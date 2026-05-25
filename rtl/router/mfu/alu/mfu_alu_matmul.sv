// Description: Stateful MatMul/Linear ALU backend.
//              The leaf owns a parameterized set of data-op packet contexts,
//              streams 64-bit weight reads per flit, and uses the shared 8-lane
//              INT8 multiplier through mfu_alu.

module mfu_alu_matmul #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = 11,
    parameter int DATA_W = 64,
    parameter int DATA_ELEM_W = 8,
    parameter int ACC_W = 32,
    parameter int MATMUL_CTX_SLOTS = 1
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic fetch_en_i,
    input  logic compute_en_i,
    input  logic release_state_i,
    input  router_ports_pkg::mfu_alu_op_t op_i,
    input  logic [FLIT_W-1:0] pkt_flit_i,
    input  router_ports_pkg::flit_meta_t pkt_meta_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  router_ports_pkg::mfu_alu_data_rsp_t data_rsp_i,
    output router_ports_pkg::mfu_alu_data_req_t data_req_o,
    output logic busy_o,
    output logic result_valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o,
    output router_ports_pkg::matmul_ctx_status_t matmul_ctx_o,
    output logic mul_req_o,
    output logic signed [7:0] mul_lhs_o [0:7],
    output logic signed [7:0] mul_rhs_o [0:7],
    input  logic mul_rsp_valid_i,
    input  logic signed [15:0] mul_product_i [0:7]
);
  import router_ports_pkg::*;

  localparam int DATA_ELEMS = DATA_W / DATA_ELEM_W;
  localparam logic [5:0] DATA_BYTES_Q = 6'(DATA_ELEMS);
  localparam logic [5:0] MAX_TASKS_Q = 6'(CNOC_MAX_TASKS);
  localparam logic signed [ACC_W-1:0] ACC_SAT_MAX = ACC_W'(127);
  localparam logic signed [ACC_W-1:0] ACC_SAT_MIN = -ACC_W'(128);
  localparam logic [31:0] SRAM_DEPTH_U32 = 32'(SRAM_DEPTH);
  localparam int MATMUL_SLOT_W = MATMUL_CTX_SLOT_W;
  localparam int MATMUL_CTX_SLOT_GUARD = 1 / ((MATMUL_CTX_SLOTS >= 1) &&
                                              (MATMUL_CTX_SLOTS <= MATMUL_CTX_SLOTS_MAX));

  typedef enum logic [2:0] {
    MM_IDLE,
    MM_RUN,
    MM_TASK_COMMIT,
    MM_DONE
  } matmul_state_e;

  matmul_state_e state_q;

  logic result_valid_q;
  logic [FLIT_W-1:0] result_flit_q;
  logic [FLIT_W-1:0] pkt_flit_q;
  flit_meta_t result_meta_q;
  flit_meta_t pkt_meta_q;
  mfu_alu_ctx_t ctx_q;
  logic [5:0] task_count_q;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] task_ids_flat_q;
  logic [15:0] weight_row_size_q;
  logic [31:0] task_weight_base_q;
  matmul_ctx_slot_t matmul_active_slot_q [MATMUL_CTX_SLOTS_MAX];
  logic signed [7:0] accum_q [MATMUL_CTX_SLOTS_MAX][CNOC_MAX_TASKS];
  logic [MATMUL_SLOT_W-1:0] active_slot_q;
  logic signed [ACC_W-1:0] task_base_q;
  logic signed [ACC_W-1:0] data_sum_q;
  logic [5:0] compute_base_idx_q;
  logic [4:0] task_idx_q;
  logic [DATA_ELEMS-1:0] rsp_lane_enable_pipe_q [0:1];
  logic rsp_lane_last_pipe_q [0:1];
  logic signed [ACC_W-1:0] lane_sum_q;

  logic data_op;
  logic head_like;
  logic tail_like;
  logic data_payload_present;
  logic data_ctx_match;
  logic data_ctx_has_free_slot;
  logic data_ctx_head_admit_ok;
  logic data_ctx_body_admit_ok;
  logic data_ctx_ok;
  logic [MATMUL_SLOT_W-1:0] data_ctx_match_slot;
  logic [MATMUL_SLOT_W-1:0] data_ctx_alloc_slot;
  logic [MATMUL_SLOT_W-1:0] active_slot_d;
  logic [5:0] effective_payload_len;
  logic [5:0] fetch_payload_len;
  logic [15:0] fetch_data_offset_u16;
  logic [15:0] fetch_input_remaining;
  logic [5:0] fetch_input_len;
  logic [5:0] input_payload_len_q;
  logic [5:0] task_count_limited;
  logic [5:0] task_idx_next;
  logic has_next_task;
  logic [31:0] current_weight_addr_full;
  logic [31:0] data_req_addr_full;
  logic [SRAM_ADDR_W-1:0] data_req_addr;
  logic data_req_valid;
  logic issue_chunk;

  logic [5:0] load_byte_offset;
  logic [5:0] load_lane_u6;
  logic [15:0] load_input_idx;
  logic [31:0] load_weight_addr_full;
  logic [7:0] load_flit_byte [0:DATA_ELEMS-1];
  logic [7:0] load_weight_byte [0:DATA_ELEMS-1];
  logic [DATA_ELEMS-1:0] lane_enable_d;
  logic lane_last_d;
  logic signed [7:0] mul_lhs_d [0:DATA_ELEMS-1];
  logic signed [7:0] mul_rhs_d [0:DATA_ELEMS-1];

  logic signed [31:0] product_q8;
  logic signed [31:0] rounded_q4;
  logic signed [ACC_W-1:0] lane_sum_d;
  logic signed [ACC_W-1:0] task_sum;
  logic signed [ACC_W-1:0] task_sum_clamped;
  logic [7:0] task_sat_byte;

  logic [CNOC_TASK_ID_W-1:0] task_id_current;
  logic [16:0] writeback_target_idx;
  logic [16:0] writeback_flit_start_idx;
  logic [16:0] writeback_flit_end_idx;
  logic [5:0] writeback_target_lane;
  logic writeback_target_in_flit;
  logic signed [ACC_W-1:0] writeback_old_psum;
  logic signed [ACC_W-1:0] writeback_new_psum;
  logic [7:0] writeback_sat_byte;
  logic [FLIT_W-1:0] result_flit_after_task;

  logic active_slot_valid;
  logic release_match;

  int unsigned load_lane_idx;
  int unsigned reduce_lane_idx;
  int unsigned reset_lane_idx;
  int unsigned reset_task_idx;
  int unsigned slot_idx;
  int unsigned active_count;

  assign data_op = op_i.is_data_op;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign tail_like = flit_is_tail_like(pkt_meta_i.flit_kind);
  assign data_payload_present = (pkt_meta_i.payload_len != 6'd0);
  assign task_count_limited =
      (ctx_i.task_count > MAX_TASKS_Q) ? MAX_TASKS_Q : ctx_i.task_count;
  assign effective_payload_len =
      (pkt_meta_q.payload_len == 6'd0) ? 6'd1 :
      (pkt_meta_q.payload_len > 6'd32) ? 6'd32 :
                                         pkt_meta_q.payload_len;
  assign task_idx_next = {1'b0, task_idx_q} + 6'd1;
  assign has_next_task = task_idx_next < task_count_q;
  assign current_weight_addr_full =
      task_weight_base_q + {16'd0, pkt_meta_q.data_offset} +
      {26'd0, compute_base_idx_q};

  always_comb begin : proc_fetch_input_len
    fetch_payload_len =
        (pkt_meta_i.payload_len > 6'd32) ? 6'd32 : pkt_meta_i.payload_len;
    fetch_data_offset_u16 = {6'd0, pkt_meta_i.data_offset};
    fetch_input_remaining = 16'd0;
    fetch_input_len = 6'd0;

    if ((fetch_payload_len != 6'd0) &&
        (fetch_data_offset_u16 < pkt_meta_i.psum_offset)) begin
      fetch_input_remaining = pkt_meta_i.psum_offset - fetch_data_offset_u16;
      if (fetch_input_remaining >= {10'd0, fetch_payload_len}) begin
        fetch_input_len = fetch_payload_len;
      end else begin
        fetch_input_len = fetch_input_remaining[5:0];
      end
    end
  end

  always_comb begin : proc_ctx_lookup
    int unsigned lookup_slot_idx;

    data_ctx_match = 1'b0;
    data_ctx_has_free_slot = 1'b0;
    data_ctx_match_slot = '0;
    data_ctx_alloc_slot = '0;
    active_slot_d = '0;
    active_count = 0;

    for (lookup_slot_idx = 0; lookup_slot_idx < MATMUL_CTX_SLOTS_MAX; lookup_slot_idx = lookup_slot_idx + 1) begin
      if ((lookup_slot_idx < MATMUL_CTX_SLOTS) && matmul_active_slot_q[lookup_slot_idx].valid) begin
        active_count = active_count + 1;
        if (!data_ctx_match &&
            (matmul_active_slot_q[lookup_slot_idx].stream_port == ctx_i.stream_port) &&
            (matmul_active_slot_q[lookup_slot_idx].stream_vc == ctx_i.stream_vc)) begin
          data_ctx_match = 1'b1;
          data_ctx_match_slot = MATMUL_SLOT_W'(lookup_slot_idx);
        end
      end
    end

    for (lookup_slot_idx = 0; lookup_slot_idx < MATMUL_CTX_SLOTS_MAX; lookup_slot_idx = lookup_slot_idx + 1) begin
      if (!data_ctx_has_free_slot &&
          (lookup_slot_idx < MATMUL_CTX_SLOTS) &&
          !matmul_active_slot_q[lookup_slot_idx].valid) begin
        data_ctx_has_free_slot = 1'b1;
        data_ctx_alloc_slot = MATMUL_SLOT_W'(lookup_slot_idx);
      end
    end

    if (data_ctx_match) begin
      active_slot_d = data_ctx_match_slot;
    end else begin
      active_slot_d = data_ctx_alloc_slot;
    end
  end

  assign data_ctx_head_admit_ok = data_ctx_match || data_ctx_has_free_slot;
  assign data_ctx_body_admit_ok = data_ctx_match;
  assign data_ctx_ok = head_like ? data_ctx_head_admit_ok : data_ctx_body_admit_ok;
  assign active_slot_valid =
      (active_slot_q < MATMUL_CTX_SLOTS) &&
      matmul_active_slot_q[active_slot_q].valid;
  assign release_match =
      data_ctx_match &&
      (data_ctx_match_slot < MATMUL_CTX_SLOTS) &&
      matmul_active_slot_q[data_ctx_match_slot].valid;

  always_comb begin : proc_ctx_status_pack
    int unsigned pack_slot_idx;

    matmul_ctx_o = '0;
    matmul_ctx_o.any_valid = 1'b0;
    matmul_ctx_o.has_free_slot = (active_count < MATMUL_CTX_SLOTS);
    matmul_ctx_o.match = data_ctx_match;
    matmul_ctx_o.match_slot = data_ctx_match_slot;
    for (pack_slot_idx = 0; pack_slot_idx < MATMUL_CTX_SLOTS_MAX; pack_slot_idx = pack_slot_idx + 1) begin
      if (pack_slot_idx < MATMUL_CTX_SLOTS) begin
        matmul_ctx_o.slot_valid[pack_slot_idx] = matmul_active_slot_q[pack_slot_idx].valid;
        matmul_ctx_o.slot_stream_port_flat[pack_slot_idx * 3 +: 3] =
            matmul_active_slot_q[pack_slot_idx].stream_port;
        matmul_ctx_o.slot_stream_vc_flat[pack_slot_idx * VC_ID_W +: VC_ID_W] =
            matmul_active_slot_q[pack_slot_idx].stream_vc;
        matmul_ctx_o.any_valid =
            matmul_ctx_o.any_valid || matmul_active_slot_q[pack_slot_idx].valid;
      end else begin
        matmul_ctx_o.slot_valid[pack_slot_idx] = 1'b0;
        matmul_ctx_o.slot_stream_port_flat[pack_slot_idx * 3 +: 3] = '0;
        matmul_ctx_o.slot_stream_vc_flat[pack_slot_idx * VC_ID_W +: VC_ID_W] = '0;
      end
    end
  end

  always_comb begin : proc_data_request
    data_req_valid = 1'b0;
    data_req_addr_full = 32'd0;
    data_req_addr = '0;

    if (fetch_en_i && data_op && (fetch_input_len != 6'd0) && data_ctx_ok &&
        (task_count_limited != 6'd0) && (state_q == MM_IDLE)) begin
      data_req_valid = 1'b1;
      data_req_addr_full = {16'd0, pkt_meta_i.data_offset};
    end else if ((state_q == MM_RUN) && issue_chunk) begin
      data_req_valid = 1'b1;
      data_req_addr_full = current_weight_addr_full;
    end

    if (data_req_addr_full < SRAM_DEPTH_U32) begin
      data_req_addr = data_req_addr_full[SRAM_ADDR_W-1:0];
    end
  end

  assign data_req_o.valid = data_req_valid;
  assign data_req_o.bank_sel = 1'b0;
  assign data_req_o.addr = data_req_addr;
  assign issue_chunk =
      compute_en_i && (state_q == MM_RUN) && (compute_base_idx_q < input_payload_len_q);
  assign mul_req_o = issue_chunk;
  assign mul_lhs_o = mul_lhs_d;
  assign mul_rhs_o = mul_rhs_d;

  always_comb begin : proc_load_operands
    load_byte_offset = pkt_meta_q.data_offset[5:0] + compute_base_idx_q;
    lane_enable_d = '0;
    lane_last_d = (compute_base_idx_q + DATA_BYTES_Q) >= input_payload_len_q;

    for (load_lane_idx = 0; load_lane_idx < DATA_ELEMS;
         load_lane_idx = load_lane_idx + 1) begin
      load_lane_u6 = 6'(load_lane_idx);
      load_input_idx =
          {6'd0, pkt_meta_q.data_offset} +
          {10'd0, compute_base_idx_q} +
          16'(load_lane_idx);
      load_weight_addr_full =
          task_weight_base_q + {16'd0, load_input_idx};
      load_flit_byte[load_lane_idx] =
          pkt_flit_q[(load_byte_offset + load_lane_u6) * 8 +: 8];
      load_weight_byte[load_lane_idx] =
          data_rsp_i.rdata[load_lane_idx * DATA_ELEM_W +: DATA_ELEM_W];
      lane_enable_d[load_lane_idx] =
          ((compute_base_idx_q + load_lane_u6) < input_payload_len_q) &&
          (load_input_idx < pkt_meta_q.psum_offset) &&
          (load_weight_addr_full < SRAM_DEPTH_U32);
      mul_lhs_d[load_lane_idx] = $signed(load_flit_byte[load_lane_idx]);
      mul_rhs_d[load_lane_idx] = $signed(load_weight_byte[load_lane_idx]);
    end
  end

  always_comb begin : proc_lane_reduce
    lane_sum_d = '0;
    product_q8 = 32'sd0;
    rounded_q4 = 32'sd0;

    for (reduce_lane_idx = 0; reduce_lane_idx < DATA_ELEMS;
         reduce_lane_idx = reduce_lane_idx + 1) begin
      product_q8 =
          {{16{mul_product_i[reduce_lane_idx][15]}}, mul_product_i[reduce_lane_idx]};
      if (product_q8 >= 32'sd0) begin
        rounded_q4 = (product_q8 + 32'sd8) >>> 4;
      end else begin
        rounded_q4 = -(((-product_q8) + 32'sd8) >>> 4);
      end
      if (rsp_lane_enable_pipe_q[1][reduce_lane_idx]) begin
        lane_sum_d = lane_sum_d + ACC_W'(rounded_q4);
      end
    end
  end

  always_comb begin : proc_task_result
    task_sum = task_base_q + data_sum_q + lane_sum_q;
    task_sat_byte = 8'd0;
    if (task_sum > ACC_SAT_MAX) begin
      task_sat_byte = 8'h7f;
    end else if (task_sum < ACC_SAT_MIN) begin
      task_sat_byte = 8'h80;
    end else begin
      task_sat_byte = task_sum[7:0];
    end
    task_sum_clamped = {{(ACC_W-8){task_sat_byte[7]}}, task_sat_byte};
  end

  always_comb begin : proc_incremental_writeback
    task_id_current =
        task_ids_flat_q[task_idx_q * CNOC_TASK_ID_W +: CNOC_TASK_ID_W];
    writeback_target_idx =
        {1'b0, pkt_meta_q.psum_offset} + {1'b0, task_id_current};
    writeback_flit_start_idx = {7'd0, pkt_meta_q.data_offset};
    writeback_flit_end_idx =
        writeback_flit_start_idx + {11'd0, effective_payload_len};
    writeback_target_lane =
        writeback_target_idx[5:0] - writeback_flit_start_idx[5:0];
    writeback_target_in_flit =
        ({1'b0, task_idx_q} < task_count_q) &&
        (writeback_target_idx >= writeback_flit_start_idx) &&
        (writeback_target_idx < writeback_flit_end_idx) &&
        (writeback_target_lane < 6'd32);
    writeback_old_psum = '0;
    writeback_new_psum = '0;
    writeback_sat_byte = 8'd0;
    result_flit_after_task = result_flit_q;

    if (writeback_target_in_flit) begin
      writeback_old_psum =
          ACC_W'($signed(result_flit_q[writeback_target_lane * 8 +: 8]));
      writeback_new_psum = writeback_old_psum + task_sum_clamped;
      if (writeback_new_psum > ACC_SAT_MAX) begin
        writeback_sat_byte = 8'h7f;
      end else if (writeback_new_psum < ACC_SAT_MIN) begin
        writeback_sat_byte = 8'h80;
      end else begin
        writeback_sat_byte = writeback_new_psum[7:0];
      end
      result_flit_after_task[writeback_target_lane * 8 +: 8] =
          writeback_sat_byte;
    end
  end

  always_ff @(posedge clk_i) begin : proc_matmul_registers
    int unsigned reset_slot_idx;

    if (reset_i) begin
      state_q <= MM_IDLE;
      result_valid_q <= 1'b0;
      result_flit_q <= '0;
      pkt_flit_q <= '0;
      result_meta_q <= '0;
      pkt_meta_q <= '0;
      ctx_q <= '0;
      task_count_q <= '0;
      task_ids_flat_q <= '0;
      weight_row_size_q <= '0;
      task_weight_base_q <= '0;
      input_payload_len_q <= '0;
      active_slot_q <= '0;
      task_base_q <= '0;
      data_sum_q <= '0;
      compute_base_idx_q <= '0;
      task_idx_q <= '0;
      lane_sum_q <= '0;
      for (reset_lane_idx = 0; reset_lane_idx < DATA_ELEMS;
           reset_lane_idx = reset_lane_idx + 1) begin
        rsp_lane_enable_pipe_q[0][reset_lane_idx] <= 1'b0;
        rsp_lane_enable_pipe_q[1][reset_lane_idx] <= 1'b0;
      end
      rsp_lane_last_pipe_q[0] <= 1'b0;
      rsp_lane_last_pipe_q[1] <= 1'b0;
      for (reset_slot_idx = 0; reset_slot_idx < MATMUL_CTX_SLOTS_MAX; reset_slot_idx = reset_slot_idx + 1) begin
        matmul_active_slot_q[reset_slot_idx] <= '0;
        for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
             reset_task_idx = reset_task_idx + 1) begin
          accum_q[reset_slot_idx][reset_task_idx] <= '0;
        end
      end
    end else begin
      result_valid_q <= 1'b0;

      if (release_state_i && data_op && tail_like && release_match) begin
        matmul_active_slot_q[data_ctx_match_slot] <= '0;
        for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
             reset_task_idx = reset_task_idx + 1) begin
          accum_q[data_ctx_match_slot][reset_task_idx] <= '0;
        end
      end

      if (fetch_en_i && data_op && data_ctx_ok && (state_q == MM_IDLE)) begin
        pkt_meta_q <= pkt_meta_i;
        ctx_q <= ctx_i;
        pkt_flit_q <= pkt_flit_i;
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
        task_count_q <= task_count_limited;
        task_ids_flat_q <= ctx_i.task_ids_flat;
        input_payload_len_q <= fetch_input_len;
        weight_row_size_q <=
            (ctx_i.weight_row_size != 16'd0) ? ctx_i.weight_row_size :
            (pkt_meta_i.psum_offset != 16'd0) ? pkt_meta_i.psum_offset :
                                                16'd1;
        task_weight_base_q <= 32'd0;
        compute_base_idx_q <= 6'd0;
        task_idx_q <= 5'd0;
        data_sum_q <= '0;
        lane_sum_q <= '0;
        rsp_lane_enable_pipe_q[0] <= '0;
        rsp_lane_enable_pipe_q[1] <= '0;
        rsp_lane_last_pipe_q[0] <= 1'b0;
        rsp_lane_last_pipe_q[1] <= 1'b0;
        active_slot_q <= active_slot_d;
        task_base_q <= head_like ? '0 :
            {{(ACC_W-8){accum_q[active_slot_d][0][7]}}, accum_q[active_slot_d][0]};

        if (head_like && !data_ctx_match) begin
          matmul_active_slot_q[active_slot_d].valid <= 1'b1;
          matmul_active_slot_q[active_slot_d].stream_port <= ctx_i.stream_port;
          matmul_active_slot_q[active_slot_d].stream_vc <= ctx_i.stream_vc;
          for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
               reset_task_idx = reset_task_idx + 1) begin
            accum_q[active_slot_d][reset_task_idx] <= '0;
          end
        end

        if ((ctx_i.task_count == 6'd0) || !data_payload_present) begin
          state_q <= MM_DONE;
        end else if (fetch_input_len == 6'd0) begin
          state_q <= MM_TASK_COMMIT;
        end else begin
          state_q <= MM_RUN;
        end
      end else begin
        unique case (state_q)
          MM_RUN: begin
            rsp_lane_enable_pipe_q[1] <= rsp_lane_enable_pipe_q[0];
            rsp_lane_last_pipe_q[1] <= rsp_lane_last_pipe_q[0];
            if (issue_chunk) begin
              rsp_lane_enable_pipe_q[0] <= lane_enable_d;
              rsp_lane_last_pipe_q[0] <= lane_last_d;
              compute_base_idx_q <= compute_base_idx_q + DATA_BYTES_Q;
            end else begin
              rsp_lane_enable_pipe_q[0] <= '0;
              rsp_lane_last_pipe_q[0] <= 1'b0;
            end

            if (mul_rsp_valid_i) begin
              if (rsp_lane_last_pipe_q[1]) begin
                lane_sum_q <= lane_sum_d;
                state_q <= MM_TASK_COMMIT;
              end else begin
                data_sum_q <= data_sum_q + lane_sum_d;
              end
            end
          end
          MM_TASK_COMMIT: begin
            accum_q[active_slot_q][task_idx_q] <= task_sat_byte;
            result_flit_q <= result_flit_after_task;
            data_sum_q <= '0;
            compute_base_idx_q <= 6'd0;
            lane_sum_q <= '0;
            rsp_lane_enable_pipe_q[0] <= '0;
            rsp_lane_enable_pipe_q[1] <= '0;
            rsp_lane_last_pipe_q[0] <= 1'b0;
            rsp_lane_last_pipe_q[1] <= 1'b0;
            if (has_next_task) begin
              task_idx_q <= task_idx_q + 5'd1;
              task_weight_base_q <=
                  task_weight_base_q + {16'd0, weight_row_size_q};
              task_base_q <=
                  {{(ACC_W-8){accum_q[active_slot_q][task_idx_next[4:0]][7]}},
                   accum_q[active_slot_q][task_idx_next[4:0]]};
              state_q <= (input_payload_len_q == 6'd0) ? MM_TASK_COMMIT : MM_RUN;
            end else begin
              state_q <= MM_DONE;
            end
          end
          MM_DONE: begin
            result_valid_q <= 1'b1;
            state_q <= MM_IDLE;
          end
          default: begin
            state_q <= MM_IDLE;
          end
        endcase
      end
    end
  end

  assign busy_o = (state_q != MM_IDLE);
  assign result_valid_o = result_valid_q;
  assign result_flit_o = result_flit_q;
  assign result_meta_o = result_meta_q;
  assign result_scalar_o = 32'sd0;

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_matmul_assertions
    if (!reset_i) begin
      if (fetch_en_i && data_op && (state_q == MM_IDLE) && !data_ctx_ok) begin
        $error("mfu_alu_matmul: data-op fetch arrived without match or free slot");
      end

      if (release_state_i && data_op && !tail_like) begin
        $error("mfu_alu_matmul: data-op state release without tail-like flit");
      end

      if (fetch_en_i && data_op && !head_like && !data_ctx_match && (state_q == MM_IDLE)) begin
        $error("mfu_alu_matmul: body/tail data-op cannot allocate a new slot");
      end
    end
  end
`endif

endmodule
