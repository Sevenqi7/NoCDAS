// Description: Stateful MatMul/Linear ALU backend.
//              The cNoC controller provides the outer FETCH/COMPUTE stage
//              enables, while this leaf owns the operation-local task progress,
//              64-bit weight reads, accumulation, and result overlay.

module mfu_alu_matmul #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = 11,
    parameter int DATA_W = 64,
    parameter int DATA_ELEM_W = 8,
    parameter int ACC_W = 32
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
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

  localparam int DATA_ELEMS = DATA_W / DATA_ELEM_W;
  localparam logic [5:0] DATA_BYTES_Q = 6'(DATA_ELEMS);
  localparam logic [5:0] MAX_TASKS_Q = 6'(CNOC_MAX_TASKS);
  localparam logic signed [ACC_W-1:0] ACC_SAT_MAX = ACC_W'(127);
  localparam logic signed [ACC_W-1:0] ACC_SAT_MIN = -ACC_W'(128);

  logic active_q;
  logic result_valid_q;
  logic [FLIT_W-1:0] result_flit_q;
  flit_meta_t result_meta_q;
  logic [5:0] task_count_q;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] task_ids_flat_q;
  logic [15:0] weight_row_size_q;
  logic [31:0] task_weight_base_q;
  logic signed [ACC_W-1:0] accum_q [NUM_PORTS][VC_NUM][CNOC_MAX_TASKS];
  logic signed [ACC_W-1:0] task_base_q;
  logic signed [ACC_W-1:0] data_sum_q;
  logic [5:0] compute_base_idx_q;
  logic [4:0] task_idx_q;
  logic [DATA_W-1:0] weight_data_q;

  logic data_op;
  logic head_like;
  logic [5:0] effective_payload_len;
  logic [5:0] task_idx_next;
  logic compute_can_issue_data;
  logic compute_has_next_data;
  logic task_complete;
  logic has_next_task;
  logic read_next_task_data;
  logic [5:0] read_lane_idx;
  logic [15:0] read_input_idx;
  logic [31:0] read_weight_addr_full;
  logic [SRAM_ADDR_W-1:0] weight_addr_clamped;

  logic data_valid;
  logic [DATA_ELEMS-1:0] data_enable;
  logic data_last;
  logic [DATA_W-1:0] data_lhs;
  logic [DATA_W-1:0] data_rhs;
  logic [15:0] compute_input_idx;
  logic [31:0] compute_weight_addr_full;
  logic signed [7:0] data_lhs_s8 [DATA_ELEMS];
  logic signed [7:0] data_rhs_s8 [DATA_ELEMS];
  logic signed [15:0] product_s16 [DATA_ELEMS];
  logic signed [31:0] product_q8 [DATA_ELEMS];
  logic signed [31:0] rounded_q4 [DATA_ELEMS];
  logic signed [ACC_W-1:0] data_contrib [DATA_ELEMS];
  logic signed [ACC_W-1:0] data_sum;
  logic data_result_valid;
  logic data_result_last;
  logic signed [ACC_W-1:0] data_result;

  logic signed [ACC_W-1:0] data_addend;
  logic signed [ACC_W-1:0] data_sum_with_result;
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

  integer data_operand_idx;
  integer data_product_idx;
  integer reset_port_idx;
  integer reset_vc_idx;
  integer reset_task_idx;

  assign data_op = op_i.is_data_op;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign effective_payload_len =
      (pkt_meta_i.payload_len == 6'd0) ? 6'd1 :
      (pkt_meta_i.payload_len > 6'd32) ? 6'd32 :
                                         pkt_meta_i.payload_len;
  assign task_idx_next = {1'b0, task_idx_q} + 6'd1;
  assign compute_can_issue_data =
      active_q && compute_en_i && (compute_base_idx_q < 6'd32);
  assign compute_has_next_data =
      compute_can_issue_data && ((compute_base_idx_q + DATA_BYTES_Q) < 6'd32);
  assign task_complete = data_result_valid && data_result_last;
  assign has_next_task = task_idx_next < task_count_q;
  assign read_next_task_data = compute_en_i && task_complete && has_next_task;
  assign data_req_o.valid =
      (fetch_en_i && data_op && (ctx_i.task_count != 6'd0)) ||
      compute_has_next_data ||
      read_next_task_data;
  assign data_req_o.bank_sel = 1'b0;

  always_comb begin : proc_read_address
    read_lane_idx = 6'd0;
    read_input_idx = {6'd0, pkt_meta_i.data_offset};
    read_weight_addr_full = {22'd0, pkt_meta_i.data_offset};
    weight_addr_clamped = '0;

    if (compute_en_i) begin
      read_lane_idx = compute_base_idx_q + DATA_BYTES_Q;
      read_input_idx = {6'd0, pkt_meta_i.data_offset} + {10'd0, read_lane_idx};
      read_weight_addr_full = task_weight_base_q + {16'd0, read_input_idx};
      if (read_next_task_data) begin
        read_lane_idx = 6'd0;
        read_input_idx = {6'd0, pkt_meta_i.data_offset};
        read_weight_addr_full =
            task_weight_base_q + {16'd0, weight_row_size_q} +
            {16'd0, read_input_idx};
      end
    end

    if (read_weight_addr_full < SRAM_DEPTH) begin
      weight_addr_clamped = read_weight_addr_full[SRAM_ADDR_W-1:0];
    end
  end

  assign data_req_o.addr = weight_addr_clamped;

  always_comb begin : proc_data_operand
    compute_input_idx =
        {6'd0, pkt_meta_i.data_offset} + {10'd0, compute_base_idx_q};
    compute_weight_addr_full = task_weight_base_q + {16'd0, compute_input_idx};
    data_valid = compute_can_issue_data;
    data_last =
        compute_can_issue_data &&
        ((compute_base_idx_q + DATA_BYTES_Q) >= 6'd32);
    data_enable = '0;
    data_lhs = '0;
    data_rhs = weight_data_q;

    if (compute_base_idx_q < 6'd32) begin
      data_lhs = pkt_flit_i[compute_base_idx_q * 8 +: DATA_W];
    end

    for (data_operand_idx = 0; data_operand_idx < DATA_ELEMS;
         data_operand_idx = data_operand_idx + 1) begin
      compute_input_idx =
          {6'd0, pkt_meta_i.data_offset} +
          {10'd0, compute_base_idx_q} +
          16'(data_operand_idx);
      compute_weight_addr_full =
          task_weight_base_q + {16'd0, compute_input_idx};
      data_enable[data_operand_idx] =
          compute_can_issue_data &&
          ((compute_base_idx_q + 6'(data_operand_idx)) < effective_payload_len) &&
          (compute_input_idx < pkt_meta_i.psum_offset) &&
          (compute_weight_addr_full < SRAM_DEPTH);
    end
  end

  always_comb begin : proc_data_product
    data_sum = '0;

    for (data_product_idx = 0; data_product_idx < DATA_ELEMS;
         data_product_idx = data_product_idx + 1) begin
      data_lhs_s8[data_product_idx] =
          data_lhs[data_product_idx*DATA_ELEM_W +: DATA_ELEM_W];
      data_rhs_s8[data_product_idx] =
          data_rhs[data_product_idx*DATA_ELEM_W +: DATA_ELEM_W];
      product_s16[data_product_idx] =
          data_lhs_s8[data_product_idx] * data_rhs_s8[data_product_idx];
      product_q8[data_product_idx] =
          {{16{product_s16[data_product_idx][15]}}, product_s16[data_product_idx]};
      rounded_q4[data_product_idx] = 32'sd0;

      if (product_q8[data_product_idx] >= 32'sd0) begin
        rounded_q4[data_product_idx] = (product_q8[data_product_idx] + 32'sd8) >>> 4;
      end else begin
        rounded_q4[data_product_idx] =
            -(((-product_q8[data_product_idx]) + 32'sd8) >>> 4);
      end

      data_contrib[data_product_idx] = ACC_W'(rounded_q4[data_product_idx]);
      if (data_enable[data_product_idx]) begin
        data_sum = data_sum + data_contrib[data_product_idx];
      end
    end
  end

  assign data_result_valid = data_valid;
  assign data_result_last = data_last;
  assign data_result = data_sum;

  always_comb begin : proc_task_result
    data_addend = data_result_valid ? data_result : '0;
    data_sum_with_result = data_sum_q + data_addend;
    task_sum = task_base_q + data_sum_with_result;
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
        {1'b0, pkt_meta_i.psum_offset} + {1'b0, task_id_current};
    writeback_flit_start_idx = {7'd0, pkt_meta_i.data_offset};
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
    if (reset_i) begin
      active_q <= 1'b0;
      result_valid_q <= 1'b0;
      result_flit_q <= '0;
      result_meta_q <= '0;
      task_count_q <= '0;
      task_ids_flat_q <= '0;
      weight_row_size_q <= '0;
      task_weight_base_q <= '0;
      task_base_q <= '0;
      data_sum_q <= '0;
      compute_base_idx_q <= '0;
      task_idx_q <= '0;
      weight_data_q <= '0;
      for (reset_port_idx = 0; reset_port_idx < NUM_PORTS;
           reset_port_idx = reset_port_idx + 1) begin
        for (reset_vc_idx = 0; reset_vc_idx < VC_NUM;
             reset_vc_idx = reset_vc_idx + 1) begin
          for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
               reset_task_idx = reset_task_idx + 1) begin
            accum_q[reset_port_idx][reset_vc_idx][reset_task_idx] <= '0;
          end
        end
      end
    end else begin
      result_valid_q <= 1'b0;

      if (fetch_en_i && data_op) begin
        active_q <= (ctx_i.task_count != 6'd0);
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
        task_count_q <= (ctx_i.task_count > MAX_TASKS_Q) ? MAX_TASKS_Q : ctx_i.task_count;
        task_ids_flat_q <= ctx_i.task_ids_flat;
        weight_row_size_q <=
            (ctx_i.weight_row_size != 16'd0) ? ctx_i.weight_row_size :
            (pkt_meta_i.psum_offset != 16'd0) ? pkt_meta_i.psum_offset :
                                                16'd1;
        task_weight_base_q <= '0;
        compute_base_idx_q <= 6'd0;
        task_idx_q <= 5'd0;
        data_sum_q <= '0;
        weight_data_q <= data_rsp_i.rdata;
        task_base_q <= head_like ? '0 : accum_q[ctx_i.out_sel][ctx_i.vc_id][0];

        if (head_like) begin
          for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
               reset_task_idx = reset_task_idx + 1) begin
            accum_q[ctx_i.out_sel][ctx_i.vc_id][reset_task_idx] <= '0;
          end
        end

        if (ctx_i.task_count == 6'd0) begin
          active_q <= 1'b0;
          result_valid_q <= 1'b1;
        end
      end

      if (compute_en_i && active_q) begin
        if (compute_has_next_data || read_next_task_data) begin
          weight_data_q <= data_rsp_i.rdata;
        end

        if (data_result_valid) begin
          data_sum_q <= data_sum_with_result;
        end

        if (compute_can_issue_data) begin
          compute_base_idx_q <= compute_base_idx_q + DATA_BYTES_Q;
        end

        if (task_complete) begin
          accum_q[ctx_i.out_sel][ctx_i.vc_id][task_idx_q] <= task_sum_clamped;
          result_flit_q <= result_flit_after_task;
          data_sum_q <= '0;
          compute_base_idx_q <= 6'd0;
          if (has_next_task) begin
            task_idx_q <= task_idx_q + 5'd1;
            task_weight_base_q <=
                task_weight_base_q + {16'd0, weight_row_size_q};
            task_base_q <=
                head_like ? '0 :
                accum_q[ctx_i.out_sel][ctx_i.vc_id][task_idx_next[4:0]];
          end else begin
            active_q <= 1'b0;
            result_valid_q <= 1'b1;
          end
        end
      end

      if (release_state_i && data_op) begin
        for (reset_task_idx = 0; reset_task_idx < CNOC_MAX_TASKS;
             reset_task_idx = reset_task_idx + 1) begin
          accum_q[ctx_i.out_sel][ctx_i.vc_id][reset_task_idx] <= '0;
        end
      end
    end
  end

  assign busy_o = active_q;
  assign result_valid_o = result_valid_q;
  assign result_flit_o = result_flit_q;
  assign result_meta_o = result_meta_q;
  assign result_scalar_o = 32'sd0;

endmodule
