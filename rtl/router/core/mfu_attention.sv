// Copyright (c) 2026
//
// Description: RTL-owned functional Attention control path for cNoC type5 flits.
//              Type4 Attention distribution writes are mirrored into a local
//              KV cache.  Type5 Attention flits capture query lanes across the
//              packet lifetime and tail flits update psum/output lanes using
//              only RTL-visible payload/KV state.  The math is intentionally a
//              compact functional model; the important ownership boundary is
//              that no C++ Packet/VCRouter state is consulted.

module mfu_attention #(
    parameter int NUM_PORTS = 5,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = 11,
    parameter int MAX_TASKS = router_ports_pkg::CNOC_MAX_TASKS,
    parameter int TASK_ID_W = router_ports_pkg::CNOC_TASK_ID_W,
    parameter int MAX_K_DIM = 128
) (
    input  logic clk,
    input  logic reset,

    input  logic        kv_wr_en_i,
    input  logic [SRAM_ADDR_W-1:0] kv_wr_addr_i,
    input  logic [31:0] kv_wr_byte_en_i,
    input  logic [FLIT_W-1:0] kv_wr_data_i,

    input  logic compute_fire_i,
    input  logic release_state_i,
    input  logic pkt_is_type5_i,
    input  logic [4:0] pkt_opcode_i,
    input  logic [FLIT_W-1:0] pkt_flit_i,
    input  router_ports_pkg::flit_meta_t pkt_meta_i,
    input  logic [VC_ID_W-1:0] pkt_vc_id_i,
    input  logic [2:0] pkt_out_sel_i,
    input  logic [15:0] kv_token_count_i,

    input  logic [5:0] task_count_i,
    input  logic [MAX_TASKS*TASK_ID_W-1:0] task_ids_flat_i,

    output logic attention_active_o,
    output logic [FLIT_W-1:0] attention_flit_o,
    output router_ports_pkg::flit_meta_t attention_meta_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam int MAX_TOKENS = (SRAM_DEPTH < 2) ? 1 : (SRAM_DEPTH / 2);

  logic [7:0] kv_bank [0:SRAM_DEPTH-1];
  logic [7:0] query_q [NUM_PORTS][VC_NUM][MAX_K_DIM];
  logic query_valid_q [NUM_PORTS][VC_NUM];
  logic attention_state_valid_q [NUM_PORTS][VC_NUM];
  logic signed [31:0] attention_running_max_q [NUM_PORTS][VC_NUM];
  logic signed [31:0] attention_running_sum_q [NUM_PORTS][VC_NUM];

  logic [9:0] data_offset;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [15:0] psum_offset;
  logic [15:0] k_dim_raw;
  logic [15:0] k_dim_safe;
  logic [15:0] k_dim_clamped;
  logic [15:0] token_stride;
  logic [15:0] max_tokens_from_sram;
  logic [15:0] active_tokens;
  logic head_like;
  logic tail_like;
  logic is_attention_op;
  logic attention_context_ready;
  logic signed [31:0] tail_running_max;
  logic signed [31:0] tail_running_sum;
  logic tail_score_valid;
  logic signed [31:0] old_running_max_q4;
  logic signed [31:0] old_running_sum_q4;
  logic signed [31:0] new_running_max_q4;
  logic signed [31:0] new_running_sum_q4;
  logic signed [31:0] old_scale_q4;
  logic signed [31:0] local_sum_exp_q4;
  logic [TASK_ID_W-1:0] task_id_by_slot [MAX_TASKS];
  logic [7:0] query_value_by_dim [MAX_K_DIM];
  logic signed [31:0] token_score_q4_by_token [MAX_TOKENS];
  logic signed [31:0] token_exp_q4_by_token [MAX_TOKENS];
  logic [16:0] flit_start_idx;
  logic [16:0] flit_end_idx;
  logic [5:0] op_query_local_lane;
  logic [31:0] op_key_addr_tmp;
  logic signed [31:0] op_query_q_tmp;
  logic signed [31:0] op_key_q_tmp;
  logic signed [31:0] op_score_accum_tmp;
  logic signed [31:0] op_score_sat_tmp;
  logic [7:0] op_sat_byte_tmp;
  logic signed [63:0] op_product_q8;
  logic signed [63:0] op_rounded_q4;
  logic signed [31:0] wb_exp_input_q4;
  logic signed [31:0] wb_exp_sat_tmp;
  logic signed [31:0] wb_mul_lhs_q;
  logic signed [31:0] wb_mul_rhs_q;
  logic signed [31:0] wb_sat_source_q;
  logic [7:0] wb_sat_byte_tmp;
  logic signed [63:0] wb_product_q8;
  logic signed [63:0] wb_rounded_q4;
  real wb_exp_value_real;
  real wb_exp_real;
  int wb_exp_quantized;

  integer reset_addr_idx;
  integer reset_port_idx;
  integer reset_vc_idx;
  integer reset_dim_idx;
  integer wr_lane_idx;
  integer q_lane_idx;
  integer task_slot_idx;
  integer token_idx;
  integer operand_task_idx;
  integer operand_dim_idx;
  integer operand_token_idx;

  assign data_offset = pkt_meta_i.data_offset;
  assign payload_len = pkt_meta_i.payload_len;
  assign effective_payload_len =
      // Padding-only tail flits carry no live query/payload lanes.  They may
      // trigger the tail-time Attention reduction, but must not override a
      // captured query lane with the zero-filled tail payload.
      (payload_len == 6'd0) ? 6'd0 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign psum_offset = pkt_meta_i.psum_offset;
  assign k_dim_raw = pkt_meta_i.k_dim;
  assign k_dim_safe =
      (k_dim_raw == 16'd0) ? 16'd1 : k_dim_raw;
  assign k_dim_clamped =
      (k_dim_safe > MAX_K_DIM[15:0]) ? MAX_K_DIM[15:0] : k_dim_safe;
  assign token_stride = k_dim_clamped << 1;
  assign max_tokens_from_sram =
      (token_stride == 16'd0) ? 16'd1 : 16'(SRAM_DEPTH / token_stride);
  assign active_tokens =
      (kv_token_count_i == 16'd0) ? 16'd0 :
      (kv_token_count_i < max_tokens_from_sram) ? kv_token_count_i :
                                                   max_tokens_from_sram;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign tail_like = flit_is_tail_like(pkt_meta_i.flit_kind);
  assign is_attention_op = pkt_is_type5_i && (pkt_opcode_i == OP_ATTENTION);
  assign attention_context_ready =
      query_valid_q[pkt_out_sel_i][pkt_vc_id_i] || head_like;
  assign attention_active_o = is_attention_op;
  assign old_running_max_q4 =
      {{24{pkt_meta_i.header_reserved[7]}}, pkt_meta_i.header_reserved[7:0]};
  assign old_running_sum_q4 =
      {{24{pkt_meta_i.header_reserved[15]}}, pkt_meta_i.header_reserved[15:8]};

  always_comb begin : proc_attention_operands
    flit_start_idx = {7'd0, data_offset};
    flit_end_idx = flit_start_idx + {11'd0, effective_payload_len};
    op_query_local_lane = 6'd0;
    op_key_addr_tmp = 32'd0;
    op_query_q_tmp = 32'sd0;
    op_key_q_tmp = 32'sd0;
    op_score_accum_tmp = 32'sd0;
    op_score_sat_tmp = 32'sd0;
    op_sat_byte_tmp = 8'd0;
    op_product_q8 = 64'sd0;
    op_rounded_q4 = 64'sd0;

    for (operand_task_idx = 0;
         operand_task_idx < MAX_TASKS;
         operand_task_idx = operand_task_idx + 1) begin
      task_id_by_slot[operand_task_idx] =
          task_ids_flat_i[operand_task_idx * TASK_ID_W +: TASK_ID_W];
    end

    for (operand_dim_idx = 0; operand_dim_idx < MAX_K_DIM; operand_dim_idx = operand_dim_idx + 1) begin
      query_value_by_dim[operand_dim_idx] =
          query_q[pkt_out_sel_i][pkt_vc_id_i][operand_dim_idx];
      // If the current flit carries this query lane, prefer the live payload so
      // tail flits can contribute their own query bytes before state is updated.
      if ((operand_dim_idx >= flit_start_idx) &&
          (operand_dim_idx < flit_end_idx)) begin
        op_query_local_lane = operand_dim_idx[5:0] - flit_start_idx[5:0];
        if (op_query_local_lane < 6'd32) begin
          query_value_by_dim[operand_dim_idx] =
              pkt_flit_i[op_query_local_lane * 8 +: 8];
        end
      end
    end

    for (operand_token_idx = 0;
         operand_token_idx < MAX_TOKENS;
         operand_token_idx = operand_token_idx + 1) begin
      op_score_accum_tmp = 32'sd0;
      token_score_q4_by_token[operand_token_idx] = 32'sd0;

      if (operand_token_idx < active_tokens) begin
        for (operand_dim_idx = 0;
             operand_dim_idx < MAX_K_DIM;
             operand_dim_idx = operand_dim_idx + 1) begin
          if (operand_dim_idx < k_dim_clamped) begin
            op_key_addr_tmp = (operand_token_idx * token_stride) + operand_dim_idx;
            op_query_q_tmp =
                {{24{query_value_by_dim[operand_dim_idx][7]}},
                 query_value_by_dim[operand_dim_idx]};
            op_key_q_tmp =
                (op_key_addr_tmp < SRAM_DEPTH) ?
                    {{24{kv_bank[op_key_addr_tmp[SRAM_ADDR_W-1:0]][7]}},
                     kv_bank[op_key_addr_tmp[SRAM_ADDR_W-1:0]]} :
                    32'sd0;
            op_product_q8 = op_query_q_tmp * op_key_q_tmp;
            if (op_product_q8 >= 64'sd0) begin
              op_rounded_q4 = (op_product_q8 + 64'sd8) >>> 4;
            end else begin
              op_rounded_q4 = -(((-op_product_q8) + 64'sd8) >>> 4);
            end
            op_score_accum_tmp = op_score_accum_tmp + op_rounded_q4[31:0];
          end
        end

        // Match the INT8 cNoC golden contract: Attention dot-product scores are
        // accumulated in Q4.4 and then clamped to one payload byte before the
        // online softmax max/exp/sum update observes them.
        if (op_score_accum_tmp > 32'sd127) begin
          op_sat_byte_tmp = 8'h7f;
        end else if (op_score_accum_tmp < -32'sd128) begin
          op_sat_byte_tmp = 8'h80;
        end else begin
          op_sat_byte_tmp = op_score_accum_tmp[7:0];
        end
        op_score_sat_tmp = {{24{op_sat_byte_tmp[7]}}, op_sat_byte_tmp};
        token_score_q4_by_token[operand_token_idx] = op_score_sat_tmp;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      for (reset_addr_idx = 0; reset_addr_idx < SRAM_DEPTH; reset_addr_idx = reset_addr_idx + 1) begin
        kv_bank[reset_addr_idx] <= 8'd0;
      end
      for (reset_port_idx = 0; reset_port_idx < NUM_PORTS; reset_port_idx = reset_port_idx + 1) begin
        for (reset_vc_idx = 0; reset_vc_idx < VC_NUM; reset_vc_idx = reset_vc_idx + 1) begin
          query_valid_q[reset_port_idx][reset_vc_idx] <= 1'b0;
          attention_state_valid_q[reset_port_idx][reset_vc_idx] <= 1'b0;
          attention_running_max_q[reset_port_idx][reset_vc_idx] <= -32'sd2147483647;
          attention_running_sum_q[reset_port_idx][reset_vc_idx] <= 32'sd0;
          for (reset_dim_idx = 0; reset_dim_idx < MAX_K_DIM; reset_dim_idx = reset_dim_idx + 1) begin
            query_q[reset_port_idx][reset_vc_idx][reset_dim_idx] <= 8'd0;
          end
        end
      end
    end else begin
      // Type4 Attention distribution writes commit into the local KV mirror.
      // This mirrors mfu_sram's KV bank so Attention compute has enough read
      // bandwidth for a functional tail-time reduction.
      if (kv_wr_en_i) begin
        for (wr_lane_idx = 0; wr_lane_idx < 32; wr_lane_idx = wr_lane_idx + 1) begin
          if (kv_wr_byte_en_i[wr_lane_idx] &&
              (kv_wr_addr_i + wr_lane_idx) < SRAM_DEPTH) begin
            kv_bank[kv_wr_addr_i + wr_lane_idx] <=
                kv_wr_data_i[wr_lane_idx * 8 +: 8];
          end
        end
      end

      if (compute_fire_i && is_attention_op) begin
        // Head/head-tail starts a fresh query stream for this output/VC.
        if (head_like) begin
          query_valid_q[pkt_out_sel_i][pkt_vc_id_i] <= 1'b1;
          attention_state_valid_q[pkt_out_sel_i][pkt_vc_id_i] <= 1'b1;
          attention_running_max_q[pkt_out_sel_i][pkt_vc_id_i] <= -32'sd2147483647;
          attention_running_sum_q[pkt_out_sel_i][pkt_vc_id_i] <= 32'sd0;
          for (q_lane_idx = 0; q_lane_idx < MAX_K_DIM; q_lane_idx = q_lane_idx + 1) begin
            query_q[pkt_out_sel_i][pkt_vc_id_i][q_lane_idx] <= 8'd0;
          end
        end

        // Any flit whose payload overlaps the query prefix updates the query
        // buffer.  Psum lanes at or after psum_offset are not query operands.
        for (q_lane_idx = 0; q_lane_idx < 32; q_lane_idx = q_lane_idx + 1) begin
          integer global_query_idx;
          global_query_idx = int'(data_offset) + q_lane_idx;
          if ((q_lane_idx < effective_payload_len) &&
              (global_query_idx < psum_offset) &&
              (global_query_idx < k_dim_clamped)) begin
            query_q[pkt_out_sel_i][pkt_vc_id_i][global_query_idx] <=
                pkt_flit_i[q_lane_idx * 8 +: 8];
          end
        end

        // Tail/head-tail is the control point where this Attention helper has
        // seen all query payload fragments for the stream.  Record the online
        // softmax running max/sum derived from RTL-owned query/KV state.
        if (tail_like && attention_context_ready) begin
          if (tail_score_valid) begin
            attention_running_max_q[pkt_out_sel_i][pkt_vc_id_i] <= new_running_max_q4;
            attention_running_sum_q[pkt_out_sel_i][pkt_vc_id_i] <= new_running_sum_q4;
          end else begin
            attention_running_max_q[pkt_out_sel_i][pkt_vc_id_i] <= -32'sd2147483647;
            attention_running_sum_q[pkt_out_sel_i][pkt_vc_id_i] <= 32'sd0;
          end
        end
      end

      if (release_state_i && is_attention_op && tail_like) begin
        query_valid_q[pkt_out_sel_i][pkt_vc_id_i] <= 1'b0;
        attention_state_valid_q[pkt_out_sel_i][pkt_vc_id_i] <= 1'b0;
        attention_running_max_q[pkt_out_sel_i][pkt_vc_id_i] <= -32'sd2147483647;
        attention_running_sum_q[pkt_out_sel_i][pkt_vc_id_i] <= 32'sd0;
      end
    end
  end

  always_comb begin : proc_attention_writeback
    logic [15:0] task_id;
    logic [16:0] target_idx;
    logic [5:0] target_lane;
    logic signed [31:0] old_output_q;
    logic signed [31:0] token_score_q;
    logic signed [31:0] token_value_q;
    logic signed [31:0] output_accum_q;
    logic signed [31:0] output_sum_q;
    logic [31:0] value_addr;
    logic [7:0] value_q_byte;

    attention_flit_o = pkt_flit_i;
    attention_meta_o = pkt_meta_i;
    task_id = 16'd0;
    target_idx = 17'd0;
    target_lane = 6'd0;
    old_output_q = 32'sd0;
    token_score_q = 32'sd0;
    token_value_q = 32'sd0;
    output_accum_q = 32'sd0;
    output_sum_q = 32'sd0;
    value_addr = 32'd0;
    value_q_byte = 8'd0;
    tail_running_max = -32'sd2147483647;
    tail_running_sum = 32'sd0;
    tail_score_valid = 1'b0;
    old_scale_q4 = 32'sd0;
    local_sum_exp_q4 = 32'sd0;
    new_running_max_q4 = old_running_max_q4;
    new_running_sum_q4 = old_running_sum_q4;
    wb_exp_input_q4 = 32'sd0;
    wb_exp_sat_tmp = 32'sd0;
    wb_mul_lhs_q = 32'sd0;
    wb_mul_rhs_q = 32'sd0;
    wb_sat_source_q = 32'sd0;
    wb_sat_byte_tmp = 8'd0;
    wb_product_q8 = 64'sd0;
    wb_rounded_q4 = 64'sd0;
    wb_exp_value_real = 0.0;
    wb_exp_real = 0.0;
    wb_exp_quantized = 0;

    for (token_idx = 0; token_idx < MAX_TOKENS; token_idx = token_idx + 1) begin
      token_exp_q4_by_token[token_idx] = 32'sd0;
    end

    // Attention output is committed on tail/head-tail.  Earlier flits only
    // populate query state and forward unchanged.
    if (is_attention_op && tail_like && attention_context_ready && active_tokens != 16'd0) begin
      for (token_idx = 0; token_idx < MAX_TOKENS; token_idx = token_idx + 1) begin
        if (token_idx < active_tokens) begin
          token_score_q = token_score_q4_by_token[token_idx];
          if (!tail_score_valid || token_score_q > tail_running_max) begin
            tail_running_max = token_score_q;
          end
          tail_running_sum = tail_running_sum + token_score_q;
          tail_score_valid = 1'b1;
        end
      end
      new_running_max_q4 =
          (old_running_max_q4 > tail_running_max) ? old_running_max_q4 : tail_running_max;

      wb_exp_input_q4 = old_running_max_q4 - new_running_max_q4;
      wb_exp_value_real = wb_exp_input_q4 / 16.0;
      wb_exp_real = $exp(wb_exp_value_real);
      wb_exp_quantized = $rtoi((wb_exp_real * 16.0) + ((wb_exp_real >= 0.0) ? 0.5 : -0.5));
      wb_exp_sat_tmp = wb_exp_quantized;
      if (wb_exp_sat_tmp > 32'sd127) begin
        wb_sat_byte_tmp = 8'h7f;
      end else if (wb_exp_sat_tmp < -32'sd128) begin
        wb_sat_byte_tmp = 8'h80;
      end else begin
        wb_sat_byte_tmp = wb_exp_sat_tmp[7:0];
      end
      old_scale_q4 = {{24{wb_sat_byte_tmp[7]}}, wb_sat_byte_tmp};

      local_sum_exp_q4 = 32'sd0;
      for (token_idx = 0; token_idx < MAX_TOKENS; token_idx = token_idx + 1) begin
        if (token_idx < active_tokens) begin
          wb_exp_input_q4 = token_score_q4_by_token[token_idx] - new_running_max_q4;
          wb_exp_value_real = wb_exp_input_q4 / 16.0;
          wb_exp_real = $exp(wb_exp_value_real);
          wb_exp_quantized = $rtoi((wb_exp_real * 16.0) + ((wb_exp_real >= 0.0) ? 0.5 : -0.5));
          wb_exp_sat_tmp = wb_exp_quantized;
          if (wb_exp_sat_tmp > 32'sd127) begin
            wb_sat_byte_tmp = 8'h7f;
          end else if (wb_exp_sat_tmp < -32'sd128) begin
            wb_sat_byte_tmp = 8'h80;
          end else begin
            wb_sat_byte_tmp = wb_exp_sat_tmp[7:0];
          end
          token_exp_q4_by_token[token_idx] = {{24{wb_sat_byte_tmp[7]}}, wb_sat_byte_tmp};

          wb_sat_source_q = local_sum_exp_q4 + token_exp_q4_by_token[token_idx];
          if (wb_sat_source_q > 32'sd127) begin
            wb_sat_byte_tmp = 8'h7f;
          end else if (wb_sat_source_q < -32'sd128) begin
            wb_sat_byte_tmp = 8'h80;
          end else begin
            wb_sat_byte_tmp = wb_sat_source_q[7:0];
          end
          local_sum_exp_q4 = {{24{wb_sat_byte_tmp[7]}}, wb_sat_byte_tmp};
        end
      end

      wb_product_q8 = old_running_sum_q4 * old_scale_q4;
      if (wb_product_q8 >= 64'sd0) begin
        wb_rounded_q4 = (wb_product_q8 + 64'sd8) >>> 4;
      end else begin
        wb_rounded_q4 = -(((-wb_product_q8) + 64'sd8) >>> 4);
      end
      wb_sat_source_q = wb_rounded_q4[31:0] + local_sum_exp_q4;
      if (wb_sat_source_q > 32'sd127) begin
        wb_sat_byte_tmp = 8'h7f;
      end else if (wb_sat_source_q < -32'sd128) begin
        wb_sat_byte_tmp = 8'h80;
      end else begin
        wb_sat_byte_tmp = wb_sat_source_q[7:0];
      end
      new_running_sum_q4 =
          {{24{wb_sat_byte_tmp[7]}}, wb_sat_byte_tmp};

      for (task_slot_idx = 0; task_slot_idx < MAX_TASKS; task_slot_idx = task_slot_idx + 1) begin
        task_id = task_id_by_slot[task_slot_idx];
        target_idx = {1'b0, psum_offset} + {1'b0, task_id};
        target_lane = target_idx[5:0] - flit_start_idx[5:0];

        if ((task_slot_idx < task_count_i) &&
            (task_id < k_dim_clamped) &&
            (target_idx >= flit_start_idx) &&
            (target_idx < flit_end_idx) &&
            (target_lane < 6'd32)) begin
          output_accum_q = 32'sd0;
          wb_mul_lhs_q =
              {{24{pkt_flit_i[(target_lane * 8) + 7]}},
               pkt_flit_i[target_lane * 8 +: 8]};
          wb_mul_rhs_q = old_scale_q4;
          wb_product_q8 = wb_mul_lhs_q * wb_mul_rhs_q;
          if (wb_product_q8 >= 64'sd0) begin
            wb_rounded_q4 = (wb_product_q8 + 64'sd8) >>> 4;
          end else begin
            wb_rounded_q4 = -(((-wb_product_q8) + 64'sd8) >>> 4);
          end
          old_output_q = wb_rounded_q4[31:0];

          for (token_idx = 0; token_idx < MAX_TOKENS; token_idx = token_idx + 1) begin
            if (token_idx < active_tokens) begin
              value_addr = (token_idx * token_stride) + k_dim_clamped + task_id;
              value_q_byte =
                  (value_addr < SRAM_DEPTH) ? kv_bank[value_addr[SRAM_ADDR_W-1:0]] : 8'd0;
              wb_mul_lhs_q = token_exp_q4_by_token[token_idx];
              wb_mul_rhs_q = {{24{value_q_byte[7]}}, value_q_byte};
              wb_product_q8 = wb_mul_lhs_q * wb_mul_rhs_q;
              if (wb_product_q8 >= 64'sd0) begin
                wb_rounded_q4 = (wb_product_q8 + 64'sd8) >>> 4;
              end else begin
                wb_rounded_q4 = -(((-wb_product_q8) + 64'sd8) >>> 4);
              end
              token_value_q = wb_rounded_q4[31:0];
              output_accum_q = output_accum_q + token_value_q;
            end
          end

          output_sum_q = old_output_q + output_accum_q;
          if (output_sum_q > 32'sd127) begin
            wb_sat_byte_tmp = 8'h7f;
          end else if (output_sum_q < -32'sd128) begin
            wb_sat_byte_tmp = 8'h80;
          end else begin
            wb_sat_byte_tmp = output_sum_q[7:0];
          end
          attention_flit_o[target_lane * 8 +: 8] = wb_sat_byte_tmp;
        end
      end

      if (new_running_max_q4 > 32'sd127) begin
        attention_meta_o.header_reserved[7:0] = 8'h7f;
      end else if (new_running_max_q4 < -32'sd128) begin
        attention_meta_o.header_reserved[7:0] = 8'h80;
      end else begin
        attention_meta_o.header_reserved[7:0] = new_running_max_q4[7:0];
      end

      if (new_running_sum_q4 > 32'sd127) begin
        attention_meta_o.header_reserved[15:8] = 8'h7f;
      end else if (new_running_sum_q4 < -32'sd128) begin
        attention_meta_o.header_reserved[15:8] = 8'h80;
      end else begin
        attention_meta_o.header_reserved[15:8] = new_running_sum_q4[7:0];
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (!reset) begin
      if (compute_fire_i && is_attention_op && !head_like &&
          !attention_state_valid_q[pkt_out_sel_i][pkt_vc_id_i]) begin
        $error("mfu_attention: body/tail Attention flit arrived without head state");
      end

      if (release_state_i && is_attention_op && tail_like &&
          attention_state_valid_q[pkt_out_sel_i][pkt_vc_id_i]) begin
        // Release is legal only after the tail/head-tail writeback point.  This
        // assertion is deliberately weak: it guards state lifetime without tying
        // the functional model to a cycle-exact softmax implementation.
        if (!query_valid_q[pkt_out_sel_i][pkt_vc_id_i]) begin
          $error("mfu_attention: release observed without a valid query stream");
        end
      end
    end
  end
`endif

endmodule
