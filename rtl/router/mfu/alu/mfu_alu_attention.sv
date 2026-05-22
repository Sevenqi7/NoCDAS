// Copyright (c) 2026
//
// Description: Stateful streaming Attention ALU leaf.
//              The leaf owns one active Attention query context plus the
//              running online-softmax/value state for that owner, and streams
//              K/V data from the shared MFU SRAM through the generic ALU data
//              request interface.  It deliberately does not mirror type4 KV
//              writes into local storage; mfu_sram is the only persistent KV
//              source.

module mfu_alu_attention #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = router_ports_pkg::CNOC_SRAM_ADDR_W,
    parameter int DATA_W = router_ports_pkg::CNOC_SRAM_DATA_W,
    parameter int MAX_TASKS = router_ports_pkg::CNOC_MAX_TASKS,
    parameter int TASK_ID_W = router_ports_pkg::CNOC_TASK_ID_W,
    parameter int MAX_K_DIM = 128
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic fetch_en_i,
    input  logic compute_en_i,
    input  logic release_state_i,
    input  router_ports_pkg::mfu_alu_op_t op_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  logic [FLIT_W-1:0] pkt_flit_i,
    input  router_ports_pkg::flit_meta_t pkt_meta_i,
    input  router_ports_pkg::mfu_alu_data_rsp_t data_rsp_i,
    output router_ports_pkg::mfu_alu_data_req_t data_req_o,
    output logic busy_o,
    output logic result_valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic attention_ctx_valid_o,
    output logic [2:0] attention_ctx_stream_port_o,
    output logic [VC_ID_W-1:0] attention_ctx_stream_vc_o,
    output logic mul_req_o,
    output logic signed [7:0] mul_lhs_o [0:7],
    output logic signed [7:0] mul_rhs_o [0:7],
    input  logic mul_rsp_valid_i,
    input  logic signed [15:0] mul_product_i [0:7]
);
  import router_ports_pkg::*;

`ifdef NONLINEAR_IMPL_DPI
  import "DPI-C" function int dpi_exp(input int x_q);
`endif

  localparam int DATA_BYTES = DATA_W / 8;
  localparam int K_DIM_IDX_W = (MAX_K_DIM <= 2) ? 1 : $clog2(MAX_K_DIM);
  localparam logic [5:0] DATA_BYTES_Q = 6'(DATA_BYTES);
  localparam logic [5:0] MAX_TASKS_Q = 6'(MAX_TASKS);
  localparam logic [31:0] SRAM_DEPTH_U32 = 32'(SRAM_DEPTH);
  localparam logic signed [31:0] SCORE_SAT_MAX = 32'sd127;
  localparam logic signed [31:0] SCORE_SAT_MIN = -32'sd128;
  localparam logic signed [31:0] NEG_INF_Q4 = -32'sd2147483647;

  typedef enum logic [3:0] {
    ATT_IDLE,
    ATT_QUERY_WRITE,
    ATT_SCORE_K,
    ATT_SCORE_MUL,
    ATT_SCORE_MUL_WAIT,
    ATT_SCORE_COMMIT,
    ATT_UPDATE_OLD_EXP,
    ATT_UPDATE_TOKEN_EXP,
    ATT_UPDATE_MUL,
    ATT_UPDATE_MUL_WAIT,
    ATT_UPDATE_COMMIT,
    ATT_VALUE_V,
    ATT_VALUE_MUL,
    ATT_VALUE_MUL_WAIT,
    ATT_VALUE_COMMIT,
    ATT_RESULT
  } attention_state_e;

  attention_state_e state_q;

  localparam int QUERY_CHUNK_BYTES = DATA_BYTES;
  localparam int QUERY_CHUNK_NUM = (MAX_K_DIM + QUERY_CHUNK_BYTES - 1) / QUERY_CHUNK_BYTES;
  localparam int QUERY_CHUNK_IDX_W = (QUERY_CHUNK_NUM <= 1) ? 1 : $clog2(QUERY_CHUNK_NUM);

  // --------------------------------------------------------------------------
  // Attention owner/query retention state.
  // --------------------------------------------------------------------------
  logic query_ctx_valid_q;
  logic [2:0] query_ctx_stream_port_q;
  logic [VC_ID_W-1:0] query_ctx_stream_vc_q;
  logic [DATA_W-1:0] query_chunk_q [0:QUERY_CHUNK_NUM-1];

  // --------------------------------------------------------------------------
  // Packet/context capture and address-generation state.
  // --------------------------------------------------------------------------
  flit_meta_t pkt_meta_q;
  mfu_alu_ctx_t ctx_q;
  logic [5:0] task_count_q;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] task_ids_flat_q;
  logic [15:0] k_dim_q;
  logic [5:0] effective_payload_len_q;
  logic [15:0] token_stride_q;
  logic [15:0] active_tokens_q;
  logic [16:0] flit_start_idx_q;
  logic [16:0] flit_end_idx_q;
  logic [15:0] token_idx_q;
  logic [31:0] token_base_addr_q;
  logic [15:0] k_base_idx_q;
  logic [4:0] task_idx_q;
  logic [DATA_W-1:0] k_data_q;
  logic [DATA_W-1:0] v_data_q;
  logic signed [31:0] score_accum_q;
  logic signed [31:0] running_max_q;
  logic signed [7:0] running_sum_q;
  logic signed [7:0] old_scale_q;
  logic signed [7:0] token_exp_q;
  logic signed [7:0] output_accum_q [MAX_TASKS];
  logic [FLIT_W-1:0] result_flit_q;
  flit_meta_t result_meta_q;
  logic result_valid_q;

  // --------------------------------------------------------------------------
  // Attention admission / owner-compatibility view for the current flit.
  // --------------------------------------------------------------------------
  logic is_attention_op;
  logic head_like;
  logic tail_like;
  logic attention_owner_match;
  logic attention_head_ctx_ok;
  logic attention_body_tail_ctx_ok;
  logic attention_ctx_ok;
  logic [5:0] effective_payload_len;
  logic [15:0] k_dim_safe;
  logic [15:0] k_dim_clamped;
  logic [15:0] token_stride;
  logic token_stride_supported;
  logic [15:0] max_tokens_clamped;
  logic [15:0] active_tokens;
  logic [5:0] task_count_limited;
  logic start_attention_tail;
  logic start_attention_passthrough;
  logic fetch_attention_ctx_ok_q;
  logic fetch_attention_head_ctx_ok_q;
  logic fetch_start_tail_q;
  logic fetch_passthrough_q;

  logic score_has_next_data;
  logic [15:0] score_next_base_idx;
  logic [31:0] token_base_addr_full;
  logic [31:0] token_base_addr_next;
  logic [31:0] score_read_addr_full;
  logic [31:0] value_read_addr_full;
  logic [31:0] value_next_read_addr_full;
  logic [31:0] next_token_read_addr_full;
  logic [31:0] data_req_full_addr;
  logic [SRAM_ADDR_W-1:0] data_req_addr;
  logic data_req_valid;

  // --------------------------------------------------------------------------
  // Score path state and shared-multiplier request/response storage.
  // --------------------------------------------------------------------------
  logic signed [18:0] score_data_sum;
  logic signed [31:0] score_sum_with_data;
  logic signed [7:0] score_sat;
  logic [7:0] score_sat_byte;
  logic signed [15:0] product_q8;
  logic signed [15:0] rounded_q4;
  logic [15:0] score_dim_idx;
  logic [QUERY_CHUNK_IDX_W-1:0] score_chunk_idx;
  logic [2:0] score_chunk_byte_idx;
  logic [DATA_W-1:0] score_query_chunk_q;
  logic signed [15:0] mul_product_q [0:7];
  logic signed [7:0] mul_lhs_comb [0:7];
  logic signed [7:0] mul_rhs_comb [0:7];

  // --------------------------------------------------------------------------
  // Online softmax update state.
  // --------------------------------------------------------------------------
  logic signed [31:0] update_new_max;
  logic signed [7:0] update_running_sum;
  logic signed [15:0] update_product_q8;
  logic signed [15:0] update_rounded_q4;
  logic signed [16:0] update_sum_source;
  logic [7:0] update_sat_byte;
  logic signed [31:0] softmax_exp_input;
  logic signed [7:0] softmax_exp_result;
  logic signed [7:0] softmax_exp_idx;
  logic signed [7:0] softmax_exp_lut_q4;
`ifdef NONLINEAR_IMPL_DPI
  int softmax_dpi_tmp;
`endif

  // --------------------------------------------------------------------------
  // Value accumulation and writeback state.
  // --------------------------------------------------------------------------
  logic [TASK_ID_W-1:0] task_id_current;
  logic [TASK_ID_W-1:0] task_id_next;
  logic [5:0] task_idx_next;
  logic has_next_task;
  logic value_has_next_task;
  logic value_last_task;
  logic token_has_next;
  logic value_task_in_flit;
  logic [16:0] value_target_idx;
  logic [5:0] value_target_lane;
  logic signed [15:0] value_old_scaled;
  logic signed [15:0] value_new_contrib;
  logic signed [16:0] value_output_sum;
  logic signed [15:0] value_product_q8;
  logic signed [15:0] value_rounded_q4;
  logic signed [7:0] value_byte_q4;
  logic [7:0] value_sat_byte;

  int unsigned reset_chunk_idx;
  int unsigned clear_chunk_idx;
  logic [QUERY_CHUNK_IDX_W-1:0] write_chunk_idx;
  int unsigned write_byte_idx;
  int unsigned reset_task_idx;
  int unsigned reset_mul_idx;
  int unsigned score_lane_idx;
  int unsigned mul_lane_idx;

  assign is_attention_op = op_i.is_attention;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign tail_like = flit_is_tail_like(pkt_meta_i.flit_kind);
  assign attention_owner_match =
      query_ctx_valid_q &&
      (query_ctx_stream_port_q == ctx_i.stream_port) &&
      (query_ctx_stream_vc_q == ctx_i.stream_vc);
  assign attention_head_ctx_ok =
      head_like && (!query_ctx_valid_q || attention_owner_match);
  assign attention_body_tail_ctx_ok =
      !head_like && attention_owner_match;
  assign attention_ctx_ok =
      attention_head_ctx_ok || attention_body_tail_ctx_ok;
  assign effective_payload_len =
      (pkt_meta_i.payload_len == 6'd0) ? 6'd0 :
      (pkt_meta_i.payload_len > 6'd32) ? 6'd32 :
                                         pkt_meta_i.payload_len;
  assign k_dim_safe = (pkt_meta_i.k_dim == 16'd0) ? 16'd1 : pkt_meta_i.k_dim;
  assign k_dim_clamped =
      (k_dim_safe > MAX_K_DIM[15:0]) ? MAX_K_DIM[15:0] : k_dim_safe;
  assign token_stride = k_dim_clamped << 1;
  assign active_tokens =
      (ctx_i.kv_token_count < max_tokens_clamped) ? ctx_i.kv_token_count :
                                                    max_tokens_clamped;
  assign task_count_limited =
      (ctx_i.task_count > MAX_TASKS_Q) ? MAX_TASKS_Q : ctx_i.task_count;
  assign start_attention_tail =
      fetch_en_i && is_attention_op && tail_like && attention_ctx_ok &&
      (active_tokens != 16'd0);
  assign start_attention_passthrough =
      fetch_en_i && is_attention_op &&
      (!tail_like || !attention_ctx_ok || (active_tokens == 16'd0));

  assign score_next_base_idx = k_base_idx_q + {10'd0, DATA_BYTES_Q};
  assign score_has_next_data =
      (state_q == ATT_SCORE_COMMIT) && compute_en_i && (score_next_base_idx < k_dim_q);
  assign token_base_addr_full = token_base_addr_q;
  assign token_base_addr_next = token_base_addr_q + {16'd0, token_stride_q};
  assign score_read_addr_full = token_base_addr_full + {16'd0, score_next_base_idx};
  assign task_idx_next = {1'b0, task_idx_q} + 6'd1;
  assign has_next_task = task_idx_next < task_count_q;
  assign value_has_next_task =
      (state_q == ATT_VALUE_COMMIT) && compute_en_i && has_next_task;
  assign value_last_task =
      (state_q == ATT_VALUE_COMMIT) && compute_en_i && !value_has_next_task;
  assign token_has_next = (token_idx_q + 16'd1) < active_tokens_q;
  assign task_id_current =
      task_ids_flat_q[task_idx_q * TASK_ID_W +: TASK_ID_W];
  assign task_id_next =
      has_next_task ? task_ids_flat_q[task_idx_next[4:0] * TASK_ID_W +: TASK_ID_W] :
                      '0;
  assign value_read_addr_full =
      token_base_addr_full + {16'd0, k_dim_q} + {16'd0, task_id_current};
  assign value_next_read_addr_full =
      token_base_addr_full + {16'd0, k_dim_q} + {16'd0, task_id_next};
  assign next_token_read_addr_full = token_base_addr_next;
  assign attention_ctx_valid_o = query_ctx_valid_q;
  assign attention_ctx_stream_port_o = query_ctx_stream_port_q;
  assign attention_ctx_stream_vc_o = query_ctx_stream_vc_q;

  // --------------------------------------------------------------------------
  // Attention token-capacity / address sideband.
  // --------------------------------------------------------------------------
  always_comb begin : proc_token_capacity
    token_stride_supported = 1'b1;
    case (token_stride)
      16'd2:   max_tokens_clamped = 16'(SRAM_DEPTH >> 1);
      16'd4:   max_tokens_clamped = 16'(SRAM_DEPTH >> 2);
      16'd8:   max_tokens_clamped = 16'(SRAM_DEPTH >> 3);
      16'd16:  max_tokens_clamped = 16'(SRAM_DEPTH >> 4);
      16'd32:  max_tokens_clamped = 16'(SRAM_DEPTH >> 5);
      16'd64:  max_tokens_clamped = 16'(SRAM_DEPTH >> 6);
      16'd128: max_tokens_clamped = 16'(SRAM_DEPTH >> 7);
      16'd256: max_tokens_clamped = 16'(SRAM_DEPTH >> 8);
      default: begin
        max_tokens_clamped = 16'd1;
        token_stride_supported = 1'b0;
      end
    endcase
    if (max_tokens_clamped == 16'd0) begin
      max_tokens_clamped = 16'd1;
    end
  end

  // --------------------------------------------------------------------------
  // Shared SRAM request generation.
  // --------------------------------------------------------------------------
  always_comb begin : proc_data_request
    data_req_valid = 1'b0;
    data_req_full_addr = 32'd0;
    data_req_addr = '0;

    if (start_attention_tail) begin
      data_req_valid = 1'b1;
      data_req_full_addr = 32'd0;
    end else if (score_has_next_data) begin
      data_req_valid = 1'b1;
      data_req_full_addr = score_read_addr_full;
    end else if ((state_q == ATT_UPDATE_COMMIT) && compute_en_i &&
                 (task_count_q != 6'd0)) begin
      data_req_valid = 1'b1;
      data_req_full_addr = value_read_addr_full;
    end else if (value_has_next_task) begin
      data_req_valid = 1'b1;
      data_req_full_addr = value_next_read_addr_full;
    end else if (value_last_task && token_has_next) begin
      data_req_valid = 1'b1;
      data_req_full_addr = next_token_read_addr_full;
    end else if ((state_q == ATT_UPDATE_COMMIT) && compute_en_i &&
                 (task_count_q == 6'd0) && token_has_next) begin
      data_req_valid = 1'b1;
      data_req_full_addr = next_token_read_addr_full;
    end

    if (data_req_full_addr < SRAM_DEPTH_U32) begin
      data_req_addr = data_req_full_addr[SRAM_ADDR_W-1:0];
    end
  end

  assign data_req_o.valid = data_req_valid;
  assign data_req_o.bank_sel = 1'b1;
  assign data_req_o.addr = data_req_addr;
  assign mul_req_o =
      (state_q == ATT_SCORE_MUL) ||
      (state_q == ATT_UPDATE_MUL) ||
      (state_q == ATT_VALUE_MUL);
  assign mul_lhs_o = mul_lhs_comb;
  assign mul_rhs_o = mul_rhs_comb;

  // --------------------------------------------------------------------------
  // Shared exp LUT for sequential online-softmax reuse.
  // --------------------------------------------------------------------------
  mfu_lut_exp exp_lut_i (
      .exp_idx_i(softmax_exp_idx),
      .exp_q4_o (softmax_exp_lut_q4)
  );

  // --------------------------------------------------------------------------
  // Shared-multiplier operand issue.
  // --------------------------------------------------------------------------
  always_comb begin : proc_mul_operands
    score_dim_idx = 16'd0;
    score_chunk_idx = '0;
    score_chunk_byte_idx = '0;
    for (mul_lane_idx = 0; mul_lane_idx < 8; mul_lane_idx = mul_lane_idx + 1) begin
      mul_lhs_comb[mul_lane_idx] = 8'sd0;
      mul_rhs_comb[mul_lane_idx] = 8'sd0;
    end

    if (state_q == ATT_SCORE_MUL) begin
      for (mul_lane_idx = 0; mul_lane_idx < DATA_BYTES; mul_lane_idx = mul_lane_idx + 1) begin
        score_dim_idx = k_base_idx_q + 16'(mul_lane_idx);
        score_chunk_idx = QUERY_CHUNK_IDX_W'(score_dim_idx[15:3]);
        score_chunk_byte_idx = score_dim_idx[2:0];
        if (score_dim_idx < k_dim_q) begin
          mul_lhs_comb[mul_lane_idx] =
              $signed(score_query_chunk_q[score_chunk_byte_idx * 8 +: 8]);
          mul_rhs_comb[mul_lane_idx] =
              $signed(k_data_q[mul_lane_idx * 8 +: 8]);
        end
      end
    end else if (state_q == ATT_UPDATE_MUL) begin
      mul_lhs_comb[0] = running_sum_q;
      mul_rhs_comb[0] = old_scale_q;
    end else if (state_q == ATT_VALUE_MUL) begin
      mul_lhs_comb[0] = output_accum_q[task_idx_q];
      mul_rhs_comb[0] = old_scale_q;
      mul_lhs_comb[1] = token_exp_q;
      mul_rhs_comb[1] = value_byte_q4;
    end
  end

  // --------------------------------------------------------------------------
  // Score accumulation datapath.
  // --------------------------------------------------------------------------
  always_comb begin : proc_score_datapath
    score_data_sum = 19'sd0;
    score_sum_with_data = score_accum_q;
    score_sat = 8'sd0;
    score_sat_byte = 8'd0;
    product_q8 = 16'sd0;
    rounded_q4 = 16'sd0;

    for (score_lane_idx = 0; score_lane_idx < DATA_BYTES;
         score_lane_idx = score_lane_idx + 1) begin
      product_q8 = mul_product_q[score_lane_idx];
      if (product_q8 >= 16'sd0) begin
        rounded_q4 = (product_q8 + 16'sd8) >>> 4;
      end else begin
        rounded_q4 = -(((-product_q8) + 16'sd8) >>> 4);
      end
      score_data_sum = score_data_sum + rounded_q4;
    end

    score_sum_with_data = score_accum_q + $signed(score_data_sum);
    if (score_sum_with_data > SCORE_SAT_MAX) begin
      score_sat_byte = 8'h7f;
    end else if (score_sum_with_data < SCORE_SAT_MIN) begin
      score_sat_byte = 8'h80;
    end else begin
      score_sat_byte = score_sum_with_data[7:0];
    end
    score_sat = score_sat_byte;
  end

  // --------------------------------------------------------------------------
  // Shared exp lookup for online softmax.
  // --------------------------------------------------------------------------
  always_comb begin : proc_softmax_exp_lookup
    softmax_exp_input = 32'sd0;
    softmax_exp_idx = 8'sd0;
    softmax_exp_result = 8'sd0;
`ifdef NONLINEAR_IMPL_DPI
    softmax_dpi_tmp = 0;
`endif

    if (state_q == ATT_UPDATE_OLD_EXP) begin
      softmax_exp_input = running_max_q - update_new_max;
    end else if (state_q == ATT_UPDATE_TOKEN_EXP) begin
      softmax_exp_input = score_accum_q - update_new_max;
    end
    softmax_exp_idx = softmax_exp_input[7:0];

`ifdef NONLINEAR_IMPL_DPI
    softmax_dpi_tmp = dpi_exp(softmax_exp_input);
    if (softmax_dpi_tmp > 127) begin
      softmax_exp_result = 8'sd127;
    end else if (softmax_dpi_tmp < -128) begin
      softmax_exp_result = -8'sd128;
    end else begin
      softmax_exp_result = 8'(softmax_dpi_tmp);
    end
`else
    softmax_exp_result = softmax_exp_lut_q4;
`endif
  end

  // --------------------------------------------------------------------------
  // Online softmax datapath.
  // --------------------------------------------------------------------------
  always_comb begin : proc_online_softmax
    update_new_max = running_max_q;
    update_running_sum = running_sum_q;
    update_product_q8 = 16'sd0;
    update_rounded_q4 = 16'sd0;
    update_sum_source = 17'sd0;
    update_sat_byte = 8'd0;

    if (score_accum_q > running_max_q) begin
      update_new_max = score_accum_q;
    end

    update_product_q8 = mul_product_q[0];
    if (update_product_q8 >= 16'sd0) begin
      update_rounded_q4 = (update_product_q8 + 16'sd8) >>> 4;
    end else begin
      update_rounded_q4 = -(((-update_product_q8) + 16'sd8) >>> 4);
    end
    update_sum_source =
        $signed(update_rounded_q4) + $signed(token_exp_q);
    if (update_sum_source > SCORE_SAT_MAX) begin
      update_sat_byte = 8'h7f;
    end else if (update_sum_source < SCORE_SAT_MIN) begin
      update_sat_byte = 8'h80;
    end else begin
      update_sat_byte = update_sum_source[7:0];
    end
    update_running_sum = update_sat_byte;
  end

  // --------------------------------------------------------------------------
  // Value accumulation datapath.
  // --------------------------------------------------------------------------
  always_comb begin : proc_value_datapath
    value_target_idx = {1'b0, pkt_meta_q.psum_offset} + {1'b0, task_id_current};
    value_target_lane = value_target_idx[5:0] - flit_start_idx_q[5:0];
    value_task_in_flit =
        ({1'b0, task_idx_q} < task_count_q) &&
        (task_id_current < k_dim_q) &&
        (value_target_idx >= flit_start_idx_q) &&
        (value_target_idx < flit_end_idx_q) &&
        (value_target_lane < 6'd32);
    value_old_scaled = 16'sd0;
    value_new_contrib = 16'sd0;
    value_output_sum = 17'sd0;
    value_product_q8 = 16'sd0;
    value_rounded_q4 = 16'sd0;
    value_byte_q4 = $signed(v_data_q[7:0]);
    value_sat_byte = 8'd0;

    value_product_q8 = mul_product_q[0];
    if (value_product_q8 >= 16'sd0) begin
      value_rounded_q4 = (value_product_q8 + 16'sd8) >>> 4;
    end else begin
      value_rounded_q4 = -(((-value_product_q8) + 16'sd8) >>> 4);
    end
    value_old_scaled = value_rounded_q4;

    value_product_q8 = mul_product_q[1];
    if (value_product_q8 >= 16'sd0) begin
      value_rounded_q4 = (value_product_q8 + 16'sd8) >>> 4;
    end else begin
      value_rounded_q4 = -(((-value_product_q8) + 16'sd8) >>> 4);
    end
    value_new_contrib = value_rounded_q4;

    value_output_sum = value_old_scaled + value_new_contrib;
    if (value_output_sum > SCORE_SAT_MAX) begin
      value_sat_byte = 8'h7f;
    end else if (value_output_sum < SCORE_SAT_MIN) begin
      value_sat_byte = 8'h80;
    end else begin
      value_sat_byte = value_output_sum[7:0];
    end
  end

  // --------------------------------------------------------------------------
  // Query chunk selection for the current score slice.
  // --------------------------------------------------------------------------
  always_comb begin : proc_score_query_chunk
    score_query_chunk_q = '0;
    if (QUERY_CHUNK_NUM == 1) begin
      score_query_chunk_q = query_chunk_q[0];
    end else if (k_base_idx_q[15:3] < QUERY_CHUNK_NUM) begin
      score_query_chunk_q = query_chunk_q[QUERY_CHUNK_IDX_W'(k_base_idx_q[15:3])];
    end
  end

  // --------------------------------------------------------------------------
  // Sequential state update.  All multi-cycle Attention progress stays here;
  // wrapper logic only supplies the shared multiplier products.
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin : proc_attention_registers
    if (reset_i) begin
      state_q <= ATT_IDLE;
      query_ctx_valid_q <= 1'b0;
      query_ctx_stream_port_q <= ROUTER_PORT_INV;
      query_ctx_stream_vc_q <= '0;
      pkt_meta_q <= '0;
      ctx_q <= '0;
      task_count_q <= '0;
      task_ids_flat_q <= '0;
      k_dim_q <= '0;
      effective_payload_len_q <= '0;
      token_stride_q <= '0;
      active_tokens_q <= '0;
      flit_start_idx_q <= '0;
      flit_end_idx_q <= '0;
      token_idx_q <= '0;
      token_base_addr_q <= 32'd0;
      k_base_idx_q <= '0;
      task_idx_q <= '0;
      k_data_q <= '0;
      v_data_q <= '0;
      score_accum_q <= 32'sd0;
      running_max_q <= NEG_INF_Q4;
      running_sum_q <= 8'sd0;
      old_scale_q <= 8'sd0;
      token_exp_q <= 8'sd0;
      fetch_attention_ctx_ok_q <= 1'b0;
      fetch_attention_head_ctx_ok_q <= 1'b0;
      fetch_start_tail_q <= 1'b0;
      fetch_passthrough_q <= 1'b0;
      result_flit_q <= '0;
      result_meta_q <= '0;
      result_valid_q <= 1'b0;
      for (reset_mul_idx = 0; reset_mul_idx < 8; reset_mul_idx = reset_mul_idx + 1) begin
        mul_product_q[reset_mul_idx] <= '0;
      end
      for (reset_task_idx = 0; reset_task_idx < MAX_TASKS;
           reset_task_idx = reset_task_idx + 1) begin
        output_accum_q[reset_task_idx] <= 8'sd0;
      end
      for (reset_chunk_idx = 0; reset_chunk_idx < QUERY_CHUNK_NUM;
           reset_chunk_idx = reset_chunk_idx + 1) begin
        query_chunk_q[reset_chunk_idx] <= '0;
      end
    end else begin
      result_valid_q <= 1'b0;

      if (fetch_en_i && is_attention_op) begin
        pkt_meta_q <= pkt_meta_i;
        ctx_q <= ctx_i;
        task_count_q <= task_count_limited;
        task_ids_flat_q <= ctx_i.task_ids_flat;
        k_dim_q <= k_dim_clamped;
        effective_payload_len_q <= effective_payload_len;
        token_stride_q <= token_stride;
        active_tokens_q <= active_tokens;
        flit_start_idx_q <= {7'd0, pkt_meta_i.data_offset};
        flit_end_idx_q <= {7'd0, pkt_meta_i.data_offset} + {11'd0, effective_payload_len};
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
        token_idx_q <= 16'd0;
        token_base_addr_q <= 32'd0;
        k_base_idx_q <= 16'd0;
        task_idx_q <= 5'd0;
        score_accum_q <= 32'sd0;
        running_max_q <=
            {{24{pkt_meta_i.header_reserved[7]}}, pkt_meta_i.header_reserved[7:0]};
        running_sum_q <= pkt_meta_i.header_reserved[15:8];
        old_scale_q <= 8'sd0;
        token_exp_q <= 8'sd0;
        k_data_q <= data_rsp_i.rdata;
        fetch_attention_ctx_ok_q <= attention_ctx_ok;
        fetch_attention_head_ctx_ok_q <= attention_head_ctx_ok;
        fetch_start_tail_q <= start_attention_tail;
        fetch_passthrough_q <= start_attention_passthrough;
        for (reset_task_idx = 0; reset_task_idx < MAX_TASKS;
             reset_task_idx = reset_task_idx + 1) begin
          output_accum_q[reset_task_idx] <= 8'sd0;
        end

        if (attention_head_ctx_ok) begin
          query_ctx_valid_q <= 1'b1;
          query_ctx_stream_port_q <= ctx_i.stream_port;
          query_ctx_stream_vc_q <= ctx_i.stream_vc;
        end

        state_q <= ATT_QUERY_WRITE;
      end

      if (state_q == ATT_QUERY_WRITE) begin
        if (fetch_attention_head_ctx_ok_q) begin
          for (clear_chunk_idx = 0; clear_chunk_idx < QUERY_CHUNK_NUM;
               clear_chunk_idx = clear_chunk_idx + 1) begin
            query_chunk_q[clear_chunk_idx] <= '0;
          end
        end

        if (fetch_attention_ctx_ok_q) begin
          for (write_byte_idx = 0; write_byte_idx < 32;
               write_byte_idx = write_byte_idx + 1) begin
            if ((write_byte_idx < effective_payload_len_q) &&
                ((pkt_meta_q.data_offset + write_byte_idx) < pkt_meta_q.psum_offset) &&
                ((pkt_meta_q.data_offset + write_byte_idx) < k_dim_q)) begin
              if (QUERY_CHUNK_NUM == 1) begin
                query_chunk_q[0][((pkt_meta_q.data_offset + write_byte_idx) & 10'h7) * 8 +: 8] <=
                    result_flit_q[write_byte_idx * 8 +: 8];
              end else begin
                write_chunk_idx =
                    QUERY_CHUNK_IDX_W'((pkt_meta_q.data_offset + write_byte_idx) >> 3);
                query_chunk_q[write_chunk_idx][((pkt_meta_q.data_offset + write_byte_idx) & 10'h7) * 8 +: 8] <=
                    result_flit_q[write_byte_idx * 8 +: 8];
              end
            end
          end
        end

        if (fetch_start_tail_q) begin
          state_q <= ATT_SCORE_K;
        end else begin
          state_q <= ATT_IDLE;
          if (fetch_passthrough_q) begin
            result_valid_q <= 1'b1;
          end
        end
      end else if (compute_en_i) begin
        unique case (state_q)
          ATT_SCORE_K: begin
            state_q <= ATT_SCORE_MUL;
          end
          ATT_SCORE_MUL: begin
            state_q <= ATT_SCORE_MUL_WAIT;
          end
          ATT_SCORE_MUL_WAIT: begin
            if (mul_rsp_valid_i) begin
              state_q <= ATT_SCORE_COMMIT;
              for (reset_mul_idx = 0; reset_mul_idx < 8; reset_mul_idx = reset_mul_idx + 1) begin
                mul_product_q[reset_mul_idx] <= mul_product_i[reset_mul_idx];
              end
            end
          end
          ATT_SCORE_COMMIT: begin
            if (score_has_next_data) begin
              score_accum_q <= score_sum_with_data;
              k_base_idx_q <= score_next_base_idx;
              k_data_q <= data_rsp_i.rdata;
              state_q <= ATT_SCORE_MUL;
            end else begin
              score_accum_q <= score_sat;
              state_q <= ATT_UPDATE_OLD_EXP;
            end
          end
          ATT_UPDATE_OLD_EXP: begin
            old_scale_q <= softmax_exp_result;
            state_q <= ATT_UPDATE_TOKEN_EXP;
          end
          ATT_UPDATE_TOKEN_EXP: begin
            token_exp_q <= softmax_exp_result;
            state_q <= ATT_UPDATE_MUL;
          end
          ATT_UPDATE_MUL: begin
            state_q <= ATT_UPDATE_MUL_WAIT;
          end
          ATT_UPDATE_MUL_WAIT: begin
            if (mul_rsp_valid_i) begin
              state_q <= ATT_UPDATE_COMMIT;
              for (reset_mul_idx = 0; reset_mul_idx < 8; reset_mul_idx = reset_mul_idx + 1) begin
                mul_product_q[reset_mul_idx] <= mul_product_i[reset_mul_idx];
              end
            end
          end
          ATT_UPDATE_COMMIT: begin
            running_max_q <= update_new_max;
            running_sum_q <= update_running_sum;
            result_meta_q.header_reserved[7:0] <= update_new_max[7:0];
            result_meta_q.header_reserved[15:8] <= update_running_sum;
            if (task_count_q != 6'd0) begin
              task_idx_q <= 5'd0;
              v_data_q <= data_rsp_i.rdata;
              state_q <= ATT_VALUE_V;
            end else if (token_has_next) begin
              token_idx_q <= token_idx_q + 16'd1;
              token_base_addr_q <= token_base_addr_next;
              k_base_idx_q <= 16'd0;
              score_accum_q <= 32'sd0;
              k_data_q <= data_rsp_i.rdata;
              state_q <= ATT_SCORE_K;
            end else begin
              state_q <= ATT_RESULT;
            end
          end
          ATT_VALUE_V: begin
            state_q <= ATT_VALUE_MUL;
          end
          ATT_VALUE_MUL: begin
            state_q <= ATT_VALUE_MUL_WAIT;
          end
          ATT_VALUE_MUL_WAIT: begin
            if (mul_rsp_valid_i) begin
              state_q <= ATT_VALUE_COMMIT;
              for (reset_mul_idx = 0; reset_mul_idx < 8; reset_mul_idx = reset_mul_idx + 1) begin
                mul_product_q[reset_mul_idx] <= mul_product_i[reset_mul_idx];
              end
            end
          end
          ATT_VALUE_COMMIT: begin
            output_accum_q[task_idx_q] <= value_sat_byte;
            if (value_task_in_flit) begin
              result_flit_q[value_target_lane * 8 +: 8] <= value_sat_byte;
            end
            if (value_has_next_task) begin
              task_idx_q <= task_idx_next[4:0];
              v_data_q <= data_rsp_i.rdata;
              state_q <= ATT_VALUE_V;
            end else if (token_has_next) begin
              token_idx_q <= token_idx_q + 16'd1;
              token_base_addr_q <= token_base_addr_next;
              k_base_idx_q <= 16'd0;
              task_idx_q <= 5'd0;
              score_accum_q <= 32'sd0;
              k_data_q <= data_rsp_i.rdata;
              state_q <= ATT_SCORE_K;
            end else begin
              state_q <= ATT_RESULT;
            end
          end
          ATT_RESULT: begin
            result_valid_q <= 1'b1;
            state_q <= ATT_IDLE;
          end
          default: begin
            state_q <= ATT_IDLE;
          end
        endcase
      end

      if (release_state_i && is_attention_op && tail_like) begin
        if (attention_owner_match) begin
          query_ctx_valid_q <= 1'b0;
          query_ctx_stream_port_q <= ROUTER_PORT_INV;
          query_ctx_stream_vc_q <= '0;
        end
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_attention_assertions
    if (!reset_i) begin
      if (fetch_en_i && is_attention_op && !head_like &&
          !attention_owner_match) begin
        $error("mfu_alu_attention: body/tail Attention flit arrived without head state");
      end

      if (release_state_i && is_attention_op && tail_like &&
          !attention_owner_match) begin
        $error("mfu_alu_attention: release observed without a valid query stream");
      end

      if (fetch_en_i && is_attention_op && !token_stride_supported) begin
        $error("mfu_alu_attention: unsupported Attention token stride");
      end
    end
  end
`endif

  assign busy_o = (state_q != ATT_IDLE);
  assign result_valid_o = result_valid_q;
  assign result_flit_o = result_flit_q;
  assign result_meta_o = result_meta_q;

endmodule
