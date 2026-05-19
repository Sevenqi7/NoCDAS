// Copyright (c) 2026
//
// Description: Stateful streaming Attention ALU leaf.
//              The leaf owns Attention query/running state and streams K/V data
//              from the shared MFU SRAM through the generic ALU data request
//              interface.  It deliberately does not mirror type4 KV writes into
//              local storage; mfu_sram is the only persistent KV source.

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
    output router_ports_pkg::flit_meta_t result_meta_o
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

  typedef enum logic [2:0] {
    ATT_IDLE,
    ATT_SCORE_K,
    ATT_UPDATE_SOFTMAX,
    ATT_VALUE_V,
    ATT_RESULT
  } attention_state_e;

  attention_state_e state_q;

  logic [7:0] query_q [NUM_PORTS][VC_NUM][MAX_K_DIM];
  logic query_valid_q [NUM_PORTS][VC_NUM];

  flit_meta_t pkt_meta_q;
  mfu_alu_ctx_t ctx_q;
  logic [5:0] task_count_q;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] task_ids_flat_q;
  logic [15:0] k_dim_q;
  logic [15:0] token_stride_q;
  logic [15:0] active_tokens_q;
  logic [16:0] flit_start_idx_q;
  logic [16:0] flit_end_idx_q;
  logic [15:0] token_idx_q;
  logic [15:0] k_base_idx_q;
  logic [4:0] task_idx_q;
  logic [DATA_W-1:0] k_data_q;
  logic [DATA_W-1:0] v_data_q;
  logic signed [31:0] score_accum_q;
  logic signed [31:0] running_max_q;
  logic signed [31:0] running_sum_q;
  logic signed [31:0] old_scale_q;
  logic signed [31:0] token_exp_q;
  logic signed [31:0] output_accum_q [MAX_TASKS];
  logic [FLIT_W-1:0] result_flit_q;
  flit_meta_t result_meta_q;
  logic result_valid_q;

  logic is_attention_op;
  logic head_like;
  logic tail_like;
  logic context_ready;
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

  logic score_has_next_data;
  logic [15:0] score_next_base_idx;
  logic [31:0] token_base_addr_full;
  logic [31:0] score_read_addr_full;
  logic [31:0] value_read_addr_full;
  logic [31:0] value_next_read_addr_full;
  logic [31:0] next_token_read_addr_full;
  logic [31:0] data_req_full_addr;
  logic [SRAM_ADDR_W-1:0] data_req_addr;
  logic data_req_valid;

  logic signed [31:0] score_data_sum;
  logic signed [31:0] score_sum_with_data;
  logic signed [31:0] score_sat;
  logic [7:0] score_sat_byte;
  logic signed [31:0] query_q4;
  logic signed [31:0] key_q4;
  logic signed [31:0] product_q8;
  logic signed [31:0] rounded_q4;
  logic [15:0] score_dim_idx;
  logic [K_DIM_IDX_W-1:0] score_query_idx;

  logic signed [31:0] update_new_max;
  logic signed [31:0] update_old_scale;
  logic signed [31:0] update_token_exp;
  logic signed [31:0] update_running_sum;
  logic signed [31:0] update_product_q8;
  logic signed [31:0] update_rounded_q4;
  logic signed [31:0] update_sum_source;
  logic [7:0] update_sat_byte;
  logic signed [31:0] update_exp_input;
  logic [7:0] update_exp_idx;
  int update_dpi_tmp;

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
  logic signed [31:0] value_old_scaled;
  logic signed [31:0] value_new_contrib;
  logic signed [31:0] value_output_sum;
  logic signed [31:0] value_product_q8;
  logic signed [31:0] value_rounded_q4;
  logic signed [31:0] value_byte_q4;
  logic [7:0] value_sat_byte;
  logic signed [31:0] value_accum_next;

  integer reset_port_idx;
  integer reset_vc_idx;
  integer reset_dim_idx;
  integer reset_task_idx;
  integer query_lane_idx;
  integer score_lane_idx;

  assign is_attention_op = op_i.is_attention;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign tail_like = flit_is_tail_like(pkt_meta_i.flit_kind);
  assign context_ready = query_valid_q[ctx_i.stream_port][ctx_i.stream_vc] || head_like;
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
      fetch_en_i && is_attention_op && tail_like && context_ready && (active_tokens != 16'd0);
  assign start_attention_passthrough =
      fetch_en_i && is_attention_op && (!tail_like || !context_ready || (active_tokens == 16'd0));

  assign score_next_base_idx = k_base_idx_q + {10'd0, DATA_BYTES_Q};
  assign score_has_next_data =
      (state_q == ATT_SCORE_K) && compute_en_i && (score_next_base_idx < k_dim_q);
  assign token_base_addr_full = {16'd0, token_idx_q} * {16'd0, token_stride_q};
  assign score_read_addr_full = token_base_addr_full + {16'd0, score_next_base_idx};
  assign task_idx_next = {1'b0, task_idx_q} + 6'd1;
  assign has_next_task = task_idx_next < task_count_q;
  assign value_has_next_task =
      (state_q == ATT_VALUE_V) && compute_en_i && has_next_task;
  assign value_last_task =
      (state_q == ATT_VALUE_V) && compute_en_i && !value_has_next_task;
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
  assign next_token_read_addr_full =
      ({16'd0, token_idx_q} + 32'd1) * {16'd0, token_stride_q};

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
    end else if ((state_q == ATT_UPDATE_SOFTMAX) && compute_en_i &&
                 (task_count_q != 6'd0)) begin
      data_req_valid = 1'b1;
      data_req_full_addr = value_read_addr_full;
    end else if (value_has_next_task) begin
      data_req_valid = 1'b1;
      data_req_full_addr = value_next_read_addr_full;
    end else if (value_last_task && token_has_next) begin
      data_req_valid = 1'b1;
      data_req_full_addr = next_token_read_addr_full;
    end else if ((state_q == ATT_UPDATE_SOFTMAX) && compute_en_i &&
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

  always_comb begin : proc_score_datapath
    score_data_sum = 32'sd0;
    score_sum_with_data = score_accum_q;
    score_sat = score_accum_q;
    score_sat_byte = 8'd0;
    query_q4 = 32'sd0;
    key_q4 = 32'sd0;
    product_q8 = 32'sd0;
    rounded_q4 = 32'sd0;
    score_dim_idx = 16'd0;
    score_query_idx = '0;

    for (score_lane_idx = 0; score_lane_idx < DATA_BYTES;
         score_lane_idx = score_lane_idx + 1) begin
      score_dim_idx = k_base_idx_q + 16'(score_lane_idx);
      score_query_idx = score_dim_idx[K_DIM_IDX_W-1:0];
      if (score_dim_idx < k_dim_q) begin
        query_q4 =
            {{24{query_q[ctx_q.stream_port][ctx_q.stream_vc][score_query_idx][7]}},
             query_q[ctx_q.stream_port][ctx_q.stream_vc][score_query_idx]};
        key_q4 =
            {{24{k_data_q[score_lane_idx * 8 + 7]}},
             k_data_q[score_lane_idx * 8 +: 8]};
        product_q8 = query_q4 * key_q4;
        if (product_q8 >= 32'sd0) begin
          rounded_q4 = (product_q8 + 32'sd8) >>> 4;
        end else begin
          rounded_q4 = -(((-product_q8) + 32'sd8) >>> 4);
        end
        score_data_sum = score_data_sum + rounded_q4;
      end
    end

    score_sum_with_data = score_accum_q + score_data_sum;
    if (score_sum_with_data > SCORE_SAT_MAX) begin
      score_sat_byte = 8'h7f;
    end else if (score_sum_with_data < SCORE_SAT_MIN) begin
      score_sat_byte = 8'h80;
    end else begin
      score_sat_byte = score_sum_with_data[7:0];
    end
    score_sat = {{24{score_sat_byte[7]}}, score_sat_byte};
  end

  always_comb begin : proc_online_softmax
    update_new_max = running_max_q;
    update_old_scale = 32'sd0;
    update_token_exp = 32'sd0;
    update_running_sum = running_sum_q;
    update_product_q8 = 32'sd0;
    update_rounded_q4 = 32'sd0;
    update_sum_source = 32'sd0;
    update_sat_byte = 8'd0;
    update_exp_input = 32'sd0;
    update_exp_idx = 8'd0;
    update_dpi_tmp = 0;

    if (score_accum_q > running_max_q) begin
      update_new_max = score_accum_q;
    end

    update_exp_input = running_max_q - update_new_max;
    update_exp_idx = update_exp_input[7:0];
`ifdef NONLINEAR_IMPL_DPI
    update_dpi_tmp = dpi_exp(update_exp_input);
`else
    case (update_exp_idx)
      8'h00: update_dpi_tmp = 32'sd16;
      8'h01: update_dpi_tmp = 32'sd17;
      8'h02: update_dpi_tmp = 32'sd18;
      8'h03: update_dpi_tmp = 32'sd19;
      8'h04: update_dpi_tmp = 32'sd21;
      8'h05: update_dpi_tmp = 32'sd22;
      8'h06: update_dpi_tmp = 32'sd23;
      8'h07: update_dpi_tmp = 32'sd25;
      8'h08: update_dpi_tmp = 32'sd26;
      8'h09: update_dpi_tmp = 32'sd28;
      8'h0a: update_dpi_tmp = 32'sd30;
      8'h0b: update_dpi_tmp = 32'sd32;
      8'h0c: update_dpi_tmp = 32'sd34;
      8'h0d: update_dpi_tmp = 32'sd36;
      8'h0e: update_dpi_tmp = 32'sd38;
      8'h0f: update_dpi_tmp = 32'sd41;
      8'h10: update_dpi_tmp = 32'sd43;
      8'h11: update_dpi_tmp = 32'sd46;
      8'h12: update_dpi_tmp = 32'sd49;
      8'h13: update_dpi_tmp = 32'sd52;
      8'h14: update_dpi_tmp = 32'sd56;
      8'h15: update_dpi_tmp = 32'sd59;
      8'h16: update_dpi_tmp = 32'sd63;
      8'h17: update_dpi_tmp = 32'sd67;
      8'h18: update_dpi_tmp = 32'sd72;
      8'h19: update_dpi_tmp = 32'sd76;
      8'h1a: update_dpi_tmp = 32'sd81;
      8'h1b: update_dpi_tmp = 32'sd86;
      8'h1c: update_dpi_tmp = 32'sd92;
      8'h1d: update_dpi_tmp = 32'sd98;
      8'h1e: update_dpi_tmp = 32'sd104;
      8'h1f: update_dpi_tmp = 32'sd111;
      8'h20: update_dpi_tmp = 32'sd118;
      8'h21: update_dpi_tmp = 32'sd126;
      8'h22: update_dpi_tmp = 32'sd127;
      8'h23: update_dpi_tmp = 32'sd127;
      8'h24: update_dpi_tmp = 32'sd127;
      8'h25: update_dpi_tmp = 32'sd127;
      8'h26: update_dpi_tmp = 32'sd127;
      8'h27: update_dpi_tmp = 32'sd127;
      8'h28: update_dpi_tmp = 32'sd127;
      8'h29: update_dpi_tmp = 32'sd127;
      8'h2a: update_dpi_tmp = 32'sd127;
      8'h2b: update_dpi_tmp = 32'sd127;
      8'h2c: update_dpi_tmp = 32'sd127;
      8'h2d: update_dpi_tmp = 32'sd127;
      8'h2e: update_dpi_tmp = 32'sd127;
      8'h2f: update_dpi_tmp = 32'sd127;
      8'h30: update_dpi_tmp = 32'sd127;
      8'h31: update_dpi_tmp = 32'sd127;
      8'h32: update_dpi_tmp = 32'sd127;
      8'h33: update_dpi_tmp = 32'sd127;
      8'h34: update_dpi_tmp = 32'sd127;
      8'h35: update_dpi_tmp = 32'sd127;
      8'h36: update_dpi_tmp = 32'sd127;
      8'h37: update_dpi_tmp = 32'sd127;
      8'h38: update_dpi_tmp = 32'sd127;
      8'h39: update_dpi_tmp = 32'sd127;
      8'h3a: update_dpi_tmp = 32'sd127;
      8'h3b: update_dpi_tmp = 32'sd127;
      8'h3c: update_dpi_tmp = 32'sd127;
      8'h3d: update_dpi_tmp = 32'sd127;
      8'h3e: update_dpi_tmp = 32'sd127;
      8'h3f: update_dpi_tmp = 32'sd127;
      8'h40: update_dpi_tmp = 32'sd127;
      8'h41: update_dpi_tmp = 32'sd127;
      8'h42: update_dpi_tmp = 32'sd127;
      8'h43: update_dpi_tmp = 32'sd127;
      8'h44: update_dpi_tmp = 32'sd127;
      8'h45: update_dpi_tmp = 32'sd127;
      8'h46: update_dpi_tmp = 32'sd127;
      8'h47: update_dpi_tmp = 32'sd127;
      8'h48: update_dpi_tmp = 32'sd127;
      8'h49: update_dpi_tmp = 32'sd127;
      8'h4a: update_dpi_tmp = 32'sd127;
      8'h4b: update_dpi_tmp = 32'sd127;
      8'h4c: update_dpi_tmp = 32'sd127;
      8'h4d: update_dpi_tmp = 32'sd127;
      8'h4e: update_dpi_tmp = 32'sd127;
      8'h4f: update_dpi_tmp = 32'sd127;
      8'h50: update_dpi_tmp = 32'sd127;
      8'h51: update_dpi_tmp = 32'sd127;
      8'h52: update_dpi_tmp = 32'sd127;
      8'h53: update_dpi_tmp = 32'sd127;
      8'h54: update_dpi_tmp = 32'sd127;
      8'h55: update_dpi_tmp = 32'sd127;
      8'h56: update_dpi_tmp = 32'sd127;
      8'h57: update_dpi_tmp = 32'sd127;
      8'h58: update_dpi_tmp = 32'sd127;
      8'h59: update_dpi_tmp = 32'sd127;
      8'h5a: update_dpi_tmp = 32'sd127;
      8'h5b: update_dpi_tmp = 32'sd127;
      8'h5c: update_dpi_tmp = 32'sd127;
      8'h5d: update_dpi_tmp = 32'sd127;
      8'h5e: update_dpi_tmp = 32'sd127;
      8'h5f: update_dpi_tmp = 32'sd127;
      8'h60: update_dpi_tmp = 32'sd127;
      8'h61: update_dpi_tmp = 32'sd127;
      8'h62: update_dpi_tmp = 32'sd127;
      8'h63: update_dpi_tmp = 32'sd127;
      8'h64: update_dpi_tmp = 32'sd127;
      8'h65: update_dpi_tmp = 32'sd127;
      8'h66: update_dpi_tmp = 32'sd127;
      8'h67: update_dpi_tmp = 32'sd127;
      8'h68: update_dpi_tmp = 32'sd127;
      8'h69: update_dpi_tmp = 32'sd127;
      8'h6a: update_dpi_tmp = 32'sd127;
      8'h6b: update_dpi_tmp = 32'sd127;
      8'h6c: update_dpi_tmp = 32'sd127;
      8'h6d: update_dpi_tmp = 32'sd127;
      8'h6e: update_dpi_tmp = 32'sd127;
      8'h6f: update_dpi_tmp = 32'sd127;
      8'h70: update_dpi_tmp = 32'sd127;
      8'h71: update_dpi_tmp = 32'sd127;
      8'h72: update_dpi_tmp = 32'sd127;
      8'h73: update_dpi_tmp = 32'sd127;
      8'h74: update_dpi_tmp = 32'sd127;
      8'h75: update_dpi_tmp = 32'sd127;
      8'h76: update_dpi_tmp = 32'sd127;
      8'h77: update_dpi_tmp = 32'sd127;
      8'h78: update_dpi_tmp = 32'sd127;
      8'h79: update_dpi_tmp = 32'sd127;
      8'h7a: update_dpi_tmp = 32'sd127;
      8'h7b: update_dpi_tmp = 32'sd127;
      8'h7c: update_dpi_tmp = 32'sd127;
      8'h7d: update_dpi_tmp = 32'sd127;
      8'h7e: update_dpi_tmp = 32'sd127;
      8'h7f: update_dpi_tmp = 32'sd127;
      8'h80: update_dpi_tmp = 32'sd0;
      8'h81: update_dpi_tmp = 32'sd0;
      8'h82: update_dpi_tmp = 32'sd0;
      8'h83: update_dpi_tmp = 32'sd0;
      8'h84: update_dpi_tmp = 32'sd0;
      8'h85: update_dpi_tmp = 32'sd0;
      8'h86: update_dpi_tmp = 32'sd0;
      8'h87: update_dpi_tmp = 32'sd0;
      8'h88: update_dpi_tmp = 32'sd0;
      8'h89: update_dpi_tmp = 32'sd0;
      8'h8a: update_dpi_tmp = 32'sd0;
      8'h8b: update_dpi_tmp = 32'sd0;
      8'h8c: update_dpi_tmp = 32'sd0;
      8'h8d: update_dpi_tmp = 32'sd0;
      8'h8e: update_dpi_tmp = 32'sd0;
      8'h8f: update_dpi_tmp = 32'sd0;
      8'h90: update_dpi_tmp = 32'sd0;
      8'h91: update_dpi_tmp = 32'sd0;
      8'h92: update_dpi_tmp = 32'sd0;
      8'h93: update_dpi_tmp = 32'sd0;
      8'h94: update_dpi_tmp = 32'sd0;
      8'h95: update_dpi_tmp = 32'sd0;
      8'h96: update_dpi_tmp = 32'sd0;
      8'h97: update_dpi_tmp = 32'sd0;
      8'h98: update_dpi_tmp = 32'sd0;
      8'h99: update_dpi_tmp = 32'sd0;
      8'h9a: update_dpi_tmp = 32'sd0;
      8'h9b: update_dpi_tmp = 32'sd0;
      8'h9c: update_dpi_tmp = 32'sd0;
      8'h9d: update_dpi_tmp = 32'sd0;
      8'h9e: update_dpi_tmp = 32'sd0;
      8'h9f: update_dpi_tmp = 32'sd0;
      8'ha0: update_dpi_tmp = 32'sd0;
      8'ha1: update_dpi_tmp = 32'sd0;
      8'ha2: update_dpi_tmp = 32'sd0;
      8'ha3: update_dpi_tmp = 32'sd0;
      8'ha4: update_dpi_tmp = 32'sd0;
      8'ha5: update_dpi_tmp = 32'sd0;
      8'ha6: update_dpi_tmp = 32'sd0;
      8'ha7: update_dpi_tmp = 32'sd0;
      8'ha8: update_dpi_tmp = 32'sd0;
      8'ha9: update_dpi_tmp = 32'sd0;
      8'haa: update_dpi_tmp = 32'sd0;
      8'hab: update_dpi_tmp = 32'sd0;
      8'hac: update_dpi_tmp = 32'sd0;
      8'had: update_dpi_tmp = 32'sd0;
      8'hae: update_dpi_tmp = 32'sd0;
      8'haf: update_dpi_tmp = 32'sd0;
      8'hb0: update_dpi_tmp = 32'sd0;
      8'hb1: update_dpi_tmp = 32'sd0;
      8'hb2: update_dpi_tmp = 32'sd0;
      8'hb3: update_dpi_tmp = 32'sd0;
      8'hb4: update_dpi_tmp = 32'sd0;
      8'hb5: update_dpi_tmp = 32'sd0;
      8'hb6: update_dpi_tmp = 32'sd0;
      8'hb7: update_dpi_tmp = 32'sd0;
      8'hb8: update_dpi_tmp = 32'sd0;
      8'hb9: update_dpi_tmp = 32'sd0;
      8'hba: update_dpi_tmp = 32'sd0;
      8'hbb: update_dpi_tmp = 32'sd0;
      8'hbc: update_dpi_tmp = 32'sd0;
      8'hbd: update_dpi_tmp = 32'sd0;
      8'hbe: update_dpi_tmp = 32'sd0;
      8'hbf: update_dpi_tmp = 32'sd0;
      8'hc0: update_dpi_tmp = 32'sd0;
      8'hc1: update_dpi_tmp = 32'sd0;
      8'hc2: update_dpi_tmp = 32'sd0;
      8'hc3: update_dpi_tmp = 32'sd0;
      8'hc4: update_dpi_tmp = 32'sd0;
      8'hc5: update_dpi_tmp = 32'sd0;
      8'hc6: update_dpi_tmp = 32'sd0;
      8'hc7: update_dpi_tmp = 32'sd0;
      8'hc8: update_dpi_tmp = 32'sd0;
      8'hc9: update_dpi_tmp = 32'sd1;
      8'hca: update_dpi_tmp = 32'sd1;
      8'hcb: update_dpi_tmp = 32'sd1;
      8'hcc: update_dpi_tmp = 32'sd1;
      8'hcd: update_dpi_tmp = 32'sd1;
      8'hce: update_dpi_tmp = 32'sd1;
      8'hcf: update_dpi_tmp = 32'sd1;
      8'hd0: update_dpi_tmp = 32'sd1;
      8'hd1: update_dpi_tmp = 32'sd1;
      8'hd2: update_dpi_tmp = 32'sd1;
      8'hd3: update_dpi_tmp = 32'sd1;
      8'hd4: update_dpi_tmp = 32'sd1;
      8'hd5: update_dpi_tmp = 32'sd1;
      8'hd6: update_dpi_tmp = 32'sd1;
      8'hd7: update_dpi_tmp = 32'sd1;
      8'hd8: update_dpi_tmp = 32'sd1;
      8'hd9: update_dpi_tmp = 32'sd1;
      8'hda: update_dpi_tmp = 32'sd1;
      8'hdb: update_dpi_tmp = 32'sd2;
      8'hdc: update_dpi_tmp = 32'sd2;
      8'hdd: update_dpi_tmp = 32'sd2;
      8'hde: update_dpi_tmp = 32'sd2;
      8'hdf: update_dpi_tmp = 32'sd2;
      8'he0: update_dpi_tmp = 32'sd2;
      8'he1: update_dpi_tmp = 32'sd2;
      8'he2: update_dpi_tmp = 32'sd2;
      8'he3: update_dpi_tmp = 32'sd3;
      8'he4: update_dpi_tmp = 32'sd3;
      8'he5: update_dpi_tmp = 32'sd3;
      8'he6: update_dpi_tmp = 32'sd3;
      8'he7: update_dpi_tmp = 32'sd3;
      8'he8: update_dpi_tmp = 32'sd4;
      8'he9: update_dpi_tmp = 32'sd4;
      8'hea: update_dpi_tmp = 32'sd4;
      8'heb: update_dpi_tmp = 32'sd4;
      8'hec: update_dpi_tmp = 32'sd5;
      8'hed: update_dpi_tmp = 32'sd5;
      8'hee: update_dpi_tmp = 32'sd5;
      8'hef: update_dpi_tmp = 32'sd6;
      8'hf0: update_dpi_tmp = 32'sd6;
      8'hf1: update_dpi_tmp = 32'sd6;
      8'hf2: update_dpi_tmp = 32'sd7;
      8'hf3: update_dpi_tmp = 32'sd7;
      8'hf4: update_dpi_tmp = 32'sd8;
      8'hf5: update_dpi_tmp = 32'sd8;
      8'hf6: update_dpi_tmp = 32'sd9;
      8'hf7: update_dpi_tmp = 32'sd9;
      8'hf8: update_dpi_tmp = 32'sd10;
      8'hf9: update_dpi_tmp = 32'sd10;
      8'hfa: update_dpi_tmp = 32'sd11;
      8'hfb: update_dpi_tmp = 32'sd12;
      8'hfc: update_dpi_tmp = 32'sd12;
      8'hfd: update_dpi_tmp = 32'sd13;
      8'hfe: update_dpi_tmp = 32'sd14;
      8'hff: update_dpi_tmp = 32'sd15;
      default: update_dpi_tmp = 32'sd0;
    endcase
`endif
    if (update_dpi_tmp > 127) begin
      update_old_scale = 32'sd127;
    end else if (update_dpi_tmp < -128) begin
      update_old_scale = -32'sd128;
    end else begin
      update_old_scale = update_dpi_tmp;
    end

    update_exp_input = score_accum_q - update_new_max;
    update_exp_idx = update_exp_input[7:0];
`ifdef NONLINEAR_IMPL_DPI
    update_dpi_tmp = dpi_exp(update_exp_input);
`else
    case (update_exp_idx)
      8'h00: update_dpi_tmp = 32'sd16;
      8'h01: update_dpi_tmp = 32'sd17;
      8'h02: update_dpi_tmp = 32'sd18;
      8'h03: update_dpi_tmp = 32'sd19;
      8'h04: update_dpi_tmp = 32'sd21;
      8'h05: update_dpi_tmp = 32'sd22;
      8'h06: update_dpi_tmp = 32'sd23;
      8'h07: update_dpi_tmp = 32'sd25;
      8'h08: update_dpi_tmp = 32'sd26;
      8'h09: update_dpi_tmp = 32'sd28;
      8'h0a: update_dpi_tmp = 32'sd30;
      8'h0b: update_dpi_tmp = 32'sd32;
      8'h0c: update_dpi_tmp = 32'sd34;
      8'h0d: update_dpi_tmp = 32'sd36;
      8'h0e: update_dpi_tmp = 32'sd38;
      8'h0f: update_dpi_tmp = 32'sd41;
      8'h10: update_dpi_tmp = 32'sd43;
      8'h11: update_dpi_tmp = 32'sd46;
      8'h12: update_dpi_tmp = 32'sd49;
      8'h13: update_dpi_tmp = 32'sd52;
      8'h14: update_dpi_tmp = 32'sd56;
      8'h15: update_dpi_tmp = 32'sd59;
      8'h16: update_dpi_tmp = 32'sd63;
      8'h17: update_dpi_tmp = 32'sd67;
      8'h18: update_dpi_tmp = 32'sd72;
      8'h19: update_dpi_tmp = 32'sd76;
      8'h1a: update_dpi_tmp = 32'sd81;
      8'h1b: update_dpi_tmp = 32'sd86;
      8'h1c: update_dpi_tmp = 32'sd92;
      8'h1d: update_dpi_tmp = 32'sd98;
      8'h1e: update_dpi_tmp = 32'sd104;
      8'h1f: update_dpi_tmp = 32'sd111;
      8'h20: update_dpi_tmp = 32'sd118;
      8'h21: update_dpi_tmp = 32'sd126;
      8'h22: update_dpi_tmp = 32'sd127;
      8'h23: update_dpi_tmp = 32'sd127;
      8'h24: update_dpi_tmp = 32'sd127;
      8'h25: update_dpi_tmp = 32'sd127;
      8'h26: update_dpi_tmp = 32'sd127;
      8'h27: update_dpi_tmp = 32'sd127;
      8'h28: update_dpi_tmp = 32'sd127;
      8'h29: update_dpi_tmp = 32'sd127;
      8'h2a: update_dpi_tmp = 32'sd127;
      8'h2b: update_dpi_tmp = 32'sd127;
      8'h2c: update_dpi_tmp = 32'sd127;
      8'h2d: update_dpi_tmp = 32'sd127;
      8'h2e: update_dpi_tmp = 32'sd127;
      8'h2f: update_dpi_tmp = 32'sd127;
      8'h30: update_dpi_tmp = 32'sd127;
      8'h31: update_dpi_tmp = 32'sd127;
      8'h32: update_dpi_tmp = 32'sd127;
      8'h33: update_dpi_tmp = 32'sd127;
      8'h34: update_dpi_tmp = 32'sd127;
      8'h35: update_dpi_tmp = 32'sd127;
      8'h36: update_dpi_tmp = 32'sd127;
      8'h37: update_dpi_tmp = 32'sd127;
      8'h38: update_dpi_tmp = 32'sd127;
      8'h39: update_dpi_tmp = 32'sd127;
      8'h3a: update_dpi_tmp = 32'sd127;
      8'h3b: update_dpi_tmp = 32'sd127;
      8'h3c: update_dpi_tmp = 32'sd127;
      8'h3d: update_dpi_tmp = 32'sd127;
      8'h3e: update_dpi_tmp = 32'sd127;
      8'h3f: update_dpi_tmp = 32'sd127;
      8'h40: update_dpi_tmp = 32'sd127;
      8'h41: update_dpi_tmp = 32'sd127;
      8'h42: update_dpi_tmp = 32'sd127;
      8'h43: update_dpi_tmp = 32'sd127;
      8'h44: update_dpi_tmp = 32'sd127;
      8'h45: update_dpi_tmp = 32'sd127;
      8'h46: update_dpi_tmp = 32'sd127;
      8'h47: update_dpi_tmp = 32'sd127;
      8'h48: update_dpi_tmp = 32'sd127;
      8'h49: update_dpi_tmp = 32'sd127;
      8'h4a: update_dpi_tmp = 32'sd127;
      8'h4b: update_dpi_tmp = 32'sd127;
      8'h4c: update_dpi_tmp = 32'sd127;
      8'h4d: update_dpi_tmp = 32'sd127;
      8'h4e: update_dpi_tmp = 32'sd127;
      8'h4f: update_dpi_tmp = 32'sd127;
      8'h50: update_dpi_tmp = 32'sd127;
      8'h51: update_dpi_tmp = 32'sd127;
      8'h52: update_dpi_tmp = 32'sd127;
      8'h53: update_dpi_tmp = 32'sd127;
      8'h54: update_dpi_tmp = 32'sd127;
      8'h55: update_dpi_tmp = 32'sd127;
      8'h56: update_dpi_tmp = 32'sd127;
      8'h57: update_dpi_tmp = 32'sd127;
      8'h58: update_dpi_tmp = 32'sd127;
      8'h59: update_dpi_tmp = 32'sd127;
      8'h5a: update_dpi_tmp = 32'sd127;
      8'h5b: update_dpi_tmp = 32'sd127;
      8'h5c: update_dpi_tmp = 32'sd127;
      8'h5d: update_dpi_tmp = 32'sd127;
      8'h5e: update_dpi_tmp = 32'sd127;
      8'h5f: update_dpi_tmp = 32'sd127;
      8'h60: update_dpi_tmp = 32'sd127;
      8'h61: update_dpi_tmp = 32'sd127;
      8'h62: update_dpi_tmp = 32'sd127;
      8'h63: update_dpi_tmp = 32'sd127;
      8'h64: update_dpi_tmp = 32'sd127;
      8'h65: update_dpi_tmp = 32'sd127;
      8'h66: update_dpi_tmp = 32'sd127;
      8'h67: update_dpi_tmp = 32'sd127;
      8'h68: update_dpi_tmp = 32'sd127;
      8'h69: update_dpi_tmp = 32'sd127;
      8'h6a: update_dpi_tmp = 32'sd127;
      8'h6b: update_dpi_tmp = 32'sd127;
      8'h6c: update_dpi_tmp = 32'sd127;
      8'h6d: update_dpi_tmp = 32'sd127;
      8'h6e: update_dpi_tmp = 32'sd127;
      8'h6f: update_dpi_tmp = 32'sd127;
      8'h70: update_dpi_tmp = 32'sd127;
      8'h71: update_dpi_tmp = 32'sd127;
      8'h72: update_dpi_tmp = 32'sd127;
      8'h73: update_dpi_tmp = 32'sd127;
      8'h74: update_dpi_tmp = 32'sd127;
      8'h75: update_dpi_tmp = 32'sd127;
      8'h76: update_dpi_tmp = 32'sd127;
      8'h77: update_dpi_tmp = 32'sd127;
      8'h78: update_dpi_tmp = 32'sd127;
      8'h79: update_dpi_tmp = 32'sd127;
      8'h7a: update_dpi_tmp = 32'sd127;
      8'h7b: update_dpi_tmp = 32'sd127;
      8'h7c: update_dpi_tmp = 32'sd127;
      8'h7d: update_dpi_tmp = 32'sd127;
      8'h7e: update_dpi_tmp = 32'sd127;
      8'h7f: update_dpi_tmp = 32'sd127;
      8'h80: update_dpi_tmp = 32'sd0;
      8'h81: update_dpi_tmp = 32'sd0;
      8'h82: update_dpi_tmp = 32'sd0;
      8'h83: update_dpi_tmp = 32'sd0;
      8'h84: update_dpi_tmp = 32'sd0;
      8'h85: update_dpi_tmp = 32'sd0;
      8'h86: update_dpi_tmp = 32'sd0;
      8'h87: update_dpi_tmp = 32'sd0;
      8'h88: update_dpi_tmp = 32'sd0;
      8'h89: update_dpi_tmp = 32'sd0;
      8'h8a: update_dpi_tmp = 32'sd0;
      8'h8b: update_dpi_tmp = 32'sd0;
      8'h8c: update_dpi_tmp = 32'sd0;
      8'h8d: update_dpi_tmp = 32'sd0;
      8'h8e: update_dpi_tmp = 32'sd0;
      8'h8f: update_dpi_tmp = 32'sd0;
      8'h90: update_dpi_tmp = 32'sd0;
      8'h91: update_dpi_tmp = 32'sd0;
      8'h92: update_dpi_tmp = 32'sd0;
      8'h93: update_dpi_tmp = 32'sd0;
      8'h94: update_dpi_tmp = 32'sd0;
      8'h95: update_dpi_tmp = 32'sd0;
      8'h96: update_dpi_tmp = 32'sd0;
      8'h97: update_dpi_tmp = 32'sd0;
      8'h98: update_dpi_tmp = 32'sd0;
      8'h99: update_dpi_tmp = 32'sd0;
      8'h9a: update_dpi_tmp = 32'sd0;
      8'h9b: update_dpi_tmp = 32'sd0;
      8'h9c: update_dpi_tmp = 32'sd0;
      8'h9d: update_dpi_tmp = 32'sd0;
      8'h9e: update_dpi_tmp = 32'sd0;
      8'h9f: update_dpi_tmp = 32'sd0;
      8'ha0: update_dpi_tmp = 32'sd0;
      8'ha1: update_dpi_tmp = 32'sd0;
      8'ha2: update_dpi_tmp = 32'sd0;
      8'ha3: update_dpi_tmp = 32'sd0;
      8'ha4: update_dpi_tmp = 32'sd0;
      8'ha5: update_dpi_tmp = 32'sd0;
      8'ha6: update_dpi_tmp = 32'sd0;
      8'ha7: update_dpi_tmp = 32'sd0;
      8'ha8: update_dpi_tmp = 32'sd0;
      8'ha9: update_dpi_tmp = 32'sd0;
      8'haa: update_dpi_tmp = 32'sd0;
      8'hab: update_dpi_tmp = 32'sd0;
      8'hac: update_dpi_tmp = 32'sd0;
      8'had: update_dpi_tmp = 32'sd0;
      8'hae: update_dpi_tmp = 32'sd0;
      8'haf: update_dpi_tmp = 32'sd0;
      8'hb0: update_dpi_tmp = 32'sd0;
      8'hb1: update_dpi_tmp = 32'sd0;
      8'hb2: update_dpi_tmp = 32'sd0;
      8'hb3: update_dpi_tmp = 32'sd0;
      8'hb4: update_dpi_tmp = 32'sd0;
      8'hb5: update_dpi_tmp = 32'sd0;
      8'hb6: update_dpi_tmp = 32'sd0;
      8'hb7: update_dpi_tmp = 32'sd0;
      8'hb8: update_dpi_tmp = 32'sd0;
      8'hb9: update_dpi_tmp = 32'sd0;
      8'hba: update_dpi_tmp = 32'sd0;
      8'hbb: update_dpi_tmp = 32'sd0;
      8'hbc: update_dpi_tmp = 32'sd0;
      8'hbd: update_dpi_tmp = 32'sd0;
      8'hbe: update_dpi_tmp = 32'sd0;
      8'hbf: update_dpi_tmp = 32'sd0;
      8'hc0: update_dpi_tmp = 32'sd0;
      8'hc1: update_dpi_tmp = 32'sd0;
      8'hc2: update_dpi_tmp = 32'sd0;
      8'hc3: update_dpi_tmp = 32'sd0;
      8'hc4: update_dpi_tmp = 32'sd0;
      8'hc5: update_dpi_tmp = 32'sd0;
      8'hc6: update_dpi_tmp = 32'sd0;
      8'hc7: update_dpi_tmp = 32'sd0;
      8'hc8: update_dpi_tmp = 32'sd0;
      8'hc9: update_dpi_tmp = 32'sd1;
      8'hca: update_dpi_tmp = 32'sd1;
      8'hcb: update_dpi_tmp = 32'sd1;
      8'hcc: update_dpi_tmp = 32'sd1;
      8'hcd: update_dpi_tmp = 32'sd1;
      8'hce: update_dpi_tmp = 32'sd1;
      8'hcf: update_dpi_tmp = 32'sd1;
      8'hd0: update_dpi_tmp = 32'sd1;
      8'hd1: update_dpi_tmp = 32'sd1;
      8'hd2: update_dpi_tmp = 32'sd1;
      8'hd3: update_dpi_tmp = 32'sd1;
      8'hd4: update_dpi_tmp = 32'sd1;
      8'hd5: update_dpi_tmp = 32'sd1;
      8'hd6: update_dpi_tmp = 32'sd1;
      8'hd7: update_dpi_tmp = 32'sd1;
      8'hd8: update_dpi_tmp = 32'sd1;
      8'hd9: update_dpi_tmp = 32'sd1;
      8'hda: update_dpi_tmp = 32'sd1;
      8'hdb: update_dpi_tmp = 32'sd2;
      8'hdc: update_dpi_tmp = 32'sd2;
      8'hdd: update_dpi_tmp = 32'sd2;
      8'hde: update_dpi_tmp = 32'sd2;
      8'hdf: update_dpi_tmp = 32'sd2;
      8'he0: update_dpi_tmp = 32'sd2;
      8'he1: update_dpi_tmp = 32'sd2;
      8'he2: update_dpi_tmp = 32'sd2;
      8'he3: update_dpi_tmp = 32'sd3;
      8'he4: update_dpi_tmp = 32'sd3;
      8'he5: update_dpi_tmp = 32'sd3;
      8'he6: update_dpi_tmp = 32'sd3;
      8'he7: update_dpi_tmp = 32'sd3;
      8'he8: update_dpi_tmp = 32'sd4;
      8'he9: update_dpi_tmp = 32'sd4;
      8'hea: update_dpi_tmp = 32'sd4;
      8'heb: update_dpi_tmp = 32'sd4;
      8'hec: update_dpi_tmp = 32'sd5;
      8'hed: update_dpi_tmp = 32'sd5;
      8'hee: update_dpi_tmp = 32'sd5;
      8'hef: update_dpi_tmp = 32'sd6;
      8'hf0: update_dpi_tmp = 32'sd6;
      8'hf1: update_dpi_tmp = 32'sd6;
      8'hf2: update_dpi_tmp = 32'sd7;
      8'hf3: update_dpi_tmp = 32'sd7;
      8'hf4: update_dpi_tmp = 32'sd8;
      8'hf5: update_dpi_tmp = 32'sd8;
      8'hf6: update_dpi_tmp = 32'sd9;
      8'hf7: update_dpi_tmp = 32'sd9;
      8'hf8: update_dpi_tmp = 32'sd10;
      8'hf9: update_dpi_tmp = 32'sd10;
      8'hfa: update_dpi_tmp = 32'sd11;
      8'hfb: update_dpi_tmp = 32'sd12;
      8'hfc: update_dpi_tmp = 32'sd12;
      8'hfd: update_dpi_tmp = 32'sd13;
      8'hfe: update_dpi_tmp = 32'sd14;
      8'hff: update_dpi_tmp = 32'sd15;
      default: update_dpi_tmp = 32'sd0;
    endcase
`endif
    if (update_dpi_tmp > 127) begin
      update_token_exp = 32'sd127;
    end else if (update_dpi_tmp < -128) begin
      update_token_exp = -32'sd128;
    end else begin
      update_token_exp = update_dpi_tmp;
    end

    update_product_q8 = running_sum_q * update_old_scale;
    if (update_product_q8 >= 32'sd0) begin
      update_rounded_q4 = (update_product_q8 + 32'sd8) >>> 4;
    end else begin
      update_rounded_q4 = -(((-update_product_q8) + 32'sd8) >>> 4);
    end
    update_sum_source = update_rounded_q4 + update_token_exp;
    if (update_sum_source > SCORE_SAT_MAX) begin
      update_sat_byte = 8'h7f;
    end else if (update_sum_source < SCORE_SAT_MIN) begin
      update_sat_byte = 8'h80;
    end else begin
      update_sat_byte = update_sum_source[7:0];
    end
    update_running_sum = {{24{update_sat_byte[7]}}, update_sat_byte};
  end

  always_comb begin : proc_value_datapath
    value_target_idx = {1'b0, pkt_meta_q.psum_offset} + {1'b0, task_id_current};
    value_target_lane = value_target_idx[5:0] - flit_start_idx_q[5:0];
    value_task_in_flit =
        ({1'b0, task_idx_q} < task_count_q) &&
        (task_id_current < k_dim_q) &&
        (value_target_idx >= flit_start_idx_q) &&
        (value_target_idx < flit_end_idx_q) &&
        (value_target_lane < 6'd32);
    value_old_scaled = 32'sd0;
    value_new_contrib = 32'sd0;
    value_output_sum = 32'sd0;
    value_product_q8 = 32'sd0;
    value_rounded_q4 = 32'sd0;
    value_byte_q4 = {{24{v_data_q[7]}}, v_data_q[7:0]};
    value_sat_byte = 8'd0;
    value_accum_next = output_accum_q[task_idx_q];

    value_product_q8 = output_accum_q[task_idx_q] * old_scale_q;
    if (value_product_q8 >= 32'sd0) begin
      value_rounded_q4 = (value_product_q8 + 32'sd8) >>> 4;
    end else begin
      value_rounded_q4 = -(((-value_product_q8) + 32'sd8) >>> 4);
    end
    value_old_scaled = value_rounded_q4;

    value_product_q8 = token_exp_q * value_byte_q4;
    if (value_product_q8 >= 32'sd0) begin
      value_rounded_q4 = (value_product_q8 + 32'sd8) >>> 4;
    end else begin
      value_rounded_q4 = -(((-value_product_q8) + 32'sd8) >>> 4);
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
    value_accum_next = {{24{value_sat_byte[7]}}, value_sat_byte};
  end

  always_ff @(posedge clk_i) begin : proc_attention_registers
    if (reset_i) begin
      state_q <= ATT_IDLE;
      pkt_meta_q <= '0;
      ctx_q <= '0;
      task_count_q <= '0;
      task_ids_flat_q <= '0;
      k_dim_q <= '0;
      token_stride_q <= '0;
      active_tokens_q <= '0;
      flit_start_idx_q <= '0;
      flit_end_idx_q <= '0;
      token_idx_q <= '0;
      k_base_idx_q <= '0;
      task_idx_q <= '0;
      k_data_q <= '0;
      v_data_q <= '0;
      score_accum_q <= 32'sd0;
      running_max_q <= NEG_INF_Q4;
      running_sum_q <= 32'sd0;
      old_scale_q <= 32'sd0;
      token_exp_q <= 32'sd0;
      result_flit_q <= '0;
      result_meta_q <= '0;
      result_valid_q <= 1'b0;
      for (reset_task_idx = 0; reset_task_idx < MAX_TASKS;
           reset_task_idx = reset_task_idx + 1) begin
        output_accum_q[reset_task_idx] <= 32'sd0;
      end
      for (reset_port_idx = 0; reset_port_idx < NUM_PORTS;
           reset_port_idx = reset_port_idx + 1) begin
        for (reset_vc_idx = 0; reset_vc_idx < VC_NUM;
             reset_vc_idx = reset_vc_idx + 1) begin
          query_valid_q[reset_port_idx][reset_vc_idx] <= 1'b0;
          for (reset_dim_idx = 0; reset_dim_idx < MAX_K_DIM;
               reset_dim_idx = reset_dim_idx + 1) begin
            query_q[reset_port_idx][reset_vc_idx][reset_dim_idx] <= 8'd0;
          end
        end
      end
    end else begin
      result_valid_q <= 1'b0;

      if (fetch_en_i && is_attention_op) begin
        pkt_meta_q <= pkt_meta_i;
        ctx_q <= ctx_i;
        task_count_q <= task_count_limited;
        task_ids_flat_q <= ctx_i.task_ids_flat;
        k_dim_q <= k_dim_clamped;
        token_stride_q <= token_stride;
        active_tokens_q <= active_tokens;
        flit_start_idx_q <= {7'd0, pkt_meta_i.data_offset};
        flit_end_idx_q <= {7'd0, pkt_meta_i.data_offset} + {11'd0, effective_payload_len};
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
        token_idx_q <= 16'd0;
        k_base_idx_q <= 16'd0;
        task_idx_q <= 5'd0;
        score_accum_q <= 32'sd0;
        running_max_q <=
            {{24{pkt_meta_i.header_reserved[7]}}, pkt_meta_i.header_reserved[7:0]};
        running_sum_q <=
            {{24{pkt_meta_i.header_reserved[15]}}, pkt_meta_i.header_reserved[15:8]};
        old_scale_q <= 32'sd0;
        token_exp_q <= 32'sd0;
        k_data_q <= data_rsp_i.rdata;
        for (reset_task_idx = 0; reset_task_idx < MAX_TASKS;
             reset_task_idx = reset_task_idx + 1) begin
          output_accum_q[reset_task_idx] <= 32'sd0;
        end

        if (head_like) begin
          query_valid_q[ctx_i.stream_port][ctx_i.stream_vc] <= 1'b1;
          for (query_lane_idx = 0; query_lane_idx < MAX_K_DIM;
               query_lane_idx = query_lane_idx + 1) begin
            query_q[ctx_i.stream_port][ctx_i.stream_vc][query_lane_idx] <= 8'd0;
          end
        end

        for (query_lane_idx = 0; query_lane_idx < 32;
             query_lane_idx = query_lane_idx + 1) begin
          if ((query_lane_idx < effective_payload_len) &&
              ((pkt_meta_i.data_offset + query_lane_idx) < pkt_meta_i.psum_offset) &&
              ((pkt_meta_i.data_offset + query_lane_idx) < k_dim_clamped)) begin
            query_q[ctx_i.stream_port][ctx_i.stream_vc]
                   [pkt_meta_i.data_offset + query_lane_idx] <=
                pkt_flit_i[query_lane_idx * 8 +: 8];
          end
        end

        if (start_attention_tail) begin
          state_q <= ATT_SCORE_K;
        end else if (start_attention_passthrough) begin
          state_q <= ATT_IDLE;
          result_valid_q <= 1'b1;
        end
      end

      if (compute_en_i) begin
        unique case (state_q)
          ATT_SCORE_K: begin
            if (score_has_next_data) begin
              score_accum_q <= score_sum_with_data;
              k_base_idx_q <= score_next_base_idx;
              k_data_q <= data_rsp_i.rdata;
            end else begin
              score_accum_q <= score_sat;
              state_q <= ATT_UPDATE_SOFTMAX;
            end
          end
          ATT_UPDATE_SOFTMAX: begin
            running_max_q <= update_new_max;
            running_sum_q <= update_running_sum;
            old_scale_q <= update_old_scale;
            token_exp_q <= update_token_exp;
            result_meta_q.header_reserved[7:0] <= update_new_max[7:0];
            result_meta_q.header_reserved[15:8] <= update_running_sum[7:0];
            if (task_count_q != 6'd0) begin
              task_idx_q <= 5'd0;
              v_data_q <= data_rsp_i.rdata;
              state_q <= ATT_VALUE_V;
            end else if (token_has_next) begin
              token_idx_q <= token_idx_q + 16'd1;
              k_base_idx_q <= 16'd0;
              score_accum_q <= 32'sd0;
              k_data_q <= data_rsp_i.rdata;
              state_q <= ATT_SCORE_K;
            end else begin
              state_q <= ATT_RESULT;
            end
          end
          ATT_VALUE_V: begin
            output_accum_q[task_idx_q] <= value_accum_next;
            if (value_task_in_flit) begin
              result_flit_q[value_target_lane * 8 +: 8] <= value_sat_byte;
            end
            if (value_has_next_task) begin
              task_idx_q <= task_idx_next[4:0];
              v_data_q <= data_rsp_i.rdata;
            end else if (token_has_next) begin
              token_idx_q <= token_idx_q + 16'd1;
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
        query_valid_q[ctx_i.stream_port][ctx_i.stream_vc] <= 1'b0;
        result_flit_q <= pkt_flit_i;
        result_meta_q <= pkt_meta_i;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_attention_assertions
    if (!reset_i) begin
      if (fetch_en_i && is_attention_op && !head_like &&
          !query_valid_q[ctx_i.stream_port][ctx_i.stream_vc]) begin
        $error("mfu_alu_attention: body/tail Attention flit arrived without head state");
      end

      if (release_state_i && is_attention_op && tail_like &&
          !query_valid_q[ctx_i.stream_port][ctx_i.stream_vc]) begin
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
