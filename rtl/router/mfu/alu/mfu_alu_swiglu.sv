// Description: SwiGLU leaf for the MFU ALU.  NONLINEAR_IMPL_DPI keeps the
//              optional DPI-C path; the default path uses a local Q4.4 SiLU LUT.

module mfu_alu_swiglu #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic start_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  logic signed [31:0] scalar_a_i,
    input  logic signed [31:0] scalar_b_i,
    output logic busy_o,
    output logic valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o,
    output logic mul_req_o,
    output logic signed [7:0] mul_lhs_o [0:7],
    output logic signed [7:0] mul_rhs_o [0:7],
    input  logic mul_rsp_valid_i,
    input  logic signed [15:0] mul_product_i [0:7]
);
  import router_ports_pkg::*;

`ifdef NONLINEAR_IMPL_DPI
  import "DPI-C" function int dpi_swiglu(input int gate_q, input int up_q);
`endif

  localparam int MUL_LANES = 8;
  localparam int PAIR_COUNT = 16;
  localparam logic [4:0] MUL_LANES_U5 = 5'd8;
  localparam logic [4:0] PAIR_COUNT_U5 = 5'd16;
  localparam logic [5:0] CNOC_MAX_TASKS_U6 = 6'd32;

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_ISSUE,
    ST_MUL,
    ST_MUL_WAIT,
    ST_COMMIT,
    ST_DONE
  } state_e;

  state_e state_q;
  logic [4:0] pair_base_q;
  logic [FLIT_W-1:0] flit_q;
  logic [FLIT_W-1:0] result_flit_q;
  flit_meta_t meta_q;
  flit_meta_t result_meta;
  logic [7:0] data_lane_byte_offset_q;
  logic [15:0] pair_update_mask_q;
  logic [5:0] effective_payload_len_q;
  logic signed [31:0] scalar_a_q;
  logic signed [31:0] scalar_b_q;

  logic [4:0] data_lane;
  logic [7:0] data_lane_byte_offset;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [5:0] task_count_limited;
  logic [5:0] task_idx_u6;
  logic [CNOC_TASK_ID_W-1:0] task_id_current;
  logic [16:0] required_flit_offset;
  logic [16:0] flit_start_idx;
  logic [16:0] flit_end_idx;
  logic [5:0] local_lane;
  logic [15:0] pair_update_mask;
  logic [4:0] issue_lane_u5;

  logic signed [7:0] mul_lhs_d [0:MUL_LANES-1];
  logic signed [7:0] mul_rhs_d [0:MUL_LANES-1];
  logic signed [7:0] mul_lhs_q [0:MUL_LANES-1];
  logic signed [7:0] mul_rhs_q [0:MUL_LANES-1];
  logic signed [15:0] mul_product_q [0:MUL_LANES-1];
  logic [MUL_LANES-1:0] lane_update_d;
  logic [MUL_LANES-1:0] lane_update_q;
  logic [3:0] lane_pair_idx_d [0:MUL_LANES-1];
  logic [3:0] lane_pair_idx_q [0:MUL_LANES-1];
`ifdef NONLINEAR_IMPL_DPI
  logic signed [31:0] lane_dpi_result_d [0:MUL_LANES-1];
  logic signed [31:0] lane_dpi_result_q [0:MUL_LANES-1];
`endif

  logic [4:0] lane_pair_idx;
  logic [7:0] lane_gate_byte;
  logic [7:0] lane_up_byte;
  logic signed [7:0] lane_lut_d [0:MUL_LANES-1];
  logic signed [15:0] lane_product;
  logic signed [15:0] lane_rounded;
  logic signed [31:0] lane_result;
  logic [31:0] lane_result_bits;
  logic [7:0] lane_sat_byte;
  logic [7:0] scalar_byte;
  logic [FLIT_W-1:0] result_flit_commit;
  logic last_round;

  int unsigned task_idx;
  int unsigned issue_lane_idx;
  int unsigned commit_lane_idx;
  int unsigned reset_lane_idx;
  int unsigned seq_lane_idx;

  assign data_lane = meta_i.data_offset[4:0];
  assign data_lane_byte_offset = {data_lane, 3'b000};
  assign payload_len = meta_i.payload_len;
  assign task_count_limited =
      (ctx_i.task_count > CNOC_MAX_TASKS_U6) ? CNOC_MAX_TASKS_U6 : ctx_i.task_count;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign busy_o = (state_q != ST_IDLE);
  assign last_round = (pair_base_q + MUL_LANES_U5) >= PAIR_COUNT_U5;
`ifdef NONLINEAR_IMPL_DPI
  assign mul_req_o = 1'b0;
`else
  assign mul_req_o = (state_q == ST_MUL);
`endif
  assign mul_lhs_o = mul_lhs_q;
  assign mul_rhs_o = mul_rhs_q;

  always_comb begin : proc_pair_update_mask
    pair_update_mask = '0;
    task_idx_u6 = '0;
    task_id_current = '0;
    required_flit_offset = '0;
    flit_start_idx = {7'd0, meta_i.data_offset};
    flit_end_idx = {7'd0, meta_i.data_offset} + {11'd0, effective_payload_len};
    local_lane = 6'd0;

    for (task_idx = 0; task_idx < CNOC_MAX_TASKS; task_idx = task_idx + 1) begin
      task_idx_u6 = task_idx[5:0];
      if (task_idx_u6 < task_count_limited) begin
        task_id_current = ctx_i.task_ids_flat[task_idx * CNOC_TASK_ID_W +: CNOC_TASK_ID_W];
        required_flit_offset = {1'b0, task_id_current} << 1;
        local_lane = required_flit_offset[5:0] - meta_i.data_offset[5:0];
        if ((required_flit_offset >= flit_start_idx) &&
            ((required_flit_offset + 17'd1) < flit_end_idx) &&
            ((local_lane + 6'd1) < 6'd32)) begin
          pair_update_mask[local_lane[4:1]] = 1'b1;
        end
      end
    end
  end

  always_comb begin : proc_issue_operands
    lane_pair_idx = '0;
    issue_lane_u5 = '0;
    lane_gate_byte = 8'd0;
    lane_up_byte = 8'd0;

    for (issue_lane_idx = 0; issue_lane_idx < MUL_LANES; issue_lane_idx = issue_lane_idx + 1) begin
      mul_lhs_d[issue_lane_idx] = '0;
      mul_rhs_d[issue_lane_idx] = '0;
      lane_lut_d[issue_lane_idx] = '0;
      lane_update_d[issue_lane_idx] = 1'b0;
      lane_pair_idx_d[issue_lane_idx] = '0;
`ifdef NONLINEAR_IMPL_DPI
      lane_dpi_result_d[issue_lane_idx] = 32'sd0;
`endif

      issue_lane_u5 = issue_lane_idx[4:0];
      lane_pair_idx = pair_base_q + issue_lane_u5;
      if (lane_pair_idx < PAIR_COUNT_U5) begin
        lane_gate_byte = flit_q[{lane_pair_idx[3:0], 4'b000} +: 8];
        lane_up_byte = flit_q[{lane_pair_idx[3:0], 4'b000} + 8 +: 8];
        lane_update_d[issue_lane_idx] =
            ((({1'b0, lane_pair_idx[3:0]} << 1) + 5'd1) < {1'b0, effective_payload_len_q}) &&
            pair_update_mask_q[lane_pair_idx[3:0]];
        lane_pair_idx_d[issue_lane_idx] = lane_pair_idx[3:0];
`ifdef NONLINEAR_IMPL_DPI
        lane_dpi_result_d[issue_lane_idx] = dpi_swiglu(
            {{24{lane_gate_byte[7]}}, lane_gate_byte},
            {{24{lane_up_byte[7]}}, lane_up_byte}
        );
`else
    case (lane_gate_byte)
          8'h00: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h01: lane_lut_d[issue_lane_idx] = 8'sd1;
          8'h02: lane_lut_d[issue_lane_idx] = 8'sd1;
          8'h03: lane_lut_d[issue_lane_idx] = 8'sd2;
          8'h04: lane_lut_d[issue_lane_idx] = 8'sd2;
          8'h05: lane_lut_d[issue_lane_idx] = 8'sd3;
          8'h06: lane_lut_d[issue_lane_idx] = 8'sd4;
          8'h07: lane_lut_d[issue_lane_idx] = 8'sd4;
          8'h08: lane_lut_d[issue_lane_idx] = 8'sd5;
          8'h09: lane_lut_d[issue_lane_idx] = 8'sd6;
          8'h0a: lane_lut_d[issue_lane_idx] = 8'sd7;
          8'h0b: lane_lut_d[issue_lane_idx] = 8'sd7;
          8'h0c: lane_lut_d[issue_lane_idx] = 8'sd8;
          8'h0d: lane_lut_d[issue_lane_idx] = 8'sd9;
          8'h0e: lane_lut_d[issue_lane_idx] = 8'sd10;
          8'h0f: lane_lut_d[issue_lane_idx] = 8'sd11;
          8'h10: lane_lut_d[issue_lane_idx] = 8'sd12;
          8'h11: lane_lut_d[issue_lane_idx] = 8'sd13;
          8'h12: lane_lut_d[issue_lane_idx] = 8'sd14;
          8'h13: lane_lut_d[issue_lane_idx] = 8'sd15;
          8'h14: lane_lut_d[issue_lane_idx] = 8'sd16;
          8'h15: lane_lut_d[issue_lane_idx] = 8'sd17;
          8'h16: lane_lut_d[issue_lane_idx] = 8'sd18;
          8'h17: lane_lut_d[issue_lane_idx] = 8'sd19;
          8'h18: lane_lut_d[issue_lane_idx] = 8'sd20;
          8'h19: lane_lut_d[issue_lane_idx] = 8'sd21;
          8'h1a: lane_lut_d[issue_lane_idx] = 8'sd22;
          8'h1b: lane_lut_d[issue_lane_idx] = 8'sd23;
          8'h1c: lane_lut_d[issue_lane_idx] = 8'sd24;
          8'h1d: lane_lut_d[issue_lane_idx] = 8'sd25;
          8'h1e: lane_lut_d[issue_lane_idx] = 8'sd26;
          8'h1f: lane_lut_d[issue_lane_idx] = 8'sd27;
          8'h20: lane_lut_d[issue_lane_idx] = 8'sd28;
          8'h21: lane_lut_d[issue_lane_idx] = 8'sd29;
          8'h22: lane_lut_d[issue_lane_idx] = 8'sd30;
          8'h23: lane_lut_d[issue_lane_idx] = 8'sd31;
          8'h24: lane_lut_d[issue_lane_idx] = 8'sd33;
          8'h25: lane_lut_d[issue_lane_idx] = 8'sd34;
          8'h26: lane_lut_d[issue_lane_idx] = 8'sd35;
          8'h27: lane_lut_d[issue_lane_idx] = 8'sd36;
          8'h28: lane_lut_d[issue_lane_idx] = 8'sd37;
          8'h29: lane_lut_d[issue_lane_idx] = 8'sd38;
          8'h2a: lane_lut_d[issue_lane_idx] = 8'sd39;
          8'h2b: lane_lut_d[issue_lane_idx] = 8'sd40;
          8'h2c: lane_lut_d[issue_lane_idx] = 8'sd41;
          8'h2d: lane_lut_d[issue_lane_idx] = 8'sd42;
          8'h2e: lane_lut_d[issue_lane_idx] = 8'sd44;
          8'h2f: lane_lut_d[issue_lane_idx] = 8'sd45;
          8'h30: lane_lut_d[issue_lane_idx] = 8'sd46;
          8'h31: lane_lut_d[issue_lane_idx] = 8'sd47;
          8'h32: lane_lut_d[issue_lane_idx] = 8'sd48;
          8'h33: lane_lut_d[issue_lane_idx] = 8'sd49;
          8'h34: lane_lut_d[issue_lane_idx] = 8'sd50;
          8'h35: lane_lut_d[issue_lane_idx] = 8'sd51;
          8'h36: lane_lut_d[issue_lane_idx] = 8'sd52;
          8'h37: lane_lut_d[issue_lane_idx] = 8'sd53;
          8'h38: lane_lut_d[issue_lane_idx] = 8'sd54;
          8'h39: lane_lut_d[issue_lane_idx] = 8'sd55;
          8'h3a: lane_lut_d[issue_lane_idx] = 8'sd56;
          8'h3b: lane_lut_d[issue_lane_idx] = 8'sd58;
          8'h3c: lane_lut_d[issue_lane_idx] = 8'sd59;
          8'h3d: lane_lut_d[issue_lane_idx] = 8'sd60;
          8'h3e: lane_lut_d[issue_lane_idx] = 8'sd61;
          8'h3f: lane_lut_d[issue_lane_idx] = 8'sd62;
          8'h40: lane_lut_d[issue_lane_idx] = 8'sd63;
          8'h41: lane_lut_d[issue_lane_idx] = 8'sd64;
          8'h42: lane_lut_d[issue_lane_idx] = 8'sd65;
          8'h43: lane_lut_d[issue_lane_idx] = 8'sd66;
          8'h44: lane_lut_d[issue_lane_idx] = 8'sd67;
          8'h45: lane_lut_d[issue_lane_idx] = 8'sd68;
          8'h46: lane_lut_d[issue_lane_idx] = 8'sd69;
          8'h47: lane_lut_d[issue_lane_idx] = 8'sd70;
          8'h48: lane_lut_d[issue_lane_idx] = 8'sd71;
          8'h49: lane_lut_d[issue_lane_idx] = 8'sd72;
          8'h4a: lane_lut_d[issue_lane_idx] = 8'sd73;
          8'h4b: lane_lut_d[issue_lane_idx] = 8'sd74;
          8'h4c: lane_lut_d[issue_lane_idx] = 8'sd75;
          8'h4d: lane_lut_d[issue_lane_idx] = 8'sd76;
          8'h4e: lane_lut_d[issue_lane_idx] = 8'sd77;
          8'h4f: lane_lut_d[issue_lane_idx] = 8'sd78;
          8'h50: lane_lut_d[issue_lane_idx] = 8'sd79;
          8'h51: lane_lut_d[issue_lane_idx] = 8'sd80;
          8'h52: lane_lut_d[issue_lane_idx] = 8'sd82;
          8'h53: lane_lut_d[issue_lane_idx] = 8'sd83;
          8'h54: lane_lut_d[issue_lane_idx] = 8'sd84;
          8'h55: lane_lut_d[issue_lane_idx] = 8'sd85;
          8'h56: lane_lut_d[issue_lane_idx] = 8'sd86;
          8'h57: lane_lut_d[issue_lane_idx] = 8'sd87;
          8'h58: lane_lut_d[issue_lane_idx] = 8'sd88;
          8'h59: lane_lut_d[issue_lane_idx] = 8'sd89;
          8'h5a: lane_lut_d[issue_lane_idx] = 8'sd90;
          8'h5b: lane_lut_d[issue_lane_idx] = 8'sd91;
          8'h5c: lane_lut_d[issue_lane_idx] = 8'sd92;
          8'h5d: lane_lut_d[issue_lane_idx] = 8'sd93;
          8'h5e: lane_lut_d[issue_lane_idx] = 8'sd94;
          8'h5f: lane_lut_d[issue_lane_idx] = 8'sd95;
          8'h60: lane_lut_d[issue_lane_idx] = 8'sd96;
          8'h61: lane_lut_d[issue_lane_idx] = 8'sd97;
          8'h62: lane_lut_d[issue_lane_idx] = 8'sd98;
          8'h63: lane_lut_d[issue_lane_idx] = 8'sd99;
          8'h64: lane_lut_d[issue_lane_idx] = 8'sd100;
          8'h65: lane_lut_d[issue_lane_idx] = 8'sd101;
          8'h66: lane_lut_d[issue_lane_idx] = 8'sd102;
          8'h67: lane_lut_d[issue_lane_idx] = 8'sd103;
          8'h68: lane_lut_d[issue_lane_idx] = 8'sd104;
          8'h69: lane_lut_d[issue_lane_idx] = 8'sd105;
          8'h6a: lane_lut_d[issue_lane_idx] = 8'sd106;
          8'h6b: lane_lut_d[issue_lane_idx] = 8'sd107;
          8'h6c: lane_lut_d[issue_lane_idx] = 8'sd108;
          8'h6d: lane_lut_d[issue_lane_idx] = 8'sd109;
          8'h6e: lane_lut_d[issue_lane_idx] = 8'sd110;
          8'h6f: lane_lut_d[issue_lane_idx] = 8'sd111;
          8'h70: lane_lut_d[issue_lane_idx] = 8'sd112;
          8'h71: lane_lut_d[issue_lane_idx] = 8'sd113;
          8'h72: lane_lut_d[issue_lane_idx] = 8'sd114;
          8'h73: lane_lut_d[issue_lane_idx] = 8'sd115;
          8'h74: lane_lut_d[issue_lane_idx] = 8'sd116;
          8'h75: lane_lut_d[issue_lane_idx] = 8'sd117;
          8'h76: lane_lut_d[issue_lane_idx] = 8'sd118;
          8'h77: lane_lut_d[issue_lane_idx] = 8'sd119;
          8'h78: lane_lut_d[issue_lane_idx] = 8'sd120;
          8'h79: lane_lut_d[issue_lane_idx] = 8'sd121;
          8'h7a: lane_lut_d[issue_lane_idx] = 8'sd122;
          8'h7b: lane_lut_d[issue_lane_idx] = 8'sd123;
          8'h7c: lane_lut_d[issue_lane_idx] = 8'sd124;
          8'h7d: lane_lut_d[issue_lane_idx] = 8'sd125;
          8'h7e: lane_lut_d[issue_lane_idx] = 8'sd126;
          8'h7f: lane_lut_d[issue_lane_idx] = 8'sd127;
          8'h80: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h81: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h82: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h83: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h84: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h85: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h86: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h87: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h88: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h89: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8a: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8b: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8c: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8d: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8e: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h8f: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h90: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h91: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h92: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h93: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h94: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h95: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h96: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h97: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h98: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h99: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9a: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9b: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9c: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9d: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9e: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'h9f: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha0: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha1: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha2: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha3: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha4: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha5: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha6: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha7: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha8: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'ha9: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'haa: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'hab: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'hac: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'had: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'hae: lane_lut_d[issue_lane_idx] = 8'sd0;
          8'haf: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb0: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb1: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb2: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb3: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb4: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb5: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb6: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb7: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb8: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hb9: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hba: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hbb: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hbc: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hbd: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hbe: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hbf: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc0: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc1: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc2: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc3: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc4: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc5: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hc6: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hc7: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hc8: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hc9: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hca: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hcb: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hcc: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hcd: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hce: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hcf: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hd0: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hd1: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hd2: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hd3: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd4: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd5: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd6: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd7: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd8: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hd9: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hda: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hdb: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hdc: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hdd: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hde: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hdf: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he0: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he1: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he2: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he3: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he4: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he5: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he6: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he7: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he8: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'he9: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hea: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'heb: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hec: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hed: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hee: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hef: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf0: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf1: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf2: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf3: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf4: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf5: lane_lut_d[issue_lane_idx] = -8'sd4;
          8'hf6: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hf7: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hf8: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hf9: lane_lut_d[issue_lane_idx] = -8'sd3;
          8'hfa: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hfb: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hfc: lane_lut_d[issue_lane_idx] = -8'sd2;
          8'hfd: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hfe: lane_lut_d[issue_lane_idx] = -8'sd1;
          8'hff: lane_lut_d[issue_lane_idx] = 8'sd0;
        endcase
        mul_lhs_d[issue_lane_idx] = lane_lut_d[issue_lane_idx];
        mul_rhs_d[issue_lane_idx] = $signed(lane_up_byte);
`endif
      end
    end
  end

  always_comb begin : proc_commit_result
    result_flit_commit = result_flit_q;
    lane_product = '0;
    lane_rounded = '0;
    lane_result = 32'sd0;
    lane_result_bits = 32'd0;
    lane_sat_byte = 8'd0;

    for (commit_lane_idx = 0; commit_lane_idx < MUL_LANES;
         commit_lane_idx = commit_lane_idx + 1) begin
      if (lane_update_q[commit_lane_idx]) begin
`ifdef NONLINEAR_IMPL_DPI
        lane_result = lane_dpi_result_q[commit_lane_idx];
`else
        lane_product = mul_product_q[commit_lane_idx];
        if (lane_product >= 16'sd0) begin
          lane_rounded = (lane_product + 16'sd8) >>> 4;
        end else begin
          lane_rounded = -(((-lane_product) + 16'sd8) >>> 4);
        end
        lane_result = $signed({{16{lane_rounded[15]}}, lane_rounded});
`endif
        lane_result_bits = $unsigned(lane_result);
        if (lane_result > 32'sd127) begin
          lane_sat_byte = 8'h7f;
        end else if (lane_result < -32'sd128) begin
          lane_sat_byte = 8'h80;
        end else begin
          lane_sat_byte = lane_result_bits[7:0];
        end
        case (lane_pair_idx_q[commit_lane_idx])
          4'd0:  result_flit_commit[7:0] = lane_sat_byte;
          4'd1:  result_flit_commit[23:16] = lane_sat_byte;
          4'd2:  result_flit_commit[39:32] = lane_sat_byte;
          4'd3:  result_flit_commit[55:48] = lane_sat_byte;
          4'd4:  result_flit_commit[71:64] = lane_sat_byte;
          4'd5:  result_flit_commit[87:80] = lane_sat_byte;
          4'd6:  result_flit_commit[103:96] = lane_sat_byte;
          4'd7:  result_flit_commit[119:112] = lane_sat_byte;
          4'd8:  result_flit_commit[135:128] = lane_sat_byte;
          4'd9:  result_flit_commit[151:144] = lane_sat_byte;
          4'd10: result_flit_commit[167:160] = lane_sat_byte;
          4'd11: result_flit_commit[183:176] = lane_sat_byte;
          4'd12: result_flit_commit[199:192] = lane_sat_byte;
          4'd13: result_flit_commit[215:208] = lane_sat_byte;
          4'd14: result_flit_commit[231:224] = lane_sat_byte;
          4'd15: result_flit_commit[247:240] = lane_sat_byte;
        endcase
      end
    end
  end

  always_comb begin : proc_result_meta
    result_meta = meta_q;
`ifdef ROUTER_ENABLE_COSIM
    result_meta.cosim.data_q = result_flit_q[data_lane_byte_offset_q +: 8];
`endif
  end

  always_ff @(posedge clk_i) begin : proc_registers
    if (reset_i) begin
      state_q <= ST_IDLE;
      pair_base_q <= '0;
      flit_q <= '0;
      result_flit_q <= '0;
      meta_q <= '0;
      data_lane_byte_offset_q <= '0;
      pair_update_mask_q <= '0;
      effective_payload_len_q <= '0;
      scalar_a_q <= 32'sd0;
      scalar_b_q <= 32'sd0;
      valid_o <= 1'b0;
      result_flit_o <= '0;
      result_meta_o <= '0;
      result_scalar_o <= 32'sd0;
      for (reset_lane_idx = 0; reset_lane_idx < MUL_LANES; reset_lane_idx = reset_lane_idx + 1) begin
        mul_lhs_q[reset_lane_idx] <= '0;
        mul_rhs_q[reset_lane_idx] <= '0;
        mul_product_q[reset_lane_idx] <= '0;
        lane_update_q[reset_lane_idx] <= 1'b0;
        lane_pair_idx_q[reset_lane_idx] <= '0;
`ifdef NONLINEAR_IMPL_DPI
        lane_dpi_result_q[reset_lane_idx] <= 32'sd0;
`endif
      end
    end else begin
      valid_o <= 1'b0;

      case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            state_q <= ST_ISSUE;
            pair_base_q <= '0;
            flit_q <= flit_i;
            result_flit_q <= flit_i;
            meta_q <= meta_i;
            data_lane_byte_offset_q <= data_lane_byte_offset;
            pair_update_mask_q <= pair_update_mask;
            effective_payload_len_q <= effective_payload_len;
            scalar_a_q <= scalar_a_i;
            scalar_b_q <= scalar_b_i;
          end
        end
        ST_ISSUE: begin
          state_q <= ST_MUL;
          for (seq_lane_idx = 0; seq_lane_idx < MUL_LANES; seq_lane_idx = seq_lane_idx + 1) begin
            mul_lhs_q[seq_lane_idx] <= mul_lhs_d[seq_lane_idx];
            mul_rhs_q[seq_lane_idx] <= mul_rhs_d[seq_lane_idx];
            lane_update_q[seq_lane_idx] <= lane_update_d[seq_lane_idx];
            lane_pair_idx_q[seq_lane_idx] <= lane_pair_idx_d[seq_lane_idx];
`ifdef NONLINEAR_IMPL_DPI
            lane_dpi_result_q[seq_lane_idx] <= lane_dpi_result_d[seq_lane_idx];
`endif
          end
        end
        ST_MUL: begin
`ifdef NONLINEAR_IMPL_DPI
          state_q <= ST_COMMIT;
`else
          state_q <= ST_MUL_WAIT;
`endif
        end
        ST_MUL_WAIT: begin
          if (mul_rsp_valid_i) begin
            state_q <= ST_COMMIT;
            for (seq_lane_idx = 0; seq_lane_idx < MUL_LANES; seq_lane_idx = seq_lane_idx + 1) begin
              mul_product_q[seq_lane_idx] <= mul_product_i[seq_lane_idx];
            end
          end
        end
        ST_COMMIT: begin
          result_flit_q <= result_flit_commit;
          if (last_round) begin
            state_q <= ST_DONE;
          end else begin
            state_q <= ST_ISSUE;
            pair_base_q <= pair_base_q + MUL_LANES_U5;
          end
        end
        ST_DONE: begin
          state_q <= ST_IDLE;
          valid_o <= 1'b1;
          result_flit_o <= result_flit_q;
          result_meta_o <= result_meta;
          scalar_byte = result_flit_q[data_lane_byte_offset_q +: 8];
          result_scalar_o <= $signed({{24{scalar_byte[7]}}, scalar_byte});
        end
        default: begin
          state_q <= ST_IDLE;
        end
      endcase
    end
  end

endmodule
