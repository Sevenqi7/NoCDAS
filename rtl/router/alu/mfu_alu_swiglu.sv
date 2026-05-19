// Description: SwiGLU leaf for the MFU ALU.  NONLINEAR_IMPL_DPI keeps the
//              legacy DPI-C path; the default path uses a local Q4.4 SiLU LUT.

module mfu_alu_swiglu #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic start_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  logic signed [31:0] scalar_a_i,
    input  logic signed [31:0] scalar_b_i,
    output logic busy_o,
    output logic valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

`ifdef NONLINEAR_IMPL_DPI
  import "DPI-C" function int dpi_swiglu(input int gate_q, input int up_q);
`endif

  integer lane_idx;
  logic signed [31:0] lhs_q;
  logic signed [31:0] rhs_q;
  logic signed [31:0] nonlinear_tmp;
  logic signed [31:0] nonlinear_lut_q4;
  logic signed [31:0] nonlinear_product_q8;
  logic signed [31:0] nonlinear_rounded_q4;
  logic signed [31:0] scalar_tmp;
  logic [7:0] sat_byte_tmp;
  logic [7:0] scalar_byte_tmp;
  logic [FLIT_W-1:0] flit_tmp;
  flit_meta_t meta_tmp;
  logic [4:0] data_lane;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [15:0] pair_update_mask;
  logic [FLIT_W-1:0] result_flit;
  flit_meta_t result_meta;
  logic signed [31:0] result_scalar;

  assign data_lane = meta_i.data_offset[4:0];
  assign payload_len = meta_i.payload_len;
  assign pair_update_mask = meta_i.header_reserved;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign busy_o = 1'b0;

  always_comb begin : proc_swiglu_result
    flit_tmp = flit_i;
    meta_tmp = meta_i;
    lhs_q = 32'sd0;
    rhs_q = 32'sd0;
    scalar_tmp = 32'sd0;
    sat_byte_tmp = 8'd0;
    scalar_byte_tmp = 8'd0;
    nonlinear_tmp = 32'sd0;
    nonlinear_lut_q4 = 32'sd0;
    nonlinear_product_q8 = 32'sd0;
    nonlinear_rounded_q4 = 32'sd0;

    for (lane_idx = 0; lane_idx < 32; lane_idx = lane_idx + 2) begin
      if (((lane_idx + 1) < effective_payload_len) &&
          pair_update_mask[lane_idx / 2]) begin
        lhs_q = {{24{flit_i[(lane_idx * 8) + 7]}}, flit_i[lane_idx * 8 +: 8]};
        rhs_q = {{24{flit_i[((lane_idx + 1) * 8) + 7]}},
                 flit_i[(lane_idx + 1) * 8 +: 8]};
`ifdef NONLINEAR_IMPL_DPI
        nonlinear_tmp = dpi_swiglu(lhs_q, rhs_q);
`else
        unique case (lhs_q[7:0])
          8'h00: nonlinear_lut_q4 = 32'sd0;
          8'h01: nonlinear_lut_q4 = 32'sd1;
          8'h02: nonlinear_lut_q4 = 32'sd1;
          8'h03: nonlinear_lut_q4 = 32'sd2;
          8'h04: nonlinear_lut_q4 = 32'sd2;
          8'h05: nonlinear_lut_q4 = 32'sd3;
          8'h06: nonlinear_lut_q4 = 32'sd4;
          8'h07: nonlinear_lut_q4 = 32'sd4;
          8'h08: nonlinear_lut_q4 = 32'sd5;
          8'h09: nonlinear_lut_q4 = 32'sd6;
          8'h0a: nonlinear_lut_q4 = 32'sd7;
          8'h0b: nonlinear_lut_q4 = 32'sd7;
          8'h0c: nonlinear_lut_q4 = 32'sd8;
          8'h0d: nonlinear_lut_q4 = 32'sd9;
          8'h0e: nonlinear_lut_q4 = 32'sd10;
          8'h0f: nonlinear_lut_q4 = 32'sd11;
          8'h10: nonlinear_lut_q4 = 32'sd12;
          8'h11: nonlinear_lut_q4 = 32'sd13;
          8'h12: nonlinear_lut_q4 = 32'sd14;
          8'h13: nonlinear_lut_q4 = 32'sd15;
          8'h14: nonlinear_lut_q4 = 32'sd16;
          8'h15: nonlinear_lut_q4 = 32'sd17;
          8'h16: nonlinear_lut_q4 = 32'sd18;
          8'h17: nonlinear_lut_q4 = 32'sd19;
          8'h18: nonlinear_lut_q4 = 32'sd20;
          8'h19: nonlinear_lut_q4 = 32'sd21;
          8'h1a: nonlinear_lut_q4 = 32'sd22;
          8'h1b: nonlinear_lut_q4 = 32'sd23;
          8'h1c: nonlinear_lut_q4 = 32'sd24;
          8'h1d: nonlinear_lut_q4 = 32'sd25;
          8'h1e: nonlinear_lut_q4 = 32'sd26;
          8'h1f: nonlinear_lut_q4 = 32'sd27;
          8'h20: nonlinear_lut_q4 = 32'sd28;
          8'h21: nonlinear_lut_q4 = 32'sd29;
          8'h22: nonlinear_lut_q4 = 32'sd30;
          8'h23: nonlinear_lut_q4 = 32'sd31;
          8'h24: nonlinear_lut_q4 = 32'sd33;
          8'h25: nonlinear_lut_q4 = 32'sd34;
          8'h26: nonlinear_lut_q4 = 32'sd35;
          8'h27: nonlinear_lut_q4 = 32'sd36;
          8'h28: nonlinear_lut_q4 = 32'sd37;
          8'h29: nonlinear_lut_q4 = 32'sd38;
          8'h2a: nonlinear_lut_q4 = 32'sd39;
          8'h2b: nonlinear_lut_q4 = 32'sd40;
          8'h2c: nonlinear_lut_q4 = 32'sd41;
          8'h2d: nonlinear_lut_q4 = 32'sd42;
          8'h2e: nonlinear_lut_q4 = 32'sd44;
          8'h2f: nonlinear_lut_q4 = 32'sd45;
          8'h30: nonlinear_lut_q4 = 32'sd46;
          8'h31: nonlinear_lut_q4 = 32'sd47;
          8'h32: nonlinear_lut_q4 = 32'sd48;
          8'h33: nonlinear_lut_q4 = 32'sd49;
          8'h34: nonlinear_lut_q4 = 32'sd50;
          8'h35: nonlinear_lut_q4 = 32'sd51;
          8'h36: nonlinear_lut_q4 = 32'sd52;
          8'h37: nonlinear_lut_q4 = 32'sd53;
          8'h38: nonlinear_lut_q4 = 32'sd54;
          8'h39: nonlinear_lut_q4 = 32'sd55;
          8'h3a: nonlinear_lut_q4 = 32'sd56;
          8'h3b: nonlinear_lut_q4 = 32'sd58;
          8'h3c: nonlinear_lut_q4 = 32'sd59;
          8'h3d: nonlinear_lut_q4 = 32'sd60;
          8'h3e: nonlinear_lut_q4 = 32'sd61;
          8'h3f: nonlinear_lut_q4 = 32'sd62;
          8'h40: nonlinear_lut_q4 = 32'sd63;
          8'h41: nonlinear_lut_q4 = 32'sd64;
          8'h42: nonlinear_lut_q4 = 32'sd65;
          8'h43: nonlinear_lut_q4 = 32'sd66;
          8'h44: nonlinear_lut_q4 = 32'sd67;
          8'h45: nonlinear_lut_q4 = 32'sd68;
          8'h46: nonlinear_lut_q4 = 32'sd69;
          8'h47: nonlinear_lut_q4 = 32'sd70;
          8'h48: nonlinear_lut_q4 = 32'sd71;
          8'h49: nonlinear_lut_q4 = 32'sd72;
          8'h4a: nonlinear_lut_q4 = 32'sd73;
          8'h4b: nonlinear_lut_q4 = 32'sd74;
          8'h4c: nonlinear_lut_q4 = 32'sd75;
          8'h4d: nonlinear_lut_q4 = 32'sd76;
          8'h4e: nonlinear_lut_q4 = 32'sd77;
          8'h4f: nonlinear_lut_q4 = 32'sd78;
          8'h50: nonlinear_lut_q4 = 32'sd79;
          8'h51: nonlinear_lut_q4 = 32'sd80;
          8'h52: nonlinear_lut_q4 = 32'sd82;
          8'h53: nonlinear_lut_q4 = 32'sd83;
          8'h54: nonlinear_lut_q4 = 32'sd84;
          8'h55: nonlinear_lut_q4 = 32'sd85;
          8'h56: nonlinear_lut_q4 = 32'sd86;
          8'h57: nonlinear_lut_q4 = 32'sd87;
          8'h58: nonlinear_lut_q4 = 32'sd88;
          8'h59: nonlinear_lut_q4 = 32'sd89;
          8'h5a: nonlinear_lut_q4 = 32'sd90;
          8'h5b: nonlinear_lut_q4 = 32'sd91;
          8'h5c: nonlinear_lut_q4 = 32'sd92;
          8'h5d: nonlinear_lut_q4 = 32'sd93;
          8'h5e: nonlinear_lut_q4 = 32'sd94;
          8'h5f: nonlinear_lut_q4 = 32'sd95;
          8'h60: nonlinear_lut_q4 = 32'sd96;
          8'h61: nonlinear_lut_q4 = 32'sd97;
          8'h62: nonlinear_lut_q4 = 32'sd98;
          8'h63: nonlinear_lut_q4 = 32'sd99;
          8'h64: nonlinear_lut_q4 = 32'sd100;
          8'h65: nonlinear_lut_q4 = 32'sd101;
          8'h66: nonlinear_lut_q4 = 32'sd102;
          8'h67: nonlinear_lut_q4 = 32'sd103;
          8'h68: nonlinear_lut_q4 = 32'sd104;
          8'h69: nonlinear_lut_q4 = 32'sd105;
          8'h6a: nonlinear_lut_q4 = 32'sd106;
          8'h6b: nonlinear_lut_q4 = 32'sd107;
          8'h6c: nonlinear_lut_q4 = 32'sd108;
          8'h6d: nonlinear_lut_q4 = 32'sd109;
          8'h6e: nonlinear_lut_q4 = 32'sd110;
          8'h6f: nonlinear_lut_q4 = 32'sd111;
          8'h70: nonlinear_lut_q4 = 32'sd112;
          8'h71: nonlinear_lut_q4 = 32'sd113;
          8'h72: nonlinear_lut_q4 = 32'sd114;
          8'h73: nonlinear_lut_q4 = 32'sd115;
          8'h74: nonlinear_lut_q4 = 32'sd116;
          8'h75: nonlinear_lut_q4 = 32'sd117;
          8'h76: nonlinear_lut_q4 = 32'sd118;
          8'h77: nonlinear_lut_q4 = 32'sd119;
          8'h78: nonlinear_lut_q4 = 32'sd120;
          8'h79: nonlinear_lut_q4 = 32'sd121;
          8'h7a: nonlinear_lut_q4 = 32'sd122;
          8'h7b: nonlinear_lut_q4 = 32'sd123;
          8'h7c: nonlinear_lut_q4 = 32'sd124;
          8'h7d: nonlinear_lut_q4 = 32'sd125;
          8'h7e: nonlinear_lut_q4 = 32'sd126;
          8'h7f: nonlinear_lut_q4 = 32'sd127;
          8'h80: nonlinear_lut_q4 = 32'sd0;
          8'h81: nonlinear_lut_q4 = 32'sd0;
          8'h82: nonlinear_lut_q4 = 32'sd0;
          8'h83: nonlinear_lut_q4 = 32'sd0;
          8'h84: nonlinear_lut_q4 = 32'sd0;
          8'h85: nonlinear_lut_q4 = 32'sd0;
          8'h86: nonlinear_lut_q4 = 32'sd0;
          8'h87: nonlinear_lut_q4 = 32'sd0;
          8'h88: nonlinear_lut_q4 = 32'sd0;
          8'h89: nonlinear_lut_q4 = 32'sd0;
          8'h8a: nonlinear_lut_q4 = 32'sd0;
          8'h8b: nonlinear_lut_q4 = 32'sd0;
          8'h8c: nonlinear_lut_q4 = 32'sd0;
          8'h8d: nonlinear_lut_q4 = 32'sd0;
          8'h8e: nonlinear_lut_q4 = 32'sd0;
          8'h8f: nonlinear_lut_q4 = 32'sd0;
          8'h90: nonlinear_lut_q4 = 32'sd0;
          8'h91: nonlinear_lut_q4 = 32'sd0;
          8'h92: nonlinear_lut_q4 = 32'sd0;
          8'h93: nonlinear_lut_q4 = 32'sd0;
          8'h94: nonlinear_lut_q4 = 32'sd0;
          8'h95: nonlinear_lut_q4 = 32'sd0;
          8'h96: nonlinear_lut_q4 = 32'sd0;
          8'h97: nonlinear_lut_q4 = 32'sd0;
          8'h98: nonlinear_lut_q4 = 32'sd0;
          8'h99: nonlinear_lut_q4 = 32'sd0;
          8'h9a: nonlinear_lut_q4 = 32'sd0;
          8'h9b: nonlinear_lut_q4 = 32'sd0;
          8'h9c: nonlinear_lut_q4 = 32'sd0;
          8'h9d: nonlinear_lut_q4 = 32'sd0;
          8'h9e: nonlinear_lut_q4 = 32'sd0;
          8'h9f: nonlinear_lut_q4 = 32'sd0;
          8'ha0: nonlinear_lut_q4 = 32'sd0;
          8'ha1: nonlinear_lut_q4 = 32'sd0;
          8'ha2: nonlinear_lut_q4 = 32'sd0;
          8'ha3: nonlinear_lut_q4 = 32'sd0;
          8'ha4: nonlinear_lut_q4 = 32'sd0;
          8'ha5: nonlinear_lut_q4 = 32'sd0;
          8'ha6: nonlinear_lut_q4 = 32'sd0;
          8'ha7: nonlinear_lut_q4 = 32'sd0;
          8'ha8: nonlinear_lut_q4 = 32'sd0;
          8'ha9: nonlinear_lut_q4 = 32'sd0;
          8'haa: nonlinear_lut_q4 = 32'sd0;
          8'hab: nonlinear_lut_q4 = 32'sd0;
          8'hac: nonlinear_lut_q4 = 32'sd0;
          8'had: nonlinear_lut_q4 = 32'sd0;
          8'hae: nonlinear_lut_q4 = 32'sd0;
          8'haf: nonlinear_lut_q4 = -32'sd1;
          8'hb0: nonlinear_lut_q4 = -32'sd1;
          8'hb1: nonlinear_lut_q4 = -32'sd1;
          8'hb2: nonlinear_lut_q4 = -32'sd1;
          8'hb3: nonlinear_lut_q4 = -32'sd1;
          8'hb4: nonlinear_lut_q4 = -32'sd1;
          8'hb5: nonlinear_lut_q4 = -32'sd1;
          8'hb6: nonlinear_lut_q4 = -32'sd1;
          8'hb7: nonlinear_lut_q4 = -32'sd1;
          8'hb8: nonlinear_lut_q4 = -32'sd1;
          8'hb9: nonlinear_lut_q4 = -32'sd1;
          8'hba: nonlinear_lut_q4 = -32'sd1;
          8'hbb: nonlinear_lut_q4 = -32'sd1;
          8'hbc: nonlinear_lut_q4 = -32'sd1;
          8'hbd: nonlinear_lut_q4 = -32'sd1;
          8'hbe: nonlinear_lut_q4 = -32'sd1;
          8'hbf: nonlinear_lut_q4 = -32'sd1;
          8'hc0: nonlinear_lut_q4 = -32'sd1;
          8'hc1: nonlinear_lut_q4 = -32'sd1;
          8'hc2: nonlinear_lut_q4 = -32'sd1;
          8'hc3: nonlinear_lut_q4 = -32'sd1;
          8'hc4: nonlinear_lut_q4 = -32'sd1;
          8'hc5: nonlinear_lut_q4 = -32'sd1;
          8'hc6: nonlinear_lut_q4 = -32'sd2;
          8'hc7: nonlinear_lut_q4 = -32'sd2;
          8'hc8: nonlinear_lut_q4 = -32'sd2;
          8'hc9: nonlinear_lut_q4 = -32'sd2;
          8'hca: nonlinear_lut_q4 = -32'sd2;
          8'hcb: nonlinear_lut_q4 = -32'sd2;
          8'hcc: nonlinear_lut_q4 = -32'sd2;
          8'hcd: nonlinear_lut_q4 = -32'sd2;
          8'hce: nonlinear_lut_q4 = -32'sd2;
          8'hcf: nonlinear_lut_q4 = -32'sd2;
          8'hd0: nonlinear_lut_q4 = -32'sd2;
          8'hd1: nonlinear_lut_q4 = -32'sd2;
          8'hd2: nonlinear_lut_q4 = -32'sd2;
          8'hd3: nonlinear_lut_q4 = -32'sd3;
          8'hd4: nonlinear_lut_q4 = -32'sd3;
          8'hd5: nonlinear_lut_q4 = -32'sd3;
          8'hd6: nonlinear_lut_q4 = -32'sd3;
          8'hd7: nonlinear_lut_q4 = -32'sd3;
          8'hd8: nonlinear_lut_q4 = -32'sd3;
          8'hd9: nonlinear_lut_q4 = -32'sd3;
          8'hda: nonlinear_lut_q4 = -32'sd3;
          8'hdb: nonlinear_lut_q4 = -32'sd3;
          8'hdc: nonlinear_lut_q4 = -32'sd3;
          8'hdd: nonlinear_lut_q4 = -32'sd4;
          8'hde: nonlinear_lut_q4 = -32'sd4;
          8'hdf: nonlinear_lut_q4 = -32'sd4;
          8'he0: nonlinear_lut_q4 = -32'sd4;
          8'he1: nonlinear_lut_q4 = -32'sd4;
          8'he2: nonlinear_lut_q4 = -32'sd4;
          8'he3: nonlinear_lut_q4 = -32'sd4;
          8'he4: nonlinear_lut_q4 = -32'sd4;
          8'he5: nonlinear_lut_q4 = -32'sd4;
          8'he6: nonlinear_lut_q4 = -32'sd4;
          8'he7: nonlinear_lut_q4 = -32'sd4;
          8'he8: nonlinear_lut_q4 = -32'sd4;
          8'he9: nonlinear_lut_q4 = -32'sd4;
          8'hea: nonlinear_lut_q4 = -32'sd4;
          8'heb: nonlinear_lut_q4 = -32'sd4;
          8'hec: nonlinear_lut_q4 = -32'sd4;
          8'hed: nonlinear_lut_q4 = -32'sd4;
          8'hee: nonlinear_lut_q4 = -32'sd4;
          8'hef: nonlinear_lut_q4 = -32'sd4;
          8'hf0: nonlinear_lut_q4 = -32'sd4;
          8'hf1: nonlinear_lut_q4 = -32'sd4;
          8'hf2: nonlinear_lut_q4 = -32'sd4;
          8'hf3: nonlinear_lut_q4 = -32'sd4;
          8'hf4: nonlinear_lut_q4 = -32'sd4;
          8'hf5: nonlinear_lut_q4 = -32'sd4;
          8'hf6: nonlinear_lut_q4 = -32'sd3;
          8'hf7: nonlinear_lut_q4 = -32'sd3;
          8'hf8: nonlinear_lut_q4 = -32'sd3;
          8'hf9: nonlinear_lut_q4 = -32'sd3;
          8'hfa: nonlinear_lut_q4 = -32'sd2;
          8'hfb: nonlinear_lut_q4 = -32'sd2;
          8'hfc: nonlinear_lut_q4 = -32'sd2;
          8'hfd: nonlinear_lut_q4 = -32'sd1;
          8'hfe: nonlinear_lut_q4 = -32'sd1;
          8'hff: nonlinear_lut_q4 = 32'sd0;
          default: nonlinear_lut_q4 = 32'sd0;
        endcase
        nonlinear_product_q8 = nonlinear_lut_q4 * rhs_q;
        if (nonlinear_product_q8 >= 32'sd0) begin
          nonlinear_rounded_q4 = (nonlinear_product_q8 + 32'sd8) >>> 4;
        end else begin
          nonlinear_rounded_q4 = -(((-nonlinear_product_q8) + 32'sd8) >>> 4);
        end
        nonlinear_tmp = nonlinear_rounded_q4;
`endif
        if (nonlinear_tmp > 127) begin
          sat_byte_tmp = 8'h7f;
        end else if (nonlinear_tmp < -128) begin
          sat_byte_tmp = 8'h80;
        end else begin
          sat_byte_tmp = nonlinear_tmp[7:0];
        end
        flit_tmp[lane_idx * 8 +: 8] = sat_byte_tmp;
      end
    end

    scalar_byte_tmp = flit_tmp[{data_lane, 3'b000} +: 8];
    scalar_tmp = {{24{scalar_byte_tmp[7]}}, scalar_byte_tmp} +
                      ((scalar_a_i ^ scalar_b_i) & 32'sd0);
    result_flit = flit_tmp;
    result_meta = meta_tmp;
    result_scalar = scalar_tmp;

`ifdef ROUTER_ENABLE_COSIM
    result_meta.cosim.data_q = result_flit[{data_lane, 3'b000} +: 8];
`endif
  end

  always_ff @(posedge clk_i) begin : proc_swiglu_registers
    if (reset_i) begin
      valid_o <= 1'b0;
      result_flit_o <= '0;
      result_meta_o <= '0;
      result_scalar_o <= 32'sd0;
    end else begin
      valid_o <= start_i;
      if (start_i) begin
        result_flit_o <= result_flit;
        result_meta_o <= result_meta;
        result_scalar_o <= result_scalar;
      end
    end
  end

endmodule
