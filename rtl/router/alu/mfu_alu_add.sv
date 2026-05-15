// Description: Functional ADD leaf for the MFU ALU.
//              Pairwise INT8 addition stays combinational for now, but this
//              leaf isolates the opcode-specific path for later RTL work.

module mfu_alu_add #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  logic signed [31:0] op_a_i,
    input  logic signed [31:0] op_b_i,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

  integer lane_idx;
  logic signed [31:0] lhs_q;
  logic signed [31:0] rhs_q;
  logic signed [31:0] lane_value_tmp;
  logic [7:0] sat_byte_tmp;
  logic [7:0] scalar_byte_tmp;
  logic [FLIT_W-1:0] flit_tmp;
  flit_meta_t meta_tmp;
  logic [4:0] data_lane;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [15:0] pair_update_mask;

  assign data_lane = meta_i.data_offset[4:0];
  assign payload_len = meta_i.payload_len;
  assign pair_update_mask = meta_i.header_reserved;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;

  always_comb begin
    flit_tmp = flit_i;
    meta_tmp = meta_i;
    lhs_q = 32'sd0;
    rhs_q = 32'sd0;
    lane_value_tmp = 32'sd0;
    sat_byte_tmp = 8'd0;
    scalar_byte_tmp = 8'd0;

    for (lane_idx = 0; lane_idx < 32; lane_idx = lane_idx + 2) begin
      if (((lane_idx + 1) < effective_payload_len) &&
          pair_update_mask[lane_idx / 2]) begin
        lhs_q = {{24{flit_i[(lane_idx * 8) + 7]}}, flit_i[lane_idx * 8 +: 8]};
        rhs_q = {{24{flit_i[((lane_idx + 1) * 8) + 7]}},
                 flit_i[(lane_idx + 1) * 8 +: 8]};
        lane_value_tmp = lhs_q + rhs_q;
        if (lane_value_tmp > 32'sd127) begin
          sat_byte_tmp = 8'h7f;
        end else if (lane_value_tmp < -32'sd128) begin
          sat_byte_tmp = 8'h80;
        end else begin
          sat_byte_tmp = lane_value_tmp[7:0];
        end
        flit_tmp[lane_idx * 8 +: 8] = sat_byte_tmp;
      end
    end

    scalar_byte_tmp = flit_tmp[{data_lane, 3'b000} +: 8];
    result_flit_o = flit_tmp;
    result_meta_o = meta_tmp;
    result_scalar_o = {{24{scalar_byte_tmp[7]}}, scalar_byte_tmp} +
                      ((op_a_i ^ op_b_i) & 32'sd0);

`ifdef ROUTER_ENABLE_COSIM
    result_meta_o.cosim.data_q = result_flit_o[{data_lane, 3'b000} +: 8];
`endif
  end

endmodule
