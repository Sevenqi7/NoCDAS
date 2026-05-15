// Description: Functional MatMul leaf for the MFU ALU.
//              MatMul keeps the current scalar multiply/round/saturate model
//              so the parent ALU can later replace it with a real RTL unit.

module mfu_alu_matmul #(
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

  logic signed [31:0] scalar_tmp;
  logic signed [31:0] rounded32_tmp;
  logic signed [63:0] product_tmp;
  logic signed [63:0] rounded_tmp;
  logic [4:0] data_lane;

  assign data_lane = meta_i.data_offset[4:0];

  always_comb begin
    result_flit_o = flit_i;
    result_meta_o = meta_i;
    scalar_tmp = 32'sd0;
    rounded32_tmp = 32'sd0;
    product_tmp = 64'sd0;
    rounded_tmp = 64'sd0;

    product_tmp = op_a_i * op_b_i;
    if (product_tmp >= 64'sd0) begin
      rounded_tmp = (product_tmp + 64'sd8) >>> 4;
    end else begin
      rounded_tmp = -(((-product_tmp) + 64'sd8) >>> 4);
    end
    rounded32_tmp = rounded_tmp[31:0];
    if (rounded32_tmp > 32'sd127) begin
      scalar_tmp = 32'sd127;
    end else if (rounded32_tmp < -32'sd128) begin
      scalar_tmp = -32'sd128;
    end else begin
      scalar_tmp = rounded32_tmp;
    end

    result_scalar_o = scalar_tmp;

`ifdef ROUTER_ENABLE_COSIM
    result_meta_o.cosim.data_q = result_flit_o[{data_lane, 3'b000} +: 8];
`endif
  end

endmodule
