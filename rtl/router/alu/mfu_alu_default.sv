// Description: Functional fallback leaf for the MFU ALU.
//              The current implementation keeps the DPI-C exp/div contract so
//              the parent ALU can later replace it with a dedicated RTL path.

module mfu_alu_default #(
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

  import "DPI-C" function int dpi_exp(input int x_q);
  import "DPI-C" function int dpi_div(input int num_q, input int den_q);

  logic signed [31:0] scalar_tmp;
  logic signed [31:0] sat_tmp;
  logic [4:0] data_lane;
  int dpi_tmp;

  assign data_lane = meta_i.data_offset[4:0];

  always_comb begin
    result_flit_o = flit_i;
    result_meta_o = meta_i;
    scalar_tmp = 32'sd0;
    sat_tmp = 32'sd0;
    dpi_tmp = 0;

    dpi_tmp = dpi_exp(op_a_i);
    sat_tmp = dpi_div(dpi_tmp, (op_b_i == 0) ? 1 : op_b_i);
    if (sat_tmp > 32'sd127) begin
      scalar_tmp = 32'sd127;
    end else if (sat_tmp < -32'sd128) begin
      scalar_tmp = -32'sd128;
    end else begin
      scalar_tmp = sat_tmp;
    end
    result_flit_o[data_lane * 8 +: 8] = scalar_tmp[7:0];
    result_meta_o = meta_i;
    result_scalar_o = scalar_tmp;

`ifdef ROUTER_ENABLE_COSIM
    result_meta_o.cosim.data_q = result_flit_o[{data_lane, 3'b000} +: 8];
`endif
  end

endmodule
