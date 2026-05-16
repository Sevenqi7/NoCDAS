// Description: Functional fallback leaf for the MFU ALU.
//              The current implementation keeps the DPI-C exp/div contract so
//              the parent ALU can later replace it with a dedicated RTL path.

module mfu_alu_default #(
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

  import "DPI-C" function int dpi_exp(input int x_q);
  import "DPI-C" function int dpi_div(input int num_q, input int den_q);

  logic signed [31:0] scalar_tmp;
  logic signed [31:0] sat_tmp;
  logic [4:0] data_lane;
  int dpi_tmp;
  logic [FLIT_W-1:0] result_flit;
  flit_meta_t result_meta;

  assign data_lane = meta_i.data_offset[4:0];
  assign busy_o = 1'b0;

  always_comb begin : proc_default_result
    result_flit = flit_i;
    result_meta = meta_i;
    scalar_tmp = 32'sd0;
    sat_tmp = 32'sd0;
    dpi_tmp = 0;

    dpi_tmp = dpi_exp(scalar_a_i);
    sat_tmp = dpi_div(dpi_tmp, (scalar_b_i == 0) ? 1 : scalar_b_i);
    if (sat_tmp > 32'sd127) begin
      scalar_tmp = 32'sd127;
    end else if (sat_tmp < -32'sd128) begin
      scalar_tmp = -32'sd128;
    end else begin
      scalar_tmp = sat_tmp;
    end
    result_flit[data_lane * 8 +: 8] = scalar_tmp[7:0];
    result_meta = meta_i;

`ifdef ROUTER_ENABLE_COSIM
    result_meta.cosim.data_q = result_flit[{data_lane, 3'b000} +: 8];
`endif
  end

  always_ff @(posedge clk_i) begin : proc_default_registers
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
        result_scalar_o <= scalar_tmp;
      end
    end
  end

endmodule
