// Description: Functional cNoC type4 store leaf.
//              The module keeps the current passthrough behavior so the parent
//              ALU can later swap this leaf for a real RTL implementation.

module mfu_alu_type4_store #(
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

  logic [4:0] data_lane;

  assign data_lane = meta_i.data_offset[4:0];

  always_comb begin
    result_flit_o = flit_i;
    result_meta_o = meta_i;
    result_scalar_o = op_a_i + (op_b_i - op_b_i);

`ifdef ROUTER_ENABLE_COSIM
    result_meta_o.cosim.data_q = result_flit_o[{data_lane, 3'b000} +: 8];
`endif
  end

endmodule
