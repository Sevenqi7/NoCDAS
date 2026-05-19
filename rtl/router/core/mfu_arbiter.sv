// Description: Selects the single cNoC flit that may enter the MFU.
//              Switch arbitration already prevents more than one MFU-bound
//              input from winning in a cycle. This module makes that ownership
//              explicit and keeps Router.sv focused on top-level composition.

module mfu_arbiter #(
    parameter int NUM_PORTS = 5,
    parameter int VC_ID_W = 3,
    parameter int FLIT_W = 256
) (
    input  logic [FLIT_W-1:0] raw_flit_i [NUM_PORTS],
    input  router_ports_pkg::flit_meta_t raw_meta_i [NUM_PORTS],
    input  router_ports_pkg::route_path_t raw_route_path_i [NUM_PORTS],
    input  logic [VC_ID_W-1:0] raw_vc_id_i [NUM_PORTS],
    input  logic [2:0] raw_stream_port_i [NUM_PORTS],
    input  logic [VC_ID_W-1:0] raw_stream_vc_i [NUM_PORTS],
    input  logic [NUM_PORTS-1:0] raw_valid_i,
    input  logic [NUM_PORTS-1:0] raw_needs_mfu_i,

    output logic [FLIT_W-1:0] mfu_flit_o,
    output router_ports_pkg::flit_meta_t mfu_meta_o,
    output router_ports_pkg::route_path_t mfu_route_path_o,
    output logic [VC_ID_W-1:0] mfu_vc_id_o,
    output logic [2:0] mfu_stream_port_o,
    output logic [VC_ID_W-1:0] mfu_stream_vc_o,
    output logic [2:0] mfu_out_sel_o,
    output logic mfu_valid_o
);
  import router_ports_pkg::*;

  int unsigned port_idx;

  always_comb begin
    mfu_flit_o = '0;
    mfu_meta_o = '0;
    mfu_route_path_o = '0;
    mfu_vc_id_o = '0;
    mfu_stream_port_o = ROUTER_PORT_INV;
    mfu_stream_vc_o = '0;
    mfu_out_sel_o = ROUTER_PORT_INV;
    mfu_valid_o = 1'b0;

    for (port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      // Pick the first raw output that both won the switch and requires MFU
      // service.  switch_allocator guarantees at most one such transfer should
      // be eligible, so this priority scan is a defensive final mux.
      if (!mfu_valid_o && raw_valid_i[port_idx] && raw_needs_mfu_i[port_idx]) begin
        mfu_flit_o = raw_flit_i[port_idx];
        mfu_meta_o = raw_meta_i[port_idx];
        mfu_route_path_o = raw_route_path_i[port_idx];
        mfu_vc_id_o = raw_vc_id_i[port_idx];
        mfu_stream_port_o = raw_stream_port_i[port_idx];
        mfu_stream_vc_o = raw_stream_vc_i[port_idx];
        mfu_out_sel_o = 3'(port_idx);
        mfu_valid_o = 1'b1;
      end
    end
  end

endmodule
