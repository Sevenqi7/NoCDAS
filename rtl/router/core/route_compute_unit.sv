// Description: Route computation for one router input port.
//              cNoC flits use a route_path_t sideband captured by input_unit.
//              This unit only receives the current hop port decoded from that
//              context, so the full source-route path does not enter the
//              common route/VC/switch metadata datapath.

module route_compute_unit (
    // Router reset.  Active high to match the imported NoC-Verilog baseline.
    input  logic reset_i,
    // Current router coordinate.
    input  logic [2:0] x_cur_i,
    input  logic [2:0] y_cur_i,
    // Kept for input_unit interface compatibility; XY routing no longer depends
    // on the incoming channel.
    input  logic [2:0] in_channel_i,
    // Metadata of the flit selected by the input VC scheduler.
    input  router_ports_pkg::flit_meta_t meta_i,
    // Source-route context owned by the selected input VC.  Head flits load
    // this state in input_unit; body/tail flits reuse it.
    input  logic       route_valid_i,
    input  logic [router_ports_pkg::META_ROUTE_LEN_W-1:0] route_len_i,
    input  logic [router_ports_pkg::META_ROUTE_PTR_W-1:0] route_ptr_i,
    input  logic [router_ports_pkg::ROUTE_PORT_W-1:0] route_port_i,
    // High when meta_i contains a valid flit.
    input  logic flit_valid_i,
    // Selected output port.
    output logic [2:0] route_sel_o
);
  import router_ports_pkg::*;

  router_port_sel_e xy_route_sel;
  router_msg_type_e msg_type;

  logic [2:0] source_route_port;
  logic       source_route_active;

  // The legacy input channel is intentionally kept for input_unit interface
  // compatibility, but typed XY/source-route routing no longer consumes it.
  // verilator lint_off UNUSED
  wire [2:0] unused_in_channel = in_channel_i;
  // verilator lint_on UNUSED

  always_comb begin
    msg_type = meta_i.msg_type;
    source_route_port = route_port_i;

    // Source route is honored only for cNoC traffic and only while the stored
    // head context still has unconsumed hops.
    source_route_active =
        flit_valid_i &&
        ((msg_type == ROUTER_MSG_DIST) || (msg_type == ROUTER_MSG_COMP)) &&
        route_valid_i &&
        (route_ptr_i < route_len_i) &&
        (int'(route_ptr_i) < ROUTE_MAX_HOPS) &&
        (source_route_port != ROUTER_PORT_INV);
  end

  xy_route_compute xy_route_compute_i (
      .reset_i(reset_i),
      .x_cur_i(x_cur_i),
      .y_cur_i(y_cur_i),
      .meta_i(meta_i),
      .flit_valid_i(flit_valid_i),
      .route_sel_o(xy_route_sel)
  );

  always_comb begin
    route_sel_o = ROUTER_PORT_LOCAL;
    // During reset, keep the output selection local/benign.
    if (!reset_i) begin
      // cNoC source routing overrides deterministic XY whenever a valid route
      // context is present for the selected input VC.
      if (source_route_active) begin
        route_sel_o = source_route_port;
      end else begin
        route_sel_o = xy_route_sel;
      end
    end
  end

endmodule
