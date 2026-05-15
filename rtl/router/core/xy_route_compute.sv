// Description: Typed deterministic XY route computation.
//              Source-routed cNoC traffic is handled in route_compute_unit; this
//              helper covers regular traffic and malformed/exhausted routes.

module xy_route_compute (
    input  logic reset_i,
    input  logic [2:0] x_cur_i,
    input  logic [2:0] y_cur_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  logic flit_valid_i,
    output router_ports_pkg::router_port_sel_e route_sel_o
);
  import router_ports_pkg::*;

  always_comb begin
    route_sel_o = ROUTER_PORT_LOCAL;
    if (!reset_i && flit_valid_i) begin
      if (x_cur_i < meta_i.dst_x) begin
        route_sel_o = ROUTER_PORT_EAST;
      end else if (x_cur_i > meta_i.dst_x) begin
        route_sel_o = ROUTER_PORT_WEST;
      end else if (y_cur_i < meta_i.dst_y) begin
        route_sel_o = ROUTER_PORT_SOUTH;
      end else if (y_cur_i > meta_i.dst_y) begin
        route_sel_o = ROUTER_PORT_NORTH;
      end else begin
        route_sel_o = ROUTER_PORT_LOCAL;
      end
    end
  end

endmodule
