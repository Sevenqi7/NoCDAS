// Description: Five-by-five Router pipeline crossbar.
//              Switch allocation owns the policy and provides one selected
//              input per output.  This module only muxes the selected pipeline
//              entry and does not inspect VC, QoS, cNoC, or MFU semantics.

module CrossBar #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM
) (
    input  router_ports_pkg::router_pipe_entry_t data_i  [NUM_PORTS],
    input  logic [2:0]                           select_i[NUM_PORTS],
    input  logic                                 valid_i [NUM_PORTS],
    output router_ports_pkg::router_pipe_entry_t data_o  [NUM_PORTS]
);
  import router_ports_pkg::*;

  always_comb begin : proc_crossbar_mux
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      data_o[out_idx] = '0;
      if (valid_i[out_idx]) begin
        case (select_i[out_idx])
          ROUTER_PORT_EAST:  data_o[out_idx] = data_i[ROUTER_PORT_EAST];
          ROUTER_PORT_WEST:  data_o[out_idx] = data_i[ROUTER_PORT_WEST];
          ROUTER_PORT_NORTH: data_o[out_idx] = data_i[ROUTER_PORT_NORTH];
          ROUTER_PORT_SOUTH: data_o[out_idx] = data_i[ROUTER_PORT_SOUTH];
          ROUTER_PORT_LOCAL: data_o[out_idx] = data_i[ROUTER_PORT_LOCAL];
          default:           data_o[out_idx] = '0;
        endcase
      end
    end
  end
endmodule
