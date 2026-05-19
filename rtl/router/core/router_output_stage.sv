// Description: Router output datapath finalization.
//              Converts crossbar-selected outputs into raw cNoC/MFU candidates,
//              advances source-route pointers, suppresses MFU-captured flits,
//              and reinserts MFU writeback results.
//              The current pipelined Router top owns output commit internally;
//              this module remains a standalone legacy/reference block rather
//              than Router.sv's active output stage.

module router_output_stage #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic [FLIT_W-1:0] xbar_flit_i [NUM_PORTS],
    input  router_ports_pkg::flit_meta_t xbar_meta_i [NUM_PORTS],
    input  router_ports_pkg::route_path_t xbar_route_path_i [NUM_PORTS],
    input  logic [2:0] routport_winner_i [NUM_PORTS],
    input  logic [VC_ID_W-1:0] vc_alloc_dst_vc_i [NUM_PORTS],

    input  logic mfu_capture_i,
    input  logic [2:0] mfu_capture_out_sel_i,
    input  logic mfu_emit_valid_i,
    input  logic [2:0] mfu_emit_out_sel_i,
    input  logic [FLIT_W-1:0] mfu_emit_flit_i,
    input  router_ports_pkg::flit_meta_t mfu_emit_meta_i,
    input  router_ports_pkg::route_path_t mfu_emit_route_path_i,
    input  logic [VC_ID_W-1:0] mfu_emit_vc_id_i,

    output logic [FLIT_W-1:0] raw_flit_o [NUM_PORTS],
    output router_ports_pkg::flit_meta_t raw_meta_o [NUM_PORTS],
    output router_ports_pkg::route_path_t raw_route_path_o [NUM_PORTS],
    output logic [VC_ID_W-1:0] raw_vc_id_o [NUM_PORTS],
    output logic [NUM_PORTS-1:0] raw_write_o,
    output router_ports_pkg::routport_data_t routport_data_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  always_comb begin : proc_raw_output
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      raw_flit_o[out_idx] = xbar_flit_i[out_idx];
      raw_meta_o[out_idx] = xbar_meta_i[out_idx];
      raw_route_path_o[out_idx] = xbar_route_path_i[out_idx];
      raw_vc_id_o[out_idx] = vc_alloc_dst_vc_i[out_idx];
      raw_write_o[out_idx] = (routport_winner_i[out_idx] != ROUTER_PORT_INV);

      // cNoC flits consume one source-route hop when they leave this router.
      // Regular traffic keeps the meta unchanged and recomputes XY every hop.
      if (((xbar_meta_i[out_idx].msg_type == ROUTER_MSG_DIST) ||
           (xbar_meta_i[out_idx].msg_type == ROUTER_MSG_COMP)) &&
          raw_write_o[out_idx] &&
          xbar_meta_i[out_idx].route_valid &&
          xbar_route_path_i[out_idx].valid &&
          (xbar_route_path_i[out_idx].route_ptr < xbar_route_path_i[out_idx].route_len)) begin
        raw_meta_o[out_idx].route_ptr = xbar_meta_i[out_idx].route_ptr + 8'd1;
        raw_route_path_o[out_idx].route_ptr = xbar_route_path_i[out_idx].route_ptr + 8'd1;
      end
    end
  end

  always_comb begin : proc_routport_output
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      routport_data_o[out_idx].flit = raw_flit_o[out_idx];
      routport_data_o[out_idx].meta = raw_meta_o[out_idx];
      routport_data_o[out_idx].route_path = raw_route_path_o[out_idx];
      routport_data_o[out_idx].vc_id = raw_vc_id_o[out_idx];
      routport_data_o[out_idx].write_req = raw_write_o[out_idx];
    end

    if (mfu_capture_i && (mfu_capture_out_sel_i < NUM_PORTS)) begin
      routport_data_o[mfu_capture_out_sel_i].flit = '0;
      routport_data_o[mfu_capture_out_sel_i].meta = '0;
      routport_data_o[mfu_capture_out_sel_i].route_path = '0;
      routport_data_o[mfu_capture_out_sel_i].write_req = 1'b0;
    end

    if (mfu_emit_valid_i && (mfu_emit_out_sel_i < NUM_PORTS)) begin
      routport_data_o[mfu_emit_out_sel_i].flit = mfu_emit_flit_i;
      routport_data_o[mfu_emit_out_sel_i].meta = mfu_emit_meta_i;
      routport_data_o[mfu_emit_out_sel_i].route_path = mfu_emit_route_path_i;
      routport_data_o[mfu_emit_out_sel_i].vc_id = mfu_emit_vc_id_i;
      routport_data_o[mfu_emit_out_sel_i].write_req = 1'b1;
    end
  end

endmodule
