// Description: Optional MatMul/Linear tracepoints for router-level debugging.
//              This module is instantiated only when RTL_DEBUG_MATMUL is set so
//              the default Router datapath stays free of verbose debug prints.

module router_matmul_debug #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W
) (
    input logic clk_i,
    input logic reset_i,
    input logic [2:0] router_x_i,
    input logic [2:0] router_y_i,

    input logic [NUM_PORTS-1:0] rinport_req_i,
    input logic [NUM_PORTS-1:0] rinport_grant_i,
    input router_ports_pkg::rinport_issue_t rin_issue_i [NUM_PORTS],
    input router_ports_pkg::flit_meta_t vc_alloc_meta_i [NUM_PORTS],
    input logic [NUM_PORTS-1:0] mfu_rinport_req_raw_i,
    input logic [NUM_PORTS-1:0] mfu_rinport_req_i,

    input router_ports_pkg::flit_meta_t raw_meta_i [NUM_PORTS],
    input logic [NUM_PORTS-1:0] raw_write_i,
    input logic [NUM_PORTS-1:0] raw_needs_mfu_i,

    input router_ports_pkg::flit_meta_t active_flit_meta_i,
    input logic [VC_ID_W-1:0] active_vc_id_i,
    input logic mfu_capture_i,
    input logic [2:0] mfu_capture_out_sel_i,
    input logic mfu_emit_valid_i,
    input logic [2:0] mfu_emit_out_sel_i,
    input router_ports_pkg::flit_meta_t mfu_emit_meta_i,
    input logic [VC_ID_W-1:0] mfu_emit_vc_id_i
);
  import router_ports_pkg::*;

  logic [META_PACKET_UID_W-1:0] issue_packet_uid [NUM_PORTS];
  logic [META_FLIT_ID_W-1:0] issue_flit_id [NUM_PORTS];
  logic [META_PACKET_UID_W-1:0] raw_packet_uid [NUM_PORTS];
  logic [META_FLIT_ID_W-1:0] raw_flit_id [NUM_PORTS];
  logic [META_PACKET_UID_W-1:0] active_packet_uid;
  logic [META_FLIT_ID_W-1:0] active_flit_id;
  logic [META_PACKET_UID_W-1:0] emit_packet_uid;
  logic [META_FLIT_ID_W-1:0] emit_flit_id;

  always_comb begin : proc_debug_ids
    for (int dbg_id_idx = 0; dbg_id_idx < NUM_PORTS; dbg_id_idx = dbg_id_idx + 1) begin
`ifdef ROUTER_ENABLE_COSIM
      issue_packet_uid[dbg_id_idx] = rin_issue_i[dbg_id_idx].meta.cosim.packet_uid;
      issue_flit_id[dbg_id_idx] = rin_issue_i[dbg_id_idx].meta.cosim.flit_id;
      raw_packet_uid[dbg_id_idx] = raw_meta_i[dbg_id_idx].cosim.packet_uid;
      raw_flit_id[dbg_id_idx] = raw_meta_i[dbg_id_idx].cosim.flit_id;
`else
      issue_packet_uid[dbg_id_idx] = '0;
      issue_flit_id[dbg_id_idx] = '0;
      raw_packet_uid[dbg_id_idx] = '0;
      raw_flit_id[dbg_id_idx] = '0;
`endif
    end
`ifdef ROUTER_ENABLE_COSIM
    active_packet_uid = active_flit_meta_i.cosim.packet_uid;
    active_flit_id = active_flit_meta_i.cosim.flit_id;
    emit_packet_uid = mfu_emit_meta_i.cosim.packet_uid;
    emit_flit_id = mfu_emit_meta_i.cosim.flit_id;
`else
    active_packet_uid = '0;
    active_flit_id = '0;
    emit_packet_uid = '0;
    emit_flit_id = '0;
`endif
  end

  always_ff @(posedge clk_i) begin
    if (!reset_i) begin
      for (int dbg_port_idx = 0; dbg_port_idx < NUM_PORTS; dbg_port_idx = dbg_port_idx + 1) begin
        if (rinport_req_i[dbg_port_idx] &&
            (rin_issue_i[dbg_port_idx].meta.msg_type == ROUTER_MSG_COMP) &&
            ((rin_issue_i[dbg_port_idx].meta.opcode == 5'd15) ||
             (rin_issue_i[dbg_port_idx].meta.opcode == 5'd0))) begin
          $display("[RTL_DEBUG_MATMUL][router_issue] router_x=%0d router_y=%0d in=%0d vc=%0d valid=%0d grant=%0d route=%0d packet_uid=%0d flit_id=%0d kind=%0d data_offset=%0d payload_len=%0d psum_offset=%0d route_ptr=%0d process_issue=%0d process_vc_meta=%0d mfu_req_raw=%0d mfu_req=%0d",
                   router_x_i,
                   router_y_i,
                   dbg_port_idx,
                   rin_issue_i[dbg_port_idx].sel_vc,
                   rin_issue_i[dbg_port_idx].valid,
                   rinport_grant_i[dbg_port_idx],
                   rin_issue_i[dbg_port_idx].route_sel,
                   issue_packet_uid[dbg_port_idx],
                   issue_flit_id[dbg_port_idx],
                   rin_issue_i[dbg_port_idx].meta.flit_kind,
                   rin_issue_i[dbg_port_idx].meta.data_offset,
                   rin_issue_i[dbg_port_idx].meta.payload_len,
                   rin_issue_i[dbg_port_idx].meta.psum_offset,
                   rin_issue_i[dbg_port_idx].meta.route_ptr,
                   rin_issue_i[dbg_port_idx].meta.process,
                   vc_alloc_meta_i[dbg_port_idx].process,
                   mfu_rinport_req_raw_i[dbg_port_idx],
                   mfu_rinport_req_i[dbg_port_idx]);
        end
      end

      for (int dbg_out_idx = 0; dbg_out_idx < NUM_PORTS; dbg_out_idx = dbg_out_idx + 1) begin
        if (raw_write_i[dbg_out_idx] &&
            (raw_meta_i[dbg_out_idx].msg_type == ROUTER_MSG_COMP) &&
            ((raw_meta_i[dbg_out_idx].opcode == 5'd15) ||
             (raw_meta_i[dbg_out_idx].opcode == 5'd0))) begin
          $display("[RTL_DEBUG_MATMUL][router_raw] router_x=%0d router_y=%0d out=%0d raw_needs_mfu=%0d packet_uid=%0d flit_id=%0d kind=%0d data_offset=%0d payload_len=%0d psum_offset=%0d process=%0d route_ptr=%0d",
                   router_x_i,
                   router_y_i,
                   dbg_out_idx,
                   raw_needs_mfu_i[dbg_out_idx],
                   raw_packet_uid[dbg_out_idx],
                   raw_flit_id[dbg_out_idx],
                   raw_meta_i[dbg_out_idx].flit_kind,
                   raw_meta_i[dbg_out_idx].data_offset,
                   raw_meta_i[dbg_out_idx].payload_len,
                   raw_meta_i[dbg_out_idx].psum_offset,
                   raw_meta_i[dbg_out_idx].process,
                   raw_meta_i[dbg_out_idx].route_ptr);
        end
      end

      if (mfu_capture_i &&
          (active_flit_meta_i.msg_type == ROUTER_MSG_COMP) &&
          ((active_flit_meta_i.opcode == 5'd15) ||
           (active_flit_meta_i.opcode == 5'd0))) begin
        $display("[RTL_DEBUG_MATMUL][router_capture] router_x=%0d router_y=%0d out=%0d vc=%0d packet_uid=%0d flit_id=%0d kind=%0d data_offset=%0d payload_len=%0d psum_offset=%0d process=%0d",
                 router_x_i,
                 router_y_i,
                 mfu_capture_out_sel_i,
                 active_vc_id_i,
                 active_packet_uid,
                 active_flit_id,
                 active_flit_meta_i.flit_kind,
                 active_flit_meta_i.data_offset,
                 active_flit_meta_i.payload_len,
                 active_flit_meta_i.psum_offset,
                 active_flit_meta_i.process);
      end

      if (mfu_emit_valid_i &&
          (mfu_emit_meta_i.msg_type == ROUTER_MSG_COMP) &&
          ((mfu_emit_meta_i.opcode == 5'd15) ||
           (mfu_emit_meta_i.opcode == 5'd0))) begin
        $display("[RTL_DEBUG_MATMUL][router_emit] router_x=%0d router_y=%0d out=%0d vc=%0d packet_uid=%0d flit_id=%0d kind=%0d data_offset=%0d payload_len=%0d psum_offset=%0d process=%0d",
                 router_x_i,
                 router_y_i,
                 mfu_emit_out_sel_i,
                 mfu_emit_vc_id_i,
                 emit_packet_uid,
                 emit_flit_id,
                 mfu_emit_meta_i.flit_kind,
                 mfu_emit_meta_i.data_offset,
                 mfu_emit_meta_i.payload_len,
                 mfu_emit_meta_i.psum_offset,
                 mfu_emit_meta_i.process);
      end
    end
  end

endmodule
