// Description: Router output commit stage.
//              This stage checks downstream flow control, creates input commit
//              events, emits raw MFU candidates, advances source-route pointers,
//              and applies MFU writeback override at the output boundary.

module router_output_stage #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic [2:0] x_cur_i,
    input  logic [2:0] y_cur_i,
    input  router_ports_pkg::router_pipe_entry_t out_entry_i [NUM_PORTS],
    input  router_ports_pkg::routport_flow_t routport_flow_i [NUM_PORTS],
    input  router_ports_pkg::attention_ctx_status_t attention_ctx_status_i,
    input  router_ports_pkg::matmul_ctx_status_t matmul_ctx_status_i,

    input  logic [NUM_PORTS-1:0] mfu_holds_routport_i,
    input  logic mfu_ingress_ready_i,
    input  logic mfu_emit_valid_i,
    input  logic [2:0] mfu_emit_out_sel_i,
    input  logic [FLIT_W-1:0] mfu_emit_flit_i,
    input  router_ports_pkg::flit_meta_t mfu_emit_meta_i,
    input  router_ports_pkg::route_path_t mfu_emit_route_path_i,
    input  logic [VC_ID_W-1:0] mfu_emit_vc_id_i,

    output logic [NUM_PORTS-1:0] output_commit_fire_o,
    output logic [NUM_PORTS-1:0] output_mfu_selected_o,
    output logic [NUM_PORTS-1:0] output_release_valid_o,
    output logic [VC_ID_W-1:0] output_release_vc_o [NUM_PORTS],
    output router_ports_pkg::router_pipe_entry_t out_entry_o [NUM_PORTS],
    output router_ports_pkg::rinport_ctrl_t rinport_ctrl_o [NUM_PORTS],

    output logic [FLIT_W-1:0] raw_flit_o [NUM_PORTS],
    output router_ports_pkg::flit_meta_t raw_meta_o [NUM_PORTS],
    output router_ports_pkg::route_path_t raw_route_path_o [NUM_PORTS],
    output logic [VC_ID_W-1:0] raw_vc_id_o [NUM_PORTS],
    output logic [2:0] raw_stream_port_o [NUM_PORTS],
    output logic [VC_ID_W-1:0] raw_stream_vc_o [NUM_PORTS],
    output logic [NUM_PORTS-1:0] raw_write_o,
    output router_ports_pkg::routport_data_t routport_data_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;
  logic [NUM_PORTS-1:0] output_regular_commit;
  logic [NUM_PORTS-1:0] output_ready_for_entry;
  logic [NUM_PORTS-1:0] output_entry_needs_mfu;
  logic [NUM_PORTS-1:0] output_ctx_ok;
  logic [NUM_PORTS-1:0] source_commit_used;
  flit_meta_t output_meta_next [NUM_PORTS];
  route_path_t output_route_path_next [NUM_PORTS];

  always_comb begin : proc_output_commit
    source_commit_used = '0;
    output_commit_fire_o = '0;
    output_mfu_selected_o = '0;
    output_release_valid_o = '0;
    output_regular_commit = '0;
    output_ready_for_entry = '0;
    output_entry_needs_mfu = '0;
    output_ctx_ok = '1;
    raw_write_o = '0;

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      routport_data_o[out_idx] = '0;
      raw_flit_o[out_idx] = '0;
      raw_meta_o[out_idx] = '0;
      raw_route_path_o[out_idx] = '0;
      raw_vc_id_o[out_idx] = '0;
      output_release_vc_o[out_idx] = '0;
      raw_stream_port_o[out_idx] = ROUTER_PORT_INV;
      raw_stream_vc_o[out_idx] = '0;
      out_entry_o[out_idx] = out_entry_i[out_idx];
      output_meta_next[out_idx] = out_entry_i[out_idx].meta;
      output_route_path_next[out_idx] = out_entry_i[out_idx].route_path;

      rinport_ctrl_o[out_idx] = '0;
      rinport_ctrl_o[out_idx].x_cur = x_cur_i;
      rinport_ctrl_o[out_idx].y_cur = y_cur_i;
      rinport_ctrl_o[out_idx].in_channel = 3'(out_idx);

      if (((out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_DIST) ||
           (out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_COMP)) &&
          out_entry_i[out_idx].valid &&
          out_entry_i[out_idx].meta.route_valid &&
          out_entry_i[out_idx].route_path.valid &&
          (out_entry_i[out_idx].route_path.route_ptr < out_entry_i[out_idx].route_path.route_len)) begin
        output_meta_next[out_idx].route_ptr = out_entry_i[out_idx].meta.route_ptr + 8'd1;
        output_route_path_next[out_idx].route_ptr = out_entry_i[out_idx].route_path.route_ptr + 8'd1;
      end
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      logic output_data_owner_match;
      logic output_attention_owner_match;
      logic output_attention_ctx_ok;
      logic output_data_ctx_ok;

      output_data_owner_match = 1'b0;
      output_attention_owner_match = 1'b0;
      output_attention_ctx_ok = 1'b1;
      output_data_ctx_ok = 1'b1;

      if (out_entry_i[out_idx].valid &&
          routport_flow_i[out_idx].state_ready &&
          routport_flow_i[out_idx].downstream_vc_credit_mask[out_entry_i[out_idx].dst_vc] &&
          !mfu_holds_routport_i[out_idx]) begin
        output_ready_for_entry[out_idx] = 1'b1;
      end

`ifdef ENABLE_CNOC_MFU
      if (out_entry_i[out_idx].valid) begin
        output_entry_needs_mfu[out_idx] =
            ((out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_COMP) &&
             out_entry_i[out_idx].meta.process) ||
            ((out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_DIST) &&
             (out_entry_i[out_idx].meta.process ||
              ((out_entry_i[out_idx].meta.dst_x == x_cur_i) &&
               (out_entry_i[out_idx].meta.dst_y == y_cur_i))));

        output_ctx_ok[out_idx] = 1'b1;
        for (int slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
          if (!output_data_owner_match &&
              matmul_ctx_status_i.slot_valid[slot_idx] &&
              (matmul_ctx_status_i.slot_stream_port_flat[slot_idx * 3 +: 3] ==
               out_entry_i[out_idx].src_port) &&
            (matmul_ctx_status_i.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] ==
               out_entry_i[out_idx].src_vc)) begin
            output_data_owner_match = 1'b1;
          end
        end

        output_attention_owner_match =
            attention_ctx_status_i.valid &&
            (attention_ctx_status_i.stream_port == out_entry_i[out_idx].src_port) &&
            (attention_ctx_status_i.stream_vc == out_entry_i[out_idx].src_vc);

        if ((out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_COMP) &&
            out_entry_i[out_idx].meta.process &&
            (out_entry_i[out_idx].meta.opcode == OP_ATTENTION)) begin
          if (out_entry_i[out_idx].head_like) begin
            output_attention_ctx_ok =
                !attention_ctx_status_i.valid || output_attention_owner_match;
          end else begin
            output_attention_ctx_ok =
                attention_ctx_status_i.valid && output_attention_owner_match;
          end
          output_ctx_ok[out_idx] = output_ctx_ok[out_idx] && output_attention_ctx_ok;
        end
        if ((out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_COMP) &&
            out_entry_i[out_idx].meta.process &&
            ((out_entry_i[out_idx].meta.opcode == OP_LINEAR) ||
             (out_entry_i[out_idx].meta.opcode == OP_MATMUL))) begin
          if (out_entry_i[out_idx].head_like) begin
            output_data_ctx_ok =
                output_data_owner_match || matmul_ctx_status_i.has_free_slot;
          end else begin
            output_data_ctx_ok = output_data_owner_match;
          end
          output_ctx_ok[out_idx] = output_ctx_ok[out_idx] && output_data_ctx_ok;
        end
      end
`else
      output_entry_needs_mfu[out_idx] = 1'b0;
`endif
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_ready_for_entry[out_idx] &&
          !source_commit_used[out_entry_i[out_idx].src_port]) begin
`ifdef ENABLE_CNOC_MFU
        if (output_entry_needs_mfu[out_idx] && mfu_ingress_ready_i && output_ctx_ok[out_idx] &&
            (output_mfu_selected_o == '0)) begin
          output_mfu_selected_o[out_idx] = 1'b1;
          output_commit_fire_o[out_idx] = 1'b1;
          source_commit_used[out_entry_i[out_idx].src_port] = 1'b1;
          raw_write_o[out_idx] = 1'b1;
        end else if (!output_entry_needs_mfu[out_idx] &&
                     !(mfu_emit_valid_i && (mfu_emit_out_sel_i == 3'(out_idx)))) begin
          output_regular_commit[out_idx] = 1'b1;
          output_commit_fire_o[out_idx] = 1'b1;
          source_commit_used[out_entry_i[out_idx].src_port] = 1'b1;
          raw_write_o[out_idx] = 1'b1;
        end
`else
        output_regular_commit[out_idx] = 1'b1;
        output_commit_fire_o[out_idx] = 1'b1;
        source_commit_used[out_entry_i[out_idx].src_port] = 1'b1;
        raw_write_o[out_idx] = 1'b1;
`endif
      end
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (raw_write_o[out_idx]) begin
        raw_flit_o[out_idx] = out_entry_i[out_idx].flit;
        raw_meta_o[out_idx] = output_meta_next[out_idx];
        raw_route_path_o[out_idx] = output_route_path_next[out_idx];
        raw_vc_id_o[out_idx] = out_entry_i[out_idx].dst_vc;
        raw_stream_port_o[out_idx] = out_entry_i[out_idx].src_port;
        raw_stream_vc_o[out_idx] = out_entry_i[out_idx].src_vc;
      end

      if (output_commit_fire_o[out_idx]) begin
        out_entry_o[out_idx] = '0;
        rinport_ctrl_o[out_entry_i[out_idx].src_port].commit_valid = 1'b1;
        rinport_ctrl_o[out_entry_i[out_idx].src_port].commit_vc = out_entry_i[out_idx].src_vc;
        rinport_ctrl_o[out_entry_i[out_idx].src_port].commit_tail_like =
            out_entry_i[out_idx].tail_like;
        if (out_entry_i[out_idx].reserved_vc && !output_mfu_selected_o[out_idx]) begin
          output_release_valid_o[out_idx] = 1'b1;
          output_release_vc_o[out_idx] = out_entry_i[out_idx].dst_vc;
        end
      end

      if (output_regular_commit[out_idx]) begin
        routport_data_o[out_idx].flit = out_entry_i[out_idx].flit;
        routport_data_o[out_idx].meta = output_meta_next[out_idx];
        routport_data_o[out_idx].route_path = output_route_path_next[out_idx];
        routport_data_o[out_idx].vc_id = out_entry_i[out_idx].dst_vc;
        routport_data_o[out_idx].write_req = 1'b1;
      end
    end

`ifdef ENABLE_CNOC_MFU
    if (mfu_emit_valid_i && (mfu_emit_out_sel_i < NUM_PORTS)) begin
      routport_data_o[mfu_emit_out_sel_i].flit = mfu_emit_flit_i;
      routport_data_o[mfu_emit_out_sel_i].meta = mfu_emit_meta_i;
      routport_data_o[mfu_emit_out_sel_i].route_path = mfu_emit_route_path_i;
      routport_data_o[mfu_emit_out_sel_i].vc_id = mfu_emit_vc_id_i;
      routport_data_o[mfu_emit_out_sel_i].write_req = 1'b1;
    end
`endif
  end

`ifndef SYNTHESIS
  always_comb begin : proc_output_assertions
    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_commit_fire_o[out_idx] && !out_entry_i[out_idx].valid) begin
        $error("router_output_stage: committed an empty output pipeline slot");
      end

`ifdef ENABLE_CNOC_MFU
      if (output_commit_fire_o[out_idx] &&
          output_mfu_selected_o[out_idx] &&
          (out_entry_i[out_idx].meta.msg_type == ROUTER_MSG_COMP) &&
          out_entry_i[out_idx].meta.process &&
          ((out_entry_i[out_idx].meta.opcode == OP_ATTENTION) ||
           (out_entry_i[out_idx].meta.opcode == OP_LINEAR) ||
           (out_entry_i[out_idx].meta.opcode == OP_MATMUL)) &&
          !output_ctx_ok[out_idx]) begin
        $error("router_output_stage: committed an owner-gated MFU entry without eligibility");
      end
`endif
    end
  end
`endif

endmodule
