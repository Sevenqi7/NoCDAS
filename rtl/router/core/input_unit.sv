// Description: Router input port with one FIFO per VC.
//              The unit accepts flits from the environment, selects one
//              non-empty VC in round-robin order, computes the requested output
//              port, and pops the selected VC when the switch allocator grants
//              the transfer.

module input_unit #(
    parameter int VC_NUM = 8,
    parameter int VC_ID_W = 3,
    parameter int FLIT_W = 256
) (
    input  logic clk,
    input  logic reset,
    input  router_ports_pkg::rinport_data_t  rinport_data_i,
    input  router_ports_pkg::rinport_ctrl_t  rinport_ctrl_i,
    input  logic [2:0] rinport_x_cur_i,
    input  logic [2:0] rinport_y_cur_i,
    input  logic [2:0] rinport_channel_i,
    output router_ports_pkg::flit_meta_t rinport_raw_meta_o,
    output router_ports_pkg::route_path_t rinport_route_path_o,
    output logic rinport_may_need_mfu_o,
    output router_ports_pkg::router_traffic_class_e rinport_traffic_class_o,
    output router_ports_pkg::rinport_issue_t rinport_issue_o,
    output router_ports_pkg::rinport_flow_t  rinport_flow_o
);
  import router_ports_pkg::*;

  localparam logic [VC_ID_W-1:0] VC_LAST = VC_ID_W'(VC_NUM - 1);

  logic [FLIT_W-1:0] vc_flit [0:VC_NUM-1];
  flit_meta_t vc_meta [0:VC_NUM-1];
  logic [2:0] vc_empty_slots [0:VC_NUM-1];
  route_path_t route_path_q [0:VC_NUM-1];
  logic [VC_NUM-1:0] pop_vec;
  logic [VC_NUM-1:0] push_vec;
  logic [VC_NUM-1:0] push_ack;

  logic [VC_ID_W-1:0] selected_vc_q;
  logic [VC_ID_W-1:0] rr_ptr_q;
  logic [VC_ID_W-1:0] selected_vc_d;
  logic selected_valid_d;
  logic pop_fire_d;
  logic flit_valid;
  logic [FLIT_W-1:0] issue_flit;
  flit_meta_t issue_meta;
  route_path_t issue_route_path;
  logic [2:0] issue_empty_slots;
  logic [2:0] issue_route_sel;
  logic issue_route_valid;
  logic [META_ROUTE_LEN_W-1:0] issue_route_len;
  logic [META_ROUTE_PTR_W-1:0] issue_route_ptr;
  logic [ROUTE_PORT_W-1:0] issue_route_port;
  logic issue_valid;
  logic [VC_ID_W-1:0] issue_sel_vc;
  logic issue_may_need_mfu;
  logic issue_is_type4;
  logic issue_is_type5;
  logic issue_head_like;
  logic issue_tail_like;
  logic input_head_like;
  logic issue_route_process_in_range;
  logic issue_route_process_mask_bit;
  router_traffic_class_e issue_traffic_class;
  router_traffic_class_e issue_msg_traffic_class;
  logic input_accept;
  logic issue_has_flit;
  int issue_route_process_ptr_idx;
  int issue_route_process_len_idx;
  int issue_route_seq_base;
  int rr_ptr_int;

  integer search_idx;
  integer vc_idx;
  integer route_vc_idx;


  genvar vc_gen_idx;

  generate
    for (vc_gen_idx = 0; vc_gen_idx < VC_NUM; vc_gen_idx = vc_gen_idx + 1) begin : vc_buffer_gen
      Buffer #(
          .FLIT_W(FLIT_W)
      ) vc_buffer_i (
          .bf_out(vc_flit[vc_gen_idx]),
          .bf_meta_out(vc_meta[vc_gen_idx]),
          .em_pl(vc_empty_slots[vc_gen_idx]),
          .clk(clk),
          .reset(reset),
          .pop(pop_vec[vc_gen_idx]),
          .push(push_vec[vc_gen_idx]),
          .bf_in(rinport_data_i.flit),
          .bf_meta_in(rinport_data_i.meta)
      );
    end
  endgenerate

  integer sel_iter;

  // Select one non-empty VC in round-robin order starting from the last granted VC.
  always_comb begin
    selected_valid_d = 1'b0;
    selected_vc_d = rr_ptr_q;
    rr_ptr_int = int'(rr_ptr_q);
    for (sel_iter = 0; sel_iter < VC_NUM; sel_iter = sel_iter + 1) begin
      search_idx = rr_ptr_int + sel_iter;
      // Wrap the VC scan inside this input port.  The selected VC is independent
      // of other input ports and therefore preserves per-port wormhole ordering.
      if (search_idx >= VC_NUM)
        search_idx = search_idx - VC_NUM;
      // Buffer empty_slots < depth means there is at least one flit buffered.
      // Keep the first non-empty VC found from the current RR pointer.
      if (!selected_valid_d && vc_empty_slots[search_idx] < 3'd4) begin
        selected_valid_d = 1'b1;
        selected_vc_d = search_idx[VC_ID_W-1:0];
      end
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      selected_vc_q <= '0;
      rr_ptr_q <= '0;
      for (route_vc_idx = 0; route_vc_idx < VC_NUM; route_vc_idx = route_vc_idx + 1) begin
        route_path_q[route_vc_idx] <= '0;
      end
    end else begin
      if (selected_valid_d)
        selected_vc_q <= selected_vc_d;

      // A granted pop consumes the selected flit, so the next arbitration starts
      // from the following VC.
      if (pop_fire_d)
        rr_ptr_q <= (selected_vc_q == VC_LAST) ? '0 : (selected_vc_q + {{(VC_ID_W-1){1'b0}}, 1'b1});
      // Even when a flit is merely presented but not granted, advance the scan to
      // avoid parking the input scheduler on one blocked VC forever.
      else if (issue_valid)
        rr_ptr_q <= (selected_vc_q == VC_LAST) ? '0 : (selected_vc_q + {{(VC_ID_W-1){1'b0}}, 1'b1});

      // Source-route context is associated with the VC that accepted the head flit.
      // The route path will be stored in the per-VC registers until the tail flit pops and clears it.
      if (pop_fire_d) begin
        if (issue_tail_like) begin
          route_path_q[selected_vc_q] <= '0;
        end
      end

      // A new head/head-tail loads source-route context into the exact VC that
      // accepted the flit.  Body/tail flits later reuse this stored context.
      if (input_accept && input_head_like) begin
        route_path_q[rinport_data_i.vc_id] <= rinport_data_i.route_path;
      end
    end
  end


  always_comb begin : proc_issue_semantics
    // Type4 distribution and type5 compute may require MFU capture.  The Router
    // top level decides whether this specific router should process type4.  For
    // type5, source-route process masks already tell this input VC whether the
    // current hop should enter the MFU.
    issue_is_type4 = (issue_meta.msg_type == ROUTER_MSG_DIST);
    issue_is_type5 = (issue_meta.msg_type == ROUTER_MSG_COMP);
    issue_head_like = flit_is_head_like(issue_meta.flit_kind);
    issue_tail_like = flit_is_tail_like(issue_meta.flit_kind);
    input_head_like = flit_is_head_like(rinport_data_i.meta.flit_kind);

    unique case (issue_meta.msg_type)
      ROUTER_MSG_COMP: issue_msg_traffic_class = ROUTER_TRAFFIC_COMP;
      ROUTER_MSG_DIST: issue_msg_traffic_class = ROUTER_TRAFFIC_DIST;
      default:         issue_msg_traffic_class = ROUTER_TRAFFIC_REGULAR;
    endcase

    // Prefer the explicit header traffic class when it is in range.  Fallback
    // to message type so a corrupted header cannot create an undefined class.
    if (issue_meta.traffic_class <= ROUTER_TRAFFIC_COMP) begin
      issue_traffic_class = issue_meta.traffic_class;
    end else begin
      issue_traffic_class = issue_msg_traffic_class;
    end

    issue_may_need_mfu = (issue_is_type5 && issue_meta.process) || issue_is_type4;
  end

  assign flit_valid = issue_meta.valid;
  assign issue_has_flit = !reset && flit_valid && (issue_empty_slots < 3'd4);


  // Issue selected VC's flit and meta to route computation and switch allocation.
  always_comb begin
    issue_flit = vc_flit[selected_vc_q];
    issue_meta = vc_meta[selected_vc_q];
    issue_route_path = route_path_q[selected_vc_q];
    issue_empty_slots = vc_empty_slots[selected_vc_q];
    issue_sel_vc = selected_vc_q;
    issue_route_valid = issue_route_path.valid;
    issue_route_len = issue_route_path.route_len;
    issue_route_ptr = issue_route_path.route_ptr;
    issue_route_process_ptr_idx = int'(issue_route_ptr);
    issue_route_process_len_idx = int'(issue_route_len);
    issue_route_seq_base = issue_route_process_ptr_idx * ROUTE_PORT_W;
    issue_route_process_in_range =
        (issue_route_process_ptr_idx < issue_route_process_len_idx) &&
        (issue_route_process_ptr_idx < ROUTE_MAX_HOPS);
    issue_route_process_mask_bit = 1'b0;
    issue_route_port = ROUTER_PORT_INV[ROUTE_PORT_W-1:0];
    if (issue_route_process_in_range) begin
      issue_route_process_mask_bit =
          issue_route_path.route_process_seq[issue_route_process_ptr_idx];
      issue_route_port =
          issue_route_path.route_seq[issue_route_seq_base +: ROUTE_PORT_W];
    end

    // Body/tail flits do not repeat the full source-route header.  Rebuild the
    // semantic meta view from the per-VC route context before the router top
    // level performs VC allocation, switch arbitration, and MFU gating.
    issue_meta.route_valid = issue_route_valid;
    issue_meta.route_ptr = issue_route_ptr;
    if (issue_meta.msg_type == ROUTER_MSG_COMP) begin
      issue_meta.process = issue_route_process_mask_bit;
    end

    // Wait allocator's grant before popping the VC buffer.
    issue_valid = 1'b0;
    pop_fire_d = 1'b0;
    if (issue_has_flit) begin
      issue_valid = 1'b1;
      pop_fire_d = rinport_ctrl_i.vc_grant;
    end

    pop_vec = '0;
    if (pop_fire_d)
      pop_vec[selected_vc_q] = 1'b1;
  end


  // VC enqueue logic
  always_comb begin
    push_vec = '0;
    push_ack = '0;
    input_accept = 1'b0;
    // During reset all backpressure signals are deasserted so the environment
    // cannot inject into a clearing FIFO.
    if (!reset) begin
      for (vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        push_ack[vc_idx] = (vc_empty_slots[vc_idx] > 3'd0);
      end
      if (rinport_data_i.push && push_ack[rinport_data_i.vc_id]) begin
        push_vec[rinport_data_i.vc_id] = 1'b1;
      end
      input_accept = rinport_data_i.push && push_ack[rinport_data_i.vc_id];
    end
  end

  route_compute_unit route_compute_i (
      .reset_i(reset),
      .x_cur_i(rinport_x_cur_i),
      .y_cur_i(rinport_y_cur_i),
      .in_channel_i(rinport_channel_i),
      .meta_i(issue_meta),
      .route_valid_i(issue_route_valid),
      .route_len_i(issue_route_len),
      .route_ptr_i(issue_route_ptr),
      .route_port_i(issue_route_port),
      .flit_valid_i(issue_has_flit),
      .route_sel_o(issue_route_sel)
  );

  // Output assignments
  assign rinport_issue_o.flit = issue_flit;
  assign rinport_issue_o.meta = issue_meta;
  assign rinport_issue_o.valid = issue_valid;
  assign rinport_issue_o.route_sel = issue_route_sel;
  // This issue bundle is consumed only by Router's route/VC/switch datapath.
  // FIFO pop remains controlled internally by pop_fire_d; exporting it would
  // create a grant -> issue -> allocator false combinational loop.
  assign rinport_issue_o.pop_fire = 1'b0;
  assign rinport_issue_o.sel_vc = issue_sel_vc;
  assign rinport_raw_meta_o = issue_meta;
  assign rinport_route_path_o = issue_route_path;
  assign rinport_may_need_mfu_o = issue_may_need_mfu;
  assign rinport_traffic_class_o = issue_traffic_class;
  assign rinport_flow_o.push_ack = push_ack;
  assign rinport_flow_o.in_accept = input_accept;


`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (!reset) begin
      // A switch grant is allowed to remove data only when the selected VC is
      // actually presenting a valid flit.  This catches flit-drop bugs where
      // arbitration and FIFO state drift apart.
      if (pop_fire_d && !issue_has_flit) begin
        $error("input_unit: attempted to pop an empty/invalid selected VC");
      end

      // The environment may only inject into a physically implemented VC.  The
      // width normally prevents this for VC_NUM=8, but the guard keeps parameter
      // changes from silently indexing outside the FIFO array.
      if (rinport_data_i.push && (int'(rinport_data_i.vc_id) >= VC_NUM)) begin
        $error("input_unit: push VC id is outside the implemented VC range");
      end

      // Body/tail source-routed flits must never invent route context.  If a
      // non-head flit carries route_valid in its meta but the input VC has no
      // stored context, route_compute_unit will intentionally fall back to XY.
      if (issue_has_flit &&
          !issue_head_like &&
          issue_meta.route_valid &&
          !route_path_q[selected_vc_q].valid) begin
        $error("input_unit: non-head flit exposed source-route meta without VC context");
      end
    end
  end
`endif

endmodule
