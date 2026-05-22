// Description: Router input port with one FIFO per VC.
//              The unit accepts flits from the environment, selects one
//              non-empty VC in round-robin order, computes the requested output
//              port, and stores accepted candidates in a local two-entry issue
//              queue.  This issue queue is selectable rather than strict FIFO:
//              the router may skip a locally blocked Attention entry so an
//              owner-matching sibling slot can still issue.  The input FIFO
//              itself still pops only after output commit, preserving wormhole
//              backpressure correctness.

module input_unit #(
    parameter int VC_NUM = 8,
    parameter int VC_ID_W = 3,
    parameter int FLIT_W = 256
) (
    input  logic clk,
    input  logic reset,
    input  router_ports_pkg::rinport_data_t  rinport_data_i,
    input  router_ports_pkg::rinport_ctrl_t  rinport_ctrl_i,
    input  router_ports_pkg::attention_ctx_status_t attention_ctx_status_i,
    input  router_ports_pkg::matmul_ctx_status_t matmul_ctx_status_i,
    input  logic [2:0] rinport_x_cur_i,
    input  logic [2:0] rinport_y_cur_i,
    input  logic [2:0] rinport_channel_i,
    input  logic issue_pop_i,
    output router_ports_pkg::flit_meta_t rinport_raw_meta_o,
    output router_ports_pkg::route_path_t rinport_route_path_o,
    output logic rinport_may_need_mfu_o,
    output router_ports_pkg::router_traffic_class_e rinport_traffic_class_o,
    output router_ports_pkg::rinport_issue_t rinport_issue_o,
    output router_ports_pkg::router_pipe_entry_t issue_entry_o,
    output router_ports_pkg::rinport_flow_t  rinport_flow_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;
  localparam logic [VC_ID_W-1:0] VC_LAST = VC_ID_W'(VC_NUM - 1);
  localparam int ISSUE_QUEUE_DEPTH = 2;

  logic [FLIT_W-1:0] vc_flit [0:VC_NUM-1];
  flit_meta_t vc_meta [0:VC_NUM-1];
  logic [2:0] vc_empty_slots [0:VC_NUM-1];
  route_path_t route_path_q [0:VC_NUM-1];
  logic [VC_NUM-1:0] inflight_q;
  logic [VC_NUM-1:0] inflight_d;
  logic [VC_NUM-1:0] pop_vec;
  logic [VC_NUM-1:0] push_vec;
  logic [VC_NUM-1:0] push_ack;

  logic [VC_ID_W-1:0] selected_vc_q;
  logic [VC_ID_W-1:0] rr_ptr_q;
  logic [VC_ID_W-1:0] selected_vc_d;
  logic selected_valid_d;
  logic issue_accept_fire;
  logic commit_fire;
  logic issue_queue_push;
  logic issue_queue_pop;
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
  router_pipe_entry_t issue_queue_q [ISSUE_QUEUE_DEPTH];
  router_pipe_entry_t issue_queue_d [ISSUE_QUEUE_DEPTH];
  router_pipe_entry_t issue_push_entry;
  logic issue_queue_has_free_slot;
  logic issue_select_valid;
  logic issue_select_slot;
  logic [ISSUE_QUEUE_DEPTH-1:0] issue_slot_ctx_ok;
  int unsigned issue_route_process_ptr_idx;
  int unsigned issue_route_process_len_idx;
  int unsigned issue_route_seq_base;
  int unsigned rr_ptr_int;

  int unsigned search_idx;
  int unsigned vc_idx;
  int unsigned route_vc_idx;
  int unsigned issue_queue_slot_idx;

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

  int unsigned sel_iter;

  // Select one non-empty VC in round-robin order starting from the last granted VC.
  always_comb begin : proc_selected_vc
    logic candidate_head_like;
    logic candidate_attention_entry;
    logic candidate_data_entry;
    logic candidate_process_bit;
    logic candidate_attention_ctx_ok;
    logic candidate_attention_owner_match;
    logic candidate_attention_head_claim_conflict;
    logic candidate_data_ctx_ok;
    logic candidate_data_owner_match;
    int candidate_route_ptr_idx;
    int candidate_route_len_idx;

    selected_valid_d = 1'b0;
    selected_vc_d = rr_ptr_q;
    rr_ptr_int = int'(rr_ptr_q);
    for (sel_iter = 0; sel_iter < VC_NUM; sel_iter = sel_iter + 1) begin
      search_idx = rr_ptr_int + sel_iter;
      // Wrap the VC scan inside this input port.  The selected VC is independent
      // of other input ports and therefore preserves per-port wormhole ordering.
      if (search_idx >= VC_NUM)
        search_idx = search_idx - VC_NUM;
      candidate_head_like = flit_is_head_like(vc_meta[search_idx].flit_kind);
      candidate_route_ptr_idx = int'(route_path_q[search_idx].route_ptr);
      candidate_route_len_idx = int'(route_path_q[search_idx].route_len);
      candidate_process_bit = 1'b0;
      if ((candidate_route_ptr_idx < candidate_route_len_idx) &&
          (candidate_route_ptr_idx < ROUTE_MAX_HOPS)) begin
        candidate_process_bit =
            route_path_q[search_idx].route_process_seq[candidate_route_ptr_idx];
      end
`ifdef ENABLE_CNOC_MFU
      candidate_attention_entry =
          (vc_meta[search_idx].msg_type == ROUTER_MSG_COMP) &&
          (vc_meta[search_idx].opcode == OP_ATTENTION) &&
          candidate_process_bit;
      candidate_attention_owner_match =
          attention_ctx_status_i.valid &&
          (attention_ctx_status_i.stream_port == rinport_channel_i) &&
          (attention_ctx_status_i.stream_vc == VC_ID_W'(search_idx));
      candidate_attention_head_claim_conflict = 1'b0;
      if (candidate_attention_entry &&
          candidate_head_like &&
          !candidate_attention_owner_match) begin
        for (int unsigned slot_idx = 0; slot_idx < ISSUE_QUEUE_DEPTH; slot_idx = slot_idx + 1) begin
          if (!candidate_attention_head_claim_conflict &&
              issue_queue_q[slot_idx].valid &&
              issue_queue_q[slot_idx].may_need_mfu &&
              (issue_queue_q[slot_idx].meta.msg_type == ROUTER_MSG_COMP) &&
              (issue_queue_q[slot_idx].meta.opcode == OP_ATTENTION) &&
              issue_queue_q[slot_idx].head_like) begin
            candidate_attention_head_claim_conflict = 1'b1;
          end
        end
      end
      candidate_data_owner_match = 1'b0;
      for (int slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
        if (!candidate_data_owner_match &&
            matmul_ctx_status_i.slot_valid[slot_idx] &&
            (matmul_ctx_status_i.slot_stream_port_flat[slot_idx * 3 +: 3] == rinport_channel_i) &&
            (matmul_ctx_status_i.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] ==
             VC_ID_W'(search_idx))) begin
          candidate_data_owner_match = 1'b1;
        end
      end
      candidate_attention_ctx_ok = 1'b1;
      candidate_data_ctx_ok = 1'b1;
      if (candidate_attention_entry) begin
        if (candidate_head_like) begin
          candidate_attention_ctx_ok =
              (!attention_ctx_status_i.valid || candidate_attention_owner_match) &&
              !candidate_attention_head_claim_conflict;
        end else begin
            candidate_attention_ctx_ok =
              attention_ctx_status_i.valid && candidate_attention_owner_match;
        end
      end
      candidate_data_entry =
          (vc_meta[search_idx].msg_type == ROUTER_MSG_COMP) &&
          ((vc_meta[search_idx].opcode == OP_LINEAR) ||
           (vc_meta[search_idx].opcode == OP_MATMUL)) &&
          candidate_process_bit;
      if (candidate_data_entry) begin
        if (candidate_head_like) begin
          candidate_data_ctx_ok =
              candidate_data_owner_match || matmul_ctx_status_i.has_free_slot;
        end else begin
          candidate_data_ctx_ok = candidate_data_owner_match;
        end
      end
`else
      candidate_attention_entry = 1'b0;
      candidate_attention_owner_match = 1'b0;
      candidate_attention_head_claim_conflict = 1'b0;
      candidate_attention_ctx_ok = 1'b1;
      candidate_data_entry = 1'b0;
      candidate_data_owner_match = 1'b0;
      candidate_data_ctx_ok = 1'b1;
`endif

      // Buffer empty_slots < depth means there is at least one flit buffered.
      // Keep the first non-empty, non-inflight, owner-eligible VC found from
      // the current RR pointer.
      if (!selected_valid_d &&
          (vc_empty_slots[search_idx] < 3'd4) &&
          !inflight_q[search_idx] &&
          candidate_attention_ctx_ok &&
          candidate_data_ctx_ok) begin
        selected_valid_d = 1'b1;
        selected_vc_d = VC_ID_W'(search_idx);
      end
    end
  end

  always_ff @(posedge clk) begin : proc_input_registers
    if (reset) begin
      selected_vc_q <= '0;
      rr_ptr_q <= '0;
      inflight_q <= '0;
      issue_queue_q[0] <= '0;
      issue_queue_q[1] <= '0;
      for (route_vc_idx = 0; route_vc_idx < VC_NUM; route_vc_idx = route_vc_idx + 1) begin
        route_path_q[route_vc_idx] <= '0;
      end
    end else begin
      inflight_q <= inflight_d;
      issue_queue_q[0] <= issue_queue_d[0];
      issue_queue_q[1] <= issue_queue_d[1];

      if (selected_valid_d)
        selected_vc_q <= selected_vc_d;

      // A committed pop consumes the flit, so the next arbitration starts from
      // the following VC.
      if (commit_fire)
        rr_ptr_q <= (selected_vc_q == VC_LAST) ? '0 : (selected_vc_q + {{(VC_ID_W-1){1'b0}}, 1'b1});
      // Once a flit enters the router pipeline, advance the scan so this input
      // can expose another non-inflight VC while the first flit waits downstream.
      else if (issue_accept_fire)
        rr_ptr_q <= (selected_vc_q == VC_LAST) ? '0 : (selected_vc_q + {{(VC_ID_W-1){1'b0}}, 1'b1});

      // Source-route context is associated with the VC that accepted the head flit.
      // The route path will be stored in the per-VC registers until the tail flit pops and clears it.
      if (commit_fire) begin
        if (rinport_ctrl_i.commit_tail_like) begin
          route_path_q[rinport_ctrl_i.commit_vc] <= '0;
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
    // Type4/type5 only have MFU semantics when ENABLE_CNOC_MFU is compiled in.
    issue_is_type4 = (issue_meta.msg_type == ROUTER_MSG_DIST);
    issue_is_type5 = (issue_meta.msg_type == ROUTER_MSG_COMP);
    issue_head_like = flit_is_head_like(issue_meta.flit_kind);
    issue_tail_like = flit_is_tail_like(issue_meta.flit_kind);
    input_head_like = flit_is_head_like(rinport_data_i.meta.flit_kind);

`ifdef ENABLE_CNOC_MFU
    case (issue_meta.msg_type)
      ROUTER_MSG_COMP: issue_msg_traffic_class = ROUTER_TRAFFIC_COMP;
      ROUTER_MSG_DIST: issue_msg_traffic_class = ROUTER_TRAFFIC_DIST;
      default:         issue_msg_traffic_class = ROUTER_TRAFFIC_REGULAR;
    endcase
`else
    issue_msg_traffic_class = ROUTER_TRAFFIC_REGULAR;
`endif

    // Prefer the explicit header traffic class when it is in range.  Fallback
    // to message type so a corrupted header cannot create an undefined class.
    if (issue_meta.traffic_class <= ROUTER_TRAFFIC_COMP) begin
      issue_traffic_class = issue_meta.traffic_class;
    end else begin
      issue_traffic_class = issue_msg_traffic_class;
    end

`ifdef ENABLE_CNOC_MFU
    issue_may_need_mfu = (issue_is_type5 && issue_meta.process) || issue_is_type4;
`else
    issue_may_need_mfu = 1'b0;
`endif
  end

  assign flit_valid = issue_meta.valid;
  assign issue_has_flit = !reset &&
                           flit_valid &&
                           (issue_empty_slots < 3'd4) &&
                           !inflight_q[selected_vc_q];


  // Issue selected VC's flit and meta to route computation and switch allocation.
  always_comb begin : proc_issue_view
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
`ifdef ENABLE_CNOC_MFU
    if (issue_meta.msg_type == ROUTER_MSG_COMP) begin
      issue_meta.process = issue_route_process_mask_bit;
    end
`endif

    issue_valid = 1'b0;
    issue_accept_fire = 1'b0;
    commit_fire = rinport_ctrl_i.commit_valid;
    if (issue_has_flit) begin
      issue_valid = 1'b1;
      issue_accept_fire = issue_queue_push;
    end

    pop_vec = '0;
    if (commit_fire)
      pop_vec[rinport_ctrl_i.commit_vc] = 1'b1;
  end

  always_comb begin : proc_inflight_next
    inflight_d = inflight_q;
    if (commit_fire) begin
      inflight_d[rinport_ctrl_i.commit_vc] = 1'b0;
    end
    if (issue_accept_fire) begin
      inflight_d[selected_vc_q] = 1'b1;
    end
  end

  always_comb begin : proc_issue_queue_select
    issue_slot_ctx_ok = '1;
    issue_select_valid = 1'b0;
    issue_select_slot = 1'b0;

    for (issue_queue_slot_idx = 0; issue_queue_slot_idx < ISSUE_QUEUE_DEPTH;
         issue_queue_slot_idx = issue_queue_slot_idx + 1) begin
`ifdef ENABLE_CNOC_MFU
      logic issue_data_owner_match;
      issue_data_owner_match = 1'b0;
      if (issue_queue_q[issue_queue_slot_idx].valid &&
          issue_queue_q[issue_queue_slot_idx].may_need_mfu &&
          (issue_queue_q[issue_queue_slot_idx].meta.msg_type == ROUTER_MSG_COMP) &&
          (issue_queue_q[issue_queue_slot_idx].meta.opcode == OP_ATTENTION)) begin
        if (issue_queue_q[issue_queue_slot_idx].head_like) begin
          issue_slot_ctx_ok[issue_queue_slot_idx] =
              !attention_ctx_status_i.valid ||
              ((attention_ctx_status_i.stream_port ==
                issue_queue_q[issue_queue_slot_idx].src_port) &&
               (attention_ctx_status_i.stream_vc ==
                issue_queue_q[issue_queue_slot_idx].src_vc));
        end else begin
          issue_slot_ctx_ok[issue_queue_slot_idx] =
              attention_ctx_status_i.valid &&
              (attention_ctx_status_i.stream_port ==
               issue_queue_q[issue_queue_slot_idx].src_port) &&
              (attention_ctx_status_i.stream_vc ==
               issue_queue_q[issue_queue_slot_idx].src_vc);
        end
      end

      if (issue_queue_q[issue_queue_slot_idx].valid &&
          issue_queue_q[issue_queue_slot_idx].may_need_mfu &&
          (issue_queue_q[issue_queue_slot_idx].meta.msg_type == ROUTER_MSG_COMP) &&
          ((issue_queue_q[issue_queue_slot_idx].meta.opcode == OP_LINEAR) ||
           (issue_queue_q[issue_queue_slot_idx].meta.opcode == OP_MATMUL))) begin
        for (int slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
          if (!issue_data_owner_match &&
              matmul_ctx_status_i.slot_valid[slot_idx] &&
              (matmul_ctx_status_i.slot_stream_port_flat[slot_idx * 3 +: 3] ==
               issue_queue_q[issue_queue_slot_idx].src_port) &&
              (matmul_ctx_status_i.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] ==
               issue_queue_q[issue_queue_slot_idx].src_vc)) begin
            issue_data_owner_match = 1'b1;
          end
        end
        if (issue_queue_q[issue_queue_slot_idx].head_like) begin
          issue_slot_ctx_ok[issue_queue_slot_idx] =
              issue_slot_ctx_ok[issue_queue_slot_idx] &&
              (issue_data_owner_match || matmul_ctx_status_i.has_free_slot);
        end else begin
          issue_slot_ctx_ok[issue_queue_slot_idx] =
              issue_slot_ctx_ok[issue_queue_slot_idx] &&
              issue_data_owner_match;
        end
      end
`endif
    end

    if (issue_queue_q[0].valid && issue_slot_ctx_ok[0]) begin
      issue_select_valid = 1'b1;
      issue_select_slot = 1'b0;
    end else if (issue_queue_q[1].valid && issue_slot_ctx_ok[1]) begin
      issue_select_valid = 1'b1;
      issue_select_slot = 1'b1;
    end
  end

  always_comb begin : proc_issue_queue_next
    logic issue_push_attention_owner_match;
    logic issue_push_attention_head_claim_conflict;

    issue_queue_d[0] = issue_queue_q[0];
    issue_queue_d[1] = issue_queue_q[1];
    issue_queue_push = 1'b0;
    issue_queue_pop = issue_pop_i && issue_select_valid;
    issue_push_entry = '0;
    issue_push_attention_owner_match = 1'b0;
    issue_push_attention_head_claim_conflict = 1'b0;
    issue_queue_has_free_slot =
        !issue_queue_q[0].valid || !issue_queue_q[1].valid;

    if (issue_queue_pop) begin
      issue_queue_d[issue_select_slot] = '0;
    end

`ifdef ENABLE_CNOC_MFU
    issue_push_attention_owner_match =
        attention_ctx_status_i.valid &&
        (attention_ctx_status_i.stream_port == rinport_channel_i) &&
        (attention_ctx_status_i.stream_vc == issue_sel_vc);
    if (issue_has_flit &&
        issue_may_need_mfu &&
        (issue_meta.msg_type == ROUTER_MSG_COMP) &&
        (issue_meta.opcode == OP_ATTENTION) &&
        issue_head_like &&
        !issue_push_attention_owner_match) begin
      for (int unsigned slot_idx = 0; slot_idx < ISSUE_QUEUE_DEPTH; slot_idx = slot_idx + 1) begin
        if (!issue_push_attention_head_claim_conflict &&
            issue_queue_d[slot_idx].valid &&
            issue_queue_d[slot_idx].may_need_mfu &&
            (issue_queue_d[slot_idx].meta.msg_type == ROUTER_MSG_COMP) &&
            (issue_queue_d[slot_idx].meta.opcode == OP_ATTENTION) &&
            issue_queue_d[slot_idx].head_like) begin
          issue_push_attention_head_claim_conflict = 1'b1;
        end
      end
    end
`endif

    if (issue_has_flit && issue_queue_has_free_slot && !issue_push_attention_head_claim_conflict) begin
      issue_queue_push = 1'b1;
      issue_push_entry.valid = 1'b1;
      issue_push_entry.flit = issue_flit;
      issue_push_entry.meta = issue_meta;
      issue_push_entry.route_path = issue_route_path;
      issue_push_entry.src_port = rinport_channel_i;
      issue_push_entry.src_vc = issue_sel_vc;
      issue_push_entry.route_sel = issue_route_sel;
      issue_push_entry.dst_vc = '0;
      issue_push_entry.traffic_class = issue_traffic_class;
      issue_push_entry.head_like = issue_head_like;
      issue_push_entry.tail_like = issue_tail_like;
      issue_push_entry.may_need_mfu = issue_may_need_mfu;
      issue_push_entry.reserved_vc = 1'b0;
      if (!issue_queue_d[0].valid) begin
        issue_queue_d[0] = issue_push_entry;
      end else begin
        issue_queue_d[1] = issue_push_entry;
      end
    end
  end


  // VC enqueue logic
  always_comb begin : proc_vc_enqueue
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
  // FIFO pop remains controlled internally by the commit sideband.
  assign rinport_issue_o.pop_fire = 1'b0;
  assign rinport_issue_o.sel_vc = issue_sel_vc;
  always_comb begin : proc_issue_entry_output
    issue_entry_o = '0;
    if (issue_select_valid) begin
      issue_entry_o = issue_queue_q[issue_select_slot];
    end
  end
  assign rinport_raw_meta_o = issue_meta;
  assign rinport_route_path_o = issue_route_path;
  assign rinport_may_need_mfu_o = issue_may_need_mfu;
  assign rinport_traffic_class_o = issue_traffic_class;
  assign rinport_flow_o.push_ack = push_ack;
  assign rinport_flow_o.in_accept = input_accept;


`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_debug_and_assertions
    if (!reset) begin
      if ($test$plusargs("CNOC_RTL_IU_DEBUG")) begin
        if ((vc_empty_slots[selected_vc_q] < 3'd4) ||
            issue_queue_q[0].valid ||
            issue_queue_q[1].valid) begin
          $display("CNOC_IU_DEBUG cycle=%0t port=%0d sel_vc=%0d sel_valid=%0d empty=%0d inflight=%b issue_has=%0d push=%0d pop=%0d pop_i=%0d q0_valid=%0d q0_vc=%0d q0_op=%0d q0_head=%0d q0_tail=%0d q0_ctx=%0d q1_valid=%0d q1_vc=%0d q1_op=%0d q1_head=%0d q1_tail=%0d q1_ctx=%0d",
                   $time,
                   rinport_channel_i,
                   selected_vc_q,
                   selected_valid_d,
                   vc_empty_slots[selected_vc_q],
                   inflight_q,
                   issue_has_flit,
                   issue_queue_push,
                   issue_queue_pop,
                   issue_pop_i,
                   issue_queue_q[0].valid,
                   issue_queue_q[0].src_vc,
                   issue_queue_q[0].meta.opcode,
                   issue_queue_q[0].head_like,
                   issue_queue_q[0].tail_like,
                   issue_slot_ctx_ok[0],
                   issue_queue_q[1].valid,
                   issue_queue_q[1].src_vc,
                   issue_queue_q[1].meta.opcode,
                   issue_queue_q[1].head_like,
                   issue_queue_q[1].tail_like,
                   issue_slot_ctx_ok[1]);
        end
      end

      if (commit_fire && !inflight_q[rinport_ctrl_i.commit_vc]) begin
        $error("input_unit: attempted to commit a VC without an inflight flit");
      end

      if (issue_accept_fire && !issue_has_flit) begin
        $error("input_unit: accepted an empty/invalid selected VC into pipeline");
      end

      if (issue_pop_i && !issue_select_valid) begin
        $error("input_unit: attempted to pop an empty issue queue");
      end

      if (issue_queue_push && !issue_queue_has_free_slot) begin
        $error("input_unit: attempted to push a full issue queue");
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
