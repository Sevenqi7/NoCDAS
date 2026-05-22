// Description: Five-port router used by the NoCDAS/Verilator co-simulation.
//              Router.sv owns top-level wiring, the output pipeline register,
//              and the optional cNoC/MFU side path.  Input buffering, issue
//              queues, VC allocation, switch allocation, crossbar muxing, and
//              output commit are implemented by the existing stage modules.

module Router #(
    parameter integer VC_NUM = 8,
    parameter integer VC_ID_W = 3,
    parameter integer FLIT_W = 256,
    parameter integer MATMUL_CTX_SLOTS = 1
)(
    input logic [2:0] X_cur, Y_cur,
    input logic clk, reset,
`ifdef ENABLE_CNOC_MFU
    input logic [5:0] cnoc_task_count,
    input logic [router_ports_pkg::CNOC_MAX_TASKS*
                 router_ports_pkg::CNOC_TASK_ID_W-1:0] cnoc_task_ids_flat,
    input logic [15:0] cnoc_weight_row_size,
`endif

    input router_ports_pkg::rinport_data_t east_input_data,
    input router_ports_pkg::rinport_data_t west_input_data,
    input router_ports_pkg::rinport_data_t north_input_data,
    input router_ports_pkg::rinport_data_t south_input_data,
    input router_ports_pkg::rinport_data_t local_input_data,
    output router_ports_pkg::rinport_flow_t east_input_flow,
    output router_ports_pkg::rinport_flow_t west_input_flow,
    output router_ports_pkg::rinport_flow_t north_input_flow,
    output router_ports_pkg::rinport_flow_t south_input_flow,
    output router_ports_pkg::rinport_flow_t local_input_flow,

    input router_ports_pkg::routport_flow_t east_output_flow,
    input router_ports_pkg::routport_flow_t west_output_flow,
    input router_ports_pkg::routport_flow_t north_output_flow,
    input router_ports_pkg::routport_flow_t south_output_flow,
    input router_ports_pkg::routport_flow_t local_output_flow,
    output router_ports_pkg::routport_data_t east_output_data,
    output router_ports_pkg::routport_data_t west_output_data,
    output router_ports_pkg::routport_data_t north_output_data,
    output router_ports_pkg::routport_data_t south_output_data,
    output router_ports_pkg::routport_data_t local_output_data
`ifdef ROUTER_ENABLE_COSIM
    ,
    output logic [31:0] cnoc_weight_bytes_stored,
    output logic [31:0] cnoc_kv_bytes_stored,
    output logic [15:0] cnoc_kv_token_count,
    output logic        cnoc_last_type4_store_valid,
    output logic        cnoc_last_type4_store_bank,
    output logic [10:0] cnoc_last_type4_store_addr,
    output logic [5:0]  cnoc_last_type4_store_bytes
`endif
);
  import router_ports_pkg::*;

  localparam integer NUM_PORTS = PORT_NUM;
  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;
  localparam router_port_sel_e PORT_SEL_EAST = ROUTER_PORT_EAST;
  localparam router_port_sel_e PORT_SEL_WEST = ROUTER_PORT_WEST;
  localparam router_port_sel_e PORT_SEL_NORTH = ROUTER_PORT_NORTH;
  localparam router_port_sel_e PORT_SEL_SOUTH = ROUTER_PORT_SOUTH;
  localparam router_port_sel_e PORT_SEL_LOCAL = ROUTER_PORT_LOCAL;
  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;

  rinport_data_t in_data [NUM_PORTS];
  rinport_flow_t in_flow [NUM_PORTS];
  rinport_ctrl_t rin_ctrl [NUM_PORTS];
  routport_flow_t out_flow [NUM_PORTS];
  routport_data_t out_data [NUM_PORTS];

  router_pipe_entry_t issue_entry [NUM_PORTS];
  router_pipe_entry_t va_entry [NUM_PORTS];
  router_pipe_entry_t crossbar_out [NUM_PORTS];
  router_pipe_entry_t commit_out_entry [NUM_PORTS];
  router_pipe_entry_t switch_out_entry [NUM_PORTS];
  router_pipe_entry_t out_q [NUM_PORTS];
  logic [NUM_PORTS-1:0] issue_pop;
  logic [NUM_PORTS-1:0] va_fire;
  logic [NUM_PORTS-1:0] output_commit_fire;
  logic [NUM_PORTS-1:0] output_mfu_selected;
  logic [NUM_PORTS-1:0] output_release_valid;
  logic [VC_ID_W-1:0] output_release_vc [NUM_PORTS];
  logic [2:0] crossbar_select [NUM_PORTS];
  logic crossbar_valid [NUM_PORTS];

  logic [FLIT_W-1:0] raw_flit [NUM_PORTS];
  flit_meta_t raw_meta [NUM_PORTS];
  route_path_t raw_route_path [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_vc_id [NUM_PORTS];
  logic [2:0] raw_stream_port [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_stream_vc [NUM_PORTS];
  logic [NUM_PORTS-1:0] raw_write;
  logic [NUM_PORTS-1:0] mfu_sa_eligible;
  attention_ctx_status_t attention_ctx_status;
  attention_ctx_status_t attention_gate_ctx_status;
  matmul_ctx_status_t matmul_ctx_status;
  matmul_ctx_status_t matmul_gate_ctx_status;
`ifdef ENABLE_CNOC_MFU
  attention_ctx_status_t attention_gate_ctx_status_d;
  matmul_ctx_status_t matmul_gate_ctx_status_d;
  logic attention_pending_valid_d;
  logic [2:0] attention_pending_stream_port_d;
  logic [VC_ID_W-1:0] attention_pending_stream_vc_d;
  logic attention_pending_valid_q;
  logic [2:0] attention_pending_stream_port_q;
  logic [VC_ID_W-1:0] attention_pending_stream_vc_q;
  logic [MATMUL_CTX_SLOTS_MAX-1:0] matmul_pending_valid_d;
  logic [2:0] matmul_pending_stream_port_d [MATMUL_CTX_SLOTS_MAX];
  logic [VC_ID_W-1:0] matmul_pending_stream_vc_d [MATMUL_CTX_SLOTS_MAX];
  logic [MATMUL_CTX_SLOTS_MAX-1:0] matmul_pending_valid_q;
  logic [2:0] matmul_pending_stream_port_q [MATMUL_CTX_SLOTS_MAX];
  logic [VC_ID_W-1:0] matmul_pending_stream_vc_q [MATMUL_CTX_SLOTS_MAX];
  logic cnoc_attention_claim_valid;
  logic [2:0] cnoc_attention_claim_src_port;
  logic [VC_ID_W-1:0] cnoc_attention_claim_src_vc;
  logic cnoc_data_claim_valid;
  logic [2:0] cnoc_data_claim_src_port;
  logic [VC_ID_W-1:0] cnoc_data_claim_src_vc;
`endif

`ifdef ENABLE_CNOC_MFU
  logic [NUM_PORTS-1:0] raw_needs_mfu;
  logic [NUM_PORTS-1:0] raw_msg_is_dist;
  logic [NUM_PORTS-1:0] raw_msg_is_comp;
  logic [NUM_PORTS-1:0] raw_type4_at_dest;
  logic [NUM_PORTS-1:0] mfu_holds_routport;
  logic mfu_ingress_ready;
  router_pipe_entry_t mfu_ingress_q [2];
  router_pipe_entry_t mfu_ingress_d [2];
  router_pipe_entry_t mfu_ingress_push_entry;
  logic mfu_ingress_front_valid;
  logic mfu_ingress_back_valid;
  logic mfu_ingress_push;
  logic mfu_ingress_pop;
  logic [FLIT_W-1:0] arb_flit_word;
  flit_meta_t arb_flit_meta;
  route_path_t arb_route_path;
  logic [VC_ID_W-1:0] arb_vc_id;
  logic [2:0] arb_stream_port;
  logic [VC_ID_W-1:0] arb_stream_vc;
  logic [2:0] arb_out_sel;
  logic arb_flit_valid;
  logic mfu_busy;
  logic mfu_capture;
  logic [2:0] mfu_capture_out_sel;
  logic mfu_emit_valid;
  logic [2:0] mfu_emit_out_sel;
  logic [FLIT_W-1:0] mfu_emit_flit;
  flit_meta_t mfu_emit_meta;
  route_path_t mfu_emit_route_path;
  logic [VC_ID_W-1:0] mfu_emit_vc_id;
`else
  localparam logic [NUM_PORTS-1:0] mfu_holds_routport = '0;
  localparam logic mfu_ingress_ready = 1'b1;
  localparam logic mfu_busy = 1'b0;
  localparam logic mfu_emit_valid = 1'b0;
  localparam logic [2:0] mfu_emit_out_sel = ROUTER_PORT_INV;
  localparam logic [FLIT_W-1:0] mfu_emit_flit = '0;
  flit_meta_t mfu_emit_meta;
  route_path_t mfu_emit_route_path;
  localparam logic [VC_ID_W-1:0] mfu_emit_vc_id = '0;
`endif

  assign in_data[PORT_SEL_EAST]  = east_input_data;
  assign in_data[PORT_SEL_WEST]  = west_input_data;
  assign in_data[PORT_SEL_NORTH] = north_input_data;
  assign in_data[PORT_SEL_SOUTH] = south_input_data;
  assign in_data[PORT_SEL_LOCAL] = local_input_data;

  assign out_flow[PORT_SEL_EAST]  = east_output_flow;
  assign out_flow[PORT_SEL_WEST]  = west_output_flow;
  assign out_flow[PORT_SEL_NORTH] = north_output_flow;
  assign out_flow[PORT_SEL_SOUTH] = south_output_flow;
  assign out_flow[PORT_SEL_LOCAL] = local_output_flow;

  assign east_output_data  = out_data[PORT_SEL_EAST];
  assign west_output_data  = out_data[PORT_SEL_WEST];
  assign north_output_data = out_data[PORT_SEL_NORTH];
  assign south_output_data = out_data[PORT_SEL_SOUTH];
  assign local_output_data = out_data[PORT_SEL_LOCAL];

  assign east_input_flow  = in_flow[PORT_SEL_EAST];
  assign west_input_flow  = in_flow[PORT_SEL_WEST];
  assign north_input_flow = in_flow[PORT_SEL_NORTH];
  assign south_input_flow = in_flow[PORT_SEL_SOUTH];
  assign local_input_flow = in_flow[PORT_SEL_LOCAL];

  genvar rinport_gen_idx;
  generate
    for (rinport_gen_idx = 0; rinport_gen_idx < NUM_PORTS; rinport_gen_idx = rinport_gen_idx + 1) begin : rinport_gen
      input_unit #(
          .VC_NUM(VC_NUM),
          .VC_ID_W(VC_ID_W),
          .FLIT_W(FLIT_W)
      ) input_unit_i (
          .clk(clk),
          .reset(reset),
          .rinport_data_i(in_data[rinport_gen_idx]),
          .rinport_ctrl_i(rin_ctrl[rinport_gen_idx]),
          .attention_ctx_status_i(attention_gate_ctx_status),
          .matmul_ctx_status_i(matmul_gate_ctx_status),
          .rinport_x_cur_i(X_cur),
          .rinport_y_cur_i(Y_cur),
          .rinport_channel_i(3'(rinport_gen_idx)),
          .issue_pop_i(issue_pop[rinport_gen_idx]),
          .rinport_raw_meta_o(),
          .rinport_route_path_o(),
          .rinport_may_need_mfu_o(),
          .rinport_traffic_class_o(),
          .rinport_issue_o(),
          .issue_entry_o(issue_entry[rinport_gen_idx]),
          .rinport_flow_o(in_flow[rinport_gen_idx])
      );
    end
  endgenerate

  vc_allocator #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W)
  ) vc_allocator_i (
      .clk(clk),
      .reset(reset),
      .issue_entry_i(issue_entry),
      .attention_ctx_status_i(attention_gate_ctx_status),
      .matmul_ctx_status_i(matmul_gate_ctx_status),
      .routport_flow_i(out_flow),
      .va_fire_i(va_fire),
      .output_release_valid_i(output_release_valid),
      .output_release_vc_i(output_release_vc),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_vc_id_i(mfu_emit_vc_id),
`ifdef ENABLE_CNOC_MFU
      .cnoc_attention_claim_valid_o(cnoc_attention_claim_valid),
      .cnoc_attention_claim_src_port_o(cnoc_attention_claim_src_port),
      .cnoc_attention_claim_src_vc_o(cnoc_attention_claim_src_vc),
      .cnoc_data_claim_valid_o(cnoc_data_claim_valid),
      .cnoc_data_claim_src_port_o(cnoc_data_claim_src_port),
      .cnoc_data_claim_src_vc_o(cnoc_data_claim_src_vc),
`endif
      .issue_pop_o(issue_pop),
      .va_entry_o(va_entry)
  );

  CrossBar #(
      .NUM_PORTS(NUM_PORTS)
  ) crossbar_i (
      .data_i(va_entry),
      .select_i(crossbar_select),
      .valid_i(crossbar_valid),
      .data_o(crossbar_out)
  );

  router_output_stage #(
      .NUM_PORTS(NUM_PORTS),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) output_stage_i (
      .x_cur_i(X_cur),
      .y_cur_i(Y_cur),
      .out_entry_i(out_q),
      .routport_flow_i(out_flow),
      .attention_ctx_status_i(attention_gate_ctx_status),
      .matmul_ctx_status_i(matmul_gate_ctx_status),
      .mfu_holds_routport_i(mfu_holds_routport),
      .mfu_ingress_ready_i(mfu_ingress_ready),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_flit_i(mfu_emit_flit),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_route_path_i(mfu_emit_route_path),
      .mfu_emit_vc_id_i(mfu_emit_vc_id),
      .output_commit_fire_o(output_commit_fire),
      .output_mfu_selected_o(output_mfu_selected),
      .output_release_valid_o(output_release_valid),
      .output_release_vc_o(output_release_vc),
      .out_entry_o(commit_out_entry),
      .rinport_ctrl_o(rin_ctrl),
      .raw_flit_o(raw_flit),
      .raw_meta_o(raw_meta),
      .raw_route_path_o(raw_route_path),
      .raw_vc_id_o(raw_vc_id),
      .raw_stream_port_o(raw_stream_port),
      .raw_stream_vc_o(raw_stream_vc),
      .raw_write_o(raw_write),
      .routport_data_o(out_data)
  );

  switch_allocator #(
      .NUM_PORTS(NUM_PORTS)
  ) switch_allocator_i (
      .clk_i(clk),
      .reset_i(reset),
      .va_entry_i(va_entry),
      .out_entry_i(commit_out_entry),
      .output_commit_fire_i(output_commit_fire),
      .mfu_issue_eligible_i(mfu_sa_eligible),
      .va_fire_o(va_fire),
      .crossbar_select_o(crossbar_select),
      .crossbar_valid_o(crossbar_valid)
  );

`ifdef ENABLE_CNOC_MFU
  always_comb begin : proc_attention_pending_next
    logic claim_found;

    attention_pending_valid_d = attention_pending_valid_q;
    attention_pending_stream_port_d = attention_pending_stream_port_q;
    attention_pending_stream_vc_d = attention_pending_stream_vc_q;
    claim_found = 1'b0;

    if (attention_pending_valid_q &&
        attention_ctx_status.valid &&
        (attention_ctx_status.stream_port == attention_pending_stream_port_q) &&
        (attention_ctx_status.stream_vc == attention_pending_stream_vc_q)) begin
      attention_pending_valid_d = 1'b0;
      attention_pending_stream_port_d = ROUTER_PORT_INV;
      attention_pending_stream_vc_d = '0;
    end

    if (!claim_found &&
        !attention_pending_valid_d &&
        !attention_ctx_status.valid &&
        cnoc_attention_claim_valid) begin
      attention_pending_valid_d = 1'b1;
      attention_pending_stream_port_d = cnoc_attention_claim_src_port;
      attention_pending_stream_vc_d = cnoc_attention_claim_src_vc;
      claim_found = 1'b1;
    end
  end

  always_comb begin : proc_matmul_pending_next
    logic claim_found;
    logic slot_match;
    logic free_found;

    matmul_pending_valid_d = matmul_pending_valid_q;
    for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
      matmul_pending_stream_port_d[slot_idx] = matmul_pending_stream_port_q[slot_idx];
      matmul_pending_stream_vc_d[slot_idx] = matmul_pending_stream_vc_q[slot_idx];
    end
    claim_found = 1'b0;
    free_found = 1'b0;

    for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
      slot_match = 1'b0;
      if (matmul_pending_valid_q[slot_idx]) begin
        for (int unsigned active_idx = 0; active_idx < MATMUL_CTX_SLOTS_MAX; active_idx = active_idx + 1) begin
          if (!slot_match &&
              matmul_ctx_status.slot_valid[active_idx] &&
              (matmul_ctx_status.slot_stream_port_flat[active_idx * 3 +: 3] ==
               matmul_pending_stream_port_q[slot_idx]) &&
              (matmul_ctx_status.slot_stream_vc_flat[active_idx * VC_ID_W +: VC_ID_W] ==
               matmul_pending_stream_vc_q[slot_idx])) begin
            slot_match = 1'b1;
          end
        end
        if (slot_match) begin
          matmul_pending_valid_d[slot_idx] = 1'b0;
          matmul_pending_stream_port_d[slot_idx] = ROUTER_PORT_INV;
          matmul_pending_stream_vc_d[slot_idx] = '0;
        end
      end
    end

    if (!claim_found && cnoc_data_claim_valid) begin
        slot_match = 1'b0;
        for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
          if (!slot_match &&
              ((matmul_pending_valid_d[slot_idx] &&
                (matmul_pending_stream_port_d[slot_idx] == cnoc_data_claim_src_port) &&
                (matmul_pending_stream_vc_d[slot_idx] == cnoc_data_claim_src_vc)) ||
               (matmul_ctx_status.slot_valid[slot_idx] &&
                (matmul_ctx_status.slot_stream_port_flat[slot_idx * 3 +: 3] ==
                 cnoc_data_claim_src_port) &&
                (matmul_ctx_status.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] ==
                 cnoc_data_claim_src_vc)))) begin
            slot_match = 1'b1;
          end
        end
        if (!slot_match) begin
          free_found = 1'b0;
          for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
            if (!free_found &&
                (slot_idx < MATMUL_CTX_SLOTS) &&
                !matmul_pending_valid_d[slot_idx] &&
                !matmul_ctx_status.slot_valid[slot_idx]) begin
              matmul_pending_valid_d[slot_idx] = 1'b1;
              matmul_pending_stream_port_d[slot_idx] = cnoc_data_claim_src_port;
              matmul_pending_stream_vc_d[slot_idx] = cnoc_data_claim_src_vc;
              free_found = 1'b1;
              claim_found = 1'b1;
            end
          end
        end
    end
  end

  // Front-end stages consume a registered owner view.  This cuts leaf context
  // feedback out of issue/VA/SA timing while keeping new VA claims visible on
  // the cycle after they are accepted.
  always_comb begin : proc_attention_gate_ctx_next
    attention_gate_ctx_status_d = attention_ctx_status;
    if (attention_pending_valid_d) begin
      attention_gate_ctx_status_d.valid = 1'b1;
      attention_gate_ctx_status_d.stream_port = attention_pending_stream_port_d;
      attention_gate_ctx_status_d.stream_vc = attention_pending_stream_vc_d;
    end
  end

  // Data-op MatMul/Linear uses a parameterized set of streaming accumulator
  // contexts. Router keeps pending slots between VA admission and leaf capture
  // so no new data-op head can claim a slot that was already committed.
  always_comb begin : proc_matmul_gate_ctx_next
    int unsigned occupied_count;

    matmul_gate_ctx_status_d = '0;
    occupied_count = 0;
    for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
      if (slot_idx < MATMUL_CTX_SLOTS) begin
        matmul_gate_ctx_status_d.slot_valid[slot_idx] = matmul_ctx_status.slot_valid[slot_idx];
        matmul_gate_ctx_status_d.slot_stream_port_flat[slot_idx * 3 +: 3] =
            matmul_ctx_status.slot_stream_port_flat[slot_idx * 3 +: 3];
        matmul_gate_ctx_status_d.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] =
            matmul_ctx_status.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W];
        if (matmul_pending_valid_d[slot_idx] &&
            !matmul_gate_ctx_status_d.slot_valid[slot_idx]) begin
          matmul_gate_ctx_status_d.slot_valid[slot_idx] = 1'b1;
          matmul_gate_ctx_status_d.slot_stream_port_flat[slot_idx * 3 +: 3] =
              matmul_pending_stream_port_d[slot_idx];
          matmul_gate_ctx_status_d.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] =
              matmul_pending_stream_vc_d[slot_idx];
        end
        if (matmul_gate_ctx_status_d.slot_valid[slot_idx]) begin
          matmul_gate_ctx_status_d.any_valid = 1'b1;
          occupied_count = occupied_count + 1;
        end
      end
    end
    matmul_gate_ctx_status_d.has_free_slot = occupied_count < MATMUL_CTX_SLOTS;
  end

  always_comb begin : proc_mfu_hold_decode
    mfu_holds_routport = '0;
    if (mfu_busy && (mfu_emit_out_sel < NUM_PORTS)) begin
      mfu_holds_routport[mfu_emit_out_sel] = 1'b1;
    end
    for (int unsigned fifo_idx = 0; fifo_idx < 2; fifo_idx = fifo_idx + 1) begin
      if (mfu_ingress_q[fifo_idx].valid &&
          (mfu_ingress_q[fifo_idx].route_sel < NUM_PORTS)) begin
        mfu_holds_routport[mfu_ingress_q[fifo_idx].route_sel] = 1'b1;
      end
    end
  end

  always_comb begin : proc_raw_mfu_decode
    for (int raw_idx = 0; raw_idx < NUM_PORTS; raw_idx = raw_idx + 1) begin
      raw_msg_is_dist[raw_idx] = (raw_meta[raw_idx].msg_type == ROUTER_MSG_DIST);
      raw_msg_is_comp[raw_idx] = (raw_meta[raw_idx].msg_type == ROUTER_MSG_COMP);
      raw_type4_at_dest[raw_idx] =
          (raw_meta[raw_idx].dst_x == X_cur) && (raw_meta[raw_idx].dst_y == Y_cur);
      raw_needs_mfu[raw_idx] =
          (raw_msg_is_comp[raw_idx] && raw_meta[raw_idx].process) ||
          (raw_msg_is_dist[raw_idx] &&
           (raw_meta[raw_idx].process || raw_type4_at_dest[raw_idx]));
    end
  end

  always_comb begin : proc_mfu_sa_eligibility
    for (int unsigned issue_idx = 0; issue_idx < NUM_PORTS; issue_idx = issue_idx + 1) begin
      mfu_sa_eligible[issue_idx] = mfu_ingress_ready;
    end
  end

  mfu_arbiter #(
      .NUM_PORTS(NUM_PORTS),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) mfu_arbiter_i (
      .raw_flit_i(raw_flit),
      .raw_meta_i(raw_meta),
      .raw_route_path_i(raw_route_path),
      .raw_vc_id_i(raw_vc_id),
      .raw_stream_port_i(raw_stream_port),
      .raw_stream_vc_i(raw_stream_vc),
      .raw_valid_i(raw_write),
      .raw_needs_mfu_i(raw_needs_mfu),
      .mfu_flit_o(arb_flit_word),
      .mfu_meta_o(arb_flit_meta),
      .mfu_route_path_o(arb_route_path),
      .mfu_vc_id_o(arb_vc_id),
      .mfu_stream_port_o(arb_stream_port),
      .mfu_stream_vc_o(arb_stream_vc),
      .mfu_out_sel_o(arb_out_sel),
      .mfu_valid_o(arb_flit_valid)
  );

  assign mfu_ingress_front_valid = mfu_ingress_q[0].valid;
  assign mfu_ingress_back_valid = mfu_ingress_q[1].valid;
  assign mfu_ingress_ready = !mfu_ingress_front_valid || !mfu_ingress_back_valid;

  always_comb begin : proc_mfu_ingress_fifo_next
    mfu_ingress_push = arb_flit_valid && mfu_ingress_ready;
    mfu_ingress_pop = mfu_capture && mfu_ingress_front_valid;
    mfu_ingress_push_entry = '0;
    mfu_ingress_push_entry.valid = 1'b1;
    mfu_ingress_push_entry.flit = arb_flit_word;
    mfu_ingress_push_entry.meta = arb_flit_meta;
    mfu_ingress_push_entry.route_path = arb_route_path;
    mfu_ingress_push_entry.dst_vc = arb_vc_id;
    mfu_ingress_push_entry.src_port = arb_stream_port;
    mfu_ingress_push_entry.src_vc = arb_stream_vc;
    mfu_ingress_push_entry.route_sel = arb_out_sel;

    mfu_ingress_d[0] = mfu_ingress_q[0];
    mfu_ingress_d[1] = mfu_ingress_q[1];

    if (mfu_ingress_pop) begin
      if (mfu_ingress_q[1].valid) begin
        mfu_ingress_d[0] = mfu_ingress_q[1];
        mfu_ingress_d[1] = '0;
      end else begin
        mfu_ingress_d[0] = '0;
        mfu_ingress_d[1] = '0;
      end
    end

    if (mfu_ingress_push) begin
      if (!mfu_ingress_front_valid) begin
        mfu_ingress_d[0] = mfu_ingress_push_entry;
        mfu_ingress_d[1] = '0;
      end else if (!mfu_ingress_back_valid) begin
        mfu_ingress_d[1] = mfu_ingress_push_entry;
      end
    end

    if (!mfu_ingress_d[0].valid && mfu_ingress_d[1].valid) begin
      mfu_ingress_d[0] = mfu_ingress_d[1];
      mfu_ingress_d[1] = '0;
    end
  end

  cnoc_mfu #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
      ,
      .MATMUL_CTX_SLOTS(MATMUL_CTX_SLOTS)
  ) cnoc_mfu_i (
      .clk(clk),
      .reset(reset),
      .raw_flit_i(mfu_ingress_q[0].flit),
      .raw_meta_i(mfu_ingress_q[0].meta),
      .raw_route_path_i(mfu_ingress_q[0].route_path),
      .raw_vc_id_i(mfu_ingress_q[0].dst_vc),
      .raw_stream_port_i(mfu_ingress_q[0].src_port),
      .raw_stream_vc_i(mfu_ingress_q[0].src_vc),
      .raw_out_sel_i(mfu_ingress_q[0].route_sel),
      .raw_valid_i(mfu_ingress_q[0].valid),
      .routport_flow_i(out_flow),
      .cnoc_task_count_i(cnoc_task_count),
      .cnoc_task_ids_flat_i(cnoc_task_ids_flat),
      .cnoc_weight_row_size_i(cnoc_weight_row_size),
      .busy_o(mfu_busy),
      .capture_o(mfu_capture),
      .capture_out_sel_o(mfu_capture_out_sel),
      .emit_valid_o(mfu_emit_valid),
      .emit_out_sel_o(mfu_emit_out_sel),
      .emit_flit_o(mfu_emit_flit),
      .emit_meta_o(mfu_emit_meta),
      .emit_route_path_o(mfu_emit_route_path),
      .emit_vc_id_o(mfu_emit_vc_id),
      .attention_ctx_o(attention_ctx_status),
      .matmul_ctx_o(matmul_ctx_status)
`ifdef ROUTER_ENABLE_COSIM
      ,
      .weight_bytes_stored_o(cnoc_weight_bytes_stored),
      .kv_bytes_stored_o(cnoc_kv_bytes_stored),
      .kv_token_count_o(cnoc_kv_token_count),
      .last_type4_store_valid_o(cnoc_last_type4_store_valid),
      .last_type4_store_bank_o(cnoc_last_type4_store_bank),
      .last_type4_store_addr_o(cnoc_last_type4_store_addr),
      .last_type4_store_bytes_o(cnoc_last_type4_store_bytes)
`endif
  );
`else
  always_comb begin : proc_disabled_mfu_defaults
    mfu_emit_meta = '0;
    mfu_emit_route_path = '0;
  end
  always_comb begin : proc_disabled_ctx_defaults
    mfu_sa_eligible = '1;
    attention_ctx_status = '0;
    attention_gate_ctx_status = '0;
    matmul_ctx_status = '0;
    matmul_ctx_status.has_free_slot = 1'b1;
    matmul_gate_ctx_status = '0;
    matmul_gate_ctx_status.has_free_slot = 1'b1;
  end

`ifdef ROUTER_ENABLE_COSIM
  assign cnoc_weight_bytes_stored = '0;
  assign cnoc_kv_bytes_stored = '0;
  assign cnoc_kv_token_count = '0;
  assign cnoc_last_type4_store_valid = 1'b0;
  assign cnoc_last_type4_store_bank = 1'b0;
  assign cnoc_last_type4_store_addr = '0;
  assign cnoc_last_type4_store_bytes = '0;
`endif
`endif

`ifdef ENABLE_CNOC_MFU
  always_ff @(posedge clk) begin : proc_cnoc_frontend_registers
    if (reset) begin
      attention_pending_valid_q <= 1'b0;
      attention_pending_stream_port_q <= ROUTER_PORT_INV;
      attention_pending_stream_vc_q <= '0;
      attention_gate_ctx_status <= '0;
      matmul_gate_ctx_status <= '0;
      matmul_gate_ctx_status.has_free_slot <= 1'b1;
      mfu_ingress_q[0] <= '0;
      mfu_ingress_q[1] <= '0;
      matmul_pending_valid_q <= '0;
      for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
        matmul_pending_stream_port_q[slot_idx] <= ROUTER_PORT_INV;
        matmul_pending_stream_vc_q[slot_idx] <= '0;
      end
    end else begin
      attention_pending_valid_q <= attention_pending_valid_d;
      attention_pending_stream_port_q <= attention_pending_stream_port_d;
      attention_pending_stream_vc_q <= attention_pending_stream_vc_d;
      attention_gate_ctx_status <= attention_gate_ctx_status_d;
      matmul_gate_ctx_status <= matmul_gate_ctx_status_d;
      mfu_ingress_q[0] <= mfu_ingress_d[0];
      mfu_ingress_q[1] <= mfu_ingress_d[1];
      matmul_pending_valid_q <= matmul_pending_valid_d;
      for (int unsigned slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
        matmul_pending_stream_port_q[slot_idx] <= matmul_pending_stream_port_d[slot_idx];
        matmul_pending_stream_vc_q[slot_idx] <= matmul_pending_stream_vc_d[slot_idx];
      end
    end
  end
`endif

  always_ff @(posedge clk) begin : proc_output_registers
    if (reset) begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        out_q[port_idx] <= '0;
      end
    end else begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        out_q[port_idx] <= switch_out_entry[port_idx];
      end
    end
  end

  always_comb begin : proc_output_slot_next
    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      switch_out_entry[port_idx] = commit_out_entry[port_idx];
      if (crossbar_valid[port_idx]) begin
        switch_out_entry[port_idx] = crossbar_out[port_idx];
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_router_assertions
    if (!reset) begin
`ifdef ENABLE_CNOC_MFU
      if (attention_pending_valid_q && attention_ctx_status.valid &&
          ((attention_ctx_status.stream_port != attention_pending_stream_port_q) ||
           (attention_ctx_status.stream_vc != attention_pending_stream_vc_q))) begin
        $error("Router: pending Attention owner diverged from active leaf owner");
      end

      for (int slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
        if (matmul_pending_valid_q[slot_idx] &&
            (matmul_pending_stream_port_q[slot_idx] == ROUTER_PORT_INV)) begin
          $error("Router: pending data-op slot lost its stream owner");
        end
      end

      if (mfu_capture && (mfu_capture_out_sel == PORT_SEL_INVALID)) begin
        $error("Router: MFU reported capture with invalid output select");
      end

      for (int issue_idx = 0; issue_idx < NUM_PORTS; issue_idx = issue_idx + 1) begin
        if (va_entry[issue_idx].valid &&
            va_entry[issue_idx].may_need_mfu &&
            ((va_entry[issue_idx].meta.msg_type != ROUTER_MSG_COMP) ||
             ((va_entry[issue_idx].meta.opcode != OP_ATTENTION) &&
              (va_entry[issue_idx].meta.opcode != OP_LINEAR) &&
              (va_entry[issue_idx].meta.opcode != OP_MATMUL))) &&
            !mfu_busy &&
            !mfu_sa_eligible[issue_idx]) begin
          $error("Router: non-owner-gated MFU issue was blocked by owner eligibility");
        end
      end
`endif
    end
  end
`endif

endmodule
