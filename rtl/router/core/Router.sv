// Description: Five-port router used by the NoCDAS/Verilator co-simulation.
//              Router.sv owns top-level wiring, the output pipeline register,
//              and the optional cNoC/MFU side path.  Input buffering, issue
//              queues, VC allocation, switch allocation, crossbar muxing, and
//              output commit are implemented by the existing stage modules.

module Router #(
    parameter integer VC_NUM = 8,
    parameter integer VC_ID_W = 3,
    parameter integer FLIT_W = 256
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

`ifndef SYNTHESIS
  initial begin
    if ($bits(flit_meta_t) != META_W) begin
      $fatal(1,
             "Router: flit_meta_t width %0d does not match META_W %0d",
             $bits(flit_meta_t),
             META_W);
    end
  end
`endif

  localparam integer NUM_PORTS = PORT_NUM;
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
  logic [2:0] crossbar_select [NUM_PORTS];
  logic crossbar_valid [NUM_PORTS];

  logic [FLIT_W-1:0] raw_flit [NUM_PORTS];
  flit_meta_t raw_meta [NUM_PORTS];
  route_path_t raw_route_path [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_vc_id [NUM_PORTS];
  logic [2:0] raw_stream_port [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_stream_vc [NUM_PORTS];
  logic [NUM_PORTS-1:0] raw_write;

`ifdef ENABLE_CNOC_MFU
  logic [NUM_PORTS-1:0] raw_needs_mfu;
  logic [NUM_PORTS-1:0] raw_msg_is_dist;
  logic [NUM_PORTS-1:0] raw_msg_is_comp;
  logic [NUM_PORTS-1:0] raw_type4_at_dest;
  logic [NUM_PORTS-1:0] mfu_holds_routport;
  logic [FLIT_W-1:0] active_flit_word;
  flit_meta_t active_flit_meta;
  route_path_t active_route_path;
  logic [VC_ID_W-1:0] active_vc_id;
  logic [2:0] active_stream_port;
  logic [VC_ID_W-1:0] active_stream_vc;
  logic [2:0] active_out_sel;
  logic active_flit_valid;
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
      .routport_flow_i(out_flow),
      .va_fire_i(va_fire),
      .output_commit_fire_i(output_commit_fire),
      .output_mfu_selected_i(output_mfu_selected),
      .output_commit_entry_i(out_q),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_vc_id_i(mfu_emit_vc_id),
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
      .mfu_holds_routport_i(mfu_holds_routport),
      .mfu_busy_i(mfu_busy),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_flit_i(mfu_emit_flit),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_route_path_i(mfu_emit_route_path),
      .mfu_emit_vc_id_i(mfu_emit_vc_id),
      .output_commit_fire_o(output_commit_fire),
      .output_mfu_selected_o(output_mfu_selected),
      .output_commit_entry_o(),
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
      .va_fire_o(va_fire),
      .crossbar_select_o(crossbar_select),
      .crossbar_valid_o(crossbar_valid)
  );

`ifdef ENABLE_CNOC_MFU
  always_comb begin : proc_mfu_hold_decode
    mfu_holds_routport = '0;
    if (mfu_busy && (mfu_emit_out_sel < NUM_PORTS)) begin
      mfu_holds_routport[mfu_emit_out_sel] = 1'b1;
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
      .mfu_flit_o(active_flit_word),
      .mfu_meta_o(active_flit_meta),
      .mfu_route_path_o(active_route_path),
      .mfu_vc_id_o(active_vc_id),
      .mfu_stream_port_o(active_stream_port),
      .mfu_stream_vc_o(active_stream_vc),
      .mfu_out_sel_o(active_out_sel),
      .mfu_valid_o(active_flit_valid)
  );

  cnoc_mfu #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) cnoc_mfu_i (
      .clk(clk),
      .reset(reset),
      .raw_flit_i(active_flit_word),
      .raw_meta_i(active_flit_meta),
      .raw_route_path_i(active_route_path),
      .raw_vc_id_i(active_vc_id),
      .raw_stream_port_i(active_stream_port),
      .raw_stream_vc_i(active_stream_vc),
      .raw_out_sel_i(active_out_sel),
      .raw_valid_i(active_flit_valid),
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
      .emit_vc_id_o(mfu_emit_vc_id)
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
  always_ff @(posedge clk) begin : proc_mfu_capture_assertions
    if (!reset) begin
`ifdef ENABLE_CNOC_MFU
      if (mfu_capture && (mfu_capture_out_sel == PORT_SEL_INVALID)) begin
        $error("Router: MFU reported capture with invalid output select");
      end
`endif
    end
  end
`endif

endmodule
