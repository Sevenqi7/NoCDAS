// Description: Five-port router used by the NoCDAS/Verilator co-simulation.
//              The module owns input VC buffering, route computation, pipelined
//              VC/switch allocation, downstream commit, and the optional cNoC
//              MFU side path. Link traversal and neighboring-router injection
//              are still modeled by the C++ co-simulation wrapper.

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
  localparam int VN_NUM_LOCAL = 2;
  localparam int VC_PER_VNET = VC_NUM / VN_NUM_LOCAL;
  localparam int VNET_URS = 0;
  localparam int VNET_LCS = 1;
  localparam int TRAFFIC_CLASS_NUM = 3;
  localparam int STARVATION_LIMIT = 20;

  typedef struct packed {
    logic valid;
    logic [FLIT_W-1:0] flit;
    flit_meta_t meta;
    route_path_t route_path;
    logic [2:0] src_port;
    logic [VC_ID_W-1:0] src_vc;
    logic [2:0] route_sel;
    logic [VC_ID_W-1:0] dst_vc;
    router_traffic_class_e traffic_class;
    logic head_like;
    logic tail_like;
    logic may_need_mfu;
    logic reserved_vc;
  } router_pipe_entry_t;

  rinport_data_t in_data [NUM_PORTS];
  rinport_flow_t in_flow [NUM_PORTS];
  rinport_ctrl_t rin_ctrl [NUM_PORTS];
  rinport_issue_t rin_issue [NUM_PORTS];
  routport_flow_t out_flow [NUM_PORTS];
  routport_data_t out_data [NUM_PORTS];
  route_path_t rinport_route_path [NUM_PORTS];
  router_traffic_class_e rinport_traffic_class [NUM_PORTS];

  router_pipe_entry_t issue_q [NUM_PORTS];
  router_pipe_entry_t issue_d [NUM_PORTS];
  router_pipe_entry_t va_q [NUM_PORTS];
  router_pipe_entry_t va_d [NUM_PORTS];
  router_pipe_entry_t out_q [NUM_PORTS];
  router_pipe_entry_t out_d [NUM_PORTS];
  router_pipe_entry_t commit_issue_d [NUM_PORTS];
  router_pipe_entry_t commit_va_d [NUM_PORTS];
  router_pipe_entry_t commit_out_d [NUM_PORTS];
  router_pipe_entry_t sa_issue_d [NUM_PORTS];
  router_pipe_entry_t sa_va_d [NUM_PORTS];
  router_pipe_entry_t sa_out_d [NUM_PORTS];
  router_pipe_entry_t va_issue_d [NUM_PORTS];
  router_pipe_entry_t va_va_d [NUM_PORTS];
  router_pipe_entry_t va_out_d [NUM_PORTS];

  logic [NUM_PORTS-1:0] issue_to_va_fire;
  logic [NUM_PORTS-1:0] issue_accept_fire;
  logic [NUM_PORTS-1:0] va_to_out_fire;
  logic [NUM_PORTS-1:0] output_commit_fire;
  logic [NUM_PORTS-1:0] output_mfu_selected;
  logic [NUM_PORTS-1:0] output_regular_commit;
  logic [NUM_PORTS-1:0] output_ready_for_entry;
  logic [NUM_PORTS-1:0] output_entry_needs_mfu;
  logic [NUM_PORTS-1:0] source_commit_used;

  logic alloc_valid_q [NUM_PORTS][VC_NUM];
  logic alloc_valid_d [NUM_PORTS][VC_NUM];
  logic alloc_valid_commit_d [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_q [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_d [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_commit_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_q [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_commit_d [NUM_PORTS][VC_NUM];
  logic reserved_vc_q [NUM_PORTS][VC_NUM];
  logic reserved_vc_d [NUM_PORTS][VC_NUM];
  logic reserved_vc_commit_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] vc_rr_ptr_q [NUM_PORTS][VN_NUM_LOCAL];
  logic [VC_ID_W-1:0] vc_rr_ptr_d [NUM_PORTS][VN_NUM_LOCAL];
  logic [2:0] switch_rr_ptr_q [NUM_PORTS];
  logic [2:0] switch_rr_ptr_d [NUM_PORTS];
  logic [2:0] switch_rr_ptr_sa_d [NUM_PORTS];
  logic [7:0] regular_starve_cnt_q [NUM_PORTS];
  logic [7:0] regular_starve_cnt_d [NUM_PORTS];
  logic [7:0] regular_starve_cnt_sa_d [NUM_PORTS];
  rinport_ctrl_t rin_ctrl_commit [NUM_PORTS];

  logic [FLIT_W-1:0] raw_flit [NUM_PORTS];
  flit_meta_t raw_meta [NUM_PORTS];
  route_path_t raw_route_path [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_vc_id [NUM_PORTS];
  logic [2:0] raw_stream_port [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_stream_vc [NUM_PORTS];
  logic [NUM_PORTS-1:0] raw_write;
  flit_meta_t output_meta_next [NUM_PORTS];
  route_path_t output_route_path_next [NUM_PORTS];

`ifdef ENABLE_CNOC_MFU
  logic [NUM_PORTS-1:0] raw_needs_mfu;
  logic [NUM_PORTS-1:0] raw_msg_is_dist;
  logic [NUM_PORTS-1:0] raw_msg_is_comp;
  logic [NUM_PORTS-1:0] raw_type4_at_dest;
  logic [NUM_PORTS-1:0] mfu_holds_routport;
  logic [NUM_PORTS-1:0] mfu_rinport_req_raw;
  logic [NUM_PORTS-1:0] mfu_rinport_req;
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
  logic [NUM_PORTS-1:0] unused_mfu_rinport_req_raw;
  flit_meta_t active_flit_meta;
  localparam logic [2:0] active_stream_port = ROUTER_PORT_INV;
  localparam logic [VC_ID_W-1:0] active_stream_vc = '0;
  localparam logic active_flit_valid = 1'b0;
  localparam logic [2:0] active_out_sel = ROUTER_PORT_INV;
  localparam logic [VC_ID_W-1:0] active_vc_id = '0;
  localparam logic [NUM_PORTS-1:0] mfu_holds_routport = '0;
  localparam logic [NUM_PORTS-1:0] mfu_rinport_req = '0;
  localparam logic mfu_busy = 1'b0;
  localparam logic mfu_capture = 1'b0;
  localparam logic [2:0] mfu_capture_out_sel = ROUTER_PORT_INV;
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
          .rinport_channel_i(rinport_gen_idx[2:0]),
          .rinport_raw_meta_o(),
          .rinport_route_path_o(rinport_route_path[rinport_gen_idx]),
`ifdef ENABLE_CNOC_MFU
          .rinport_may_need_mfu_o(mfu_rinport_req_raw[rinport_gen_idx]),
`else
          .rinport_may_need_mfu_o(unused_mfu_rinport_req_raw[rinport_gen_idx]),
`endif
          .rinport_traffic_class_o(rinport_traffic_class[rinport_gen_idx]),
          .rinport_issue_o(rin_issue[rinport_gen_idx]),
          .rinport_flow_o(in_flow[rinport_gen_idx])
      );
    end
  endgenerate

`ifdef ENABLE_CNOC_MFU
  always_comb begin : proc_mfu_hold_decode
    mfu_holds_routport = '0;
    if (mfu_busy && (mfu_emit_out_sel < NUM_PORTS)) begin
      mfu_holds_routport[mfu_emit_out_sel] = 1'b1;
    end
    mfu_rinport_req = mfu_rinport_req_raw;
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
    active_flit_meta = '0;
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

  always_comb begin : proc_output_data
    source_commit_used = '0;
    output_commit_fire = '0;
    output_mfu_selected = '0;
    output_regular_commit = '0;
    output_ready_for_entry = '0;
    output_entry_needs_mfu = '0;
    raw_write = '0;

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      out_data[out_idx] = '0;
      raw_flit[out_idx] = '0;
      raw_meta[out_idx] = '0;
      raw_route_path[out_idx] = '0;
      raw_vc_id[out_idx] = '0;
      raw_stream_port[out_idx] = ROUTER_PORT_INV;
      raw_stream_vc[out_idx] = '0;
      output_meta_next[out_idx] = out_q[out_idx].meta;
      output_route_path_next[out_idx] = out_q[out_idx].route_path;

      if (((out_q[out_idx].meta.msg_type == ROUTER_MSG_DIST) ||
           (out_q[out_idx].meta.msg_type == ROUTER_MSG_COMP)) &&
          out_q[out_idx].valid &&
          out_q[out_idx].meta.route_valid &&
          out_q[out_idx].route_path.valid &&
          (out_q[out_idx].route_path.route_ptr < out_q[out_idx].route_path.route_len)) begin
        output_meta_next[out_idx].route_ptr = out_q[out_idx].meta.route_ptr + 8'd1;
        output_route_path_next[out_idx].route_ptr = out_q[out_idx].route_path.route_ptr + 8'd1;
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (out_q[out_idx].valid &&
          out_flow[out_idx].state_ready &&
          out_flow[out_idx].downstream_vc_credit_mask[out_q[out_idx].dst_vc] &&
          !mfu_holds_routport[out_idx]) begin
        output_ready_for_entry[out_idx] = 1'b1;
      end

`ifdef ENABLE_CNOC_MFU
      if (out_q[out_idx].valid) begin
        output_entry_needs_mfu[out_idx] =
            ((out_q[out_idx].meta.msg_type == ROUTER_MSG_COMP) &&
             out_q[out_idx].meta.process) ||
            ((out_q[out_idx].meta.msg_type == ROUTER_MSG_DIST) &&
             (out_q[out_idx].meta.process ||
              ((out_q[out_idx].meta.dst_x == X_cur) &&
               (out_q[out_idx].meta.dst_y == Y_cur))));
      end
`else
      output_entry_needs_mfu[out_idx] = 1'b0;
`endif
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_ready_for_entry[out_idx] &&
          !source_commit_used[out_q[out_idx].src_port]) begin
`ifdef ENABLE_CNOC_MFU
        if (output_entry_needs_mfu[out_idx] && !mfu_busy && (output_mfu_selected == '0)) begin
          output_mfu_selected[out_idx] = 1'b1;
          output_commit_fire[out_idx] = 1'b1;
          source_commit_used[out_q[out_idx].src_port] = 1'b1;
          raw_write[out_idx] = 1'b1;
        end else if (!output_entry_needs_mfu[out_idx] &&
                     !(mfu_emit_valid && (mfu_emit_out_sel == 3'(out_idx)))) begin
          output_regular_commit[out_idx] = 1'b1;
          output_commit_fire[out_idx] = 1'b1;
          source_commit_used[out_q[out_idx].src_port] = 1'b1;
          raw_write[out_idx] = 1'b1;
        end
`else
        output_regular_commit[out_idx] = 1'b1;
        output_commit_fire[out_idx] = 1'b1;
        source_commit_used[out_q[out_idx].src_port] = 1'b1;
        raw_write[out_idx] = 1'b1;
`endif
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (raw_write[out_idx]) begin
        raw_flit[out_idx] = out_q[out_idx].flit;
        raw_meta[out_idx] = output_meta_next[out_idx];
        raw_route_path[out_idx] = output_route_path_next[out_idx];
        raw_vc_id[out_idx] = out_q[out_idx].dst_vc;
        raw_stream_port[out_idx] = out_q[out_idx].src_port;
        raw_stream_vc[out_idx] = out_q[out_idx].src_vc;
      end

      if (output_regular_commit[out_idx]) begin
        out_data[out_idx].flit = out_q[out_idx].flit;
        out_data[out_idx].meta = output_meta_next[out_idx];
        out_data[out_idx].route_path = output_route_path_next[out_idx];
        out_data[out_idx].vc_id = out_q[out_idx].dst_vc;
        out_data[out_idx].write_req = 1'b1;
      end
    end

`ifdef ENABLE_CNOC_MFU
    if (mfu_emit_valid && (mfu_emit_out_sel < NUM_PORTS)) begin
      out_data[mfu_emit_out_sel].flit = mfu_emit_flit;
      out_data[mfu_emit_out_sel].meta = mfu_emit_meta;
      out_data[mfu_emit_out_sel].route_path = mfu_emit_route_path;
      out_data[mfu_emit_out_sel].vc_id = mfu_emit_vc_id;
      out_data[mfu_emit_out_sel].write_req = 1'b1;
    end
`endif
  end

  always_comb begin : proc_commit_next
    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      commit_issue_d[port_idx] = issue_q[port_idx];
      commit_va_d[port_idx] = va_q[port_idx];
      commit_out_d[port_idx] = out_q[port_idx];
      rin_ctrl_commit[port_idx] = '0;
      rin_ctrl_commit[port_idx].x_cur = X_cur;
      rin_ctrl_commit[port_idx].y_cur = Y_cur;
      rin_ctrl_commit[port_idx].in_channel = 3'(port_idx);
    end

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        alloc_valid_commit_d[in_idx][vc_idx] = alloc_valid_q[in_idx][vc_idx];
        alloc_out_port_commit_d[in_idx][vc_idx] = alloc_out_port_q[in_idx][vc_idx];
        alloc_dst_vc_commit_d[in_idx][vc_idx] = alloc_dst_vc_q[in_idx][vc_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        reserved_vc_commit_d[out_idx][vc_idx] = reserved_vc_q[out_idx][vc_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_commit_fire[out_idx]) begin
        commit_out_d[out_idx] = '0;
        rin_ctrl_commit[out_q[out_idx].src_port].commit_valid = 1'b1;
        rin_ctrl_commit[out_q[out_idx].src_port].commit_vc = out_q[out_idx].src_vc;
        rin_ctrl_commit[out_q[out_idx].src_port].commit_tail_like = out_q[out_idx].tail_like;

        if (out_q[out_idx].reserved_vc && !output_mfu_selected[out_idx]) begin
          reserved_vc_commit_d[out_idx][out_q[out_idx].dst_vc] = 1'b0;
        end

        if (out_q[out_idx].head_like && !out_q[out_idx].tail_like) begin
          alloc_valid_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] = 1'b1;
          alloc_out_port_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] =
              3'(out_idx);
          alloc_dst_vc_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] =
              out_q[out_idx].dst_vc;
        end

        if (out_q[out_idx].tail_like) begin
          alloc_valid_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] = 1'b0;
          alloc_out_port_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] =
              PORT_SEL_INVALID;
          alloc_dst_vc_commit_d[out_q[out_idx].src_port][out_q[out_idx].src_vc] = '0;
        end
      end
    end

`ifdef ENABLE_CNOC_MFU
    if (mfu_emit_valid &&
        (mfu_emit_out_sel < NUM_PORTS) &&
        flit_is_head_like(mfu_emit_meta.flit_kind)) begin
      reserved_vc_commit_d[mfu_emit_out_sel][mfu_emit_vc_id] = 1'b0;
    end
`endif
  end

  always_comb begin : proc_switch_select
    router_traffic_class_e selected_class;
    logic [2:0] selected_winner;
    logic selected_found_winner;
    logic selected_force_regular;
    logic selected_regular_waiting;
    int selected_candidate_port;

    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      sa_issue_d[port_idx] = commit_issue_d[port_idx];
      sa_va_d[port_idx] = commit_va_d[port_idx];
      sa_out_d[port_idx] = commit_out_d[port_idx];
      va_to_out_fire[port_idx] = 1'b0;
      switch_rr_ptr_sa_d[port_idx] = switch_rr_ptr_q[port_idx];
      regular_starve_cnt_sa_d[port_idx] = regular_starve_cnt_q[port_idx];
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (!out_q[out_idx].valid || output_commit_fire[out_idx]) begin
        selected_winner = PORT_SEL_INVALID;
        selected_found_winner = 1'b0;
        selected_regular_waiting = 1'b0;
        selected_force_regular = 1'b0;

        for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
          if (va_q[in_idx].valid &&
              (va_q[in_idx].route_sel == 3'(out_idx)) &&
              (va_q[in_idx].traffic_class == ROUTER_TRAFFIC_REGULAR)) begin
            selected_regular_waiting = 1'b1;
          end
        end

        if (selected_regular_waiting &&
            (regular_starve_cnt_q[out_idx] >= STARVATION_LIMIT[7:0])) begin
          selected_force_regular = 1'b1;
        end

        for (int prio_idx = 0; prio_idx < TRAFFIC_CLASS_NUM; prio_idx = prio_idx + 1) begin
          selected_class = ROUTER_TRAFFIC_REGULAR;
          if (selected_force_regular) begin
            case (prio_idx[1:0])
              2'd0: selected_class = ROUTER_TRAFFIC_REGULAR;
              2'd1: selected_class = ROUTER_TRAFFIC_COMP;
              default: selected_class = ROUTER_TRAFFIC_DIST;
            endcase
          end else begin
            case (prio_idx[1:0])
              2'd0: selected_class = ROUTER_TRAFFIC_COMP;
              2'd1: selected_class = ROUTER_TRAFFIC_DIST;
              default: selected_class = ROUTER_TRAFFIC_REGULAR;
            endcase
          end

          for (int rr_iter = 0; rr_iter < NUM_PORTS; rr_iter = rr_iter + 1) begin
            selected_candidate_port = int'(switch_rr_ptr_q[out_idx]) + rr_iter;
            if (selected_candidate_port >= NUM_PORTS) begin
              selected_candidate_port = selected_candidate_port - NUM_PORTS;
            end

            if (!selected_found_winner &&
                commit_va_d[selected_candidate_port].valid &&
                (commit_va_d[selected_candidate_port].route_sel == 3'(out_idx)) &&
                (commit_va_d[selected_candidate_port].traffic_class == selected_class)) begin
              selected_winner = 3'(selected_candidate_port);
              selected_found_winner = 1'b1;
            end
          end
        end

        if (selected_winner != PORT_SEL_INVALID) begin
          sa_out_d[out_idx] = commit_va_d[selected_winner];
          sa_va_d[selected_winner] = '0;
          va_to_out_fire[selected_winner] = 1'b1;

          switch_rr_ptr_sa_d[out_idx] =
              (selected_winner == 3'(NUM_PORTS - 1)) ? '0 : (selected_winner + 3'd1);

          if (commit_va_d[selected_winner].traffic_class == ROUTER_TRAFFIC_REGULAR) begin
            regular_starve_cnt_sa_d[out_idx] = 8'd0;
          end else if (selected_regular_waiting &&
                       (regular_starve_cnt_q[out_idx] < STARVATION_LIMIT[7:0])) begin
            regular_starve_cnt_sa_d[out_idx] = regular_starve_cnt_q[out_idx] + 8'd1;
          end
        end else if (!selected_regular_waiting) begin
          regular_starve_cnt_sa_d[out_idx] = 8'd0;
        end
      end
    end
  end

  always_comb begin : proc_vc_alloc_next
    logic selected_head_ready;
    logic selected_body_ready;
    logic selected_va_ready;
    logic selected_found_vc;
    logic [1:0] selected_vnet;
    logic selected_vnet_idx;
    logic [VC_NUM-1:0] selected_vnet_mask;
    logic [VC_NUM-1:0] selected_alloc_mask;
    logic [VC_ID_W-1:0] selected_dst_vc;
    int selected_vnet_base;
    int selected_vnet_last;
    int selected_start_offset;
    int selected_candidate_offset;
    int selected_candidate_vc;

    selected_head_ready = 1'b0;
    selected_body_ready = 1'b0;
    selected_va_ready = 1'b0;
    selected_found_vc = 1'b0;
    selected_vnet = 2'(VNET_URS);
    selected_vnet_idx = 1'b0;
    selected_vnet_mask = '0;
    selected_alloc_mask = '0;
    selected_dst_vc = '0;
    selected_vnet_base = 0;
    selected_vnet_last = 0;
    selected_start_offset = 0;
    selected_candidate_offset = 0;
    selected_candidate_vc = 0;

    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      va_issue_d[port_idx] = sa_issue_d[port_idx];
      va_va_d[port_idx] = sa_va_d[port_idx];
      va_out_d[port_idx] = sa_out_d[port_idx];
      issue_to_va_fire[port_idx] = 1'b0;
      switch_rr_ptr_d[port_idx] = switch_rr_ptr_sa_d[port_idx];
      regular_starve_cnt_d[port_idx] = regular_starve_cnt_sa_d[port_idx];
    end

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        alloc_valid_d[in_idx][vc_idx] = alloc_valid_commit_d[in_idx][vc_idx];
        alloc_out_port_d[in_idx][vc_idx] = alloc_out_port_commit_d[in_idx][vc_idx];
        alloc_dst_vc_d[in_idx][vc_idx] = alloc_dst_vc_commit_d[in_idx][vc_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        reserved_vc_d[out_idx][vc_idx] = reserved_vc_commit_d[out_idx][vc_idx];
      end
      for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
        vc_rr_ptr_d[out_idx][vnet_idx] = vc_rr_ptr_q[out_idx][vnet_idx];
      end
    end

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      if (sa_issue_d[in_idx].valid && (!sa_va_d[in_idx].valid || va_to_out_fire[in_idx])) begin
        selected_head_ready = 1'b0;
        selected_body_ready = 1'b0;
        selected_va_ready = 1'b0;
        selected_found_vc = 1'b0;
        selected_vnet = 2'(VNET_URS);
        selected_vnet_idx = 1'b0;
        selected_vnet_mask = '0;
        selected_alloc_mask = '0;
        selected_dst_vc = '0;
        selected_vnet_base = 0;
        selected_vnet_last = 0;
        selected_start_offset = 0;
        selected_candidate_offset = 0;
        selected_candidate_vc = 0;

`ifdef ENABLE_CNOC_MFU
        if ((sa_issue_d[in_idx].traffic_class == ROUTER_TRAFFIC_COMP) ||
            (sa_issue_d[in_idx].traffic_class == ROUTER_TRAFFIC_DIST)) begin
          selected_vnet = 2'(VNET_LCS);
        end else if (sa_issue_d[in_idx].meta.vnet < VN_NUM_LOCAL) begin
          selected_vnet = sa_issue_d[in_idx].meta.vnet[1:0];
        end
`else
        if (sa_issue_d[in_idx].meta.vnet < VN_NUM_LOCAL) begin
          selected_vnet = sa_issue_d[in_idx].meta.vnet[1:0];
        end
`endif

        selected_vnet_idx = selected_vnet[0];
        selected_vnet_base = int'(selected_vnet) * VC_PER_VNET;
        selected_vnet_last = selected_vnet_base + VC_PER_VNET - 1;
        for (int vc_offset = 0; vc_offset < VC_PER_VNET; vc_offset = vc_offset + 1) begin
          if ((selected_vnet_base + vc_offset) < VC_NUM) begin
            selected_vnet_mask[selected_vnet_base + vc_offset] = 1'b1;
          end
        end

        if (sa_issue_d[in_idx].route_sel < NUM_PORTS) begin
          if (sa_issue_d[in_idx].head_like) begin
            selected_alloc_mask =
                out_flow[sa_issue_d[in_idx].route_sel].downstream_vc_idle_mask &
                out_flow[sa_issue_d[in_idx].route_sel].downstream_vc_credit_mask &
                selected_vnet_mask;

            for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
              if (reserved_vc_d[sa_issue_d[in_idx].route_sel][vc_idx]) begin
                selected_alloc_mask[vc_idx] = 1'b0;
              end
            end

            selected_start_offset =
                int'(vc_rr_ptr_q[sa_issue_d[in_idx].route_sel][selected_vnet_idx]) -
                selected_vnet_base;
            if ((selected_start_offset < 0) || (selected_start_offset >= VC_PER_VNET)) begin
              selected_start_offset = 0;
            end

            for (int vc_iter = 0; vc_iter < VC_PER_VNET; vc_iter = vc_iter + 1) begin
              selected_candidate_offset = selected_start_offset + vc_iter;
              if (selected_candidate_offset >= VC_PER_VNET) begin
                selected_candidate_offset = selected_candidate_offset - VC_PER_VNET;
              end
              selected_candidate_vc = selected_vnet_base + selected_candidate_offset;
              if (!selected_found_vc &&
                  (selected_candidate_vc < VC_NUM) &&
                  selected_alloc_mask[selected_candidate_vc]) begin
                selected_dst_vc = VC_ID_W'(selected_candidate_vc);
                selected_found_vc = 1'b1;
              end
            end

            selected_head_ready = selected_found_vc;
            selected_va_ready = selected_head_ready;
          end else begin
            selected_body_ready =
                alloc_valid_q[sa_issue_d[in_idx].src_port][sa_issue_d[in_idx].src_vc] &&
                (alloc_out_port_q[sa_issue_d[in_idx].src_port][sa_issue_d[in_idx].src_vc] ==
                 sa_issue_d[in_idx].route_sel);
            selected_dst_vc =
                alloc_dst_vc_q[sa_issue_d[in_idx].src_port][sa_issue_d[in_idx].src_vc];
            selected_va_ready = selected_body_ready;
          end
        end

        if (selected_va_ready) begin
          va_va_d[in_idx] = sa_issue_d[in_idx];
          va_va_d[in_idx].dst_vc = selected_dst_vc;
          va_va_d[in_idx].reserved_vc = sa_issue_d[in_idx].head_like;
          va_issue_d[in_idx] = '0;
          issue_to_va_fire[in_idx] = 1'b1;

          if (sa_issue_d[in_idx].head_like) begin
            reserved_vc_d[sa_issue_d[in_idx].route_sel][selected_dst_vc] = 1'b1;
            if (int'(selected_dst_vc) >= selected_vnet_last) begin
              vc_rr_ptr_d[sa_issue_d[in_idx].route_sel][selected_vnet_idx] =
                  VC_ID_W'(selected_vnet_base);
            end else begin
              vc_rr_ptr_d[sa_issue_d[in_idx].route_sel][selected_vnet_idx] =
                  selected_dst_vc + {{(VC_ID_W-1){1'b0}}, 1'b1};
            end
          end
        end
      end
    end
  end

  always_comb begin : proc_issue_accept
    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      issue_d[port_idx] = va_issue_d[port_idx];
      va_d[port_idx] = va_va_d[port_idx];
      out_d[port_idx] = va_out_d[port_idx];
      issue_accept_fire[port_idx] = 1'b0;
      rin_ctrl[port_idx] = rin_ctrl_commit[port_idx];
    end

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      if (rin_issue[in_idx].valid && (!issue_q[in_idx].valid || issue_to_va_fire[in_idx])) begin
        issue_accept_fire[in_idx] = 1'b1;
        rin_ctrl[in_idx].issue_accept = 1'b1;
        issue_d[in_idx] = '0;
        issue_d[in_idx].valid = 1'b1;
        issue_d[in_idx].flit = rin_issue[in_idx].flit;
        issue_d[in_idx].meta = rin_issue[in_idx].meta;
        issue_d[in_idx].route_path = rinport_route_path[in_idx];
        issue_d[in_idx].src_port = 3'(in_idx);
        issue_d[in_idx].src_vc = rin_issue[in_idx].sel_vc;
        issue_d[in_idx].route_sel = rin_issue[in_idx].route_sel;
        issue_d[in_idx].traffic_class = rinport_traffic_class[in_idx];
        issue_d[in_idx].head_like = flit_is_head_like(rin_issue[in_idx].meta.flit_kind);
        issue_d[in_idx].tail_like = flit_is_tail_like(rin_issue[in_idx].meta.flit_kind);
        issue_d[in_idx].may_need_mfu = mfu_rinport_req[in_idx];
      end
    end
  end

  always_ff @(posedge clk) begin : proc_registers
    if (reset) begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        issue_q[port_idx] <= '0;
        va_q[port_idx] <= '0;
        out_q[port_idx] <= '0;
        switch_rr_ptr_q[port_idx] <= 3'(port_idx);
        regular_starve_cnt_q[port_idx] <= '0;
      end

      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= 1'b0;
          alloc_out_port_q[in_idx][vc_idx] <= PORT_SEL_INVALID;
          alloc_dst_vc_q[in_idx][vc_idx] <= '0;
        end
      end

      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          reserved_vc_q[out_idx][vc_idx] <= 1'b0;
        end
        for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= VC_ID_W'(vnet_idx * VC_PER_VNET);
        end
      end
    end else begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        issue_q[port_idx] <= issue_d[port_idx];
        va_q[port_idx] <= va_d[port_idx];
        out_q[port_idx] <= out_d[port_idx];
        switch_rr_ptr_q[port_idx] <= switch_rr_ptr_d[port_idx];
        regular_starve_cnt_q[port_idx] <= regular_starve_cnt_d[port_idx];
      end

      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= alloc_valid_d[in_idx][vc_idx];
          alloc_out_port_q[in_idx][vc_idx] <= alloc_out_port_d[in_idx][vc_idx];
          alloc_dst_vc_q[in_idx][vc_idx] <= alloc_dst_vc_d[in_idx][vc_idx];
        end
      end

      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          reserved_vc_q[out_idx][vc_idx] <= reserved_vc_d[out_idx][vc_idx];
        end
        for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= vc_rr_ptr_d[out_idx][vnet_idx];
        end
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_pipeline_assertions
    if (!reset) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        if (output_commit_fire[out_idx] && !out_q[out_idx].valid) begin
          $error("Router: committed an empty output pipeline slot");
        end
      end
    end
  end
`endif

endmodule
