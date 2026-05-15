// Description: Five-port router used by the NoCDAS/Verilator co-simulation.
//              The module owns input VC buffering, route computation, switch
//              allocation, downstream VC allocation, the crossbar datapath, and
//              the cNoC MFU side path.  Link traversal and neighboring-router
//              injection are still modeled by the C++ co-simulation wrapper.

module Router #(
    parameter integer VC_NUM = 8,
    parameter integer VC_ID_W = 3,
    parameter integer FLIT_W = 256,
    parameter integer MFU_LAT_LINEAR = 0,
    parameter integer MFU_LAT_MATMUL = 0,
    parameter integer MFU_LAT_ADD = 0,
    parameter integer MFU_LAT_SWIGLU = 0,
    parameter integer MFU_LAT_GEGLU = 0,
    parameter integer MFU_LAT_ATTENTION = 8,
    parameter integer MFU_LAT_DEFAULT = 0,
    parameter integer MFU_LAT_TYPE4_STORE = 0
)(
    input logic [2:0] X_cur, Y_cur,
    input logic clk, reset,
    input logic [5:0] cnoc_task_count,
    input logic [router_ports_pkg::CNOC_MAX_TASKS*
                 router_ports_pkg::CNOC_TASK_ID_W-1:0] cnoc_task_ids_flat,
    input logic [15:0] cnoc_weight_row_size,

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
    output router_ports_pkg::routport_data_t local_output_data,

    // Debug outputs for cNoC state tracking.  
    output logic [31:0] cnoc_weight_bytes_stored,
    output logic [31:0] cnoc_kv_bytes_stored,
    output logic [15:0] cnoc_kv_token_count,
    output logic        cnoc_last_type4_store_valid,
    output logic        cnoc_last_type4_store_bank,
    output logic [10:0] cnoc_last_type4_store_addr,
    output logic [5:0]  cnoc_last_type4_store_bytes
);
  import router_ports_pkg::*;

  // cNoC control path (type4/type5) is progressively moved into RTL.
  // Legacy packets (type0/type1/type2/type3) keep the forwarding datapath.
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

  // ---------------------------------------------------
  // Local semantic constants (enum-style, Verilog-safe)
  // ---------------------------------------------------
  localparam integer NUM_PORTS = PORT_NUM;
  localparam router_port_sel_e PORT_SEL_EAST = ROUTER_PORT_EAST;
  localparam router_port_sel_e PORT_SEL_WEST = ROUTER_PORT_WEST;
  localparam router_port_sel_e PORT_SEL_NORTH = ROUTER_PORT_NORTH;
  localparam router_port_sel_e PORT_SEL_SOUTH = ROUTER_PORT_SOUTH;
  localparam router_port_sel_e PORT_SEL_LOCAL = ROUTER_PORT_LOCAL;

  // -----------------------------------------
  // Top-level channel bundles
  // -----------------------------------------
  rinport_data_t in_data [NUM_PORTS];
  rinport_flow_t in_flow [NUM_PORTS];
  rinport_ctrl_t rin_ctrl [NUM_PORTS];
  rinport_issue_t rin_issue [NUM_PORTS];
  routport_flow_t out_flow [NUM_PORTS];
  routport_data_t out_data [NUM_PORTS];
  logic [NUM_PORTS-1:0] routport_ready;
  logic [NUM_PORTS-1:0] rinport_req;
  logic [2:0] rinport_route_sel [NUM_PORTS];
  router_traffic_class_e rinport_traffic_class [NUM_PORTS];
  logic [NUM_PORTS-1:0] rinport_grant;
  flit_meta_t iu_meta_filtered [NUM_PORTS];
  route_path_t rinport_route_path [NUM_PORTS];

  // -----------------------------------------
  // Input stage / crossbar datapath
  // -----------------------------------------
  wire [FLIT_W-1:0] iu_flit_e, iu_flit_w, iu_flit_n, iu_flit_s, iu_flit_j;
  flit_meta_t iu_meta_e, iu_meta_w, iu_meta_n, iu_meta_s, iu_meta_j;
  wire [FLIT_W-1:0] xbar_flit_e, xbar_flit_w, xbar_flit_n, xbar_flit_s, xbar_flit_j;
  flit_meta_t xbar_meta_e, xbar_meta_w, xbar_meta_n, xbar_meta_s, xbar_meta_j;
  logic [FLIT_W-1:0] xbar_flit [NUM_PORTS];
  flit_meta_t xbar_meta [NUM_PORTS];
  route_path_t xbar_route_path [NUM_PORTS];

  logic [2:0] xbar_sel_e, xbar_sel_w, xbar_sel_n, xbar_sel_s, xbar_sel_j;

  // Router-owned downstream VC allocation is isolated in vc_allocator.
  logic [NUM_PORTS-1:0] vc_route_ready [NUM_PORTS];
  logic [VC_ID_W-1:0] vc_alloc_dst_vc [NUM_PORTS];
  flit_meta_t vc_alloc_meta [NUM_PORTS];
  logic [VC_ID_W-1:0] vc_alloc_src_vc [NUM_PORTS];
  logic [2:0] routport_winner [NUM_PORTS];

  // -----------------------------------------
  // VC IDs and raw output-stage signals
  // -----------------------------------------
  logic [FLIT_W-1:0] raw_flit [NUM_PORTS];
  flit_meta_t raw_meta [NUM_PORTS];
  route_path_t raw_route_path [NUM_PORTS];
  logic [VC_ID_W-1:0] raw_vc_id [NUM_PORTS];
  logic [NUM_PORTS-1:0] raw_write;
  logic [NUM_PORTS-1:0] raw_needs_mfu;
  logic [NUM_PORTS-1:0] raw_msg_is_dist;
  logic [NUM_PORTS-1:0] raw_msg_is_comp;
  logic [NUM_PORTS-1:0] raw_type4_at_dest;

  // -----------------------------------------
  // cNoC selected flit + MFU path
  // -----------------------------------------
  reg [FLIT_W-1:0] active_flit_word;
  flit_meta_t active_flit_meta;
  route_path_t active_route_path;
  reg [VC_ID_W-1:0] active_vc_id;
  reg [2:0] active_out_sel;
  reg active_flit_valid;

  wire mfu_busy;
  wire mfu_capture;
  wire [2:0] mfu_capture_out_sel;
  wire mfu_emit_valid;
  wire [2:0] mfu_emit_out_sel;
  wire [FLIT_W-1:0] mfu_emit_flit;
  flit_meta_t mfu_emit_meta;
  route_path_t mfu_emit_route_path;
  wire [VC_ID_W-1:0] mfu_emit_vc_id;
  logic [NUM_PORTS-1:0] mfu_holds_routport;
  logic [NUM_PORTS-1:0] mfu_rinport_req_raw;
  logic [NUM_PORTS-1:0] mfu_rinport_req;

  always_comb begin : proc_raw_mfu_decode
    for (int raw_idx = 0; raw_idx < NUM_PORTS; raw_idx = raw_idx + 1) begin
      raw_msg_is_dist[raw_idx] = (raw_meta[raw_idx].msg_type == ROUTER_MSG_DIST);
      raw_msg_is_comp[raw_idx] = (raw_meta[raw_idx].msg_type == ROUTER_MSG_COMP);
      raw_type4_at_dest[raw_idx] =
          (raw_meta[raw_idx].dst_x == X_cur) && (raw_meta[raw_idx].dst_y == Y_cur);

      // Type5 compute packets enter the MFU only when the current source-route
      // hop's process mask is set.  Source-route revisits carry process=0 and
      // bypass the MFU without requiring a router-local associative table.
      raw_needs_mfu[raw_idx] =
          (raw_msg_is_comp[raw_idx] && raw_meta[raw_idx].process) ||
          // Type4 distribution packets enter the MFU either when explicitly
          // marked for processing or when they reach their destination router,
          // because the destination may be the final storage target.
          (raw_msg_is_dist[raw_idx] &&
           (raw_meta[raw_idx].process || raw_type4_at_dest[raw_idx]));
    end
  end

  always_comb begin
    mfu_holds_routport = '0;
    if (mfu_busy && (mfu_emit_out_sel < NUM_PORTS)) begin
      // A flit captured by the MFU already consumed a downstream VC decision
      // before leaving the raw crossbar path.  Hold the selected output closed
      // to raw traffic until the MFU emits, otherwise a later bypass flit can
      // consume the same downstream C++ mirror VC and strand the MFU flit.
      mfu_holds_routport[mfu_emit_out_sel] = 1'b1;
    end
  end

  assign mfu_rinport_req = mfu_rinport_req_raw;

`ifdef RTL_DEBUG_MATMUL
  router_matmul_debug #(
      .NUM_PORTS(NUM_PORTS),
      .VC_ID_W(VC_ID_W)
  ) router_matmul_debug_i (
      .clk_i(clk),
      .reset_i(reset),
      .router_x_i(X_cur),
      .router_y_i(Y_cur),
      .rinport_req_i(rinport_req),
      .rinport_grant_i(rinport_grant),
      .rin_issue_i(rin_issue),
      .vc_alloc_meta_i(vc_alloc_meta),
      .mfu_rinport_req_raw_i(mfu_rinport_req_raw),
      .mfu_rinport_req_i(mfu_rinport_req),
      .raw_meta_i(raw_meta),
      .raw_write_i(raw_write),
      .raw_needs_mfu_i(raw_needs_mfu),
      .active_flit_meta_i(active_flit_meta),
      .active_vc_id_i(active_vc_id),
      .mfu_capture_i(mfu_capture),
      .mfu_capture_out_sel_i(mfu_capture_out_sel),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_vc_id_i(mfu_emit_vc_id)
  );
`endif

  mfu_arbiter #(
      .NUM_PORTS(NUM_PORTS),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) mfu_arbiter_i (
      .raw_flit_i(raw_flit),
      .raw_meta_i(raw_meta),
      .raw_route_path_i(raw_route_path),
      .raw_vc_id_i(raw_vc_id),
      .raw_valid_i(raw_write),
      .raw_needs_mfu_i(raw_needs_mfu),
      .mfu_flit_o(active_flit_word),
      .mfu_meta_o(active_flit_meta),
      .mfu_route_path_o(active_route_path),
      .mfu_vc_id_o(active_vc_id),
      .mfu_out_sel_o(active_out_sel),
      .mfu_valid_o(active_flit_valid)
  );

  // -----------------------------------------
  // Directional bundled ports <-> internal channel arrays
  // -----------------------------------------
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

  always_comb begin
    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      vc_alloc_src_vc[port_idx] = rin_issue[port_idx].sel_vc;
      rinport_req[port_idx] = rin_issue[port_idx].valid;
      rinport_route_sel[port_idx] = rin_issue[port_idx].route_sel;
      routport_ready[port_idx] = out_flow[port_idx].state_ready &&
                                 !mfu_holds_routport[port_idx];

      rin_ctrl[port_idx] = '0;
      rin_ctrl[port_idx].vc_grant = rinport_grant[port_idx];
    end
  end

  vc_allocator #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W)
  ) vc_allocator_i (
      .clk(clk),
      .reset(reset),
      .rinport_meta_i(vc_alloc_meta),
      .rinport_vc_id_i(vc_alloc_src_vc),
      .routport_flow_i(out_flow),
      .routport_winner_i(routport_winner),
      .route_vc_ready_o(vc_route_ready),
      .routport_vc_id_o(vc_alloc_dst_vc)
  );

  switch_allocator #(
      .NUM_PORTS(NUM_PORTS)
  ) switch_allocator_i (
      .clk_i(clk),
      .reset_i(reset),
      .routport_ready_i(routport_ready),
      .rinport_req_i(rinport_req),
      .rinport_route_sel_i(rinport_route_sel),
      .rinport_traffic_class_i(rinport_traffic_class),
      .route_vc_ready_i(vc_route_ready),
      .mfu_rinport_req_i(mfu_rinport_req),
      .mfu_busy_i(mfu_busy),
      .routport_winner_o(routport_winner),
      .rinport_grant_o(rinport_grant)
  );

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
          .rinport_raw_meta_o(vc_alloc_meta[rinport_gen_idx]),
          .rinport_route_path_o(rinport_route_path[rinport_gen_idx]),
          .rinport_may_need_mfu_o(mfu_rinport_req_raw[rinport_gen_idx]),
          .rinport_traffic_class_o(rinport_traffic_class[rinport_gen_idx]),
          .rinport_issue_o(rin_issue[rinport_gen_idx]),
          .rinport_flow_o(in_flow[rinport_gen_idx])
      );
    end
  endgenerate

  assign iu_flit_e = rin_issue[PORT_SEL_EAST].flit;
  assign iu_flit_w = rin_issue[PORT_SEL_WEST].flit;
  assign iu_flit_n = rin_issue[PORT_SEL_NORTH].flit;
  assign iu_flit_s = rin_issue[PORT_SEL_SOUTH].flit;
  assign iu_flit_j = rin_issue[PORT_SEL_LOCAL].flit;

  always_comb begin
    for (integer filter_idx = 0; filter_idx < NUM_PORTS; filter_idx = filter_idx + 1) begin
      iu_meta_filtered[filter_idx] = rin_issue[filter_idx].meta;
    end
  end

  assign iu_meta_e = iu_meta_filtered[PORT_SEL_EAST];
  assign iu_meta_w = iu_meta_filtered[PORT_SEL_WEST];
  assign iu_meta_n = iu_meta_filtered[PORT_SEL_NORTH];
  assign iu_meta_s = iu_meta_filtered[PORT_SEL_SOUTH];
  assign iu_meta_j = iu_meta_filtered[PORT_SEL_LOCAL];

  always @(*) begin
    // Crossbar select is exactly the switch allocator winner for each output.
    // A value of ROUTER_PORT_INV produces an idle raw_write below.
    xbar_sel_e = routport_winner[PORT_SEL_EAST];
    xbar_sel_w = routport_winner[PORT_SEL_WEST];
    xbar_sel_n = routport_winner[PORT_SEL_NORTH];
    xbar_sel_s = routport_winner[PORT_SEL_SOUTH];
    xbar_sel_j = routport_winner[PORT_SEL_LOCAL];
  end

  assign xbar_flit[PORT_SEL_EAST] = xbar_flit_e;
  assign xbar_flit[PORT_SEL_WEST] = xbar_flit_w;
  assign xbar_flit[PORT_SEL_NORTH] = xbar_flit_n;
  assign xbar_flit[PORT_SEL_SOUTH] = xbar_flit_s;
  assign xbar_flit[PORT_SEL_LOCAL] = xbar_flit_j;

  assign xbar_meta[PORT_SEL_EAST] = xbar_meta_e;
  assign xbar_meta[PORT_SEL_WEST] = xbar_meta_w;
  assign xbar_meta[PORT_SEL_NORTH] = xbar_meta_n;
  assign xbar_meta[PORT_SEL_SOUTH] = xbar_meta_s;
  assign xbar_meta[PORT_SEL_LOCAL] = xbar_meta_j;

  always_comb begin : proc_route_path_xbar
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      xbar_route_path[out_idx] = '0;
      if (routport_winner[out_idx] < NUM_PORTS) begin
        xbar_route_path[out_idx] = rinport_route_path[routport_winner[out_idx]];
      end
    end
  end

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

  cnoc_mfu #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .MFU_LAT_LINEAR(MFU_LAT_LINEAR),
      .MFU_LAT_MATMUL(MFU_LAT_MATMUL),
      .MFU_LAT_ADD(MFU_LAT_ADD),
      .MFU_LAT_SWIGLU(MFU_LAT_SWIGLU),
      .MFU_LAT_GEGLU(MFU_LAT_GEGLU),
      .MFU_LAT_ATTENTION(MFU_LAT_ATTENTION),
      .MFU_LAT_DEFAULT(MFU_LAT_DEFAULT),
      .MFU_LAT_TYPE4_STORE(MFU_LAT_TYPE4_STORE)
  ) cnoc_mfu_i (
      .clk(clk),
      .reset(reset),
      .raw_flit_i(active_flit_word),
      .raw_meta_i(active_flit_meta),
      .raw_route_path_i(active_route_path),
      .raw_vc_id_i(active_vc_id),
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
      .emit_vc_id_o(mfu_emit_vc_id),
      .weight_bytes_stored_o(cnoc_weight_bytes_stored),
      .kv_bytes_stored_o(cnoc_kv_bytes_stored),
      .kv_token_count_o(cnoc_kv_token_count),
      .last_type4_store_valid_o(cnoc_last_type4_store_valid),
      .last_type4_store_bank_o(cnoc_last_type4_store_bank),
      .last_type4_store_addr_o(cnoc_last_type4_store_addr),
      .last_type4_store_bytes_o(cnoc_last_type4_store_bytes)
  );

  router_output_stage #(
      .NUM_PORTS(NUM_PORTS),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) router_output_stage_i (
      .xbar_flit_i(xbar_flit),
      .xbar_meta_i(xbar_meta),
      .xbar_route_path_i(xbar_route_path),
      .routport_winner_i(routport_winner),
      .vc_alloc_dst_vc_i(vc_alloc_dst_vc),
      .mfu_capture_i(mfu_capture),
      .mfu_capture_out_sel_i(mfu_capture_out_sel),
      .mfu_emit_valid_i(mfu_emit_valid),
      .mfu_emit_out_sel_i(mfu_emit_out_sel),
      .mfu_emit_flit_i(mfu_emit_flit),
      .mfu_emit_meta_i(mfu_emit_meta),
      .mfu_emit_route_path_i(mfu_emit_route_path),
      .mfu_emit_vc_id_i(mfu_emit_vc_id),
      .raw_flit_o(raw_flit),
      .raw_meta_o(raw_meta),
      .raw_route_path_o(raw_route_path),
      .raw_vc_id_o(raw_vc_id),
      .raw_write_o(raw_write),
      .routport_data_o(out_data)
  );
  
  CrossBar #(.FLIT_W(FLIT_W)) X(
    .OE(xbar_flit_e), .OW(xbar_flit_w), .ON(xbar_flit_n), .OS(xbar_flit_s), .Eject(xbar_flit_j),
    .OE_META(xbar_meta_e), .OW_META(xbar_meta_w), .ON_META(xbar_meta_n), .OS_META(xbar_meta_s), .Eject_META(xbar_meta_j),
    .S_E(xbar_sel_e), .S_W(xbar_sel_w), .S_N(xbar_sel_n), .S_S(xbar_sel_s), .S_Ejec(xbar_sel_j),
    .IE(iu_flit_e), .IW(iu_flit_w), .IN(iu_flit_n), .IS(iu_flit_s), .Inject(iu_flit_j),
    .IE_META(iu_meta_e), .IW_META(iu_meta_w), .IN_META(iu_meta_n), .IS_META(iu_meta_s), .Inject_META(iu_meta_j)
  );
  
endmodule
