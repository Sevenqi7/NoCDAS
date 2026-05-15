`timescale 1ns/1ps

module router_unit_tb;
  import router_ports_pkg::*;

  localparam int NUM_PORTS = 5;

  logic clk;
  logic reset;
  int errors;
  string tb_suite;
  string tb_wavefile;
  int tb_seed;
  bit tb_run_router;
  bit tb_run_mfu;
  bit tb_run_stress;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  task automatic check(input logic cond, input string msg);
    begin
      if (!cond) begin
        $display("[TB][FAIL] %s", msg);
        errors++;
      end
    end
  endtask

  function automatic flit_meta_t make_meta(
      input router_flit_kind_e flit_kind,
      input router_msg_type_e msg_type,
      input router_traffic_class_e traffic_class,
      input logic [2:0] dst_x,
      input logic [2:0] dst_y,
      input logic route_valid,
      input logic [7:0] route_len,
      input logic [7:0] route_ptr,
      input logic [2:0] route_port0,
      input logic process,
      input logic [4:0] opcode,
      input logic [9:0] data_offset,
      input logic [7:0] data_q
  );
    flit_meta_t meta;
    begin
      meta = '0;
      meta.valid = 1'b1;
      meta.flit_kind = flit_kind;
      meta.msg_type = msg_type;
      meta.opcode = opcode;
      meta.data_offset = data_offset;
`ifdef ROUTER_ENABLE_COSIM
      meta.cosim.data_q = data_q;
`endif
      meta.dst_x = dst_x;
      meta.dst_y = dst_y;
      meta.process = process;
      meta.route_valid = route_valid;
      meta.route_ptr = route_ptr;
      meta.vnet = (traffic_class == ROUTER_TRAFFIC_REGULAR) ? 2'd0 : 2'd1;
      meta.payload_len = 6'd1;
      meta.traffic_class = traffic_class;
      meta.header_reserved = 16'hffff;
`ifdef ROUTER_ENABLE_COSIM
      meta.cosim.packet_uid = {22'd0, data_offset};
      meta.cosim.flit_id = {6'd0, data_offset};
`endif
      make_meta = meta;
    end
  endfunction

  function automatic route_path_t make_route_path(
      input logic route_valid,
      input logic [7:0] route_len,
      input logic [7:0] route_ptr,
      input logic [2:0] route_port0,
      input logic process
  );
    route_path_t route_path;
    begin
      route_path = '0;
      route_path.valid = route_valid;
      route_path.route_len = route_len;
      route_path.route_ptr = route_ptr;
      route_path.route_seq[0 +: ROUTE_PORT_W] = route_port0;
      if (route_valid && (int'(route_ptr) < ROUTE_MAX_HOPS)) begin
        route_path.route_process_seq[int'(route_ptr)] = process;
      end
      make_route_path = route_path;
    end
  endfunction

  // --------------------------------------------------------------------------
  // Route compute and input VC source-route context.
  // --------------------------------------------------------------------------
  rinport_data_t iu_data;
  rinport_ctrl_t iu_ctrl;
  rinport_issue_t iu_issue;
  rinport_flow_t iu_flow;
  flit_meta_t iu_raw_meta;
  route_path_t iu_route_path;
  logic iu_may_need_mfu;
  router_traffic_class_e iu_traffic_class;
  flit_meta_t iu_revisit_meta;
  route_path_t iu_revisit_route_path;
  flit_meta_t iu_exhausted_route_meta;
  route_path_t iu_exhausted_route_path;

  input_unit #(
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) input_unit_i (
      .clk(clk),
      .reset(reset),
      .rinport_data_i(iu_data),
      .rinport_ctrl_i(iu_ctrl),
      .rinport_x_cur_i(iu_ctrl.x_cur),
      .rinport_y_cur_i(iu_ctrl.y_cur),
      .rinport_channel_i(iu_ctrl.in_channel),
      .rinport_raw_meta_o(iu_raw_meta),
      .rinport_route_path_o(iu_route_path),
      .rinport_may_need_mfu_o(iu_may_need_mfu),
      .rinport_traffic_class_o(iu_traffic_class),
      .rinport_issue_o(iu_issue),
      .rinport_flow_o(iu_flow)
  );

  task automatic iu_push(input flit_meta_t meta, input logic [VC_ID_W-1:0] vc_id);
    begin
      iu_push_with_route(meta, '0, vc_id);
    end
  endtask

  task automatic iu_push_with_route(
      input flit_meta_t meta,
      input route_path_t route_path,
      input logic [VC_ID_W-1:0] vc_id
  );
    begin
      @(negedge clk);
      iu_data.flit = '0;
      iu_data.meta = meta;
      iu_data.route_path = route_path;
      iu_data.vc_id = vc_id;
      iu_data.push = 1'b1;
      @(negedge clk);
      iu_data.push = 1'b0;
      iu_data.meta = '0;
      iu_data.route_path = '0;
      iu_data.vc_id = '0;
    end
  endtask

  task automatic iu_expect_and_pop(input logic [2:0] expected_route, input string label);
    int wait_count;
    begin
      wait_count = 0;
      while (!iu_issue.valid && wait_count < 8) begin
        @(negedge clk);
        wait_count++;
      end
      check(iu_issue.valid, {label, ": input unit did not issue"});
      check(iu_issue.route_sel == expected_route, {label, ": unexpected route_sel"});
      iu_ctrl.vc_grant = 1'b1;
      @(negedge clk);
      iu_ctrl.vc_grant = 1'b0;
    end
  endtask

  // --------------------------------------------------------------------------
  // Switch allocator QoS and starvation.
  // --------------------------------------------------------------------------
  logic [NUM_PORTS-1:0] sw_rout_ready;
  logic [NUM_PORTS-1:0] sw_req;
  logic [2:0] sw_route_sel [NUM_PORTS];
  router_traffic_class_e sw_class [NUM_PORTS];
  logic [NUM_PORTS-1:0] sw_vc_ready [NUM_PORTS];
  logic [NUM_PORTS-1:0] sw_mfu_req;
  logic sw_mfu_busy;
  logic [2:0] sw_winner [NUM_PORTS];
  logic [NUM_PORTS-1:0] sw_grant;

  switch_allocator #(
      .NUM_PORTS(NUM_PORTS),
      .STARVATION_LIMIT(2)
  ) switch_allocator_i (
      .clk_i(clk),
      .reset_i(reset),
      .routport_ready_i(sw_rout_ready),
      .rinport_req_i(sw_req),
      .rinport_route_sel_i(sw_route_sel),
      .rinport_traffic_class_i(sw_class),
      .route_vc_ready_i(sw_vc_ready),
      .mfu_rinport_req_i(sw_mfu_req),
      .mfu_busy_i(sw_mfu_busy),
      .routport_winner_o(sw_winner),
      .rinport_grant_o(sw_grant)
  );

  task automatic clear_switch_inputs();
    int i;
    int j;
    begin
      sw_rout_ready = '1;
      sw_req = '0;
      sw_mfu_req = '0;
      sw_mfu_busy = 1'b0;
      for (i = 0; i < NUM_PORTS; i++) begin
        sw_route_sel[i] = ROUTER_PORT_LOCAL;
        sw_class[i] = ROUTER_TRAFFIC_REGULAR;
        sw_vc_ready[i] = '0;
        for (j = 0; j < NUM_PORTS; j++) begin
          sw_vc_ready[i][j] = 1'b1;
        end
      end
    end
  endtask

  // --------------------------------------------------------------------------
  // VC allocator downstream VC round-robin.
  // --------------------------------------------------------------------------
  flit_meta_t vca_meta [NUM_PORTS];
  logic [VC_ID_W-1:0] vca_src_vc [NUM_PORTS];
  routport_flow_t vca_flow [NUM_PORTS];
  logic [2:0] vca_winner [NUM_PORTS];
  logic [NUM_PORTS-1:0] vca_ready [NUM_PORTS];
  logic [VC_ID_W-1:0] vca_dst_vc [NUM_PORTS];

  vc_allocator #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W)
  ) vc_allocator_i (
      .clk(clk),
      .reset(reset),
      .rinport_meta_i(vca_meta),
      .rinport_vc_id_i(vca_src_vc),
      .routport_flow_i(vca_flow),
      .routport_winner_i(vca_winner),
      .route_vc_ready_o(vca_ready),
      .routport_vc_id_o(vca_dst_vc)
  );

  task automatic clear_vc_allocator_inputs();
    int i;
    begin
      for (i = 0; i < NUM_PORTS; i++) begin
        vca_meta[i] = '0;
        vca_src_vc[i] = '0;
        vca_winner[i] = ROUTER_PORT_INV;
        vca_flow[i].state_ready = 1'b1;
        vca_flow[i].downstream_vc_idle_mask = '1;
        vca_flow[i].downstream_vc_credit_mask = '1;
      end
    end
  endtask

  // --------------------------------------------------------------------------
  // MFU storage and writeback helpers.
  // --------------------------------------------------------------------------
  logic mfu_sram_wr_en;
  logic mfu_sram_wr_bank_sel;
  logic [10:0] mfu_sram_wr_addr;
  logic [31:0] mfu_sram_wr_byte_en;
  logic [255:0] mfu_sram_wr_data;
  logic mfu_sram_rd_bank_sel;
  logic [10:0] mfu_sram_rd_addr;

  logic mfu_if_emit_valid;
  logic mfu_if_type4;
  logic [4:0] mfu_if_opcode;
  logic [9:0] mfu_if_data_idx;
  logic [5:0] mfu_if_payload_len;
  logic [255:0] mfu_if_payload;
  logic [10:0] mfu_if_storage_addr;
  logic [5:0] mfu_if_store_bytes;

  mfu_sram_if mfu_sram_if_i (
      .emit_valid_i(mfu_if_emit_valid),
      .pkt_is_type4_i(mfu_if_type4),
      .pkt_opcode_i(mfu_if_opcode),
      .pkt_data_idx_i(mfu_if_data_idx),
      .pkt_payload_len_i(mfu_if_payload_len),
      .pkt_payload_i(mfu_if_payload),
      .pkt_storage_addr_i(mfu_if_storage_addr),
      .pkt_store_bytes_i(mfu_if_store_bytes),
      .sram_wr_en_o(mfu_sram_wr_en),
      .sram_wr_bank_sel_o(mfu_sram_wr_bank_sel),
      .sram_wr_addr_o(mfu_sram_wr_addr),
      .sram_wr_byte_en_o(mfu_sram_wr_byte_en),
      .sram_wr_data_o(mfu_sram_wr_data),
      .sram_rd_bank_sel_o(mfu_sram_rd_bank_sel),
      .sram_rd_addr_o(mfu_sram_rd_addr)
  );

  flit_meta_t wb_meta_in;
  flit_meta_t wb_meta_out;
  logic [FLIT_W-1:0] wb_flit_in;
  logic [FLIT_W-1:0] wb_flit_out;
  flit_meta_t wb_alu_meta;
  logic [FLIT_W-1:0] wb_alu_flit;
  logic wb_matmul_active;
  logic [FLIT_W-1:0] wb_matmul_flit;

  mfu_writeback #(
      .FLIT_W(FLIT_W)
  ) mfu_writeback_i (
      .pkt_flit_i(wb_flit_in),
      .pkt_meta_i(wb_meta_in),
      .pkt_is_type5_i(1'b1),
      .alu_flit_i(wb_alu_flit),
      .alu_meta_i(wb_alu_meta),
      .matmul_active_i(wb_matmul_active),
      .matmul_flit_i(wb_matmul_flit),
      .attention_active_i(1'b0),
      .attention_flit_i('0),
      .attention_meta_i('0),
      .emit_flit_o(wb_flit_out),
      .emit_meta_o(wb_meta_out)
  );

  logic alu0_start;
  logic alu0_busy;
  logic alu0_valid;
  logic [FLIT_W-1:0] alu0_flit_in;
  flit_meta_t alu0_meta_in;
  logic [FLIT_W-1:0] alu0_flit_out;
  flit_meta_t alu0_meta_out;
  logic signed [31:0] alu0_scalar_out;

  mfu_alu #(
      .FLIT_W(FLIT_W),
      .LAT_ADD(0)
  ) mfu_alu_zero_lat_i (
      .clk_i(clk),
      .reset_i(reset),
      .start_i(alu0_start),
      .flit_i(alu0_flit_in),
      .meta_i(alu0_meta_in),
      .op_a_i(32'sd0),
      .op_b_i(32'sd0),
      .busy_o(alu0_busy),
      .result_valid_o(alu0_valid),
      .result_flit_o(alu0_flit_out),
      .result_meta_o(alu0_meta_out),
      .result_scalar_o(alu0_scalar_out)
  );

  logic alu3_start;
  logic alu3_busy;
  logic alu3_valid;
  logic [FLIT_W-1:0] alu3_flit_in;
  flit_meta_t alu3_meta_in;
  logic [FLIT_W-1:0] alu3_flit_out;
  flit_meta_t alu3_meta_out;
  logic signed [31:0] alu3_scalar_out;

  mfu_alu #(
      .FLIT_W(FLIT_W),
      .LAT_GEGLU(3)
  ) mfu_alu_three_lat_i (
      .clk_i(clk),
      .reset_i(reset),
      .start_i(alu3_start),
      .flit_i(alu3_flit_in),
      .meta_i(alu3_meta_in),
      .op_a_i(32'sd0),
      .op_b_i(32'sd0),
      .busy_o(alu3_busy),
      .result_valid_o(alu3_valid),
      .result_flit_o(alu3_flit_out),
      .result_meta_o(alu3_meta_out),
      .result_scalar_o(alu3_scalar_out)
  );

  logic [FLIT_W-1:0] alu_leaf_flit_in;
  flit_meta_t alu_leaf_meta_in;
  logic signed [31:0] alu_leaf_op_a;
  logic signed [31:0] alu_leaf_op_b;
  logic [FLIT_W-1:0] alu_type4_flit_out;
  flit_meta_t alu_type4_meta_out;
  logic signed [31:0] alu_type4_scalar_out;
  logic [FLIT_W-1:0] alu_linear_flit_out;
  flit_meta_t alu_linear_meta_out;
  logic signed [31:0] alu_linear_scalar_out;
  logic [FLIT_W-1:0] alu_matmul_flit_out;
  flit_meta_t alu_matmul_meta_out;
  logic signed [31:0] alu_matmul_scalar_out;
  logic [FLIT_W-1:0] alu_add_flit_out;
  flit_meta_t alu_add_meta_out;
  logic signed [31:0] alu_add_scalar_out;
  logic [FLIT_W-1:0] alu_swiglu_flit_out;
  flit_meta_t alu_swiglu_meta_out;
  logic signed [31:0] alu_swiglu_scalar_out;
  logic [FLIT_W-1:0] alu_geglu_flit_out;
  flit_meta_t alu_geglu_meta_out;
  logic signed [31:0] alu_geglu_scalar_out;
  logic [FLIT_W-1:0] alu_attention_flit_out;
  flit_meta_t alu_attention_meta_out;
  logic signed [31:0] alu_attention_scalar_out;
  logic [FLIT_W-1:0] alu_default_flit_out;
  flit_meta_t alu_default_meta_out;
  logic signed [31:0] alu_default_scalar_out;

  mfu_alu_type4_store #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_type4_store_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_type4_flit_out),
      .result_meta_o(alu_type4_meta_out),
      .result_scalar_o(alu_type4_scalar_out)
  );

  mfu_alu_linear #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_linear_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_linear_flit_out),
      .result_meta_o(alu_linear_meta_out),
      .result_scalar_o(alu_linear_scalar_out)
  );

  mfu_alu_matmul #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_matmul_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_matmul_flit_out),
      .result_meta_o(alu_matmul_meta_out),
      .result_scalar_o(alu_matmul_scalar_out)
  );

  mfu_alu_add #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_add_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_add_flit_out),
      .result_meta_o(alu_add_meta_out),
      .result_scalar_o(alu_add_scalar_out)
  );

  mfu_alu_swiglu #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_swiglu_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_swiglu_flit_out),
      .result_meta_o(alu_swiglu_meta_out),
      .result_scalar_o(alu_swiglu_scalar_out)
  );

  mfu_alu_geglu #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_geglu_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_geglu_flit_out),
      .result_meta_o(alu_geglu_meta_out),
      .result_scalar_o(alu_geglu_scalar_out)
  );

  mfu_alu_attention #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_attention_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_attention_flit_out),
      .result_meta_o(alu_attention_meta_out),
      .result_scalar_o(alu_attention_scalar_out)
  );

  mfu_alu_default #(
      .FLIT_W(FLIT_W)
  ) mfu_alu_default_i (
      .flit_i(alu_leaf_flit_in),
      .meta_i(alu_leaf_meta_in),
      .op_a_i(alu_leaf_op_a),
      .op_b_i(alu_leaf_op_b),
      .result_flit_o(alu_default_flit_out),
      .result_meta_o(alu_default_meta_out),
      .result_scalar_o(alu_default_scalar_out)
  );

  logic mm_weight_wr_en;
  logic [10:0] mm_weight_wr_addr;
  logic [31:0] mm_weight_wr_byte_en;
  logic [FLIT_W-1:0] mm_weight_wr_data;
  logic mm_compute_fire;
  logic mm_release_state;
  logic [FLIT_W-1:0] mm_flit_in;
  flit_meta_t mm_meta_in;
  logic [FLIT_W-1:0] mm_flit_out;
  logic mm_active;
  logic [15:0] mm_weight_row_size;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] mm_task_ids_flat;
  logic att_kv_wr_en;
  logic [10:0] att_kv_wr_addr;
  logic [31:0] att_kv_wr_byte_en;
  logic [FLIT_W-1:0] att_kv_wr_data;
  logic att_compute_fire;
  logic att_release_state;
  logic [FLIT_W-1:0] att_flit_in;
  flit_meta_t att_meta_in;
  logic [FLIT_W-1:0] att_flit_out;
  flit_meta_t att_meta_out;
  logic att_active;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] att_task_ids_flat;

  mfu_matmul #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W)
  ) mfu_matmul_i (
      .clk(clk),
      .reset(reset),
      .weight_wr_en_i(mm_weight_wr_en),
      .weight_wr_addr_i(mm_weight_wr_addr),
      .weight_wr_byte_en_i(mm_weight_wr_byte_en),
      .weight_wr_data_i(mm_weight_wr_data),
      .compute_fire_i(mm_compute_fire),
      .release_state_i(mm_release_state),
      .pkt_is_type5_i(1'b1),
      .pkt_opcode_i(5'd15),
      .pkt_flit_i(mm_flit_in),
      .pkt_meta_i(mm_meta_in),
      .pkt_vc_id_i(3'd0),
      .pkt_out_sel_i(ROUTER_PORT_LOCAL),
      .task_count_i(6'd2),
      .task_ids_flat_i(mm_task_ids_flat),
      .weight_row_size_i(mm_weight_row_size),
      .matmul_active_o(mm_active),
      .matmul_flit_o(mm_flit_out)
  );

  mfu_attention #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .MAX_K_DIM(4)
  ) mfu_attention_i (
      .clk(clk),
      .reset(reset),
      .kv_wr_en_i(att_kv_wr_en),
      .kv_wr_addr_i(att_kv_wr_addr),
      .kv_wr_byte_en_i(att_kv_wr_byte_en),
      .kv_wr_data_i(att_kv_wr_data),
      .compute_fire_i(att_compute_fire),
      .release_state_i(att_release_state),
      .pkt_is_type5_i(1'b1),
      .pkt_opcode_i(5'd23),
      .pkt_flit_i(att_flit_in),
      .pkt_meta_i(att_meta_in),
      .pkt_vc_id_i(3'd0),
      .pkt_out_sel_i(ROUTER_PORT_LOCAL),
      .kv_token_count_i(16'd1),
      .task_count_i(6'd2),
      .task_ids_flat_i(att_task_ids_flat),
      .attention_active_o(att_active),
      .attention_flit_o(att_flit_out),
      .attention_meta_o(att_meta_out)
  );

  // --------------------------------------------------------------------------
  // Router top-level smoke coverage.
  // --------------------------------------------------------------------------
  rinport_data_t top_east_input_data;
  rinport_data_t top_west_input_data;
  rinport_data_t top_north_input_data;
  rinport_data_t top_south_input_data;
  rinport_data_t top_local_input_data;
  rinport_flow_t top_east_input_flow;
  rinport_flow_t top_west_input_flow;
  rinport_flow_t top_north_input_flow;
  rinport_flow_t top_south_input_flow;
  rinport_flow_t top_local_input_flow;
  routport_flow_t top_east_output_flow;
  routport_flow_t top_west_output_flow;
  routport_flow_t top_north_output_flow;
  routport_flow_t top_south_output_flow;
  routport_flow_t top_local_output_flow;
  routport_data_t top_east_output_data;
  routport_data_t top_west_output_data;
  routport_data_t top_north_output_data;
  routport_data_t top_south_output_data;
  routport_data_t top_local_output_data;
  logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] top_task_ids_flat;
  logic [31:0] top_weight_bytes_stored;
  logic [31:0] top_kv_bytes_stored;
  logic [15:0] top_kv_token_count;
  logic top_last_store_valid;
  logic top_last_store_bank;
  logic [10:0] top_last_store_addr;
  logic [5:0] top_last_store_bytes;
  bit top_parallel_seen;
  logic [FLIT_W-1:0] top_dup_flit;
  logic [FLIT_W-1:0] top_dup_processed_flit;
  flit_meta_t top_dup_meta;
  route_path_t top_dup_route_path;
  int top_wait_idx;

  Router #(
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .MFU_LAT_ATTENTION(0)
  ) router_top_i (
      .X_cur(3'd1),
      .Y_cur(3'd1),
      .clk(clk),
      .reset(reset),
      .cnoc_task_count(6'd2),
      .cnoc_task_ids_flat(top_task_ids_flat),
      .cnoc_weight_row_size(16'd2),
      .east_input_data(top_east_input_data),
      .west_input_data(top_west_input_data),
      .north_input_data(top_north_input_data),
      .south_input_data(top_south_input_data),
      .local_input_data(top_local_input_data),
      .east_input_flow(top_east_input_flow),
      .west_input_flow(top_west_input_flow),
      .north_input_flow(top_north_input_flow),
      .south_input_flow(top_south_input_flow),
      .local_input_flow(top_local_input_flow),
      .east_output_flow(top_east_output_flow),
      .west_output_flow(top_west_output_flow),
      .north_output_flow(top_north_output_flow),
      .south_output_flow(top_south_output_flow),
      .local_output_flow(top_local_output_flow),
      .east_output_data(top_east_output_data),
      .west_output_data(top_west_output_data),
      .north_output_data(top_north_output_data),
      .south_output_data(top_south_output_data),
      .local_output_data(top_local_output_data),
      .cnoc_weight_bytes_stored(top_weight_bytes_stored),
      .cnoc_kv_bytes_stored(top_kv_bytes_stored),
      .cnoc_kv_token_count(top_kv_token_count),
      .cnoc_last_type4_store_valid(top_last_store_valid),
      .cnoc_last_type4_store_bank(top_last_store_bank),
      .cnoc_last_type4_store_addr(top_last_store_addr),
      .cnoc_last_type4_store_bytes(top_last_store_bytes)
  );

  task automatic top_clear_inputs();
    begin
      top_east_input_data = '0;
      top_west_input_data = '0;
      top_north_input_data = '0;
      top_south_input_data = '0;
      top_local_input_data = '0;
    end
  endtask

  task automatic top_ready_all_outputs();
    begin
      top_east_output_flow.state_ready = 1'b1;
      top_west_output_flow.state_ready = 1'b1;
      top_north_output_flow.state_ready = 1'b1;
      top_south_output_flow.state_ready = 1'b1;
      top_local_output_flow.state_ready = 1'b1;
      top_east_output_flow.downstream_vc_idle_mask = '1;
      top_west_output_flow.downstream_vc_idle_mask = '1;
      top_north_output_flow.downstream_vc_idle_mask = '1;
      top_south_output_flow.downstream_vc_idle_mask = '1;
      top_local_output_flow.downstream_vc_idle_mask = '1;
      top_east_output_flow.downstream_vc_credit_mask = '1;
      top_west_output_flow.downstream_vc_credit_mask = '1;
      top_north_output_flow.downstream_vc_credit_mask = '1;
      top_south_output_flow.downstream_vc_credit_mask = '1;
      top_local_output_flow.downstream_vc_credit_mask = '1;
    end
  endtask

  task automatic top_drive_input(
      input router_port_sel_e port_sel,
      input logic [FLIT_W-1:0] flit,
      input flit_meta_t meta,
      input logic [VC_ID_W-1:0] vc_id,
      input logic push
  );
    begin
      top_drive_input_with_route(port_sel, flit, meta, '0, vc_id, push);
    end
  endtask

  task automatic top_drive_input_with_route(
      input router_port_sel_e port_sel,
      input logic [FLIT_W-1:0] flit,
      input flit_meta_t meta,
      input route_path_t route_path,
      input logic [VC_ID_W-1:0] vc_id,
      input logic push
  );
    begin
      unique case (port_sel)
        ROUTER_PORT_EAST: begin
          top_east_input_data.flit = flit;
          top_east_input_data.meta = meta;
          top_east_input_data.route_path = route_path;
          top_east_input_data.vc_id = vc_id;
          top_east_input_data.push = push;
        end
        ROUTER_PORT_WEST: begin
          top_west_input_data.flit = flit;
          top_west_input_data.meta = meta;
          top_west_input_data.route_path = route_path;
          top_west_input_data.vc_id = vc_id;
          top_west_input_data.push = push;
        end
        ROUTER_PORT_NORTH: begin
          top_north_input_data.flit = flit;
          top_north_input_data.meta = meta;
          top_north_input_data.route_path = route_path;
          top_north_input_data.vc_id = vc_id;
          top_north_input_data.push = push;
        end
        ROUTER_PORT_SOUTH: begin
          top_south_input_data.flit = flit;
          top_south_input_data.meta = meta;
          top_south_input_data.route_path = route_path;
          top_south_input_data.vc_id = vc_id;
          top_south_input_data.push = push;
        end
        default: begin
          top_local_input_data.flit = flit;
          top_local_input_data.meta = meta;
          top_local_input_data.route_path = route_path;
          top_local_input_data.vc_id = vc_id;
          top_local_input_data.push = push;
        end
      endcase
    end
  endtask

  function automatic logic top_output_write(input router_port_sel_e port_sel);
    begin
      unique case (port_sel)
        ROUTER_PORT_EAST:  top_output_write = top_east_output_data.write_req;
        ROUTER_PORT_WEST:  top_output_write = top_west_output_data.write_req;
        ROUTER_PORT_NORTH: top_output_write = top_north_output_data.write_req;
        ROUTER_PORT_SOUTH: top_output_write = top_south_output_data.write_req;
        default:           top_output_write = top_local_output_data.write_req;
      endcase
    end
  endfunction

  function automatic logic [FLIT_W-1:0] top_output_flit(input router_port_sel_e port_sel);
    begin
      unique case (port_sel)
        ROUTER_PORT_EAST:  top_output_flit = top_east_output_data.flit;
        ROUTER_PORT_WEST:  top_output_flit = top_west_output_data.flit;
        ROUTER_PORT_NORTH: top_output_flit = top_north_output_data.flit;
        ROUTER_PORT_SOUTH: top_output_flit = top_south_output_data.flit;
        default:           top_output_flit = top_local_output_data.flit;
      endcase
    end
  endfunction

  function automatic flit_meta_t top_output_meta(input router_port_sel_e port_sel);
    begin
      unique case (port_sel)
        ROUTER_PORT_EAST:  top_output_meta = top_east_output_data.meta;
        ROUTER_PORT_WEST:  top_output_meta = top_west_output_data.meta;
        ROUTER_PORT_NORTH: top_output_meta = top_north_output_data.meta;
        ROUTER_PORT_SOUTH: top_output_meta = top_south_output_data.meta;
        default:           top_output_meta = top_local_output_data.meta;
      endcase
    end
  endfunction

  function automatic logic top_input_ack(
      input router_port_sel_e port_sel,
      input logic [VC_ID_W-1:0] vc_id
  );
    begin
      unique case (port_sel)
        ROUTER_PORT_EAST:  top_input_ack = top_east_input_flow.push_ack[vc_id];
        ROUTER_PORT_WEST:  top_input_ack = top_west_input_flow.push_ack[vc_id];
        ROUTER_PORT_NORTH: top_input_ack = top_north_input_flow.push_ack[vc_id];
        ROUTER_PORT_SOUTH: top_input_ack = top_south_input_flow.push_ack[vc_id];
        default:           top_input_ack = top_local_input_flow.push_ack[vc_id];
      endcase
    end
  endfunction

  task automatic top_expect_output(
      input router_port_sel_e port_sel,
      input logic [FLIT_W-1:0] expected_flit,
      input string msg
  );
    bit seen;
    begin
      seen = 1'b0;
      for (top_wait_idx = 0; top_wait_idx < 12; top_wait_idx = top_wait_idx + 1) begin
        #1;
        if (top_output_write(port_sel)) begin
          seen = 1'b1;
          check(top_output_flit(port_sel) == expected_flit, msg);
        end
        @(negedge clk);
      end
      check(seen, msg);
    end
  endtask

  task automatic top_expect_output_process(
      input router_port_sel_e port_sel,
      input logic [FLIT_W-1:0] expected_flit,
      input logic expected_process,
      input string msg
  );
    bit seen;
    begin
      seen = 1'b0;
      for (top_wait_idx = 0; top_wait_idx < 12; top_wait_idx = top_wait_idx + 1) begin
        #1;
        if (top_output_write(port_sel)) begin
          seen = 1'b1;
          check(top_output_flit(port_sel) == expected_flit, msg);
          check(top_output_meta(port_sel).process == expected_process,
                {msg, ": process bit mismatch"});
        end
        @(negedge clk);
      end
      check(seen, msg);
    end
  endtask

  task automatic top_stress_observe_outputs(
      inout bit seen [0:31],
      input logic [FLIT_W-1:0] expected [0:31],
      input int expected_count
  );
    router_port_sel_e port_sel;
    int idx;
    bit matched;
    begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        port_sel = router_port_sel_e'(port_idx);
        if (top_output_write(port_sel)) begin
          matched = 1'b0;
          for (idx = 0; idx < expected_count; idx = idx + 1) begin
            if (!seen[idx] && top_output_flit(port_sel) == expected[idx]) begin
              seen[idx] = 1'b1;
              matched = 1'b1;
              break;
            end
          end
          if (!matched) begin
            $display("[TB][DEBUG] stress unexpected output time=%0t port=%0d flit=%h expected_count=%0d",
                     $time, port_idx, top_output_flit(port_sel), expected_count);
          end
          check(matched, "stress scoreboard observed duplicate or unexpected output flit");
        end
      end
    end
  endtask

  task automatic run_top_stress(input int seed);
    localparam int STRESS_FLITS = 24;
    logic [FLIT_W-1:0] expected [0:31];
    bit seen [0:31];
    int accepted;
    int emitted;
    int src_port_raw;
    int dst_port_raw;
    int stall_port_raw;
    router_port_sel_e src_port;
    router_port_sel_e dst_port;
    logic [2:0] dst_x;
    logic [2:0] dst_y;
    logic [VC_ID_W-1:0] vc_id;
    begin
      $display("[TB][INFO] running stress suite with seed=%0d", seed);
      void'($urandom(seed));
      accepted = 0;
      emitted = 0;
      for (int idx = 0; idx < 32; idx = idx + 1) begin
        expected[idx] = '0;
        seen[idx] = 1'b0;
      end

      reset = 1'b1;
      top_clear_inputs();
      top_ready_all_outputs();
      repeat (2) @(negedge clk);
      reset = 1'b0;
      repeat (2) @(negedge clk);

      for (int flit_idx = 0; flit_idx < STRESS_FLITS; flit_idx = flit_idx + 1) begin
        // Keep the stress source at LOCAL so the generated traffic is legal for
        // a single-router fixture.  Random ingress-port U-turns can be blocked
        // by realistic NoC turn restrictions and would make this a topology
        // policy test rather than a no-drop scoreboard.
        src_port_raw = ROUTER_PORT_LOCAL;
        dst_port_raw = $urandom_range(0, NUM_PORTS - 1);
        stall_port_raw = $urandom_range(0, NUM_PORTS - 1);
        src_port = router_port_sel_e'(src_port_raw);
        dst_port = router_port_sel_e'(dst_port_raw);
        vc_id = flit_idx[VC_ID_W-1:0];

        // Temporarily remove one downstream credit to exercise backpressure
        // without making the generated flit permanently unroutable.
        top_ready_all_outputs();
        unique case (router_port_sel_e'(stall_port_raw))
          ROUTER_PORT_EAST:  top_east_output_flow.downstream_vc_credit_mask = '0;
          ROUTER_PORT_WEST:  top_west_output_flow.downstream_vc_credit_mask = '0;
          ROUTER_PORT_NORTH: top_north_output_flow.downstream_vc_credit_mask = '0;
          ROUTER_PORT_SOUTH: top_south_output_flow.downstream_vc_credit_mask = '0;
          default:           top_local_output_flow.downstream_vc_credit_mask = '0;
        endcase

        unique case (dst_port)
          ROUTER_PORT_EAST: begin
            dst_x = 3'd2;
            dst_y = 3'd1;
          end
          ROUTER_PORT_WEST: begin
            dst_x = 3'd0;
            dst_y = 3'd1;
          end
          ROUTER_PORT_NORTH: begin
            dst_x = 3'd1;
            dst_y = 3'd0;
          end
          ROUTER_PORT_SOUTH: begin
            dst_x = 3'd1;
            dst_y = 3'd2;
          end
          default: begin
            dst_x = 3'd1;
            dst_y = 3'd1;
          end
        endcase

        expected[flit_idx] = '0;
        expected[flit_idx][31:0] = 32'h5A00_0000 | flit_idx[31:0];

        top_drive_input(src_port,
                        expected[flit_idx],
                        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ,
                                  ROUTER_TRAFFIC_REGULAR, dst_x, dst_y, 1'b0, 8'd0,
                                  8'd0, ROUTER_PORT_INV, 1'b0, 5'd0, 10'd0, 8'h00),
                        vc_id,
                        1'b1);
        #1;
        check(top_input_ack(src_port, vc_id),
              "stress input flit was not accepted by its selected VC");
        @(posedge clk);
        #1;
        top_clear_inputs();
        top_ready_all_outputs();
        accepted++;
        // The top-level router presents a newly unblocked buffered flit through
        // the combinational crossbar before the following clock edge pops it.
        // Count that transfer once here, then advance through the pop edge
        // before generating the next flit.
        #1;
        top_stress_observe_outputs(seen, expected, flit_idx + 1);
        for (int local_drain_idx = 0; local_drain_idx < 64 && !seen[flit_idx];
             local_drain_idx = local_drain_idx + 1) begin
          // Sample outputs in the stable half-cycle before the next posedge
          // consumes them.  Sampling both immediately after push and after pop
          // can double-count a single combinational transfer.
          @(negedge clk);
          #1;
          top_stress_observe_outputs(seen, expected, flit_idx + 1);
        end
        if (!seen[flit_idx]) begin
          $display("[TB][DEBUG] stress missing flit_idx=%0d src_port=%0d dst_port=%0d stall_port=%0d vc_id=%0d expected=%h",
                   flit_idx, src_port_raw, dst_port_raw, stall_port_raw,
                   vc_id, expected[flit_idx]);
        end
        check(seen[flit_idx], "stress did not emit the most recently accepted flit");
        // Advance through the pop edge after observing a flit before driving the
        // next one.
        @(posedge clk);
        @(negedge clk);
      end

      for (int drain_idx = 0; drain_idx < 96; drain_idx = drain_idx + 1) begin
        @(negedge clk);
        #1;
        top_stress_observe_outputs(seen, expected, STRESS_FLITS);
      end

      for (int flit_idx = 0; flit_idx < STRESS_FLITS; flit_idx = flit_idx + 1) begin
        if (seen[flit_idx]) emitted++;
      end
      check(accepted == STRESS_FLITS, "stress did not accept every generated flit");
      check(emitted == STRESS_FLITS, "stress did not emit every accepted flit");
    end
  endtask

  initial begin
    errors = 0;
    tb_suite = "all";
    tb_wavefile = "router_unit_tb.vcd";
    tb_seed = 1;
    void'($value$plusargs("suite=%s", tb_suite));
    void'($value$plusargs("seed=%d", tb_seed));
    void'($value$plusargs("wavefile=%s", tb_wavefile));
    tb_run_router = (tb_suite == "all") || (tb_suite == "router");
    tb_run_mfu = (tb_suite == "all") || (tb_suite == "mfu");
    tb_run_stress = (tb_suite == "all") || (tb_suite == "stress");
    if ($test$plusargs("waves")) begin
      $dumpfile(tb_wavefile);
      $dumpvars(0, router_unit_tb);
    end
    $display("[TB][INFO] suite=%s seed=%0d", tb_suite, tb_seed);
    reset = 1'b1;

    iu_data = '0;
    iu_ctrl = '0;
    iu_ctrl.x_cur = 3'd0;
    iu_ctrl.y_cur = 3'd0;
    iu_ctrl.in_channel = ROUTER_PORT_LOCAL;
    clear_switch_inputs();
    clear_vc_allocator_inputs();
    mfu_if_emit_valid = 1'b0;
    mfu_if_type4 = 1'b0;
    mfu_if_opcode = '0;
    mfu_if_data_idx = '0;
    mfu_if_payload_len = '0;
    mfu_if_payload = '0;
    mfu_if_storage_addr = '0;
    mfu_if_store_bytes = '0;
    wb_meta_in = '0;
    wb_flit_in = '0;
    wb_alu_meta = '0;
    wb_alu_flit = '0;
    wb_matmul_active = 1'b0;
    wb_matmul_flit = '0;
    alu0_start = 1'b0;
    alu0_flit_in = '0;
    alu0_meta_in = '0;
    alu3_start = 1'b0;
    alu3_flit_in = '0;
    alu3_meta_in = '0;
    alu_leaf_flit_in = '0;
    alu_leaf_meta_in = '0;
    alu_leaf_op_a = '0;
    alu_leaf_op_b = '0;
    mm_weight_wr_en = 1'b0;
    mm_weight_wr_addr = '0;
    mm_weight_wr_byte_en = '0;
    mm_weight_wr_data = '0;
    mm_compute_fire = 1'b0;
    mm_release_state = 1'b0;
    mm_flit_in = '0;
    mm_meta_in = '0;
    mm_weight_row_size = 16'd2;
    mm_task_ids_flat = '0;
    mm_task_ids_flat[0 +: CNOC_TASK_ID_W] = 16'd0;
    mm_task_ids_flat[CNOC_TASK_ID_W +: CNOC_TASK_ID_W] = 16'd1;
    att_kv_wr_en = 1'b0;
    att_kv_wr_addr = '0;
    att_kv_wr_byte_en = '0;
    att_kv_wr_data = '0;
    att_compute_fire = 1'b0;
    att_release_state = 1'b0;
    att_flit_in = '0;
    att_meta_in = '0;
    att_task_ids_flat = '0;
    att_task_ids_flat[0 +: CNOC_TASK_ID_W] = 16'd0;
    att_task_ids_flat[CNOC_TASK_ID_W +: CNOC_TASK_ID_W] = 16'd1;
    top_task_ids_flat = '0;
    top_task_ids_flat[0 +: CNOC_TASK_ID_W] = 16'd0;
    top_task_ids_flat[CNOC_TASK_ID_W +: CNOC_TASK_ID_W] = 16'd1;
    top_clear_inputs();
    top_ready_all_outputs();

    repeat (4) @(negedge clk);
    reset = 1'b0;
    repeat (2) @(negedge clk);

    if (tb_run_router) begin
    // Input unit: head establishes source route, body/tail reuse it, tail clears it.
    iu_push_with_route(
        make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                  3'd2, 3'd0, 1'b1, 8'd2, 8'd0, ROUTER_PORT_WEST,
                  1'b1, 5'd15, 10'd3, 8'h11),
        make_route_path(1'b1, 8'd2, 8'd0, ROUTER_PORT_WEST, 1'b1),
        3'd0);
    iu_expect_and_pop(ROUTER_PORT_WEST, "source-route head");

    iu_push(make_meta(ROUTER_FLIT_BODY, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                      3'd2, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                      1'b1, 5'd15, 10'd4, 8'h22), 3'd0);
    iu_expect_and_pop(ROUTER_PORT_WEST, "source-route body");

    iu_push(make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                      3'd2, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                      1'b1, 5'd15, 10'd5, 8'h33), 3'd0);
    iu_expect_and_pop(ROUTER_PORT_WEST, "source-route tail");

    iu_push(make_meta(ROUTER_FLIT_BODY, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                      3'd2, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                      1'b1, 5'd15, 10'd6, 8'h44), 3'd0);
    iu_expect_and_pop(ROUTER_PORT_EAST, "source-route context clear");

    iu_revisit_meta = make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                                3'd2, 3'd0, 1'b1, 8'd5, 8'd4, ROUTER_PORT_EAST,
                                1'b1, 5'd15, 10'd7, 8'h55);
    iu_revisit_route_path =
        make_route_path(1'b1, 8'd5, 8'd4, ROUTER_PORT_EAST, 1'b0);
    iu_revisit_route_path.route_seq[1 * ROUTE_PORT_W +: ROUTE_PORT_W] =
        ROUTER_PORT_WEST;
    iu_revisit_route_path.route_seq[2 * ROUTE_PORT_W +: ROUTE_PORT_W] =
        ROUTER_PORT_EAST;
    iu_revisit_route_path.route_seq[3 * ROUTE_PORT_W +: ROUTE_PORT_W] =
        ROUTER_PORT_WEST;
    iu_revisit_route_path.route_seq[4 * ROUTE_PORT_W +: ROUTE_PORT_W] =
        ROUTER_PORT_EAST;
    iu_revisit_route_path.route_process_seq[4] = 1'b0;
    iu_push_with_route(iu_revisit_meta, iu_revisit_route_path, 3'd0);
    while (!iu_issue.valid) @(negedge clk);
    check(iu_issue.route_sel == ROUTER_PORT_EAST,
          "source-route revisit test did not use current route pointer");
    check(iu_issue.meta.process == 1'b0,
          "input_unit did not apply source-route process mask on revisit");
    check(!iu_may_need_mfu,
          "input_unit should not request MFU when current source-route hop process bit is clear");
    iu_ctrl.vc_grant = 1'b1;
    @(negedge clk);
    iu_ctrl.vc_grant = 1'b0;

    // Source-route process mask: the first hop has process=1 and enters the
    // MFU; a later hop for the same flit identity has process=0 and bypasses
    // MFU.  This replaces the old router-local processed-flit table.
    top_clear_inputs();
    top_ready_all_outputs();
    top_dup_flit = '0;
    top_dup_flit[7:0] = 8'd16;
    top_dup_flit[15:8] = 8'd16;
    top_dup_processed_flit = top_dup_flit;
    top_dup_processed_flit[7:0] = 8'd32;
    top_dup_meta =
        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                  3'd2, 3'd1, 1'b1, 8'd2, 8'd0, ROUTER_PORT_EAST,
                  1'b1, 5'd18, 10'd12, 8'h10);
`ifdef ROUTER_ENABLE_COSIM
    top_dup_meta.cosim.packet_uid = 32'h1234_5678;
    top_dup_meta.cosim.flit_id = 16'h0042;
`endif
    top_dup_meta.payload_len = 6'd2;
    top_dup_meta.header_reserved = 16'h0001;
    top_dup_route_path =
        make_route_path(1'b1, 8'd2, 8'd0, ROUTER_PORT_EAST, 1'b1);
    top_drive_input_with_route(ROUTER_PORT_LOCAL,
                               top_dup_flit,
                               top_dup_meta,
                               top_dup_route_path,
                               3'd0,
                               1'b1);
    #1;
    check(top_input_ack(ROUTER_PORT_LOCAL, 3'd0),
          "source-route process-mask first flit was not accepted");
    @(posedge clk);
    #1;
    top_clear_inputs();
    top_expect_output_process(ROUTER_PORT_EAST,
                              top_dup_processed_flit,
                              1'b1,
                              "source-route process-mask first compute");
    @(posedge clk);
    @(negedge clk);

    top_dup_meta.route_ptr = 8'd1;
    top_dup_meta.process = 1'b0;
    top_dup_route_path =
        make_route_path(1'b1, 8'd2, 8'd1, ROUTER_PORT_EAST, 1'b0);
    top_dup_route_path.route_process_seq = '0;
    top_dup_route_path.route_seq[1 * ROUTE_PORT_W +: ROUTE_PORT_W] =
        ROUTER_PORT_EAST;
    top_drive_input_with_route(ROUTER_PORT_LOCAL,
                               top_dup_flit,
                               top_dup_meta,
                               top_dup_route_path,
                               3'd0,
                               1'b1);
    #1;
    check(top_input_ack(ROUTER_PORT_LOCAL, 3'd0),
          "source-route process-mask revisit flit was not accepted");
    @(posedge clk);
    #1;
    top_clear_inputs();
    top_expect_output_process(ROUTER_PORT_EAST,
                              top_dup_flit,
                              1'b0,
                              "source-route process-mask revisit bypass");

    // Head-tail source-route packets establish and release their VC route
    // context in one transfer.  The next body-like flit on the same VC must
    // therefore fall back to XY rather than inheriting the old route.
    iu_push_with_route(
        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                  3'd2, 3'd0, 1'b1, 8'd1, 8'd0, ROUTER_PORT_WEST,
                  1'b1, 5'd15, 10'd8, 8'h66),
        make_route_path(1'b1, 8'd1, 8'd0, ROUTER_PORT_WEST, 1'b1),
        3'd0);
    iu_expect_and_pop(ROUTER_PORT_WEST, "source-route head-tail");

    iu_push(make_meta(ROUTER_FLIT_BODY, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                      3'd2, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                      1'b1, 5'd15, 10'd9, 8'h77), 3'd0);
    iu_expect_and_pop(ROUTER_PORT_EAST, "source-route head-tail context clear");

    // If the stored route pointer has already consumed route_len, source routing
    // is inactive and the router must use deterministic XY.  This avoids reading
    // stale route_seq entries past the valid path.
    iu_exhausted_route_meta =
        make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                  3'd2, 3'd0, 1'b1, 8'd1, 8'd1, ROUTER_PORT_WEST,
                  1'b1, 5'd15, 10'd10, 8'h88);
    iu_exhausted_route_path =
        make_route_path(1'b1, 8'd1, 8'd1, ROUTER_PORT_WEST, 1'b1);
    iu_push_with_route(iu_exhausted_route_meta, iu_exhausted_route_path, 3'd0);
    iu_expect_and_pop(ROUTER_PORT_EAST, "source-route exhausted head falls back to XY");

    iu_push(make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                      3'd2, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                      1'b1, 5'd15, 10'd11, 8'h99), 3'd0);
    iu_expect_and_pop(ROUTER_PORT_EAST, "source-route exhausted tail falls back to XY");

    // QoS: type5 wins over type4 and regular.
    clear_switch_inputs();
    sw_req[0] = 1'b1;
    sw_req[1] = 1'b1;
    sw_req[2] = 1'b1;
    sw_route_sel[0] = ROUTER_PORT_EAST;
    sw_route_sel[1] = ROUTER_PORT_EAST;
    sw_route_sel[2] = ROUTER_PORT_EAST;
    sw_class[0] = ROUTER_TRAFFIC_REGULAR;
    sw_class[1] = ROUTER_TRAFFIC_DIST;
    sw_class[2] = ROUTER_TRAFFIC_COMP;
    #1;
    check(sw_winner[ROUTER_PORT_EAST] == 3'd2, "QoS did not prioritize type5");

    // MFU busy blocks an MFU-bound candidate from bypassing.
    sw_mfu_req = '0;
    sw_mfu_req[2] = 1'b1;
    sw_mfu_busy = 1'b1;
    #1;
    check(sw_winner[ROUTER_PORT_EAST] == 3'd1, "MFU busy did not block type5 MFU candidate");

    // Multiple independent outputs may fire in the same cycle when they use
    // different inputs.  This guards against accidentally serializing the whole
    // crossbar while modeling the single-entry MFU side path.
    clear_switch_inputs();
    sw_req[0] = 1'b1;
    sw_req[1] = 1'b1;
    sw_route_sel[0] = ROUTER_PORT_EAST;
    sw_route_sel[1] = ROUTER_PORT_WEST;
    sw_class[0] = ROUTER_TRAFFIC_REGULAR;
    sw_class[1] = ROUTER_TRAFFIC_REGULAR;
    #1;
    check(sw_winner[ROUTER_PORT_EAST] == 3'd0, "switch allocator did not grant east output");
    check(sw_winner[ROUTER_PORT_WEST] == 3'd1, "switch allocator did not grant west output");
    check(sw_grant[0] && sw_grant[1], "switch allocator did not grant both independent inputs");

    // The MFU is single-entry: if two MFU-bound flits target different outputs,
    // only one may be granted.  The other must wait instead of bypassing.
    clear_switch_inputs();
    sw_req[0] = 1'b1;
    sw_req[1] = 1'b1;
    sw_route_sel[0] = ROUTER_PORT_EAST;
    sw_route_sel[1] = ROUTER_PORT_WEST;
    sw_class[0] = ROUTER_TRAFFIC_COMP;
    sw_class[1] = ROUTER_TRAFFIC_COMP;
    sw_mfu_req[0] = 1'b1;
    sw_mfu_req[1] = 1'b1;
    #1;
    check((sw_grant[0] ^ sw_grant[1]) == 1'b1,
          "switch allocator granted more than one MFU-bound input");

    // Starvation guard: after two LCS grants with a regular request waiting,
    // the regular request is forced through.
    clear_switch_inputs();
    sw_req[0] = 1'b1;
    sw_req[1] = 1'b1;
    sw_route_sel[0] = ROUTER_PORT_EAST;
    sw_route_sel[1] = ROUTER_PORT_EAST;
    sw_class[0] = ROUTER_TRAFFIC_REGULAR;
    sw_class[1] = ROUTER_TRAFFIC_COMP;
    repeat (2) @(posedge clk);
    #1;
    check(sw_winner[ROUTER_PORT_EAST] == 3'd0, "starvation guard did not force regular traffic");

    // VC allocator: regular vnet0 and cNoC vnet1 both rotate within their own pools.
    clear_vc_allocator_inputs();
    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ, ROUTER_TRAFFIC_REGULAR,
                  3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b0, 5'd0, 10'd0, 8'h00);
    vca_winner[ROUTER_PORT_EAST] = ROUTER_PORT_LOCAL;
    #1;
    check(vca_dst_vc[ROUTER_PORT_EAST] == 3'd0, "VC allocator regular first grant was not VC0");
    @(posedge clk);
    #1;
    check(vca_dst_vc[ROUTER_PORT_EAST] == 3'd1, "VC allocator regular RR did not advance to VC1");

    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                  3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b1, 5'd15, 10'd0, 8'h00);
    #1;
    check(vca_dst_vc[ROUTER_PORT_EAST] == 3'd4, "VC allocator cNoC first grant was not LCS VC4");
    @(posedge clk);
    #1;
    check(vca_dst_vc[ROUTER_PORT_EAST] == 3'd5, "VC allocator cNoC RR did not advance to VC5");

    // Negative VC allocation: no idle VC means a head cannot transfer.
    vca_flow[ROUTER_PORT_EAST].downstream_vc_idle_mask = '0;
    #1;
    check(!vca_ready[ROUTER_PORT_LOCAL][ROUTER_PORT_EAST], "head transfer ready despite no idle VC");

    // Body/tail flits reuse the VC allocated by their head.  They must be
    // blocked by credit on that recorded downstream VC and tail must release the
    // mapping afterwards.
    clear_vc_allocator_inputs();
    vca_src_vc[ROUTER_PORT_LOCAL] = 3'd2;
    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_MEM_READ, ROUTER_TRAFFIC_REGULAR,
                  3'd0, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b0, 5'd0, 10'd0, 8'h00);
    vca_winner[ROUTER_PORT_WEST] = ROUTER_PORT_LOCAL;
    #1;
    check(vca_dst_vc[ROUTER_PORT_WEST] == 3'd0,
          "VC allocator body/tail test did not start with regular VC0");
    @(posedge clk);
    #1;
    vca_winner[ROUTER_PORT_WEST] = ROUTER_PORT_INV;

    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_BODY, ROUTER_MSG_MEM_READ, ROUTER_TRAFFIC_REGULAR,
                  3'd0, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b0, 5'd0, 10'd0, 8'h00);
    vca_flow[ROUTER_PORT_WEST].downstream_vc_idle_mask = '0;
    vca_flow[ROUTER_PORT_WEST].downstream_vc_credit_mask = 8'b0000_0001;
    #1;
    check(vca_ready[ROUTER_PORT_LOCAL][ROUTER_PORT_WEST],
          "VC allocator body did not reuse head allocation with credit");
    check(vca_dst_vc[ROUTER_PORT_WEST] == 3'd0,
          "VC allocator body changed downstream VC");

    vca_flow[ROUTER_PORT_WEST].downstream_vc_credit_mask = '0;
    #1;
    check(!vca_ready[ROUTER_PORT_LOCAL][ROUTER_PORT_WEST],
          "VC allocator body ready despite missing downstream credit");

    vca_flow[ROUTER_PORT_WEST].downstream_vc_credit_mask = 8'b0000_0001;
    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_MEM_READ, ROUTER_TRAFFIC_REGULAR,
                  3'd0, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b0, 5'd0, 10'd0, 8'h00);
    vca_winner[ROUTER_PORT_WEST] = ROUTER_PORT_LOCAL;
    @(posedge clk);
    #1;
    vca_winner[ROUTER_PORT_WEST] = ROUTER_PORT_INV;
    vca_meta[ROUTER_PORT_LOCAL] =
        make_meta(ROUTER_FLIT_BODY, ROUTER_MSG_MEM_READ, ROUTER_TRAFFIC_REGULAR,
                  3'd0, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                  1'b0, 5'd0, 10'd0, 8'h00);
    #1;
    check(!vca_ready[ROUTER_PORT_LOCAL][ROUTER_PORT_WEST],
          "VC allocator body remained ready after tail released allocation");

    end

    if (tb_run_mfu) begin
    // MFU storage/writeback helpers.
    mfu_if_emit_valid = 1'b1;
    mfu_if_type4 = 1'b1;
    mfu_if_opcode = 5'd23;  // attention -> KV bank
    mfu_if_data_idx = 10'd17;
    mfu_if_payload_len = 6'd2;
    mfu_if_store_bytes = 6'd2;
    mfu_if_payload[7:0] = 8'hA5;
    mfu_if_payload[15:8] = 8'h5A;
    mfu_if_storage_addr = 11'd17;
    #1;
    check(mfu_sram_wr_en, "type4 did not assert SRAM write enable");
    check(mfu_sram_wr_bank_sel, "attention type4 did not select KV bank");
    check(mfu_sram_wr_addr == 11'd17, "type4 SRAM write address mismatch");
    check(mfu_sram_wr_byte_en[1:0] == 2'b11, "type4 SRAM byte enables mismatch");
    check(mfu_sram_wr_data[15:0] == 16'h5AA5, "type4 SRAM write data mismatch");

    mfu_if_payload_len = 6'd63;
    mfu_if_storage_addr = 11'd2040;
    mfu_if_store_bytes = 6'd32;
    #1;
    check(mfu_sram_wr_byte_en == 32'hffff_ffff,
          "type4 payload_len > 32 was not clamped to one 256-bit flit");
    check(mfu_sram_wr_addr == 11'd2040,
          "type4 storage base address should come from data_offset-derived address");

    mfu_if_store_bytes = 6'd8;
    #1;
    check(mfu_sram_wr_byte_en == 32'h0000_00ff,
          "type4 store crossing SRAM end should only enable in-range bytes");

    mfu_if_store_bytes = 6'd0;
    #1;
    check(!mfu_sram_wr_en && mfu_sram_wr_byte_en == 32'h0000_0000,
          "type4 fully out-of-range store should become a deterministic no-op");

    mfu_if_payload_len = 6'd0;
    mfu_if_payload = '0;
    mfu_if_store_bytes = 6'd1;
    #1;
    check(!mfu_sram_wr_en && mfu_sram_wr_byte_en == 32'h0000_0000,
          "type4 zero payload_len flit should not write SRAM/KV bytes");

    // Non-Attention type4 storage uses the same data_offset-derived address
    // model as the MatMul helper's weight bank.  The bank selector must stay on
    // the weight side, unlike Attention's KV-cache writes.
    mfu_if_emit_valid = 1'b1;
    mfu_if_type4 = 1'b1;
    mfu_if_opcode = 5'd15;
    mfu_if_payload_len = 6'd3;
    mfu_if_store_bytes = 6'd3;
    mfu_if_storage_addr = 11'd64;
    mfu_if_payload = '0;
    mfu_if_payload[7:0] = 8'h01;
    mfu_if_payload[15:8] = 8'h02;
    mfu_if_payload[23:16] = 8'h03;
    #1;
    check(mfu_sram_wr_en, "weight type4 did not assert SRAM write enable");
    check(!mfu_sram_wr_bank_sel, "weight type4 incorrectly selected KV bank");
    check(mfu_sram_wr_addr == 11'd64, "weight type4 write address mismatch");
    check(mfu_sram_wr_byte_en[2:0] == 3'b111, "weight type4 byte enables mismatch");
    check(mfu_sram_wr_data[23:0] == 24'h030201, "weight type4 write data mismatch");

    // Latency-aware ALU: latency 0 returns an ADD result on the next sampled edge
    // without holding busy, while latency 3 holds busy until the configured delay
    // expires.
    alu0_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                             3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                             1'b1, 5'd18, 10'd0, 8'h00);
    alu0_meta_in.payload_len = 6'd4;
    alu0_flit_in = '0;
    alu0_flit_in[7:0] = 8'd10;
    alu0_flit_in[15:8] = 8'd32;
    alu0_flit_in[23:16] = 8'hf0;
    alu0_flit_in[31:24] = 8'd4;
    @(negedge clk);
    alu0_start = 1'b1;
    @(posedge clk);
    #1;
    check(alu0_valid, "latency-0 ALU did not assert result_valid");
    check(!alu0_busy, "latency-0 ALU should not remain busy");
    check(alu0_flit_out[7:0] == 8'd42, "latency-0 ADD lane0 result mismatch");
    check(alu0_flit_out[23:16] == 8'hf4, "latency-0 ADD lane1 result mismatch");
    alu0_start = 1'b0;
    @(negedge clk);

    alu3_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                             3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                             1'b1, 5'd24, 10'd0, 8'h00);
    alu3_meta_in.payload_len = 6'd2;
    alu3_flit_in = '0;
    alu3_flit_in[7:0] = 8'd16;
    alu3_flit_in[15:8] = 8'd16;
    @(negedge clk);
    alu3_start = 1'b1;
    @(posedge clk);
    #1;
    check(!alu3_valid && alu3_busy, "latency-3 ALU should be busy after start");
    alu3_start = 1'b0;
    @(posedge clk);
    #1;
    check(!alu3_valid && alu3_busy, "latency-3 ALU completed too early at cycle 1");
    @(posedge clk);
    #1;
    check(!alu3_valid && alu3_busy, "latency-3 ALU completed too early at cycle 2");
    @(posedge clk);
    #1;
    check(alu3_valid && !alu3_busy, "latency-3 ALU did not complete at cycle 3");

    // Leaf smoke: each opcode-specific module keeps the current functional
    // behavior so the parent ALU shell can later swap in a real RTL pipeline.
    alu_leaf_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_DIST, ROUTER_TRAFFIC_DIST,
                                 3'd1, 3'd1, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                                 1'b0, 5'd23, 10'd0, 8'hA5);
    alu_leaf_meta_in.payload_len = 6'd1;
    alu_leaf_flit_in = '0;
    alu_leaf_flit_in[7:0] = 8'hA5;
    alu_leaf_op_a = 32'sd77;
    alu_leaf_op_b = 32'sd5;
    #1;
    check(alu_type4_flit_out == alu_leaf_flit_in, "type4 leaf did not passthrough flit");
    check(alu_type4_scalar_out == 32'sd77, "type4 leaf scalar mismatch");

    alu_leaf_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                                 3'd1, 3'd1, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                                 1'b0, 5'd0, 10'd0, 8'h00);
    alu_leaf_meta_in.payload_len = 6'd1;
    alu_leaf_op_a = 32'sd16;
    alu_leaf_op_b = 32'sd16;
    #1;
    check(alu_linear_scalar_out == 32'sd16, "linear leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd15;
    #1;
    check(alu_matmul_scalar_out == 32'sd16, "matmul leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd18;
    alu_leaf_meta_in.payload_len = 6'd2;
    alu_leaf_meta_in.header_reserved = 16'h0001;
    alu_leaf_flit_in = '0;
    alu_leaf_flit_in[7:0] = 8'd1;
    alu_leaf_flit_in[15:8] = 8'd2;
    #1;
    check(alu_add_flit_out[7:0] == 8'd3, "add leaf lane0 mismatch");
    check(alu_add_scalar_out == 32'sd3, "add leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd21;
    alu_leaf_flit_in = '0;
    alu_leaf_flit_in[7:0] = 8'd0;
    alu_leaf_flit_in[15:8] = 8'd8;
    #1;
    check(alu_swiglu_flit_out[7:0] == 8'd0, "swiglu leaf lane0 mismatch");
    check(alu_swiglu_scalar_out == 32'sd0, "swiglu leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd24;
    #1;
    check(alu_geglu_flit_out[7:0] == 8'd0, "geglu leaf lane0 mismatch");
    check(alu_geglu_scalar_out == 32'sd0, "geglu leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd23;
    alu_leaf_meta_in.payload_len = 6'd1;
    alu_leaf_flit_in = '0;
    alu_leaf_flit_in[7:0] = 8'd5;
    alu_leaf_op_a = 32'sd0;
    alu_leaf_op_b = 32'sd0;
    #1;
    check(alu_attention_flit_out[7:0] == 8'd0, "attention leaf lane0 mismatch");
    check(alu_attention_scalar_out == 32'sd0, "attention leaf scalar mismatch");

    alu_leaf_meta_in.opcode = 5'd31;
    alu_leaf_flit_in = '0;
    alu_leaf_flit_in[7:0] = 8'h5A;
    alu_leaf_op_a = 32'sd0;
    alu_leaf_op_b = 32'sd16;
    #1;
    check(alu_default_flit_out[7:0] == 8'd16, "default leaf lane0 mismatch");
    check(alu_default_scalar_out == 32'sd16, "default leaf scalar mismatch");

    wb_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd18, 10'd0, 8'h00);
    wb_meta_in.payload_len = 6'd4;
    wb_flit_in = '0;
    wb_flit_in[7:0] = 8'd10;
    wb_flit_in[15:8] = 8'd32;
    wb_flit_in[23:16] = 8'hf0;  // -16
    wb_flit_in[31:24] = 8'd4;
    wb_alu_flit = alu0_flit_out;
    wb_alu_meta = alu0_meta_out;
    #1;
`ifdef ROUTER_ENABLE_COSIM
    check(wb_meta_out.cosim.data_q == 8'd42,
          "type5 writeback did not update data_q");
`endif
    check(wb_flit_out[7:0] == 8'd42, "type5 ADD writeback did not update pair 0 result");
    check(wb_flit_out[23:16] == 8'hf4, "type5 ADD writeback did not update pair 1 result");
    check(wb_flit_out[15:8] == 8'd32, "type5 ADD writeback should preserve rhs lane");

    wb_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd1, 10'd5, 8'h00);
    wb_meta_in.payload_len = 6'd6;
    wb_flit_in = '0;
    wb_alu_flit = '0;
    wb_alu_flit[47:40] = 8'd42;
    wb_alu_meta = wb_meta_in;
`ifdef ROUTER_ENABLE_COSIM
    wb_alu_meta.cosim.data_q = 8'd42;
`endif
    #1;
    check(wb_flit_out[47:40] == 8'd42, "type5 scalar writeback did not honor data_offset lane");
    check(wb_flit_out[7:0] == 8'd0, "type5 scalar writeback incorrectly used lane 0");

    // MatMul helper: type4 writes task-local rows, input flit accumulates, and
    // the later psum flit receives each assigned task's partial sum.
    @(negedge clk);
    mm_weight_wr_data = '0;
    mm_weight_wr_data[7:0] = 8'd16;    // task0, input0
    mm_weight_wr_data[15:8] = 8'd32;   // task0, input1
    mm_weight_wr_data[23:16] = 8'd48;  // task1, input0
    mm_weight_wr_data[31:24] = 8'd64;  // task1, input1
    mm_weight_wr_addr = 11'd0;
    mm_weight_wr_byte_en = 32'h0000_000f;
    mm_weight_wr_en = 1'b1;
    @(negedge clk);
    mm_weight_wr_en = 1'b0;

    mm_meta_in = make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd0, 8'h00);
    mm_meta_in.payload_len = 6'd2;
    mm_meta_in.psum_offset = 16'd2;
    mm_flit_in = '0;
    mm_flit_in[7:0] = 8'd16;
    mm_flit_in[15:8] = 8'd16;
    @(negedge clk);
    mm_compute_fire = 1'b1;
    @(negedge clk);
    mm_compute_fire = 1'b0;

    mm_meta_in = make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd2, 8'h00);
    mm_meta_in.payload_len = 6'd2;
    mm_meta_in.psum_offset = 16'd2;
    mm_flit_in = '0;
    #1;
    check(mm_active, "MatMul helper did not recognize MATMUL type5");
    check(mm_flit_out[7:0] == 8'd48, "MatMul psum lane0 mismatch");
    check(mm_flit_out[15:8] == 8'd112, "MatMul psum lane1 mismatch");

    // Same-flit contract: if a flit carries both input lanes and the target
    // psum lanes, the helper must use the current flit's multiply result for
    // the writeback.  This covers head-tail packets and small synthetic layers.
    @(negedge clk);
    mm_release_state = 1'b1;
    @(negedge clk);
    mm_release_state = 1'b0;
    mm_weight_row_size = 16'd2;
    mm_meta_in = make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd0, 8'h00);
    mm_meta_in.payload_len = 6'd4;
    mm_meta_in.psum_offset = 16'd2;
    mm_flit_in = '0;
    mm_flit_in[7:0] = 8'd16;
    mm_flit_in[15:8] = 8'd16;
    @(negedge clk);
    mm_compute_fire = 1'b1;
    #1;
    check(mm_flit_out[23:16] == 8'd48,
          "MatMul same-flit psum lane0 did not observe current input chunk");
    check(mm_flit_out[31:24] == 8'd112,
          "MatMul same-flit psum lane1 did not observe current input chunk");
    @(negedge clk);
    mm_compute_fire = 1'b0;

    // Off-flit contract: a router cannot write a packet-global psum entry unless
    // the outgoing flit actually carries that byte.  The contribution is kept in
    // router-local accumulator state and becomes visible only when the psum flit
    // passes through the MFU.
    @(negedge clk);
    mm_release_state = 1'b1;
    @(negedge clk);
    mm_release_state = 1'b0;
    mm_meta_in = make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd0, 8'h00);
    mm_meta_in.payload_len = 6'd2;
    mm_meta_in.psum_offset = 16'd4;
    mm_flit_in = '0;
    mm_flit_in[7:0] = 8'd16;
    mm_flit_in[15:8] = 8'd16;
    @(negedge clk);
    mm_compute_fire = 1'b1;
    #1;
    check(mm_flit_out[7:0] == 8'd16,
          "MatMul off-flit writeback corrupted input lane0");
    check(mm_flit_out[15:8] == 8'd16,
          "MatMul off-flit writeback corrupted input lane1");
    check(mm_flit_out[39:32] == 8'd0,
          "MatMul off-flit writeback updated psum lane before it was carried");
    @(negedge clk);
    mm_compute_fire = 1'b0;
    mm_meta_in = make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd4, 8'h00);
    mm_meta_in.payload_len = 6'd0;
    mm_meta_in.psum_offset = 16'd4;

    // MatMul layout with an explicit non-zero data_offset.  This models the
    // hardware address contract task_slot * row_size + data_offset + lane and
    // guards against flattening every chunk to row base zero.
    @(negedge clk);
    mm_release_state = 1'b1;
    @(negedge clk);
    mm_release_state = 1'b0;
    mm_weight_row_size = 16'd4;
    mm_weight_wr_data = '0;
    mm_weight_wr_data[7:0] = 8'd32;    // task0, input2 at address 2
    mm_weight_wr_data[15:8] = 8'd32;   // task0, input3 at address 3
    mm_weight_wr_addr = 11'd2;
    mm_weight_wr_byte_en = 32'h0000_0003;
    mm_weight_wr_en = 1'b1;
    @(negedge clk);
    mm_weight_wr_en = 1'b0;
    mm_weight_wr_data = '0;
    mm_weight_wr_data[7:0] = 8'd64;    // task1, input2 at address 6
    mm_weight_wr_data[15:8] = 8'd64;   // task1, input3 at address 7
    mm_weight_wr_addr = 11'd6;
    mm_weight_wr_byte_en = 32'h0000_0003;
    mm_weight_wr_en = 1'b1;
    @(negedge clk);
    mm_weight_wr_en = 1'b0;

    mm_meta_in = make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd2, 8'h00);
    mm_meta_in.payload_len = 6'd2;
    mm_meta_in.psum_offset = 16'd4;
    mm_flit_in = '0;
    mm_flit_in[7:0] = 8'd16;
    mm_flit_in[15:8] = 8'd16;
    @(negedge clk);
    mm_compute_fire = 1'b1;
    @(negedge clk);
    mm_compute_fire = 1'b0;

    mm_meta_in = make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                           3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                           1'b1, 5'd15, 10'd4, 8'h00);
    mm_meta_in.payload_len = 6'd2;
    mm_meta_in.psum_offset = 16'd4;
    mm_flit_in = '0;
    #1;
    check(mm_flit_out[7:0] == 8'd64,
          "MatMul non-zero data_offset psum lane0 mismatch");
    check(mm_flit_out[15:8] == 8'd127,
          "MatMul non-zero data_offset psum lane1 mismatch");
    @(negedge clk);
    mm_release_state = 1'b1;
    @(negedge clk);
    mm_release_state = 1'b0;

    // Attention helper: KV cache is loaded by type4-like writes, query lanes are
    // captured from type5 head/body payload, and tail updates psum lanes using
    // only RTL-owned query/KV state.  This is a compact functional control test,
    // not a claim of final softmax numerical equivalence.
    @(negedge clk);
    att_kv_wr_data = '0;
    att_kv_wr_data[7:0] = 8'd16;    // token0 K0 = 1.0
    att_kv_wr_data[15:8] = 8'd16;   // token0 K1 = 1.0
    att_kv_wr_data[23:16] = 8'd16;  // token0 V0 = 1.0
    att_kv_wr_data[31:24] = 8'd32;  // token0 V1 = 2.0
    att_kv_wr_addr = 11'd0;
    att_kv_wr_byte_en = 32'h0000_000f;
    att_kv_wr_en = 1'b1;
    @(negedge clk);
    att_kv_wr_en = 1'b0;

    att_meta_in = make_meta(ROUTER_FLIT_HEAD, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                            3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                            1'b1, 5'd23, 10'd0, 8'h00);
    att_meta_in.payload_len = 6'd2;
    att_meta_in.psum_offset = 16'd2;
    att_meta_in.k_dim = 16'd2;
    att_flit_in = '0;
    att_flit_in[7:0] = 8'd16;   // Q0 = 1.0
    att_flit_in[15:8] = 8'd16;  // Q1 = 1.0
    @(negedge clk);
    att_compute_fire = 1'b1;
    @(negedge clk);
    att_compute_fire = 1'b0;

    att_meta_in = make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                            3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                            1'b1, 5'd23, 10'd2, 8'h00);
    att_meta_in.payload_len = 6'd2;
    att_meta_in.psum_offset = 16'd2;
    att_meta_in.k_dim = 16'd2;
    att_flit_in = '0;
    #1;
    // With one local token, the Q4.4 softmax weight is 1.0.  The functional
    // Attention helper therefore forwards the value vector [1.0, 2.0] into the
    // psum lanes and records running_max=2.0/running_sum=1.0 in header_reserved.
    check(att_flit_out[7:0] == 8'd16, "Attention functional output lane0 mismatch");
    check(att_flit_out[15:8] == 8'd32, "Attention functional output lane1 mismatch");
    check(att_meta_out.header_reserved[7:0] == 8'd32,
          "Attention running max sideband mismatch");
    check(att_meta_out.header_reserved[15:8] == 8'd16,
          "Attention running sum sideband mismatch");
    @(negedge clk);
    att_release_state = 1'b1;
    @(negedge clk);
    att_release_state = 1'b0;

    // After tail release, the per-output/per-VC Attention context must be gone.
    // A tail-like flit observed combinationally without compute_fire must not
    // reuse the previous query state and fabricate an output.
    att_meta_in = make_meta(ROUTER_FLIT_TAIL, ROUTER_MSG_COMP, ROUTER_TRAFFIC_COMP,
                            3'd1, 3'd0, 1'b0, 8'd0, 8'd0, ROUTER_PORT_INV,
                            1'b1, 5'd23, 10'd2, 8'h00);
    att_meta_in.payload_len = 6'd2;
    att_meta_in.psum_offset = 16'd2;
    att_meta_in.k_dim = 16'd2;
    att_flit_in = '0;
    #1;
    check(att_flit_out[7:0] == 8'd0 && att_flit_out[15:8] == 8'd0,
          "Attention release did not clear output-producing context");

    end

    if (tb_run_router) begin
    // Router top-level: regular XY path must travel through the real
    // input-unit/VC-allocator/switch/crossbar chain, not a submodule-only path.
    reset = 1'b1;
    top_clear_inputs();
    top_ready_all_outputs();
    repeat (2) @(negedge clk);
    reset = 1'b0;
    repeat (2) @(negedge clk);

    top_drive_input(ROUTER_PORT_LOCAL,
                    256'h0000_0000_0000_0000_0000_0000_0000_00a1,
                    make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ,
                              ROUTER_TRAFFIC_REGULAR, 3'd2, 3'd1, 1'b0, 8'd0,
                              8'd0, ROUTER_PORT_INV, 1'b0, 5'd0, 10'd0, 8'h00),
                    3'd0, 1'b1);
    @(negedge clk);
    top_clear_inputs();
    top_expect_output(ROUTER_PORT_EAST,
                      256'h0000_0000_0000_0000_0000_0000_0000_00a1,
                      "Router top-level regular XY east output mismatch");

    // Source-routed cNoC traffic can intentionally choose a non-XY hop.  This
    // checks that the top-level wiring preserves the input VC route context and
    // forwards by the header sequence.
    top_drive_input_with_route(
        ROUTER_PORT_LOCAL,
        256'h0000_0000_0000_0000_0000_0000_0000_00b2,
        make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_DIST,
                  ROUTER_TRAFFIC_DIST, 3'd3, 3'd1, 1'b1, 8'd1,
                  8'd0, ROUTER_PORT_WEST, 1'b0, 5'd23, 10'd0, 8'h00),
        make_route_path(1'b1, 8'd1, 8'd0, ROUTER_PORT_WEST, 1'b0),
        3'd1,
        1'b1);
    @(negedge clk);
    top_clear_inputs();
    top_expect_output(ROUTER_PORT_WEST,
                      256'h0000_0000_0000_0000_0000_0000_0000_00b2,
                      "Router top-level source-route west output mismatch");

    // VC allocation must respect downstream idle/credit.  The head-tail flit
    // remains buffered while east has no usable downstream VC and moves only
    // after the flow-control mask is restored.
    top_east_output_flow.downstream_vc_idle_mask = '0;
    top_east_output_flow.downstream_vc_credit_mask = '0;
    top_drive_input(ROUTER_PORT_LOCAL,
                    256'h0000_0000_0000_0000_0000_0000_0000_00c3,
                    make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ,
                              ROUTER_TRAFFIC_REGULAR, 3'd2, 3'd1, 1'b0, 8'd0,
                              8'd0, ROUTER_PORT_INV, 1'b0, 5'd0, 10'd0, 8'h00),
                    3'd2, 1'b1);
    @(negedge clk);
    top_clear_inputs();
    repeat (3) begin
      #1;
      check(!top_east_output_data.write_req,
            "Router top-level emitted despite missing downstream VC credit");
      @(negedge clk);
    end
    top_ready_all_outputs();
    top_expect_output(ROUTER_PORT_EAST,
                      256'h0000_0000_0000_0000_0000_0000_0000_00c3,
                      "Router top-level credit-released flit mismatch");

    // Different input ports targeting different output ports should be able to
    // transfer in the same router cycle.  This guards against reintroducing the
    // old single-output overwrite/drop behavior at the top level.
    top_drive_input(ROUTER_PORT_EAST,
                    256'h0000_0000_0000_0000_0000_0000_0000_00d4,
                    make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ,
                              ROUTER_TRAFFIC_REGULAR, 3'd1, 3'd1, 1'b0, 8'd0,
                              8'd0, ROUTER_PORT_INV, 1'b0, 5'd0, 10'd0, 8'h00),
                    3'd0, 1'b1);
    top_drive_input(ROUTER_PORT_WEST,
                    256'h0000_0000_0000_0000_0000_0000_0000_00e5,
                    make_meta(ROUTER_FLIT_HEAD_TAIL, ROUTER_MSG_MEM_READ,
                              ROUTER_TRAFFIC_REGULAR, 3'd2, 3'd1, 1'b0, 8'd0,
                              8'd0, ROUTER_PORT_INV, 1'b0, 5'd0, 10'd0, 8'h00),
                    3'd0, 1'b1);
    @(negedge clk);
    top_clear_inputs();
    top_parallel_seen = 1'b0;
    for (top_wait_idx = 0; top_wait_idx < 12; top_wait_idx = top_wait_idx + 1) begin
      #1;
      if (top_local_output_data.write_req && top_east_output_data.write_req) begin
        top_parallel_seen = 1'b1;
        check(top_local_output_data.flit ==
              256'h0000_0000_0000_0000_0000_0000_0000_00d4,
              "Router top-level parallel local flit mismatch");
        check(top_east_output_data.flit ==
              256'h0000_0000_0000_0000_0000_0000_0000_00e5,
              "Router top-level parallel east flit mismatch");
      end
      @(negedge clk);
    end
    check(top_parallel_seen, "Router top-level did not emit independent outputs in parallel");
    end

    if (tb_run_stress) begin
      run_top_stress(tb_seed);
    end

    if (errors == 0) begin
      $display("[TB][PASS] router_unit_tb completed.");
      $finish;
    end else begin
      $display("[TB][FAIL] router_unit_tb found %0d errors.", errors);
      $fatal(1);
    end
  end

endmodule
