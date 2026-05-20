// Description: Pipeline VC allocation stage for the Router main datapath.
//              The module owns per-input VA slots, packet-to-downstream-VC
//              mappings, downstream VC reservations, and per-output/vnet VC
//              round-robin pointers.  Input issue queues pop only when a flit
//              successfully enters the VA stage; packet mappings are updated
//              only from output commit events.

module vc_allocator #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W
) (
    input logic clk,
    input logic reset,

    input router_ports_pkg::router_pipe_entry_t issue_entry_i [NUM_PORTS],
    input router_ports_pkg::routport_flow_t routport_flow_i [NUM_PORTS],
    input logic [NUM_PORTS-1:0] va_fire_i,

    input logic [NUM_PORTS-1:0] output_commit_fire_i,
    input logic [NUM_PORTS-1:0] output_mfu_selected_i,
    input router_ports_pkg::router_pipe_entry_t output_commit_entry_i [NUM_PORTS],

    input logic mfu_emit_valid_i,
    input logic [2:0] mfu_emit_out_sel_i,
    input router_ports_pkg::flit_meta_t mfu_emit_meta_i,
    input logic [VC_ID_W-1:0] mfu_emit_vc_id_i,

    output logic [NUM_PORTS-1:0] issue_pop_o,
    output router_ports_pkg::router_pipe_entry_t va_entry_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;
  localparam int VN_NUM_LOCAL = 2;
  localparam int VC_PER_VNET = VC_NUM / VN_NUM_LOCAL;
  localparam int VNET_URS = 0;
  localparam int VNET_LCS = 1;

  router_pipe_entry_t va_q [NUM_PORTS];
  router_pipe_entry_t va_d [NUM_PORTS];

  logic alloc_valid_q [NUM_PORTS][VC_NUM];
  logic alloc_valid_d [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_q [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_q [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_d [NUM_PORTS][VC_NUM];
  logic reserved_vc_q [NUM_PORTS][VC_NUM];
  logic reserved_vc_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] vc_rr_ptr_q [NUM_PORTS][VN_NUM_LOCAL];
  logic [VC_ID_W-1:0] vc_rr_ptr_d [NUM_PORTS][VN_NUM_LOCAL];

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

    issue_pop_o = '0;

    for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      va_d[port_idx] = va_q[port_idx];
      if (va_fire_i[port_idx]) begin
        va_d[port_idx] = '0;
      end
    end

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        alloc_valid_d[in_idx][vc_idx] = alloc_valid_q[in_idx][vc_idx];
        alloc_out_port_d[in_idx][vc_idx] = alloc_out_port_q[in_idx][vc_idx];
        alloc_dst_vc_d[in_idx][vc_idx] = alloc_dst_vc_q[in_idx][vc_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        reserved_vc_d[out_idx][vc_idx] = reserved_vc_q[out_idx][vc_idx];
      end
      for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
        vc_rr_ptr_d[out_idx][vnet_idx] = vc_rr_ptr_q[out_idx][vnet_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_commit_fire_i[out_idx]) begin
        if (output_commit_entry_i[out_idx].reserved_vc && !output_mfu_selected_i[out_idx]) begin
          reserved_vc_d[out_idx][output_commit_entry_i[out_idx].dst_vc] = 1'b0;
        end

        if (output_commit_entry_i[out_idx].head_like && !output_commit_entry_i[out_idx].tail_like) begin
          alloc_valid_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] = 1'b1;
          alloc_out_port_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] = 3'(out_idx);
          alloc_dst_vc_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] =
              output_commit_entry_i[out_idx].dst_vc;
        end

        if (output_commit_entry_i[out_idx].tail_like) begin
          alloc_valid_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] = 1'b0;
          alloc_out_port_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] = PORT_SEL_INVALID;
          alloc_dst_vc_d[output_commit_entry_i[out_idx].src_port][output_commit_entry_i[out_idx].src_vc] = '0;
        end
      end
    end

`ifdef ENABLE_CNOC_MFU
    if (mfu_emit_valid_i &&
        (mfu_emit_out_sel_i < NUM_PORTS) &&
        flit_is_head_like(mfu_emit_meta_i.flit_kind)) begin
      reserved_vc_d[mfu_emit_out_sel_i][mfu_emit_vc_id_i] = 1'b0;
    end
`endif

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      if (issue_entry_i[in_idx].valid && (!va_d[in_idx].valid)) begin
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
        if ((issue_entry_i[in_idx].traffic_class == ROUTER_TRAFFIC_COMP) ||
            (issue_entry_i[in_idx].traffic_class == ROUTER_TRAFFIC_DIST)) begin
          selected_vnet = 2'(VNET_LCS);
        end else if (issue_entry_i[in_idx].meta.vnet < VN_NUM_LOCAL) begin
          selected_vnet = issue_entry_i[in_idx].meta.vnet[1:0];
        end
`else
        if (issue_entry_i[in_idx].meta.vnet < VN_NUM_LOCAL) begin
          selected_vnet = issue_entry_i[in_idx].meta.vnet[1:0];
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

        if (issue_entry_i[in_idx].route_sel < NUM_PORTS) begin
          if (issue_entry_i[in_idx].head_like) begin
            selected_alloc_mask =
                routport_flow_i[issue_entry_i[in_idx].route_sel].downstream_vc_idle_mask &
                routport_flow_i[issue_entry_i[in_idx].route_sel].downstream_vc_credit_mask &
                selected_vnet_mask;

            for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
              if (reserved_vc_d[issue_entry_i[in_idx].route_sel][vc_idx]) begin
                selected_alloc_mask[vc_idx] = 1'b0;
              end
            end

            selected_start_offset =
                int'(vc_rr_ptr_q[issue_entry_i[in_idx].route_sel][selected_vnet_idx]) -
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
                alloc_valid_q[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] &&
                (alloc_out_port_q[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] ==
                 issue_entry_i[in_idx].route_sel);
            selected_dst_vc =
                alloc_dst_vc_q[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc];
            selected_va_ready = selected_body_ready;
          end
        end

        if (selected_va_ready) begin
          va_d[in_idx] = issue_entry_i[in_idx];
          va_d[in_idx].dst_vc = selected_dst_vc;
          va_d[in_idx].reserved_vc = issue_entry_i[in_idx].head_like;
          issue_pop_o[in_idx] = 1'b1;

          if (issue_entry_i[in_idx].head_like) begin
            reserved_vc_d[issue_entry_i[in_idx].route_sel][selected_dst_vc] = 1'b1;
            if (int'(selected_dst_vc) >= selected_vnet_last) begin
              vc_rr_ptr_d[issue_entry_i[in_idx].route_sel][selected_vnet_idx] =
                  VC_ID_W'(selected_vnet_base);
            end else begin
              vc_rr_ptr_d[issue_entry_i[in_idx].route_sel][selected_vnet_idx] =
                  selected_dst_vc + {{(VC_ID_W-1){1'b0}}, 1'b1};
            end
          end
        end
      end
    end
  end

  assign va_entry_o = va_q;

  always_ff @(posedge clk) begin : proc_registers
    if (reset) begin
      for (int port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        va_q[port_idx] <= '0;
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
        va_q[port_idx] <= va_d[port_idx];
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
  always_ff @(posedge clk) begin : proc_assertions
    if (!reset) begin
      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        if (issue_pop_o[in_idx] && !issue_entry_i[in_idx].valid) begin
          $error("vc_allocator: popped an invalid issue entry");
        end
      end
    end
  end
`endif

endmodule
