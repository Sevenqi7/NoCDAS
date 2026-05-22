// Description: Pipeline VC allocation stage for the Router main datapath.
//              The module owns per-input VA slots, packet-to-downstream-VC
//              mappings, downstream VC reservations, and per-output/vnet VC
//              round-robin pointers.  Input issue queues pop only when a flit
//              successfully enters the VA stage; packet stream mappings are
//              established and released when head/tail enter the VA buffer.

module vc_allocator #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W
) (
    input logic clk,
    input logic reset,

    input router_ports_pkg::router_pipe_entry_t issue_entry_i [NUM_PORTS],
    input router_ports_pkg::attention_ctx_status_t attention_ctx_status_i,
    input router_ports_pkg::matmul_ctx_status_t matmul_ctx_status_i,
    input router_ports_pkg::routport_flow_t routport_flow_i [NUM_PORTS],
    input logic [NUM_PORTS-1:0] va_fire_i,

    input logic [NUM_PORTS-1:0] output_release_valid_i,
    input logic [VC_ID_W-1:0] output_release_vc_i [NUM_PORTS],

    input logic mfu_emit_valid_i,
    input logic [2:0] mfu_emit_out_sel_i,
    input router_ports_pkg::flit_meta_t mfu_emit_meta_i,
    input logic [VC_ID_W-1:0] mfu_emit_vc_id_i,

`ifdef ENABLE_CNOC_MFU
    output logic cnoc_attention_claim_valid_o,
    output logic [2:0] cnoc_attention_claim_src_port_o,
    output logic [VC_ID_W-1:0] cnoc_attention_claim_src_vc_o,
    output logic cnoc_data_claim_valid_o,
    output logic [2:0] cnoc_data_claim_src_port_o,
    output logic [VC_ID_W-1:0] cnoc_data_claim_src_vc_o,
`endif
    output logic [NUM_PORTS-1:0] issue_pop_o,
    output router_ports_pkg::router_pipe_entry_t va_entry_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;
  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;
  localparam int VA_BUF_DEPTH = 2;
  localparam int VN_NUM_LOCAL = 2;
  localparam int VC_PER_VNET = VC_NUM / VN_NUM_LOCAL;
  localparam int VNET_URS = 0;
  localparam int VNET_LCS = 1;

  router_pipe_entry_t va_q [NUM_PORTS][VA_BUF_DEPTH];
  router_pipe_entry_t va_d [NUM_PORTS][VA_BUF_DEPTH];
  logic va_buf_has_free_slot [NUM_PORTS];
  logic va_buf_front_valid [NUM_PORTS];
  logic va_buf_back_valid [NUM_PORTS];
  logic va_buf_pop [NUM_PORTS];
  logic va_buf_push [NUM_PORTS];

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
`ifdef ENABLE_CNOC_MFU
  logic cnoc_attention_claim_valid_q;
  logic cnoc_attention_claim_valid_d;
  logic [2:0] cnoc_attention_claim_src_port_q;
  logic [2:0] cnoc_attention_claim_src_port_d;
  logic [VC_ID_W-1:0] cnoc_attention_claim_src_vc_q;
  logic [VC_ID_W-1:0] cnoc_attention_claim_src_vc_d;
  logic cnoc_data_claim_valid_q;
  logic cnoc_data_claim_valid_d;
  logic [2:0] cnoc_data_claim_src_port_q;
  logic [2:0] cnoc_data_claim_src_port_d;
  logic [VC_ID_W-1:0] cnoc_data_claim_src_vc_q;
  logic [VC_ID_W-1:0] cnoc_data_claim_src_vc_d;
`endif

`ifdef ENABLE_CNOC_MFU
  assign cnoc_attention_claim_valid_o = cnoc_attention_claim_valid_q;
  assign cnoc_attention_claim_src_port_o = cnoc_attention_claim_src_port_q;
  assign cnoc_attention_claim_src_vc_o = cnoc_attention_claim_src_vc_q;
  assign cnoc_data_claim_valid_o = cnoc_data_claim_valid_q;
  assign cnoc_data_claim_src_port_o = cnoc_data_claim_src_port_q;
  assign cnoc_data_claim_src_vc_o = cnoc_data_claim_src_vc_q;
`endif

  always_comb begin : proc_vc_alloc_next
    logic selected_head_ready;
    logic selected_body_ready;
    logic selected_va_ready;
    logic selected_ctx_ok;
    logic selected_found_vc;
    logic [1:0] selected_vnet;
    logic selected_vnet_idx;
    logic [VC_NUM-1:0] selected_vnet_mask;
    logic [VC_NUM-1:0] selected_alloc_mask;
    logic [VC_ID_W-1:0] selected_dst_vc;
    logic attention_ctx_valid;
    logic [2:0] attention_ctx_stream_port;
    logic [VC_ID_W-1:0] attention_ctx_stream_vc;
    logic selected_attention_entry;
    logic selected_attention_owner_match;
    logic selected_attention_ctx_ok;
    logic selected_data_entry;
    logic selected_data_owner_match;
    logic selected_data_ctx_ok;
    router_pipe_entry_t va_push_entry;
    int selected_vnet_base;
    int selected_vnet_last;
    int selected_start_offset;
    int selected_candidate_offset;
    int selected_candidate_vc;

    selected_head_ready = 1'b0;
    selected_body_ready = 1'b0;
    selected_va_ready = 1'b0;
    selected_ctx_ok = 1'b1;
    selected_found_vc = 1'b0;
    selected_vnet = 2'(VNET_URS);
    selected_vnet_idx = 1'b0;
    selected_vnet_mask = '0;
    selected_alloc_mask = '0;
    selected_dst_vc = '0;
    attention_ctx_valid = attention_ctx_status_i.valid;
    attention_ctx_stream_port = attention_ctx_status_i.stream_port;
    attention_ctx_stream_vc = attention_ctx_status_i.stream_vc;
    selected_attention_entry = 1'b0;
    selected_attention_owner_match = 1'b0;
    selected_attention_ctx_ok = 1'b1;
    selected_data_entry = 1'b0;
    selected_data_owner_match = 1'b0;
    selected_data_ctx_ok = 1'b1;
    va_push_entry = '0;
    selected_vnet_base = 0;
    selected_vnet_last = 0;
    selected_start_offset = 0;
    selected_candidate_offset = 0;
    selected_candidate_vc = 0;

    issue_pop_o = '0;
`ifdef ENABLE_CNOC_MFU
    cnoc_attention_claim_valid_d = 1'b0;
    cnoc_attention_claim_src_port_d = ROUTER_PORT_INV;
    cnoc_attention_claim_src_vc_d = '0;
    cnoc_data_claim_valid_d = 1'b0;
    cnoc_data_claim_src_port_d = ROUTER_PORT_INV;
    cnoc_data_claim_src_vc_d = '0;
`endif

    for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      va_buf_front_valid[port_idx] = va_q[port_idx][0].valid;
      va_buf_back_valid[port_idx] = va_q[port_idx][1].valid;
      va_buf_has_free_slot[port_idx] =
          !va_q[port_idx][0].valid || !va_q[port_idx][1].valid;
      va_buf_pop[port_idx] = va_fire_i[port_idx] && va_q[port_idx][0].valid;
      va_buf_push[port_idx] = 1'b0;

      va_d[port_idx][0] = va_q[port_idx][0];
      va_d[port_idx][1] = va_q[port_idx][1];

      if (va_buf_pop[port_idx]) begin
        if (va_q[port_idx][1].valid) begin
          va_d[port_idx][0] = va_q[port_idx][1];
          va_d[port_idx][1] = '0;
        end else begin
          va_d[port_idx][0] = '0;
          va_d[port_idx][1] = '0;
        end
      end
    end

    for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        alloc_valid_d[in_idx][vc_idx] = alloc_valid_q[in_idx][vc_idx];
        alloc_out_port_d[in_idx][vc_idx] = alloc_out_port_q[in_idx][vc_idx];
        alloc_dst_vc_d[in_idx][vc_idx] = alloc_dst_vc_q[in_idx][vc_idx];
      end
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        reserved_vc_d[out_idx][vc_idx] = reserved_vc_q[out_idx][vc_idx];
      end
      for (int unsigned vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
        vc_rr_ptr_d[out_idx][vnet_idx] = vc_rr_ptr_q[out_idx][vnet_idx];
      end
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (output_release_valid_i[out_idx]) begin
        reserved_vc_d[out_idx][output_release_vc_i[out_idx]] = 1'b0;
      end
    end

`ifdef ENABLE_CNOC_MFU
    if (mfu_emit_valid_i &&
        (mfu_emit_out_sel_i < NUM_PORTS) &&
        flit_is_head_like(mfu_emit_meta_i.flit_kind)) begin
      reserved_vc_d[mfu_emit_out_sel_i][mfu_emit_vc_id_i] = 1'b0;
    end
`endif

    for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      if (issue_entry_i[in_idx].valid && va_buf_has_free_slot[in_idx]) begin
        selected_head_ready = 1'b0;
        selected_body_ready = 1'b0;
        selected_va_ready = 1'b0;
        selected_ctx_ok = 1'b1;
        selected_found_vc = 1'b0;
        selected_vnet = 2'(VNET_URS);
        selected_vnet_idx = 1'b0;
        selected_vnet_mask = '0;
        selected_alloc_mask = '0;
        selected_dst_vc = '0;
        selected_attention_entry = 1'b0;
        selected_attention_owner_match = 1'b0;
        selected_attention_ctx_ok = 1'b1;
        selected_data_entry = 1'b0;
        selected_data_owner_match = 1'b0;
        selected_data_ctx_ok = 1'b1;
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
`ifdef ENABLE_CNOC_MFU
          selected_attention_entry =
              issue_entry_i[in_idx].may_need_mfu &&
              (issue_entry_i[in_idx].meta.msg_type == ROUTER_MSG_COMP) &&
              (issue_entry_i[in_idx].meta.opcode == OP_ATTENTION);
          selected_data_entry =
              issue_entry_i[in_idx].may_need_mfu &&
              (issue_entry_i[in_idx].meta.msg_type == ROUTER_MSG_COMP) &&
              ((issue_entry_i[in_idx].meta.opcode == OP_LINEAR) ||
               (issue_entry_i[in_idx].meta.opcode == OP_MATMUL));
          selected_attention_owner_match =
              attention_ctx_valid &&
              (attention_ctx_stream_port == issue_entry_i[in_idx].src_port) &&
              (attention_ctx_stream_vc == issue_entry_i[in_idx].src_vc);
          for (int slot_idx = 0; slot_idx < MATMUL_CTX_SLOTS_MAX; slot_idx = slot_idx + 1) begin
            if (!selected_data_owner_match &&
                matmul_ctx_status_i.slot_valid[slot_idx] &&
                (matmul_ctx_status_i.slot_stream_port_flat[slot_idx * 3 +: 3] ==
                 issue_entry_i[in_idx].src_port) &&
                (matmul_ctx_status_i.slot_stream_vc_flat[slot_idx * VC_ID_W +: VC_ID_W] ==
                 issue_entry_i[in_idx].src_vc)) begin
              selected_data_owner_match = 1'b1;
            end
          end
          if (selected_attention_entry) begin
            if (issue_entry_i[in_idx].head_like) begin
              selected_attention_ctx_ok =
                  !attention_ctx_valid || selected_attention_owner_match;
            end else begin
              selected_attention_ctx_ok =
                  attention_ctx_valid && selected_attention_owner_match;
            end
          end
          if (selected_data_entry) begin
            if (issue_entry_i[in_idx].head_like) begin
              selected_data_ctx_ok =
                  selected_data_owner_match || matmul_ctx_status_i.has_free_slot;
            end else begin
              selected_data_ctx_ok = selected_data_owner_match;
            end
          end
`else
          selected_attention_entry = 1'b0;
          selected_attention_owner_match = 1'b0;
          selected_attention_ctx_ok = 1'b1;
          selected_data_entry = 1'b0;
          selected_data_owner_match = 1'b0;
          selected_data_ctx_ok = 1'b1;
`endif

          if (issue_entry_i[in_idx].head_like) begin
            selected_alloc_mask =
                routport_flow_i[issue_entry_i[in_idx].route_sel].downstream_vc_idle_mask &
                routport_flow_i[issue_entry_i[in_idx].route_sel].downstream_vc_credit_mask &
                selected_vnet_mask;

            for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
              if (reserved_vc_q[issue_entry_i[in_idx].route_sel][vc_idx]) begin
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

        selected_ctx_ok = selected_attention_ctx_ok && selected_data_ctx_ok;
        selected_va_ready = selected_va_ready && selected_ctx_ok;

        if (selected_va_ready) begin
          va_push_entry = issue_entry_i[in_idx];
          va_push_entry.dst_vc = selected_dst_vc;
          va_push_entry.reserved_vc = issue_entry_i[in_idx].head_like;
          va_buf_push[in_idx] = 1'b1;
          issue_pop_o[in_idx] = 1'b1;

          // Refill eligibility is based only on the pre-cycle occupancy view.
          // Even if va_fire_i pops the front entry this cycle, a full buffer
          // must not treat that newly-freed slot as push-eligible.
          if (!va_buf_front_valid[in_idx]) begin
            va_d[in_idx][0] = va_push_entry;
            va_d[in_idx][1] = '0;
          end else if (!va_buf_back_valid[in_idx]) begin
            va_d[in_idx][1] = va_push_entry;
          end

          if (selected_attention_entry && issue_entry_i[in_idx].head_like &&
              !attention_ctx_valid) begin
            attention_ctx_valid = 1'b1;
            attention_ctx_stream_port = issue_entry_i[in_idx].src_port;
            attention_ctx_stream_vc = issue_entry_i[in_idx].src_vc;
          end
`ifdef ENABLE_CNOC_MFU
          if (issue_entry_i[in_idx].head_like) begin
            if (selected_attention_entry && !cnoc_attention_claim_valid_d) begin
              cnoc_attention_claim_valid_d = 1'b1;
              cnoc_attention_claim_src_port_d = issue_entry_i[in_idx].src_port;
              cnoc_attention_claim_src_vc_d = issue_entry_i[in_idx].src_vc;
            end
            if (selected_data_entry && !cnoc_data_claim_valid_d) begin
              cnoc_data_claim_valid_d = 1'b1;
              cnoc_data_claim_src_port_d = issue_entry_i[in_idx].src_port;
              cnoc_data_claim_src_vc_d = issue_entry_i[in_idx].src_vc;
            end
          end
`endif
          if (issue_entry_i[in_idx].head_like && !issue_entry_i[in_idx].tail_like) begin
            alloc_valid_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] = 1'b1;
            alloc_out_port_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] =
                issue_entry_i[in_idx].route_sel;
            alloc_dst_vc_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] =
                selected_dst_vc;
          end else if (issue_entry_i[in_idx].tail_like) begin
            alloc_valid_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] = 1'b0;
            alloc_out_port_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] =
                PORT_SEL_INVALID;
            alloc_dst_vc_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc] = '0;
          end
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

    for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      if (!va_d[port_idx][0].valid && va_d[port_idx][1].valid) begin
        va_d[port_idx][0] = va_d[port_idx][1];
        va_d[port_idx][1] = '0;
      end
    end
  end

  always_comb begin : proc_va_entry_output
    for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      va_entry_o[port_idx] = va_q[port_idx][0];
    end
  end

  always_ff @(posedge clk) begin : proc_registers
    if (reset) begin
      for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        va_q[port_idx][0] <= '0;
        va_q[port_idx][1] <= '0;
      end

      for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= 1'b0;
          alloc_out_port_q[in_idx][vc_idx] <= PORT_SEL_INVALID;
          alloc_dst_vc_q[in_idx][vc_idx] <= '0;
        end
      end

      for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          reserved_vc_q[out_idx][vc_idx] <= 1'b0;
        end
        for (int unsigned vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= VC_ID_W'(vnet_idx * VC_PER_VNET);
        end
      end
`ifdef ENABLE_CNOC_MFU
      cnoc_attention_claim_valid_q <= 1'b0;
      cnoc_attention_claim_src_port_q <= ROUTER_PORT_INV;
      cnoc_attention_claim_src_vc_q <= '0;
      cnoc_data_claim_valid_q <= 1'b0;
      cnoc_data_claim_src_port_q <= ROUTER_PORT_INV;
      cnoc_data_claim_src_vc_q <= '0;
`endif
    end else begin
      for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
        va_q[port_idx][0] <= va_d[port_idx][0];
        va_q[port_idx][1] <= va_d[port_idx][1];
      end

      for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= alloc_valid_d[in_idx][vc_idx];
          alloc_out_port_q[in_idx][vc_idx] <= alloc_out_port_d[in_idx][vc_idx];
          alloc_dst_vc_q[in_idx][vc_idx] <= alloc_dst_vc_d[in_idx][vc_idx];
        end
      end

      for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int unsigned vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          reserved_vc_q[out_idx][vc_idx] <= reserved_vc_d[out_idx][vc_idx];
        end
        for (int unsigned vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= vc_rr_ptr_d[out_idx][vnet_idx];
        end
      end
`ifdef ENABLE_CNOC_MFU
      cnoc_attention_claim_valid_q <= cnoc_attention_claim_valid_d;
      cnoc_attention_claim_src_port_q <= cnoc_attention_claim_src_port_d;
      cnoc_attention_claim_src_vc_q <= cnoc_attention_claim_src_vc_d;
      cnoc_data_claim_valid_q <= cnoc_data_claim_valid_d;
      cnoc_data_claim_src_port_q <= cnoc_data_claim_src_port_d;
      cnoc_data_claim_src_vc_q <= cnoc_data_claim_src_vc_d;
`endif
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_assertions
    if (!reset) begin
      for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        if (issue_pop_o[in_idx] && !issue_entry_i[in_idx].valid) begin
          $error("vc_allocator: popped an invalid issue entry");
        end
        if (issue_pop_o[in_idx] &&
            !issue_entry_i[in_idx].head_like &&
            issue_entry_i[in_idx].tail_like &&
            alloc_valid_d[issue_entry_i[in_idx].src_port][issue_entry_i[in_idx].src_vc]) begin
          $error("vc_allocator: tail-like VA admit kept stream mapping live");
        end
        if (va_fire_i[in_idx] && !va_q[in_idx][0].valid) begin
          $error("vc_allocator: attempted to consume an empty VA front slot");
        end
        if (va_q[in_idx][1].valid && !va_q[in_idx][0].valid) begin
          $error("vc_allocator: VA buffer back slot valid while front slot empty");
        end
      end
    end
  end
`endif

endmodule
