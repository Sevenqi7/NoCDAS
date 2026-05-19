// Description: Downstream VC allocator.
//              Head flits allocate an idle downstream VC in the same virtual
//              network.  Body/tail flits reuse the recorded allocation until
//              the tail releases the mapping.
//              The current pipelined Router top owns commit-driven VC
//              allocation internally; this module remains as standalone
//              unit-tested reference coverage.

module vc_allocator #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W
) (
    input logic clk,
    input logic reset,

    input router_ports_pkg::flit_meta_t rinport_meta_i [NUM_PORTS],
    input logic [VC_ID_W-1:0] rinport_vc_id_i [NUM_PORTS],
    input router_ports_pkg::routport_flow_t routport_flow_i [NUM_PORTS],
    input logic [2:0] routport_winner_i [NUM_PORTS],

    output logic [NUM_PORTS-1:0] route_vc_ready_o [NUM_PORTS],
    output logic [VC_ID_W-1:0] routport_vc_id_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;

  // NoCDAS currently uses two vnets with an equal number of VCs per vnet.
  localparam int VN_NUM_LOCAL = 2;
  localparam int VC_PER_VNET = VC_NUM / VN_NUM_LOCAL;
  localparam int VNET_URS = 0;
  localparam int VNET_LCS = 1;

  logic alloc_valid_q [NUM_PORTS][VC_NUM];
  logic alloc_valid_d [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_q [NUM_PORTS][VC_NUM];
  logic [2:0] alloc_out_port_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_q [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] alloc_dst_vc_d [NUM_PORTS][VC_NUM];
  logic [VC_ID_W-1:0] vc_rr_ptr_q [NUM_PORTS][VN_NUM_LOCAL];
  logic [VC_ID_W-1:0] vc_rr_ptr_d [NUM_PORTS][VN_NUM_LOCAL];

  logic [1:0] preferred_vnet [NUM_PORTS];
  logic [VC_NUM-1:0] vnet_mask [NUM_PORTS];
  logic [VC_NUM-1:0] head_alloc_mask [NUM_PORTS][NUM_PORTS];
  logic [VC_ID_W-1:0] selected_dst_vc [NUM_PORTS][NUM_PORTS];
  logic transfer_ready [NUM_PORTS][NUM_PORTS];
  router_traffic_class_e rinport_traffic_class [NUM_PORTS];
  logic rinport_is_lcs [NUM_PORTS];
  logic rinport_head_like [NUM_PORTS];
  logic rinport_tail_like [NUM_PORTS];

  always_comb begin : proc_rinport_meta_decode
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      unique case (rinport_meta_i[in_idx].msg_type)
        ROUTER_MSG_COMP: rinport_traffic_class[in_idx] = ROUTER_TRAFFIC_COMP;
        ROUTER_MSG_DIST: rinport_traffic_class[in_idx] = ROUTER_TRAFFIC_DIST;
        default:         rinport_traffic_class[in_idx] = ROUTER_TRAFFIC_REGULAR;
      endcase

      if (rinport_meta_i[in_idx].traffic_class <= ROUTER_TRAFFIC_COMP) begin
        rinport_traffic_class[in_idx] = rinport_meta_i[in_idx].traffic_class;
      end

      rinport_is_lcs[in_idx] =
          (rinport_traffic_class[in_idx] == ROUTER_TRAFFIC_COMP) ||
          (rinport_traffic_class[in_idx] == ROUTER_TRAFFIC_DIST);
      rinport_head_like[in_idx] = flit_is_head_like(rinport_meta_i[in_idx].flit_kind);
      rinport_tail_like[in_idx] = flit_is_tail_like(rinport_meta_i[in_idx].flit_kind);
    end
  end

  // Decode each input's target vnet once, then reuse the decoded value for VC
  // masks, selected VC calculation, and state updates.
  always_comb begin : proc_vnet_decode
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      preferred_vnet[in_idx] = 2'(VNET_URS);

`ifdef ENABLE_CNOC_MFU
      // cNoC distribution/compute traffic always uses the LCS VC pool.  This
      // preserves the NoCDAS split between regular traffic and high-priority
      // cNoC traffic even when the incoming header vnet field is stale.
      if (rinport_is_lcs[in_idx]) begin
        preferred_vnet[in_idx] = 2'(VNET_LCS);

      // Regular traffic keeps the vnet selected by the packet header when it is
      // within the implemented vnet range.
      end else if (rinport_meta_i[in_idx].vnet < VN_NUM_LOCAL) begin
        preferred_vnet[in_idx] = rinport_meta_i[in_idx].vnet[1:0];
      end
`else
      if (rinport_meta_i[in_idx].vnet < VN_NUM_LOCAL) begin
        preferred_vnet[in_idx] = rinport_meta_i[in_idx].vnet[1:0];
      end
`endif
    end
  end

  always_comb begin : proc_vnet_mask
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      int vnet_base;

      vnet_mask[in_idx] = '0;
      vnet_base = int'(preferred_vnet[in_idx]) * VC_PER_VNET;

      for (int vc_offset = 0; vc_offset < VC_PER_VNET; vc_offset = vc_offset + 1) begin
        if ((vnet_base + vc_offset) < VC_NUM) begin
          vnet_mask[in_idx][vnet_base + vc_offset] = 1'b1;
        end
      end
    end
  end

  always_comb begin : proc_head_alloc_mask
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        // A head may allocate a downstream VC only if that VC is both idle
        // packet-wise and has space for at least one flit.  The vnet mask
        // enforces traffic-class separation.
        head_alloc_mask[in_idx][out_idx] =
            routport_flow_i[out_idx].downstream_vc_idle_mask &
            routport_flow_i[out_idx].downstream_vc_credit_mask &
            vnet_mask[in_idx];
      end
    end
  end

  always_comb begin : proc_selected_dst_vc
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        int vnet_idx;
        int vnet_base;
        int start_offset;
        int candidate_offset;
        int candidate_vc;
        logic found_vc;

        selected_dst_vc[in_idx][out_idx] = '0;
        vnet_idx = 0;
        vnet_base = 0;
        start_offset = 0;
        candidate_offset = 0;
        candidate_vc = 0;
        found_vc = 1'b0;

        // Fallback to the lowest set bit if the vnet pointer is malformed.
        for (int vc_idx = VC_NUM - 1; vc_idx >= 0; vc_idx = vc_idx - 1) begin
          if (head_alloc_mask[in_idx][out_idx][vc_idx]) begin
            selected_dst_vc[in_idx][out_idx] = VC_ID_W'(vc_idx);
          end
        end

        // Normal case: route allocation is indexed by output port and vnet.  The
        // pointer rotates only within the selected vnet pool so regular traffic
        // and cNoC traffic cannot steal each other's VCs.
        vnet_idx = int'(preferred_vnet[in_idx]);
        if (vnet_idx < VN_NUM_LOCAL) begin
          vnet_base = vnet_idx * VC_PER_VNET;
          start_offset = int'(vc_rr_ptr_q[out_idx][vnet_idx]) - vnet_base;

          // If reset or parameter changes leave the pointer outside the selected
          // vnet, restart from that vnet's first VC instead of wrapping globally.
          if ((start_offset < 0) || (start_offset >= VC_PER_VNET)) begin
            start_offset = 0;
          end

          for (int vc_iter = 0; vc_iter < VC_PER_VNET; vc_iter = vc_iter + 1) begin
            candidate_offset = start_offset + vc_iter;
            if (candidate_offset >= VC_PER_VNET) begin
              candidate_offset = candidate_offset - VC_PER_VNET;
            end

            candidate_vc = vnet_base + candidate_offset;
            if (!found_vc &&
                (candidate_vc < VC_NUM) &&
                head_alloc_mask[in_idx][out_idx][candidate_vc]) begin
              selected_dst_vc[in_idx][out_idx] = VC_ID_W'(candidate_vc);
              found_vc = 1'b1;
            end
          end
        end
      end
    end
  end

  always_comb begin : proc_transfer_ready
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        logic [VC_ID_W-1:0] src_vc;

        src_vc = rinport_vc_id_i[in_idx];
        transfer_ready[in_idx][out_idx] = 1'b0;

        // The output link must be present/ready before either head allocation or
        // body/tail credit checks are meaningful.
        if (routport_flow_i[out_idx].state_ready) begin
          // Head and head-tail flits allocate a new downstream VC.
          if (rinport_head_like[in_idx]) begin
            transfer_ready[in_idx][out_idx] = |head_alloc_mask[in_idx][out_idx];

          // Body/tail flits must reuse the downstream VC allocated by their head,
          // and they may proceed only when that VC has credit this cycle.
          end else if (alloc_valid_q[in_idx][src_vc] &&
                       (alloc_out_port_q[in_idx][src_vc] == 3'(out_idx))) begin
            transfer_ready[in_idx][out_idx] =
                routport_flow_i[out_idx].downstream_vc_credit_mask[
                    alloc_dst_vc_q[in_idx][src_vc]
                ];
          end
        end
      end
    end
  end

  always_comb begin : proc_outputs
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      route_vc_ready_o[in_idx] = '0;
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        route_vc_ready_o[in_idx][out_idx] = transfer_ready[in_idx][out_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      // The selected output port publishes the destination VC for the input that
      // won that output's switch allocation.
      routport_vc_id_o[out_idx] = '0;
      if (routport_winner_i[out_idx] < NUM_PORTS) begin
        if (rinport_head_like[routport_winner_i[out_idx]]) begin
          routport_vc_id_o[out_idx] = selected_dst_vc[routport_winner_i[out_idx]][out_idx];
        end else begin
          routport_vc_id_o[out_idx] =
              alloc_dst_vc_q[routport_winner_i[out_idx]][rinport_vc_id_i[routport_winner_i[out_idx]]];
        end
      end
    end
  end

  always_comb begin : proc_alloc_state
    int winner_in_port;
    int winner_vnet_idx;
    int winner_vnet_base;
    int winner_vnet_last;
    logic [VC_ID_W-1:0] winner_src_vc;
    logic [VC_ID_W-1:0] winner_dst_vc;
    logic winner_head_like;
    logic winner_tail_like;

    winner_in_port = 0;
    winner_vnet_idx = 0;
    winner_vnet_base = 0;
    winner_vnet_last = 0;
    winner_src_vc = '0;
    winner_dst_vc = '0;
    winner_head_like = 1'b0;
    winner_tail_like = 1'b0;

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
        alloc_valid_d[in_idx][vc_idx] = alloc_valid_q[in_idx][vc_idx];
        alloc_out_port_d[in_idx][vc_idx] = alloc_out_port_q[in_idx][vc_idx];
        alloc_dst_vc_d[in_idx][vc_idx] = alloc_dst_vc_q[in_idx][vc_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
        vc_rr_ptr_d[out_idx][vnet_idx] = vc_rr_ptr_q[out_idx][vnet_idx];
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      winner_in_port = 0;
      winner_vnet_idx = 0;
      winner_vnet_base = 0;
      winner_vnet_last = 0;
      winner_src_vc = '0;
      winner_dst_vc = '0;
      winner_head_like = 1'b0;
      winner_tail_like = 1'b0;

      if (routport_winner_i[out_idx] != PORT_SEL_INVALID) begin
        winner_in_port = int'(routport_winner_i[out_idx]);
        if (winner_in_port < NUM_PORTS) begin
          winner_src_vc = rinport_vc_id_i[winner_in_port];
          winner_dst_vc = selected_dst_vc[winner_in_port][out_idx];
          winner_vnet_idx = int'(preferred_vnet[winner_in_port]);
          winner_head_like = rinport_head_like[winner_in_port];
          winner_tail_like = rinport_tail_like[winner_in_port];

          // Head/head-tail won the switch this cycle.  Consume one RR slot in the
          // chosen downstream vnet.  Multi-flit packets also remember the
          // allocation so body/tail flits remain ordered on the same downstream VC.
          if (winner_head_like && (winner_vnet_idx < VN_NUM_LOCAL)) begin
            winner_vnet_base = winner_vnet_idx * VC_PER_VNET;
            winner_vnet_last = winner_vnet_base + VC_PER_VNET - 1;
            if (int'(winner_dst_vc) >= winner_vnet_last) begin
              vc_rr_ptr_d[out_idx][winner_vnet_idx] = VC_ID_W'(winner_vnet_base);
            end else begin
              vc_rr_ptr_d[out_idx][winner_vnet_idx] =
                  winner_dst_vc + {{(VC_ID_W-1){1'b0}}, 1'b1};
            end

            if (!winner_tail_like) begin
              alloc_valid_d[winner_in_port][winner_src_vc] = 1'b1;
              alloc_out_port_d[winner_in_port][winner_src_vc] = 3'(out_idx);
              alloc_dst_vc_d[winner_in_port][winner_src_vc] = winner_dst_vc;
            end
          end

          // Tail and head-tail flits close the packet lifetime at this input VC.
          // A head-tail flit may advance the RR pointer above but does not keep a
          // stored VC allocation.
          if (winner_tail_like) begin
            alloc_valid_d[winner_in_port][winner_src_vc] = 1'b0;
            alloc_out_port_d[winner_in_port][winner_src_vc] = PORT_SEL_INVALID;
            alloc_dst_vc_d[winner_in_port][winner_src_vc] = '0;
          end
        end
      end
    end
  end

  always_ff @(posedge clk) begin : proc_registers
    // Reset clears all packet-to-downstream-VC mappings and positions every
    // output/vnet RR pointer at the first VC in that vnet.
    if (reset) begin
      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= 1'b0;
          alloc_out_port_q[in_idx][vc_idx] <= PORT_SEL_INVALID;
          alloc_dst_vc_q[in_idx][vc_idx] <= '0;
        end
      end

      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= VC_ID_W'(vnet_idx * VC_PER_VNET);
        end
      end
    end else begin
      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        for (int vc_idx = 0; vc_idx < VC_NUM; vc_idx = vc_idx + 1) begin
          alloc_valid_q[in_idx][vc_idx] <= alloc_valid_d[in_idx][vc_idx];
          alloc_out_port_q[in_idx][vc_idx] <= alloc_out_port_d[in_idx][vc_idx];
          alloc_dst_vc_q[in_idx][vc_idx] <= alloc_dst_vc_d[in_idx][vc_idx];
        end
      end

      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        for (int vnet_idx = 0; vnet_idx < VN_NUM_LOCAL; vnet_idx = vnet_idx + 1) begin
          vc_rr_ptr_q[out_idx][vnet_idx] <= vc_rr_ptr_d[out_idx][vnet_idx];
        end
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_assertions
    if (!reset) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        if (routport_winner_i[out_idx] != PORT_SEL_INVALID) begin
          logic [2:0] dbg_in_port;
          logic [VC_ID_W-1:0] dbg_src_vc;

          dbg_in_port = routport_winner_i[out_idx];
          dbg_src_vc = rinport_vc_id_i[dbg_in_port];

          // Switch allocation must never select a transfer whose downstream VC
          // precondition was false.  Otherwise the input FIFO would pop a flit
          // that cannot legally be accepted by the next router.
          if (!transfer_ready[dbg_in_port][out_idx]) begin
            $error("vc_allocator: output winner lacks downstream VC/credit readiness");
          end

          // Body/tail flits are wormhole followers.  They must reuse the
          // allocation created by the head and must not allocate a fresh VC.
          if (!rinport_head_like[dbg_in_port] &&
              (!alloc_valid_q[dbg_in_port][dbg_src_vc] ||
               (alloc_out_port_q[dbg_in_port][dbg_src_vc] != 3'(out_idx)))) begin
            $error("vc_allocator: body/tail transfer without matching head allocation");
          end
        end
      end
    end
  end
`endif

endmodule
