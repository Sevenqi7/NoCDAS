// Description: Per-output switch allocator for the five-port router.
//              Each output port runs an independent round-robin arbitration
//              over input ports that request that output.  cNoC MFU traffic is
//              additionally gated so that at most one MFU-bound flit is
//              captured in a cycle.

module switch_allocator #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int STARVATION_LIMIT = 20
) (
    // Router clock and active-high reset.
    input  logic clk_i,
    input  logic reset_i,
    // Output ports that can accept a transfer this cycle.
    input  logic [NUM_PORTS-1:0] routport_ready_i,
    // Input ports currently presenting a valid flit.
    input  logic [NUM_PORTS-1:0] rinport_req_i,
    // Requested output port for each input port.
    input  logic [2:0] rinport_route_sel_i [NUM_PORTS],
    // QoS class for each input port's selected flit.
    input  router_ports_pkg::router_traffic_class_e rinport_traffic_class_i [NUM_PORTS],
    // VC allocator readiness matrix indexed as [input_port][output_port].
    input  logic [NUM_PORTS-1:0] route_vc_ready_i [NUM_PORTS],
    // Input ports whose selected flit must pass through the MFU.
    input  logic [NUM_PORTS-1:0] mfu_rinport_req_i,
    // MFU cannot capture a new flit while busy.
    input  logic mfu_busy_i,
    // Winner input for each output port.
    output logic [2:0] routport_winner_o [NUM_PORTS],
    // Pop grant returned to each input port.
    output logic [NUM_PORTS-1:0] rinport_grant_o
);
  import router_ports_pkg::*;

  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;
  localparam int STARVATION_LIMIT_Q = STARVATION_LIMIT;
  localparam int TRAFFIC_CLASS_NUM = 3;

  logic [2:0] rr_ptr_q [NUM_PORTS];
  logic [2:0] rr_ptr_d [NUM_PORTS];
  logic [2:0] mfu_rr_ptr_q;
  logic [2:0] mfu_rr_ptr_d;
  logic [2:0] mfu_winner;
  int unsigned regular_starve_cnt_q [NUM_PORTS];
  int unsigned regular_starve_cnt_d [NUM_PORTS];

  logic [NUM_PORTS-1:0] request_to_output [NUM_PORTS];
  logic [NUM_PORTS-1:0] request_has_vc [NUM_PORTS];
  logic [NUM_PORTS-1:0] request_output_ready [NUM_PORTS];
  logic [NUM_PORTS-1:0] request_eligible [NUM_PORTS];
  logic [NUM_PORTS-1:0] mfu_candidate_by_class [TRAFFIC_CLASS_NUM];
  logic [NUM_PORTS-1:0] mfu_allowed;
  logic [NUM_PORTS-1:0] regular_waiting;
  logic [NUM_PORTS-1:0] force_regular;
  router_traffic_class_e priority_class [NUM_PORTS][TRAFFIC_CLASS_NUM];

  always_comb begin : proc_request_masks
    for (int prio_idx = 0; prio_idx < TRAFFIC_CLASS_NUM; prio_idx = prio_idx + 1) begin
      mfu_candidate_by_class[prio_idx] = '0;
    end

    regular_waiting = '0;

    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        request_to_output[in_idx][out_idx] =
            rinport_req_i[in_idx] &&
            (rinport_route_sel_i[in_idx] == out_idx[2:0]);

        request_has_vc[in_idx][out_idx] =
            request_to_output[in_idx][out_idx] &&
            route_vc_ready_i[in_idx][out_idx];

        request_output_ready[in_idx][out_idx] =
            request_has_vc[in_idx][out_idx] &&
            routport_ready_i[out_idx];

        // The starvation counter should advance only when a regular packet could
        // have used this output but lost to higher-priority traffic.
        if (request_has_vc[in_idx][out_idx] &&
            (rinport_traffic_class_i[in_idx] == ROUTER_TRAFFIC_REGULAR)) begin
          regular_waiting[out_idx] = 1'b1;
        end
      end

      // MFU arbitration is a global side arbitration: the flit must need MFU
      // service, must be able to route to its output, and must belong to the
      // priority class currently being scanned.
      if (mfu_rinport_req_i[in_idx] &&
          (rinport_route_sel_i[in_idx] < NUM_PORTS) &&
          request_output_ready[in_idx][rinport_route_sel_i[in_idx]]) begin
        unique case (rinport_traffic_class_i[in_idx])
          ROUTER_TRAFFIC_COMP:    mfu_candidate_by_class[ROUTER_TRAFFIC_COMP][in_idx] = 1'b1;
          ROUTER_TRAFFIC_DIST:    mfu_candidate_by_class[ROUTER_TRAFFIC_DIST][in_idx] = 1'b1;
          default:                mfu_candidate_by_class[ROUTER_TRAFFIC_REGULAR][in_idx] = 1'b1;
        endcase
      end
    end
  end

  always_comb begin : proc_mfu_arb
    int candidate_idx;

    candidate_idx = 0;
    mfu_winner = PORT_SEL_INVALID;

    // First choose at most one MFU-bound input globally.  This prevents two
    // switch winners from entering the single MFU in the same cycle.
    if (!mfu_busy_i) begin
      for (int prio_idx = int'(ROUTER_TRAFFIC_COMP);
           prio_idx >= int'(ROUTER_TRAFFIC_REGULAR);
           prio_idx = prio_idx - 1) begin
        for (int rr_iter = 0; rr_iter < NUM_PORTS; rr_iter = rr_iter + 1) begin
          candidate_idx = int'(mfu_rr_ptr_q) + rr_iter;
          if (candidate_idx >= NUM_PORTS) begin
            candidate_idx = candidate_idx - NUM_PORTS;
          end

          if ((mfu_winner == PORT_SEL_INVALID) &&
              mfu_candidate_by_class[prio_idx][candidate_idx]) begin
            mfu_winner = candidate_idx[2:0];
          end
        end
      end
    end
  end

  always_comb begin : proc_mfu_allowed
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      // MFU-bound flits cannot bypass the MFU.  They may leave the crossbar only
      // when the MFU is free and this input is the single MFU winner.
      if (mfu_rinport_req_i[in_idx]) begin
        mfu_allowed[in_idx] = !mfu_busy_i && (in_idx[2:0] == mfu_winner);
      end else begin
        mfu_allowed[in_idx] = 1'b1;
      end
    end
  end

  always_comb begin : proc_request_eligible
    for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        // Final per-output eligibility: request present, route matches this
        // output, downstream VC is available, and MFU gating does not block.
        request_eligible[in_idx][out_idx] =
            request_has_vc[in_idx][out_idx] &&
            mfu_allowed[in_idx];
      end
    end
  end

  always_comb begin : proc_priority_order
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      force_regular[out_idx] =
          regular_waiting[out_idx] &&
          (regular_starve_cnt_q[out_idx] >= STARVATION_LIMIT_Q);

      // When the starvation guard fires, regular traffic is temporarily scanned
      // first, then normal cNoC priority resumes for the remaining slots.
      if (force_regular[out_idx]) begin
        priority_class[out_idx][0] = ROUTER_TRAFFIC_REGULAR;
        priority_class[out_idx][1] = ROUTER_TRAFFIC_COMP;
        priority_class[out_idx][2] = ROUTER_TRAFFIC_DIST;

      // Normal QoS order: type5 compute > type4 distribution > regular.
      end else begin
        priority_class[out_idx][0] = ROUTER_TRAFFIC_COMP;
        priority_class[out_idx][1] = ROUTER_TRAFFIC_DIST;
        priority_class[out_idx][2] = ROUTER_TRAFFIC_REGULAR;
      end
    end
  end

  always_comb begin : proc_switch_grant
    int candidate_idx;

    candidate_idx = 0;
    rinport_grant_o = '0;

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      routport_winner_o[out_idx] = PORT_SEL_INVALID;
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      // Each output port arbitrates independently, allowing multiple outputs to
      // fire in the same cycle when their winners are different input ports.
      if (routport_ready_i[out_idx]) begin
        for (int prio_idx = 0; prio_idx < TRAFFIC_CLASS_NUM; prio_idx = prio_idx + 1) begin
          for (int rr_iter = 0; rr_iter < NUM_PORTS; rr_iter = rr_iter + 1) begin
            candidate_idx = int'(rr_ptr_q[out_idx]) + rr_iter;
            if (candidate_idx >= NUM_PORTS) begin
              candidate_idx = candidate_idx - NUM_PORTS;
            end

            // The first eligible request in priority/RR order wins this output.
            if ((routport_winner_o[out_idx] == PORT_SEL_INVALID) &&
                request_eligible[candidate_idx][out_idx] &&
                (rinport_traffic_class_i[candidate_idx] == priority_class[out_idx][prio_idx])) begin
              routport_winner_o[out_idx] = candidate_idx[2:0];
            end
          end
        end
      end
    end

    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      // Convert per-output winners back into per-input pop grants.  The router
      // architecture assumes an input requests only one output in a cycle.
      if (routport_winner_o[out_idx] != PORT_SEL_INVALID) begin
        rinport_grant_o[routport_winner_o[out_idx]] = 1'b1;
      end
    end
  end

  always_comb begin : proc_switch_state
    for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      rr_ptr_d[out_idx] = rr_ptr_q[out_idx];
      regular_starve_cnt_d[out_idx] = regular_starve_cnt_q[out_idx];

      // A granted output advances its RR pointer to the input after the winner.
      if (routport_winner_o[out_idx] != PORT_SEL_INVALID) begin
        rr_ptr_d[out_idx] =
            (routport_winner_o[out_idx] == (NUM_PORTS - 1)) ?
            '0 : (routport_winner_o[out_idx] + 3'd1);

        // Serving regular traffic clears the starvation counter for that output.
        if (rinport_traffic_class_i[routport_winner_o[out_idx]] == ROUTER_TRAFFIC_REGULAR) begin
          regular_starve_cnt_d[out_idx] = 8'd0;

        // Serving higher-priority traffic while regular traffic is waiting
        // increments the guard counter until it reaches the configured limit.
        end else if (regular_waiting[out_idx] &&
                     (regular_starve_cnt_q[out_idx] < STARVATION_LIMIT_Q)) begin
          regular_starve_cnt_d[out_idx] = regular_starve_cnt_q[out_idx] + 8'd1;
        end

      // If no regular request is waiting, stale starvation history is cleared.
      end else if (!regular_waiting[out_idx]) begin
        regular_starve_cnt_d[out_idx] = 8'd0;
      end
    end

    mfu_rr_ptr_d = mfu_rr_ptr_q;
    // The MFU winner pointer rotates only when a cNoC/MFU flit is selected,
    // preserving the previous fairness behavior across MFU-bound traffic.
    if (mfu_winner != PORT_SEL_INVALID) begin
      mfu_rr_ptr_d = (mfu_winner == (NUM_PORTS - 1)) ? '0 : (mfu_winner + 3'd1);
    end
  end

  always_ff @(posedge clk_i) begin : proc_registers
    // Reset uses deterministic staggered RR pointers so simultaneous requests do
    // not always favor input 0 on every output after reset.
    if (reset_i) begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        rr_ptr_q[out_idx] <= out_idx[2:0];
        regular_starve_cnt_q[out_idx] <= 8'd0;
      end
      mfu_rr_ptr_q <= '0;
    end else begin
      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        rr_ptr_q[out_idx] <= rr_ptr_d[out_idx];
        regular_starve_cnt_q[out_idx] <= regular_starve_cnt_d[out_idx];
      end
      mfu_rr_ptr_q <= mfu_rr_ptr_d;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_assertions
    int grant_count;

    if (!reset_i) begin
      for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        // A single router input has one selected VC and therefore can only be
        // granted once per cycle.  Multiple grants would duplicate/pop the same
        // flit into more than one output.
        grant_count = 0;
        for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
          if (routport_winner_o[out_idx] == in_idx[2:0]) begin
            grant_count++;
          end
        end
        if (grant_count > 1) begin
          $error("switch_allocator: one input granted to multiple outputs");
        end
      end

      for (int out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        if (routport_winner_o[out_idx] != PORT_SEL_INVALID) begin
          // Winners must correspond to a real request for this output.  This is
          // the core no-phantom-flit invariant for switch allocation.
          if (!request_eligible[routport_winner_o[out_idx]][out_idx]) begin
            $error("switch_allocator: output winner does not satisfy request eligibility");
          end
        end
      end

      if (mfu_busy_i) begin
        for (int in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
          // MFU-bound flits cannot bypass while the single-entry MFU is busy.
          // If this fires, a cNoC flit was popped without being capturable.
          if (mfu_rinport_req_i[in_idx] && rinport_grant_o[in_idx]) begin
            $error("switch_allocator: granted MFU-bound flit while MFU busy");
          end
        end
      end
    end
  end
`endif

endmodule
