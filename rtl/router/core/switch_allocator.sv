// Description: Pipeline switch allocation stage for the Router main datapath.
//              The module arbitrates valid VA entries into per-output pipeline
//              slots.  It owns per-output round-robin pointers and the regular
//              traffic starvation guard, while CrossBar owns only data muxing.

module switch_allocator #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int STARVATION_LIMIT = 20
) (
    input  logic clk_i,
    input  logic reset_i,

    input  router_ports_pkg::router_pipe_entry_t va_entry_i [NUM_PORTS],
    input  router_ports_pkg::router_pipe_entry_t out_entry_i [NUM_PORTS],
    input  logic [NUM_PORTS-1:0] output_commit_fire_i,

    output logic [NUM_PORTS-1:0] va_fire_o,
    output logic [2:0] crossbar_select_o [NUM_PORTS],
    output logic crossbar_valid_o [NUM_PORTS]
);
  import router_ports_pkg::*;

  localparam logic [2:0] PORT_SEL_INVALID = ROUTER_PORT_INV;
  localparam int TRAFFIC_CLASS_NUM = 3;

  logic [2:0] switch_rr_ptr_q [NUM_PORTS];
  logic [2:0] switch_rr_ptr_d [NUM_PORTS];
  logic [7:0] regular_starve_cnt_q [NUM_PORTS];
  logic [7:0] regular_starve_cnt_d [NUM_PORTS];

  always_comb begin : proc_switch_select
    router_traffic_class_e selected_class;
    logic [2:0] selected_winner;
    logic selected_found_winner;
    logic selected_force_regular;
    logic selected_regular_waiting;
    int unsigned selected_candidate_port;

    selected_class = ROUTER_TRAFFIC_REGULAR;
    selected_winner = PORT_SEL_INVALID;
    selected_found_winner = 1'b0;
    selected_force_regular = 1'b0;
    selected_regular_waiting = 1'b0;
    selected_candidate_port = 0;

    va_fire_o = '0;

    for (int unsigned port_idx = 0; port_idx < NUM_PORTS; port_idx = port_idx + 1) begin
      crossbar_select_o[port_idx] = PORT_SEL_INVALID;
      crossbar_valid_o[port_idx] = 1'b0;
      switch_rr_ptr_d[port_idx] = switch_rr_ptr_q[port_idx];
      regular_starve_cnt_d[port_idx] = regular_starve_cnt_q[port_idx];
    end

    for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
      if (!out_entry_i[out_idx].valid || output_commit_fire_i[out_idx]) begin
        selected_winner = PORT_SEL_INVALID;
        selected_found_winner = 1'b0;
        selected_regular_waiting = 1'b0;
        selected_force_regular = 1'b0;

        for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
          if (va_entry_i[in_idx].valid &&
              (va_entry_i[in_idx].route_sel == 3'(out_idx)) &&
              (va_entry_i[in_idx].traffic_class == ROUTER_TRAFFIC_REGULAR)) begin
            selected_regular_waiting = 1'b1;
          end
        end

        if (selected_regular_waiting &&
            (regular_starve_cnt_q[out_idx] >= STARVATION_LIMIT[7:0])) begin
          selected_force_regular = 1'b1;
        end

        for (int unsigned prio_idx = 0; prio_idx < TRAFFIC_CLASS_NUM; prio_idx = prio_idx + 1) begin
          selected_class = ROUTER_TRAFFIC_REGULAR;
          if (selected_force_regular) begin
            case (2'(prio_idx))
              2'd0: selected_class = ROUTER_TRAFFIC_REGULAR;
              2'd1: selected_class = ROUTER_TRAFFIC_COMP;
              default: selected_class = ROUTER_TRAFFIC_DIST;
            endcase
          end else begin
            case (2'(prio_idx))
              2'd0: selected_class = ROUTER_TRAFFIC_COMP;
              2'd1: selected_class = ROUTER_TRAFFIC_DIST;
              default: selected_class = ROUTER_TRAFFIC_REGULAR;
            endcase
          end

          for (int unsigned rr_iter = 0; rr_iter < NUM_PORTS; rr_iter = rr_iter + 1) begin
            selected_candidate_port = int'(switch_rr_ptr_q[out_idx]) + rr_iter;
            if (selected_candidate_port >= NUM_PORTS) begin
              selected_candidate_port = selected_candidate_port - NUM_PORTS;
            end

            if (!selected_found_winner &&
                va_entry_i[selected_candidate_port].valid &&
                (va_entry_i[selected_candidate_port].route_sel == 3'(out_idx)) &&
                (va_entry_i[selected_candidate_port].traffic_class == selected_class)) begin
              selected_winner = 3'(selected_candidate_port);
              selected_found_winner = 1'b1;
            end
          end
        end

        if (selected_winner != PORT_SEL_INVALID) begin
          crossbar_select_o[out_idx] = selected_winner;
          crossbar_valid_o[out_idx] = 1'b1;
          va_fire_o[selected_winner] = 1'b1;

          switch_rr_ptr_d[out_idx] =
              (selected_winner == 3'(NUM_PORTS - 1)) ? '0 : (selected_winner + 3'd1);

          if (va_entry_i[selected_winner].traffic_class == ROUTER_TRAFFIC_REGULAR) begin
            regular_starve_cnt_d[out_idx] = 8'd0;
          end else if (selected_regular_waiting &&
                       (regular_starve_cnt_q[out_idx] < STARVATION_LIMIT[7:0])) begin
            regular_starve_cnt_d[out_idx] = regular_starve_cnt_q[out_idx] + 8'd1;
          end
        end else if (!selected_regular_waiting) begin
          regular_starve_cnt_d[out_idx] = 8'd0;
        end
      end
    end
  end

  always_ff @(posedge clk_i) begin : proc_registers
    if (reset_i) begin
      for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        switch_rr_ptr_q[out_idx] <= 3'(out_idx);
        regular_starve_cnt_q[out_idx] <= '0;
      end
    end else begin
      for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
        switch_rr_ptr_q[out_idx] <= switch_rr_ptr_d[out_idx];
        regular_starve_cnt_q[out_idx] <= regular_starve_cnt_d[out_idx];
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_assertions
    int grant_count;

    if (!reset_i) begin
      for (int unsigned in_idx = 0; in_idx < NUM_PORTS; in_idx = in_idx + 1) begin
        grant_count = 0;
        for (int unsigned out_idx = 0; out_idx < NUM_PORTS; out_idx = out_idx + 1) begin
          if (crossbar_valid_o[out_idx] && (crossbar_select_o[out_idx] == 3'(in_idx))) begin
            grant_count++;
          end
        end
        if (grant_count > 1) begin
          $error("switch_allocator: one VA entry selected by multiple outputs");
        end
      end
    end
  end
`endif

endmodule
