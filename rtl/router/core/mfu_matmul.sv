// Copyright (c) 2026
//
// Description: RTL-owned MatMul/Linear accumulation path for cNoC type5 flits.
//              Type4 distribution writes are mirrored into a local INT8 weight
//              bank.  Type5 input flits accumulate per assigned router task and
//              later psum flits are updated when their psum lanes pass through
//              the MFU.  This models the hardware-visible state movement
//              instead of asking the C++ wrapper to recompute the result.

module mfu_matmul #(
    parameter int NUM_PORTS = 5,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = 11,
    parameter int MAX_TASKS = router_ports_pkg::CNOC_MAX_TASKS,
    parameter int TASK_ID_W = router_ports_pkg::CNOC_TASK_ID_W
) (
    input  logic clk,
    input  logic reset,

    input  logic        weight_wr_en_i,
    input  logic [SRAM_ADDR_W-1:0] weight_wr_addr_i,
    input  logic [31:0] weight_wr_byte_en_i,
    input  logic [FLIT_W-1:0] weight_wr_data_i,

    input  logic compute_fire_i,
    input  logic release_state_i,
    input  logic pkt_is_type5_i,
    input  logic [4:0] pkt_opcode_i,
    input  logic [FLIT_W-1:0] pkt_flit_i,
    input  router_ports_pkg::flit_meta_t pkt_meta_i,
    input  logic [VC_ID_W-1:0] pkt_vc_id_i,
    input  logic [2:0] pkt_out_sel_i,

    input  logic [5:0] task_count_i,
    input  logic [MAX_TASKS*TASK_ID_W-1:0] task_ids_flat_i,
    input  logic [15:0] weight_row_size_i,

    output logic matmul_active_o,
    output logic [FLIT_W-1:0] matmul_flit_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;

  logic signed [31:0] accum_q [NUM_PORTS][VC_NUM][MAX_TASKS];
  logic [7:0] weight_bank [0:SRAM_DEPTH-1];

  logic [9:0] data_offset;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [15:0] psum_offset;
  logic [15:0] weight_row_size;
  logic head_like;
  logic tail_like;
  logic is_matmul_op;

  integer reset_port_idx;
  integer reset_vc_idx;
  integer reset_task_idx;
  integer reset_weight_idx;
  integer lane_idx;
  integer task_slot_idx;
  integer comb_task_slot_idx;
  integer wr_lane_idx;
  logic signed [31:0] comb_accum_q;
  logic [15:0] comb_task_id;
  logic [16:0] comb_target_idx;
  logic [16:0] comb_flit_start_idx;
  logic [16:0] comb_flit_end_idx;
  logic [5:0] comb_target_lane;
  logic signed [31:0] comb_old_psum_q;
  logic signed [31:0] comb_new_psum_q;
  logic [7:0] comb_new_psum_sat_byte;
  logic [TASK_ID_W-1:0] task_id_by_slot [MAX_TASKS];
  logic signed [31:0] lane_accum_by_task [MAX_TASKS];
  logic signed [31:0] base_accum_by_task [MAX_TASKS];
  logic signed [31:0] next_accum_by_task [MAX_TASKS];
  integer accum_task_idx;
  integer accum_lane_idx;
  logic [15:0] accum_input_idx;
  logic [31:0] accum_weight_addr_full;
  logic signed [31:0] accum_lhs_q;
  logic signed [31:0] accum_rhs_q;
  logic signed [31:0] accum_sum_q;
  logic signed [63:0] accum_product_q8;
  logic signed [63:0] accum_rounded_q4;
  logic [7:0] accum_sat_byte;
`ifdef RTL_DEBUG_MATMUL
  integer dbg_lane_idx;
  integer dbg_task_slot_idx;
  logic [15:0] dbg_task_id;
  logic [16:0] dbg_target_idx;
  logic [16:0] dbg_flit_start_idx;
  logic [16:0] dbg_flit_end_idx;
  logic signed [31:0] dbg_old_accum_q;
  logic signed [31:0] dbg_lane_accum_q;
  logic signed [31:0] dbg_next_accum_q;
  logic dbg_target_in_current_flit;
  logic [META_PACKET_UID_W-1:0] dbg_packet_uid;
  logic [META_FLIT_ID_W-1:0] dbg_flit_id;
`endif

  assign data_offset = pkt_meta_i.data_offset;
  assign payload_len = pkt_meta_i.payload_len;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign psum_offset = pkt_meta_i.psum_offset;
  assign weight_row_size =
      // Prefer the wrapper-provided row size when available; it reflects the
      // number of input elements assigned to each local task.
      (weight_row_size_i != 16'd0) ? weight_row_size_i :
      // Fallback for synthetic tests: psum_offset equals input-vector length.
      (psum_offset != 16'd0) ? psum_offset :
      // Degenerate safety default to avoid divide-by-zero style addressing.
                               16'd1;
  assign head_like = flit_is_head_like(pkt_meta_i.flit_kind);
  assign tail_like = flit_is_tail_like(pkt_meta_i.flit_kind);
  assign is_matmul_op = (pkt_opcode_i == OP_MATMUL) || (pkt_opcode_i == OP_LINEAR);
  assign matmul_active_o = pkt_is_type5_i && is_matmul_op;

  always_comb begin : proc_task_lane_accum
    accum_input_idx = 16'd0;
    accum_weight_addr_full = 32'd0;
    accum_lhs_q = 32'sd0;
    accum_rhs_q = 32'sd0;
    accum_sum_q = 32'sd0;
    accum_product_q8 = 64'sd0;
    accum_rounded_q4 = 64'sd0;
    accum_sat_byte = 8'd0;

    for (accum_task_idx = 0; accum_task_idx < MAX_TASKS; accum_task_idx = accum_task_idx + 1) begin
      task_id_by_slot[accum_task_idx] =
          task_ids_flat_i[accum_task_idx * TASK_ID_W +: TASK_ID_W];
      lane_accum_by_task[accum_task_idx] = 32'sd0;
      base_accum_by_task[accum_task_idx] =
          head_like ? 32'sd0 :
                      accum_q[pkt_out_sel_i][pkt_vc_id_i][accum_task_idx];

      for (accum_lane_idx = 0; accum_lane_idx < 32; accum_lane_idx = accum_lane_idx + 1) begin
        accum_input_idx = {6'd0, data_offset} + accum_lane_idx[15:0];
        accum_weight_addr_full = (accum_task_idx * weight_row_size) + accum_input_idx;
        // Accumulate only valid input lanes before psum_offset.  Lanes at
        // or beyond psum_offset are psum/output lanes and must not be used as
        // new input activations.
        if ((accum_lane_idx < effective_payload_len) &&
            (accum_input_idx < psum_offset) &&
            (accum_weight_addr_full < SRAM_DEPTH)) begin
          accum_lhs_q =
              {{24{pkt_flit_i[(accum_lane_idx * 8) + 7]}},
               pkt_flit_i[accum_lane_idx * 8 +: 8]};
          accum_rhs_q =
              {{24{weight_bank[accum_weight_addr_full[SRAM_ADDR_W-1:0]][7]}},
               weight_bank[accum_weight_addr_full[SRAM_ADDR_W-1:0]]};
          accum_product_q8 = accum_lhs_q * accum_rhs_q;
          if (accum_product_q8 >= 64'sd0) begin
            accum_rounded_q4 = (accum_product_q8 + 64'sd8) >>> 4;
          end else begin
            accum_rounded_q4 = -(((-accum_product_q8) + 64'sd8) >>> 4);
          end
          lane_accum_by_task[accum_task_idx] =
              lane_accum_by_task[accum_task_idx] + accum_rounded_q4[31:0];
        end
      end

      accum_sum_q = base_accum_by_task[accum_task_idx] + lane_accum_by_task[accum_task_idx];
      // Current co-sim payload contract stores psums as INT8 lanes.  Clamp each
      // temporal chunk so the RTL trace remains comparable with quant golden.
      if (accum_sum_q > 32'sd127) begin
        accum_sat_byte = 8'h7f;
      end else if (accum_sum_q < -32'sd128) begin
        accum_sat_byte = 8'h80;
      end else begin
        accum_sat_byte = accum_sum_q[7:0];
      end
      next_accum_by_task[accum_task_idx] = {{24{accum_sat_byte[7]}}, accum_sat_byte};
    end
  end

  always_ff @(posedge clk) begin
    // Reset clears both distributed weights and per-stream accumulators.
    if (reset) begin
      for (reset_weight_idx = 0; reset_weight_idx < SRAM_DEPTH; reset_weight_idx = reset_weight_idx + 1) begin
        weight_bank[reset_weight_idx] <= 8'd0;
      end
      for (reset_port_idx = 0; reset_port_idx < NUM_PORTS; reset_port_idx = reset_port_idx + 1) begin
        for (reset_vc_idx = 0; reset_vc_idx < VC_NUM; reset_vc_idx = reset_vc_idx + 1) begin
          for (reset_task_idx = 0; reset_task_idx < MAX_TASKS; reset_task_idx = reset_task_idx + 1) begin
            accum_q[reset_port_idx][reset_vc_idx][reset_task_idx] <= 32'sd0;
          end
        end
      end
    end else begin
      // Type4 non-Attention distribution writes the local INT8 weight bank.  The
      // byte enable lets a partially full flit update only valid lanes.
      if (weight_wr_en_i) begin
        for (wr_lane_idx = 0; wr_lane_idx < 32; wr_lane_idx = wr_lane_idx + 1) begin
          // Drop out-of-range bytes deterministically rather than wrapping.
          if (weight_wr_byte_en_i[wr_lane_idx] &&
              (weight_wr_addr_i + wr_lane_idx) < SRAM_DEPTH) begin
            weight_bank[weight_wr_addr_i + wr_lane_idx] <=
                weight_wr_data_i[wr_lane_idx * 8 +: 8];
          end
        end
      end

      // Type5 MatMul/Linear input flits accumulate local partial sums for every
      // task assigned to this router.
      if (compute_fire_i && matmul_active_o) begin
        for (task_slot_idx = 0; task_slot_idx < MAX_TASKS; task_slot_idx = task_slot_idx + 1) begin
          // Only configured task slots are active.  Unused slots stay zero.
          if (task_slot_idx < task_count_i) begin
            accum_q[pkt_out_sel_i][pkt_vc_id_i][task_slot_idx] <=
                next_accum_by_task[task_slot_idx];
          end else begin
            accum_q[pkt_out_sel_i][pkt_vc_id_i][task_slot_idx] <= 32'sd0;
          end
        end
      end

      // Tail/head-tail release clears the per-output/per-VC accumulator state
      // once the final psum writeback has left the MFU.
      if (release_state_i && matmul_active_o && tail_like) begin
        for (task_slot_idx = 0; task_slot_idx < MAX_TASKS; task_slot_idx = task_slot_idx + 1) begin
          accum_q[pkt_out_sel_i][pkt_vc_id_i][task_slot_idx] <= 32'sd0;
        end
      end
    end
  end

`ifdef RTL_DEBUG_MATMUL
  always_ff @(posedge clk) begin
    if (!reset) begin
      // Optional debug hook for correlating RTL-owned MatMul state with the C++
      // quantized golden.  Enable with CMake -DRTL_DEBUG_MATMUL=ON; it is
      // intentionally absent from normal regressions.
      if (weight_wr_en_i) begin
        for (dbg_lane_idx = 0; dbg_lane_idx < 32; dbg_lane_idx = dbg_lane_idx + 1) begin
          if (weight_wr_byte_en_i[dbg_lane_idx] &&
              (weight_wr_addr_i + dbg_lane_idx) < SRAM_DEPTH) begin
            $display("[RTL_DEBUG_MATMUL][weight] addr=%0d lane=%0d value=%0d",
                     weight_wr_addr_i + dbg_lane_idx,
                     dbg_lane_idx,
                     $signed(weight_wr_data_i[dbg_lane_idx * 8 +: 8]));
          end
        end
      end

      if (compute_fire_i && matmul_active_o) begin
        for (dbg_task_slot_idx = 0;
             dbg_task_slot_idx < MAX_TASKS;
             dbg_task_slot_idx = dbg_task_slot_idx + 1) begin
          if (dbg_task_slot_idx < task_count_i) begin
            dbg_task_id = task_id_by_slot[dbg_task_slot_idx];
            dbg_target_idx = {1'b0, psum_offset} + {1'b0, dbg_task_id};
            dbg_flit_start_idx = {7'd0, data_offset};
            dbg_flit_end_idx = dbg_flit_start_idx + {11'd0, effective_payload_len};
            dbg_old_accum_q = base_accum_by_task[dbg_task_slot_idx];
            dbg_lane_accum_q = lane_accum_by_task[dbg_task_slot_idx];
            dbg_next_accum_q = next_accum_by_task[dbg_task_slot_idx];
            dbg_target_in_current_flit =
                (dbg_target_idx >= dbg_flit_start_idx) &&
                (dbg_target_idx < dbg_flit_end_idx);
`ifdef ROUTER_ENABLE_COSIM
            dbg_packet_uid = pkt_meta_i.cosim.packet_uid;
            dbg_flit_id = pkt_meta_i.cosim.flit_id;
`else
            dbg_packet_uid = '0;
            dbg_flit_id = '0;
`endif

            $display("[RTL_DEBUG_MATMUL][compute] out=%0d vc=%0d packet_uid=%0d flit_id=%0d flit_kind=%0d data_offset=%0d payload_len=%0d psum_offset=%0d weight_row_size=%0d task_slot=%0d task_id=%0d lane_accum=%0d old_accum=%0d next_accum=%0d target_index=%0d target_in_current_flit=%0d emit_byte=%0d",
                     pkt_out_sel_i,
                     pkt_vc_id_i,
                     dbg_packet_uid,
                     dbg_flit_id,
                     pkt_meta_i.flit_kind,
                     data_offset,
                     effective_payload_len,
                     psum_offset,
                     weight_row_size,
                     dbg_task_slot_idx,
                     dbg_task_id,
                     dbg_lane_accum_q,
                     dbg_old_accum_q,
                     dbg_next_accum_q,
                     dbg_target_idx,
                     dbg_target_in_current_flit,
                     dbg_target_in_current_flit ?
                         $signed(matmul_flit_o[(dbg_target_idx[4:0] - data_offset[4:0]) * 8 +: 8]) :
                         0);
          end
        end
      end
    end
  end
`endif

  always_comb begin
    matmul_flit_o = pkt_flit_i;
    comb_task_id = 16'd0;
    comb_target_idx = 17'd0;
    comb_flit_start_idx = 17'd0;
    comb_flit_end_idx = 17'd0;
    comb_target_lane = 6'd0;
    comb_accum_q = 32'sd0;
    comb_old_psum_q = 32'sd0;
    comb_new_psum_q = 32'sd0;
    comb_new_psum_sat_byte = 8'd0;

    // Combinational writeback overlays accumulated psums onto the outgoing flit
    // only when that flit carries the target psum indices.
    if (matmul_active_o) begin
      for (comb_task_slot_idx = 0; comb_task_slot_idx < MAX_TASKS; comb_task_slot_idx = comb_task_slot_idx + 1) begin
        comb_task_id = task_id_by_slot[comb_task_slot_idx];
        comb_target_idx = {1'b0, psum_offset} + {1'b0, comb_task_id};
        comb_flit_start_idx = {7'd0, data_offset};
        comb_flit_end_idx = comb_flit_start_idx + {11'd0, effective_payload_len};
        comb_target_lane = comb_target_idx[5:0] - comb_flit_start_idx[5:0];
        comb_accum_q = accum_q[pkt_out_sel_i][pkt_vc_id_i][comb_task_slot_idx];
        if (compute_fire_i && matmul_active_o && (comb_task_slot_idx < task_count_i)) begin
          // Writeback and accumulator update observe the same flit.  Use the
          // next accumulator value here so a head-tail or psum-carrying flit can
          // emit the result of its own current multiply work instead of the
          // previous cycle's stale accumulator.
          comb_accum_q = next_accum_by_task[comb_task_slot_idx];
        end
        comb_old_psum_q = 32'sd0;
        comb_new_psum_q = 32'sd0;

        // A task updates the outgoing payload only when its psum target index is
        // located inside the current flit's payload window.
        if ((comb_task_slot_idx < task_count_i) &&
            (comb_target_idx >= comb_flit_start_idx) &&
            (comb_target_idx < comb_flit_end_idx) &&
            (comb_target_lane < 6'd32)) begin
          comb_old_psum_q =
              {{24{pkt_flit_i[(comb_target_lane * 8) + 7]}},
               pkt_flit_i[comb_target_lane * 8 +: 8]};
          comb_new_psum_q =
              comb_old_psum_q + comb_accum_q;
          if (comb_new_psum_q > 32'sd127) begin
            comb_new_psum_sat_byte = 8'h7f;
          end else if (comb_new_psum_q < -32'sd128) begin
            comb_new_psum_sat_byte = 8'h80;
          end else begin
            comb_new_psum_sat_byte = comb_new_psum_q[7:0];
          end
          matmul_flit_o[comb_target_lane * 8 +: 8] = comb_new_psum_sat_byte;
        end
      end
    end
  end

endmodule
