// Description: cNoC MFU datapath wrapper.
//              Captures cNoC flits selected by the router datapath, performs
//              the functional MFU operation, updates metadata, and re-emits the
//              flit toward the originally selected output port.

module cnoc_mfu #(
    parameter int NUM_PORTS = 5,
    parameter int VC_NUM = 8,
    parameter int VC_ID_W = 3,
    parameter int FLIT_W = 256,
    parameter int MFU_LAT_LINEAR = 0,
    parameter int MFU_LAT_MATMUL = 0,
    parameter int MFU_LAT_ADD = 0,
    parameter int MFU_LAT_SWIGLU = 0,
    parameter int MFU_LAT_GEGLU = 0,
    parameter int MFU_LAT_ATTENTION = 8,
    parameter int MFU_LAT_DEFAULT = 0,
    parameter int MFU_LAT_TYPE4_STORE = 0
) (
    input  logic clk,
    input  logic reset,

    input  logic [FLIT_W-1:0] raw_flit_i,
    input  router_ports_pkg::flit_meta_t raw_meta_i,
    input  router_ports_pkg::route_path_t raw_route_path_i,
    input  logic [VC_ID_W-1:0] raw_vc_id_i,
    input  logic [2:0] raw_out_sel_i,
    input  logic raw_valid_i,

    input  router_ports_pkg::routport_flow_t routport_flow_i [NUM_PORTS],
    input  logic [5:0] cnoc_task_count_i,
    input  logic [router_ports_pkg::CNOC_MAX_TASKS*
                  router_ports_pkg::CNOC_TASK_ID_W-1:0] cnoc_task_ids_flat_i,
    input  logic [15:0] cnoc_weight_row_size_i,

    output logic busy_o,
    output logic capture_o,
    output logic [2:0] capture_out_sel_o,

    output logic emit_valid_o,
    output logic [2:0] emit_out_sel_o,
    output logic [FLIT_W-1:0] emit_flit_o,
    output router_ports_pkg::flit_meta_t emit_meta_o,
    output router_ports_pkg::route_path_t emit_route_path_o,
    output logic [VC_ID_W-1:0] emit_vc_id_o,

    output logic [31:0] weight_bytes_stored_o,
    output logic [31:0] kv_bytes_stored_o,
    output logic [15:0] kv_token_count_o,
    output logic        last_type4_store_valid_o,
    output logic        last_type4_store_bank_o,
    output logic [10:0] last_type4_store_addr_o,
    output logic [5:0]  last_type4_store_bytes_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OPCODE_ATTENTION = 5'd23;
  localparam int SRAM_DEPTH = 2048;
  localparam logic [31:0] SRAM_DEPTH_U32 = 32'd2048;
  localparam int SRAM_ADDR_W = 11;
  localparam int SINK_TOKENS = 4;
  localparam logic [15:0] SINK_TOKENS_Q = 16'd4;

  // Single-entry cNoC packet register.  The MFU owns one flit until ALU/MFU
  // work finishes and the original output port has downstream credit.
  logic pkt_valid_q;
  logic [FLIT_W-1:0] pkt_flit_q;
  flit_meta_t pkt_meta_q;
  route_path_t pkt_route_path_q;
  logic [VC_ID_W-1:0] pkt_vc_q;
  logic [2:0] pkt_out_sel_q;

  // DECODE stage fields from the registered cNoC flit.
  logic [4:0] decode_opcode;
  logic [9:0] decode_data_idx;
  logic decode_is_type4;
  logic decode_is_type5;
  logic decode_tail_like;
  logic [4:0] decoded_opcode_q;
  logic [9:0] decoded_data_idx_q;
  logic [5:0] decoded_payload_len_q;
  logic [15:0] decoded_k_dim_q;
  logic decoded_is_type4_q;
  logic decoded_is_type5_q;
  logic decoded_tail_like_q;
  logic [4:0] decoded_data_lane;
  logic signed [7:0] decoded_payload_lane;

  logic fsm_busy;
  logic decode_en;
  logic fetch_en;
  logic compute_en;
  logic write_back_en;
  logic compute_done;
  logic compute_fire;
  logic alu_start;
  logic alu_busy;
  logic alu_result_valid;
  logic emit_ready;

  // Type4 distribution path: payload bytes are written into either the weight
  // SRAM bank or the Attention KV bank.
  logic [7:0] sram_rdata;
  logic sram_wr_en;
  logic sram_wr_bank_sel;
  logic [SRAM_ADDR_W-1:0] sram_wr_addr;
  logic [31:0] sram_wr_byte_en;
  logic [255:0] sram_wr_data;
  logic sram_rd_bank_sel;
  logic [SRAM_ADDR_W-1:0] sram_rd_addr;
  logic [SRAM_ADDR_W-1:0] storage_wr_addr;
  logic [15:0] kv_token_count_q;
  logic [15:0] kv_token_stride;
  logic [15:0] kv_max_tokens_raw;
  logic [15:0] kv_max_tokens_safe;
  logic [15:0] kv_sink_limit;
  logic [31:0] kv_max_tokens_full;
  logic [15:0] kv_ring_span;
  logic [15:0] kv_slot;
  logic [31:0] storage_wr_addr_full;
  logic [5:0] effective_payload_bytes;
  logic [31:0] bytes_to_sram_end_full;
  logic [5:0] bytes_to_sram_end;
  logic [5:0] type4_store_bytes;
  logic matmul_weight_wr_en;
  logic attention_kv_wr_en;

  // Type5 compute/writeback path.  ALU results are latched so writeback only
  // observes a result after the latency-aware ALU has asserted result_valid.
  logic signed [31:0] fetch_operand_a_q;
  logic signed [31:0] fetch_operand_b_q;
  logic [FLIT_W-1:0] alu_result_flit;
  flit_meta_t alu_result_meta;
  logic [FLIT_W-1:0] processed_flit_q;
  flit_meta_t processed_meta_q;
  logic [FLIT_W-1:0] writeback_flit;
  logic [FLIT_W-1:0] matmul_writeback_flit;
  logic [FLIT_W-1:0] attention_writeback_flit;
  flit_meta_t attention_writeback_meta;
  logic matmul_active;
  logic attention_active;
  logic type5_tail_emit;

  // Status bridge for NoCDAS phase gating and trace diagnostics.  These are not
  // used to make RTL routing/storage/compute decisions.
  logic [31:0] weight_bytes_stored_q;
  logic [31:0] kv_bytes_stored_q;
  logic last_type4_store_valid_q;
  logic last_type4_store_bank_q;
  logic [SRAM_ADDR_W-1:0] last_type4_store_addr_q;
  logic [5:0] last_type4_store_bytes_q;

  // Capture/control path.  Non-cNoC traffic is intentionally invisible here and
  // remains on Router.sv's regular datapath.
  assign capture_o = raw_valid_i && !pkt_valid_q && !fsm_busy;
  assign capture_out_sel_o = raw_out_sel_i;
  assign busy_o = pkt_valid_q || fsm_busy || alu_busy;
  assign alu_start = pkt_valid_q && compute_en && !alu_busy && !alu_result_valid;
  assign compute_done = alu_result_valid;
  assign compute_fire = alu_result_valid;
  assign emit_ready =
      routport_flow_i[pkt_out_sel_q].state_ready &&
      routport_flow_i[pkt_out_sel_q].downstream_vc_credit_mask[pkt_vc_q];
  assign emit_valid_o = pkt_valid_q && write_back_en && emit_ready;
  assign emit_out_sel_o = pkt_out_sel_q;
  assign emit_flit_o = writeback_flit;
  assign emit_route_path_o = pkt_route_path_q;
  assign emit_vc_id_o = pkt_vc_q;

  // Type4 storage/KV address generation.
  assign kv_token_stride = (decoded_k_dim_q == 16'd0) ? 16'd1 : (decoded_k_dim_q << 1);
  assign kv_max_tokens_full = (kv_token_stride == 16'd0) ? 32'd1 : (SRAM_DEPTH / kv_token_stride);
  assign kv_max_tokens_raw =
      // Degenerate k_dim/storage cases still expose at least one safe logical
      // token slot through kv_max_tokens_safe below.
      (kv_max_tokens_full == 32'd0) ? 16'd0 :
      // Clamp large capacities into the 16-bit token-count domain.
      (kv_max_tokens_full > 32'h0000_ffff) ? 16'hffff :
      // Normal case: SRAM depth divided by K/V token stride.
                                             kv_max_tokens_full[15:0];
  assign kv_max_tokens_safe = (kv_max_tokens_raw == 16'd0) ? 16'd1 : kv_max_tokens_raw;
  assign kv_sink_limit =
      // Preserve up to SINK_TOKENS initial tokens, but never more than the local
      // SRAM can hold.
      (kv_max_tokens_safe > SINK_TOKENS_Q) ? SINK_TOKENS_Q : kv_max_tokens_safe;
  assign kv_ring_span =
      // If there is space after sink tokens, later tokens rotate through that
      // ring.  Otherwise all rolling writes collapse to the final safe slot.
      (kv_max_tokens_safe > kv_sink_limit) ? (kv_max_tokens_safe - kv_sink_limit) : 16'd1;
  assign kv_slot =
      // Initial prefill tokens are pinned as sink tokens.
      (kv_token_count_q < kv_sink_limit) ?
      kv_token_count_q :
      // Later tokens use ring-buffer replacement after the sink region.
      (kv_max_tokens_safe > kv_sink_limit) ?
      (kv_sink_limit + ((kv_token_count_q - kv_sink_limit) % kv_ring_span)) :
      // Fully degenerate case: one usable token slot remains.
      (kv_max_tokens_safe - 16'd1);
  assign storage_wr_addr_full =
      // Attention type4 packets load KV-cache slots.  The address is token slot
      // base plus the flit's data offset within K/V.
      (decoded_opcode_q == OPCODE_ATTENTION) ?
      ((kv_slot * kv_token_stride) + decoded_data_idx_q) :
      // Non-Attention type4 packets load the linear/MatMul weight bank directly
      // at data_offset.
      decoded_data_idx_q;
  assign storage_wr_addr = storage_wr_addr_full[SRAM_ADDR_W-1:0];
  assign effective_payload_bytes =
      // Padding-only flits carry no SRAM/KV bytes.  Tail flits may still mark a
      // logical token complete, but they must not overwrite address 0 through
      // an implied scalar payload lane.
      (decoded_payload_len_q == 6'd0) ? 6'd0 :
      // A 256-bit flit can carry at most 32 INT8 lanes.
      (decoded_payload_len_q > 6'd32) ? 6'd32 :
                                        decoded_payload_len_q;
  assign bytes_to_sram_end_full =
      // Fully out-of-range writes become no-ops rather than wrapping in SRAM.
      (storage_wr_addr_full >= SRAM_DEPTH) ? 32'd0 :
      // Otherwise limit the write to the remaining bytes in the local bank.
                                             (SRAM_DEPTH_U32 - storage_wr_addr_full);
  assign bytes_to_sram_end =
      (bytes_to_sram_end_full > 32'd32) ? 6'd32 :
                                          bytes_to_sram_end_full[5:0];
  assign type4_store_bytes =
      // Write no more than both the valid payload bytes and the remaining SRAM
      // capacity.
      (effective_payload_bytes < bytes_to_sram_end) ? effective_payload_bytes :
                                                      bytes_to_sram_end;
  assign matmul_weight_wr_en = sram_wr_en && !sram_wr_bank_sel;
  assign attention_kv_wr_en = sram_wr_en && sram_wr_bank_sel;

  // Type5 ALU/MFU side inputs.
  assign decoded_data_lane = decoded_data_idx_q[4:0];
  assign decoded_payload_lane = pkt_flit_q[{decoded_data_lane, 3'b000} +: 8];
  assign type5_tail_emit = emit_valid_o && decoded_is_type5_q && decoded_tail_like_q;

  // Status bridge outputs.
  assign weight_bytes_stored_o = weight_bytes_stored_q;
  assign kv_bytes_stored_o = kv_bytes_stored_q;
  assign kv_token_count_o = kv_token_count_q;
  assign last_type4_store_valid_o = last_type4_store_valid_q;
  assign last_type4_store_bank_o = last_type4_store_bank_q;
  assign last_type4_store_addr_o = last_type4_store_addr_q;
  assign last_type4_store_bytes_o = last_type4_store_bytes_q;

  always_ff @(posedge clk) begin : proc_mfu_registers
    // Reset clears the one-entry MFU pipeline and the status bridge counters.
    if (reset) begin
      pkt_valid_q <= 1'b0;
      pkt_flit_q <= '0;
      pkt_meta_q <= '0;
      pkt_route_path_q <= '0;
      pkt_vc_q <= '0;
      pkt_out_sel_q <= '0;
      decoded_opcode_q <= 5'd0;
      decoded_data_idx_q <= 10'd0;
      decoded_payload_len_q <= 6'd0;
      decoded_k_dim_q <= 16'd0;
      decoded_is_type4_q <= 1'b0;
      decoded_is_type5_q <= 1'b0;
      decoded_tail_like_q <= 1'b0;
      fetch_operand_a_q <= 32'sd0;
      fetch_operand_b_q <= 32'sd0;
      processed_flit_q <= '0;
      processed_meta_q <= '0;
      kv_token_count_q <= 16'd0;
      weight_bytes_stored_q <= 32'd0;
      kv_bytes_stored_q <= 32'd0;
      last_type4_store_valid_q <= 1'b0;
      last_type4_store_bank_q <= 1'b0;
      last_type4_store_addr_q <= '0;
      last_type4_store_bytes_q <= '0;
    end else begin
      // last_type4_store_* is a one-cycle event pulse used by the wrapper/status
      // bridge; clear it unless a type4 emit below refreshes it.
      last_type4_store_valid_q <= 1'b0;

      // Capture a newly selected cNoC flit when the MFU pipeline is empty.  The
      // raw flit is removed from the normal output path by Router.sv in the same
      // cycle through mfu_capture.
      if (capture_o) begin
        pkt_valid_q <= 1'b1;
        pkt_flit_q <= raw_flit_i;
        pkt_meta_q <= raw_meta_i;
        pkt_route_path_q <= raw_route_path_i;
        pkt_vc_q <= raw_vc_id_i;
        pkt_out_sel_q <= raw_out_sel_i;
      end

      // DECODE is the first stage that interprets the locked flit metadata.
      if (decode_en) begin
        decoded_opcode_q <= decode_opcode;
        decoded_data_idx_q <= decode_data_idx;
        decoded_payload_len_q <= pkt_meta_q.payload_len;
        decoded_k_dim_q <= pkt_meta_q.k_dim;
        decoded_is_type4_q <= decode_is_type4;
        decoded_is_type5_q <= decode_is_type5;
        decoded_tail_like_q <= decode_tail_like;
      end

      // FETCH latches scalar operands before COMPUTE can start the ALU backend.
      if (fetch_en) begin
        fetch_operand_a_q <= {{24{decoded_payload_lane[7]}}, decoded_payload_lane};
        fetch_operand_b_q <= {{24{sram_rdata[7]}}, sram_rdata};
      end

      // Latch the ALU payload/meta exactly when the latency-aware ALU declares
      // the functional result visible to the rest of the MFU pipeline.
      if (alu_result_valid) begin
        processed_flit_q <= alu_result_flit;
        processed_meta_q <= alu_result_meta;
      end

      // emit_valid_o means the processed flit has both completed MFU work and
      // has downstream credit to leave this router.
      if (emit_valid_o) begin
        // Type4 emit is the point where the local SRAM/KV write has committed.
        // Update the status bridge from the same transaction.
        if (decoded_is_type4_q) begin
          last_type4_store_valid_q <= 1'b1;
          last_type4_store_bank_q <= (decoded_opcode_q == OPCODE_ATTENTION);
          last_type4_store_addr_q <= storage_wr_addr;
          last_type4_store_bytes_q <= type4_store_bytes;
          // Attention type4 packets fill the KV-cache bank.
          if (decoded_opcode_q == OPCODE_ATTENTION) begin
            kv_bytes_stored_q <= kv_bytes_stored_q + {26'd0, type4_store_bytes};
          // Other type4 packets fill the weight bank.
          end else begin
            weight_bytes_stored_q <= weight_bytes_stored_q + {26'd0, type4_store_bytes};
          end
        end
        // Tail/head-tail of an Attention type4 packet marks one logical KV token
        // complete, so the next token advances to the next sink/ring slot.
        if (decoded_is_type4_q && decoded_opcode_q == OPCODE_ATTENTION &&
            decoded_tail_like_q) begin
          kv_token_count_q <= kv_token_count_q + 16'd1;
        end
        // The one-entry MFU pipeline is now free to capture the next cNoC flit.
        pkt_valid_q <= 1'b0;
      end
    end
  end

  cnoc_decode dec_pkt_i (
      .flit_meta(pkt_meta_q),
      .flit_type(),
      .msg_type(),
      .opcode(decode_opcode),
      .data_idx(decode_data_idx),
      .is_type4(decode_is_type4),
      .is_type5(decode_is_type5),
      .is_tail(decode_tail_like)
  );

  mfu_ctrl_fsm fsm_mfu_i (
      .clk(clk),
      .reset(reset),
      .start(capture_o),
      .compute_done(compute_done),
      .write_back_ready(emit_ready),
      .busy(fsm_busy),
      .decode_en(decode_en),
      .fetch_en(fetch_en),
      .compute_en(compute_en),
      .write_back_en(write_back_en)
  );

  mfu_sram #(
      .DATA_W(8),
      .DEPTH(SRAM_DEPTH),
      .ADDR_W(SRAM_ADDR_W)
  ) mfu_mem_i (
      .clk(clk),
      .reset(reset),
      .wr_en(sram_wr_en),
      .wr_bank_sel(sram_wr_bank_sel),
      .wr_addr(sram_wr_addr),
      .wr_byte_en(sram_wr_byte_en),
      .wr_data(sram_wr_data),
      .rd_bank_sel(sram_rd_bank_sel),
      .rd_addr(sram_rd_addr),
      .rd_data(sram_rdata)
  );

  mfu_sram_if #(
      .ADDR_W(SRAM_ADDR_W)
  ) mfu_sram_if_i (
      .emit_valid_i(emit_valid_o),
      .pkt_is_type4_i(decoded_is_type4_q),
      .pkt_opcode_i(decoded_opcode_q),
      .pkt_data_idx_i(decoded_data_idx_q),
      .pkt_payload_len_i(decoded_payload_len_q),
      .pkt_payload_i(pkt_flit_q[255:0]),
      .pkt_storage_addr_i(storage_wr_addr),
      .pkt_store_bytes_i(type4_store_bytes),
      .sram_wr_en_o(sram_wr_en),
      .sram_wr_bank_sel_o(sram_wr_bank_sel),
      .sram_wr_addr_o(sram_wr_addr),
      .sram_wr_byte_en_o(sram_wr_byte_en),
      .sram_wr_data_o(sram_wr_data),
      .sram_rd_bank_sel_o(sram_rd_bank_sel),
      .sram_rd_addr_o(sram_rd_addr)
  );

  mfu_writeback #(
      .FLIT_W(FLIT_W)
  ) mfu_writeback_i (
      .pkt_flit_i(pkt_flit_q),
      .pkt_meta_i(pkt_meta_q),
      .pkt_is_type5_i(decoded_is_type5_q),
      .alu_flit_i(processed_flit_q),
      .alu_meta_i(processed_meta_q),
      .matmul_active_i(matmul_active),
      .matmul_flit_i(matmul_writeback_flit),
      .attention_active_i(attention_active),
      .attention_flit_i(attention_writeback_flit),
      .attention_meta_i(attention_writeback_meta),
      .emit_flit_o(writeback_flit),
      .emit_meta_o(emit_meta_o)
  );

  mfu_matmul #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W)
  ) mfu_matmul_i (
      .clk(clk),
      .reset(reset),
      .weight_wr_en_i(matmul_weight_wr_en),
      .weight_wr_addr_i(sram_wr_addr),
      .weight_wr_byte_en_i(sram_wr_byte_en),
      .weight_wr_data_i(sram_wr_data),
      .compute_fire_i(compute_fire),
      .release_state_i(type5_tail_emit),
      .pkt_is_type5_i(decoded_is_type5_q),
      .pkt_opcode_i(decoded_opcode_q),
      .pkt_flit_i(pkt_flit_q),
      .pkt_meta_i(pkt_meta_q),
      .pkt_vc_id_i(pkt_vc_q),
      .pkt_out_sel_i(pkt_out_sel_q),
      .task_count_i(cnoc_task_count_i),
      .task_ids_flat_i(cnoc_task_ids_flat_i),
      .weight_row_size_i(cnoc_weight_row_size_i),
      .matmul_active_o(matmul_active),
      .matmul_flit_o(matmul_writeback_flit)
  );

  mfu_attention #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W)
  ) mfu_attention_i (
      .clk(clk),
      .reset(reset),
      .kv_wr_en_i(attention_kv_wr_en),
      .kv_wr_addr_i(sram_wr_addr),
      .kv_wr_byte_en_i(sram_wr_byte_en),
      .kv_wr_data_i(sram_wr_data),
      .compute_fire_i(compute_fire),
      .release_state_i(type5_tail_emit),
      .pkt_is_type5_i(decoded_is_type5_q),
      .pkt_opcode_i(decoded_opcode_q),
      .pkt_flit_i(pkt_flit_q),
      .pkt_meta_i(pkt_meta_q),
      .pkt_vc_id_i(pkt_vc_q),
      .pkt_out_sel_i(pkt_out_sel_q),
      .kv_token_count_i(kv_token_count_q),
      .task_count_i(cnoc_task_count_i),
      .task_ids_flat_i(cnoc_task_ids_flat_i),
      .attention_active_o(attention_active),
      .attention_flit_o(attention_writeback_flit),
      .attention_meta_o(attention_writeback_meta)
  );

  mfu_alu #(
      .FLIT_W(FLIT_W),
      .LAT_LINEAR(MFU_LAT_LINEAR),
      .LAT_MATMUL(MFU_LAT_MATMUL),
      .LAT_ADD(MFU_LAT_ADD),
      .LAT_SWIGLU(MFU_LAT_SWIGLU),
      .LAT_GEGLU(MFU_LAT_GEGLU),
      .LAT_ATTENTION(MFU_LAT_ATTENTION),
      .LAT_DEFAULT(MFU_LAT_DEFAULT),
      .LAT_TYPE4_STORE(MFU_LAT_TYPE4_STORE)
  ) mfu_datapath_alu_i (
      .clk_i(clk),
      .reset_i(reset),
      .start_i(alu_start),
      .flit_i(pkt_flit_q),
      .meta_i(pkt_meta_q),
      .op_a_i(fetch_operand_a_q),
      .op_b_i(fetch_operand_b_q),
      .busy_o(alu_busy),
      .result_valid_o(alu_result_valid),
      .result_flit_o(alu_result_flit),
      .result_meta_o(alu_result_meta),
      .result_scalar_o()
  );

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin : proc_assertions
    if (!reset) begin
      // The MFU is intentionally modeled as a single-entry side pipeline in
      // this prototype.  A new capture while pkt_valid_q is set would overwrite
      // the in-flight cNoC flit and lose its storage/compute side effect.
      if (capture_o && pkt_valid_q) begin
        $error("cnoc_mfu: captured a second flit while pipeline entry is occupied");
      end

      // Upstream arbitration owns the MFU-bound decision.  A non-cNoC flit
      // reaching DECODE means that Router.sv/mfu_arbiter selected the wrong
      // side-pipeline transaction.
      if (decode_en && pkt_valid_q && !(decode_is_type4 || decode_is_type5)) begin
        $error("cnoc_mfu: decoded a non-cNoC flit in the MFU side pipeline");
      end

      // writeback is allowed to emit only after the latency-aware ALU/MFU path
      // has produced a visible result and downstream credit is available.
      if (emit_valid_o && !write_back_en) begin
        $error("cnoc_mfu: emitted outside WRITE_BACK state");
      end

      if (emit_valid_o && !emit_ready) begin
        $error("cnoc_mfu: emitted without downstream ready/credit");
      end

      // Type4 storage side effects are committed only on emit, so the write
      // enable must never fire for non-type4 packets.
      if (sram_wr_en && !decoded_is_type4_q) begin
        $error("cnoc_mfu: SRAM write asserted for non-type4 packet");
      end
    end
  end
`endif

endmodule
