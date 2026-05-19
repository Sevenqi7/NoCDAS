// Description: cNoC MFU datapath wrapper.
//              Captures cNoC flits selected by the router datapath, performs
//              the functional MFU operation, updates metadata, and re-emits the
//              flit toward the originally selected output port.

module cnoc_mfu #(
    parameter int NUM_PORTS = 5,
    parameter int VC_NUM = 8,
    parameter int VC_ID_W = 3,
    parameter int FLIT_W = 256,
    parameter int MFU_MATMUL_ACC_W = 32
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
    output logic [VC_ID_W-1:0] emit_vc_id_o

`ifdef ROUTER_ENABLE_COSIM
    ,
    output logic [31:0] weight_bytes_stored_o,
    output logic [31:0] kv_bytes_stored_o,
    output logic [15:0] kv_token_count_o,
    output logic        last_type4_store_valid_o,
    output logic        last_type4_store_bank_o,
    output logic [10:0] last_type4_store_addr_o,
    output logic [5:0]  last_type4_store_bytes_o
`endif
);
  import router_ports_pkg::*;

  localparam logic [4:0] OPCODE_LINEAR = 5'd0;
  localparam logic [4:0] OPCODE_MATMUL = 5'd15;
  localparam logic [4:0] OPCODE_ATTENTION = 5'd23;
  localparam int SRAM_DEPTH = 2048;
  localparam logic [31:0] SRAM_DEPTH_U32 = 32'd2048;
  localparam int SRAM_ADDR_W = 11;
  localparam int ALU_DATA_W = 64;
  localparam int ALU_DATA_ELEM_W = 8;
  localparam int TYPE4_KV_SINK_TOKENS = 4;
  localparam logic [15:0] TYPE4_KV_SINK_TOKENS_Q = 16'd4;

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

  logic fsm_busy;
  logic decode_en;
  logic fetch_en;
  logic compute_en;
  logic write_back_en;
  logic compute_done;
  logic emit_ready;
  logic decoded_is_data_op;

  // Type4 distribution path: payload bytes are written into either the weight
  // SRAM bank or the Attention KV bank.
  logic [ALU_DATA_W-1:0] sram_rdata;
  logic sram_wr_en;
  logic sram_wr_bank_sel;
  logic [SRAM_ADDR_W-1:0] sram_wr_addr;
  logic [31:0] sram_wr_byte_en;
  logic [255:0] sram_wr_data;
  logic sram_rd_bank_sel;
  logic [SRAM_ADDR_W-1:0] sram_rd_addr;
  logic type4_bank_sel;
  logic [SRAM_ADDR_W-1:0] type4_wr_addr;
  logic [15:0] type4_kv_token_count_q;
  logic [15:0] type4_kv_stride;
  logic [15:0] type4_kv_max_tokens_raw;
  logic [15:0] type4_kv_max_tokens_safe;
  logic [15:0] type4_kv_sink_limit;
  logic [31:0] type4_kv_max_tokens_full;
  logic [15:0] type4_kv_slot_q;
  logic [15:0] type4_kv_slot_next;
  logic [15:0] type4_kv_slot;
  logic [15:0] type4_kv_last_slot;
  logic type4_kv_has_ring_region;
  logic [31:0] type4_wr_addr_full;
  logic [5:0] type4_payload_bytes;
  logic [31:0] type4_bytes_to_sram_end_full;
  logic [5:0] type4_bytes_to_sram_end;
  logic [5:0] type4_store_bytes;

  // Type5 compute/writeback path.  ALU results are latched so writeback only
  // observes a result after the latency-aware ALU has asserted result_valid.
  logic [FLIT_W-1:0] writeback_flit;
  logic type5_tail_emit;
  logic mfu_sram_if_rd_bank_sel;
  logic [SRAM_ADDR_W-1:0] mfu_sram_if_rd_addr;
  mfu_alu_ctrl_t alu_ctrl;
  mfu_alu_ctx_t alu_ctx;
  mfu_alu_data_rsp_t alu_data_rsp;
  mfu_alu_data_req_t alu_data_req;
  mfu_alu_op_t alu_op;
  mfu_alu_result_t alu_result;

`ifdef ROUTER_ENABLE_COSIM
  // Status bridge for NoCDAS phase gating and trace diagnostics.  These are not
  // used to make RTL routing/storage/compute decisions.
  logic [31:0] weight_bytes_stored_q;
  logic [31:0] kv_bytes_stored_q;
  logic last_type4_store_valid_q;
  logic last_type4_store_bank_q;
  logic [SRAM_ADDR_W-1:0] last_type4_store_addr_q;
  logic [5:0] last_type4_store_bytes_q;
`endif

  // Capture/control path.  Non-cNoC traffic is intentionally invisible here and
  // remains on Router.sv's regular datapath.
  assign capture_o = raw_valid_i && !pkt_valid_q && !fsm_busy;
  assign capture_out_sel_o = raw_out_sel_i;
  assign busy_o = pkt_valid_q || fsm_busy || alu_result.busy;
  assign compute_done = decoded_is_type4_q ? 1'b1 : alu_result.valid;
  assign emit_ready =
      routport_flow_i[pkt_out_sel_q].state_ready &&
      routport_flow_i[pkt_out_sel_q].downstream_vc_credit_mask[pkt_vc_q];
  assign emit_valid_o = pkt_valid_q && write_back_en && emit_ready;
  assign emit_out_sel_o = pkt_out_sel_q;
  assign emit_flit_o = writeback_flit;
  assign emit_route_path_o = pkt_route_path_q;
  assign emit_vc_id_o = pkt_vc_q;

  always_comb begin : proc_type4_storage
    type4_bank_sel = (decoded_opcode_q == OPCODE_ATTENTION);
    type4_kv_stride = (decoded_k_dim_q == 16'd0) ? 16'd1 : (decoded_k_dim_q << 1);
    type4_kv_max_tokens_full =
        (type4_kv_stride == 16'd0) ? 32'd1 : (SRAM_DEPTH / type4_kv_stride);
    type4_kv_max_tokens_raw =
        (type4_kv_max_tokens_full == 32'd0) ? 16'd0 :
        (type4_kv_max_tokens_full > 32'h0000_ffff) ? 16'hffff :
                                                     type4_kv_max_tokens_full[15:0];
    type4_kv_max_tokens_safe =
        (type4_kv_max_tokens_raw == 16'd0) ? 16'd1 : type4_kv_max_tokens_raw;
    type4_kv_sink_limit =
        (type4_kv_max_tokens_safe > TYPE4_KV_SINK_TOKENS_Q) ?
        TYPE4_KV_SINK_TOKENS_Q :
        type4_kv_max_tokens_safe;
    type4_kv_last_slot = type4_kv_max_tokens_safe - 16'd1;
    type4_kv_has_ring_region = type4_kv_max_tokens_safe > type4_kv_sink_limit;

    if (type4_kv_slot_q < type4_kv_max_tokens_safe) begin
      type4_kv_slot = type4_kv_slot_q;
    end else begin
      type4_kv_slot = type4_kv_last_slot;
    end

    type4_kv_slot_next = type4_kv_slot;
    if (!type4_kv_has_ring_region) begin
      type4_kv_slot_next = type4_kv_last_slot;
    end else if ((type4_kv_slot + 16'd1) < type4_kv_sink_limit) begin
      type4_kv_slot_next = type4_kv_slot + 16'd1;
    end else if (type4_kv_slot < type4_kv_sink_limit) begin
      type4_kv_slot_next = type4_kv_sink_limit;
    end else if ((type4_kv_slot + 16'd1) < type4_kv_max_tokens_safe) begin
      type4_kv_slot_next = type4_kv_slot + 16'd1;
    end else begin
      type4_kv_slot_next = type4_kv_sink_limit;
    end

    if (type4_bank_sel) begin
      type4_wr_addr_full = (type4_kv_slot * type4_kv_stride) + decoded_data_idx_q;
    end else begin
      type4_wr_addr_full = decoded_data_idx_q;
    end
    type4_wr_addr = type4_wr_addr_full[SRAM_ADDR_W-1:0];

    if (decoded_payload_len_q == 6'd0) begin
      type4_payload_bytes = 6'd0;
    end else if (decoded_payload_len_q > 6'd32) begin
      type4_payload_bytes = 6'd32;
    end else begin
      type4_payload_bytes = decoded_payload_len_q;
    end

    if (type4_wr_addr_full >= SRAM_DEPTH) begin
      type4_bytes_to_sram_end_full = 32'd0;
    end else begin
      type4_bytes_to_sram_end_full = SRAM_DEPTH_U32 - type4_wr_addr_full;
    end
    if (type4_bytes_to_sram_end_full > 32'd32) begin
      type4_bytes_to_sram_end = 6'd32;
    end else begin
      type4_bytes_to_sram_end = type4_bytes_to_sram_end_full[5:0];
    end

    if (type4_payload_bytes < type4_bytes_to_sram_end) begin
      type4_store_bytes = type4_payload_bytes;
    end else begin
      type4_store_bytes = type4_bytes_to_sram_end;
    end
  end

  // Type5 ALU/MFU side inputs.
  assign decoded_is_data_op =
      decoded_is_type5_q &&
      ((decoded_opcode_q == OPCODE_LINEAR) || (decoded_opcode_q == OPCODE_MATMUL));
  assign type5_tail_emit = emit_valid_o && decoded_is_type5_q && decoded_tail_like_q;
  assign alu_ctrl.start =
      pkt_valid_q && compute_en && decoded_is_type5_q && !decoded_is_data_op &&
      !alu_result.busy && !alu_result.valid;
  assign alu_ctrl.fetch_en = fetch_en;
  assign alu_ctrl.compute_en = compute_en;
  assign alu_ctrl.state_release = type5_tail_emit;
  always_comb begin : proc_alu_op
    alu_op = '0;
    alu_op.opcode = decoded_opcode_q;
    alu_op.msg_type = decoded_is_type5_q ? ROUTER_MSG_COMP :
                                           pkt_meta_q.msg_type;
    alu_op.is_type5 = decoded_is_type5_q;
    alu_op.is_data_op = decoded_is_data_op;
    alu_op.is_attention = decoded_is_type5_q && (decoded_opcode_q == OPCODE_ATTENTION);
  end
  assign alu_ctx.vc_id = pkt_vc_q;
  assign alu_ctx.out_sel = pkt_out_sel_q;
  assign alu_ctx.kv_token_count = type4_kv_token_count_q;
  assign alu_ctx.task_count = cnoc_task_count_i;
  assign alu_ctx.task_ids_flat = cnoc_task_ids_flat_i;
  assign alu_ctx.weight_row_size = cnoc_weight_row_size_i;
  assign alu_data_rsp.rdata = sram_rdata;

`ifdef ROUTER_ENABLE_COSIM
  // Status bridge outputs.
  assign weight_bytes_stored_o = weight_bytes_stored_q;
  assign kv_bytes_stored_o = kv_bytes_stored_q;
  assign kv_token_count_o = type4_kv_token_count_q;
  assign last_type4_store_valid_o = last_type4_store_valid_q;
  assign last_type4_store_bank_o = last_type4_store_bank_q;
  assign last_type4_store_addr_o = last_type4_store_addr_q;
  assign last_type4_store_bytes_o = last_type4_store_bytes_q;
`endif

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
      type4_kv_token_count_q <= 16'd0;
      type4_kv_slot_q <= 16'd0;
`ifdef ROUTER_ENABLE_COSIM
      weight_bytes_stored_q <= 32'd0;
      kv_bytes_stored_q <= 32'd0;
      last_type4_store_valid_q <= 1'b0;
      last_type4_store_bank_q <= 1'b0;
      last_type4_store_addr_q <= '0;
      last_type4_store_bytes_q <= '0;
`endif
    end else begin
`ifdef ROUTER_ENABLE_COSIM
      // last_type4_store_* is a one-cycle event pulse used by the wrapper/status
      // bridge; clear it unless a type4 emit below refreshes it.
      last_type4_store_valid_q <= 1'b0;
`endif
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

      // emit_valid_o means the processed flit has both completed MFU work and
      // has downstream credit to leave this router.
      if (emit_valid_o) begin
`ifdef ROUTER_ENABLE_COSIM
        // Type4 emit is the point where the local SRAM/KV write has committed.
        // Update the status bridge from the same transaction.
        if (decoded_is_type4_q) begin
          last_type4_store_valid_q <= 1'b1;
          last_type4_store_bank_q <= (decoded_opcode_q == OPCODE_ATTENTION);
          last_type4_store_addr_q <= type4_wr_addr;
          last_type4_store_bytes_q <= type4_store_bytes;
          // Attention type4 packets fill the KV-cache bank.
          if (decoded_opcode_q == OPCODE_ATTENTION) begin
            kv_bytes_stored_q <= kv_bytes_stored_q + {26'd0, type4_store_bytes};
          // Other type4 packets fill the weight bank.
          end else begin
            weight_bytes_stored_q <= weight_bytes_stored_q + {26'd0, type4_store_bytes};
          end
        end
`endif
        // Tail/head-tail of an Attention type4 packet marks one logical KV token
        // complete, so the next token advances to the next sink/ring slot.
        if (decoded_is_type4_q && decoded_opcode_q == OPCODE_ATTENTION &&
            decoded_tail_like_q) begin
          type4_kv_token_count_q <= type4_kv_token_count_q + 16'd1;
          type4_kv_slot_q <= type4_kv_slot_next;
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
      .DATA_W(ALU_DATA_W),
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
      .type4_valid_i(decoded_is_type4_q),
      .type4_bank_sel_i(type4_bank_sel),
      .type4_addr_i(type4_wr_addr),
      .type4_store_bytes_i(type4_store_bytes),
      .type4_payload_i(pkt_flit_q[255:0]),
      .scalar_opcode_i(decoded_opcode_q),
      .scalar_data_idx_i(decoded_data_idx_q),
      .sram_wr_en_o(sram_wr_en),
      .sram_wr_bank_sel_o(sram_wr_bank_sel),
      .sram_wr_addr_o(sram_wr_addr),
      .sram_wr_byte_en_o(sram_wr_byte_en),
      .sram_wr_data_o(sram_wr_data),
      .sram_rd_bank_sel_o(mfu_sram_if_rd_bank_sel),
      .sram_rd_addr_o(mfu_sram_if_rd_addr)
  );

  mfu_writeback #(
      .FLIT_W(FLIT_W)
  ) mfu_writeback_i (
      .pkt_flit_i(pkt_flit_q),
      .pkt_meta_i(pkt_meta_q),
      .pkt_is_type5_i(decoded_is_type5_q),
      .alu_flit_i(alu_result.flit),
      .alu_meta_i(alu_result.meta),
      .emit_flit_o(writeback_flit),
      .emit_meta_o(emit_meta_o)
  );

  mfu_alu #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W),
      .DATA_W(ALU_DATA_W),
      .DATA_ELEM_W(ALU_DATA_ELEM_W),
      .DATA_ACC_W(MFU_MATMUL_ACC_W)
  ) mfu_datapath_alu_i (
      .clk_i(clk),
      .reset_i(reset),
      .ctrl_i(alu_ctrl),
      .op_i(alu_op),
      .flit_i(pkt_flit_q),
      .meta_i(pkt_meta_q),
      .ctx_i(alu_ctx),
      .data_rsp_i(alu_data_rsp),
      .data_req_o(alu_data_req),
      .result_o(alu_result)
  );

  assign sram_rd_bank_sel =
      alu_data_req.valid ? alu_data_req.bank_sel : mfu_sram_if_rd_bank_sel;
  assign sram_rd_addr =
      alu_data_req.valid ? alu_data_req.addr : mfu_sram_if_rd_addr;

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
