// Description: Unified MFU ALU wrapper.
//              The wrapper only decodes the predecoded op struct from
//              cnoc_mfu, starts the matching leaf/backend, multiplexes the
//              shared INT8 multiplier operands, and muxes the resulting
//              busy/valid/result bundle plus the Attention owner status.

module mfu_alu #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = router_ports_pkg::CNOC_SRAM_ADDR_W,
    parameter int DATA_W = router_ports_pkg::CNOC_SRAM_DATA_W,
    parameter int DATA_ELEM_W = 8,
    parameter int DATA_ACC_W = 32,
    parameter int MATMUL_CTX_SLOTS = 1
) (
    input  logic clk_i,
    input  logic reset_i,
    input  router_ports_pkg::mfu_alu_ctrl_t ctrl_i,
    input  router_ports_pkg::mfu_alu_op_t op_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  router_ports_pkg::mfu_alu_data_rsp_t data_rsp_i,
    output router_ports_pkg::mfu_alu_data_req_t data_req_o,
    output router_ports_pkg::mfu_alu_result_t result_o,
    output router_ports_pkg::attention_ctx_status_t attention_ctx_o,
    output router_ports_pkg::matmul_ctx_status_t matmul_ctx_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ADD       = 5'd18;
  localparam logic [4:0] OP_SWIGLU    = 5'd21;
  localparam logic [4:0] OP_GEGLU     = 5'd24;
  localparam int MUL_LANES = 8;
  localparam int MUL_SRC_MATMUL = 0;
  localparam int MUL_SRC_SWIGLU = 1;
  localparam int MUL_SRC_GEGLU = 2;
  localparam int MUL_SRC_ATTENTION = 3;

  logic signed [31:0] scalar_a;
  logic signed [31:0] scalar_b;
  mfu_alu_data_req_t matmul_data_req;
  mfu_alu_data_req_t attention_data_req;

  logic add_start;
  logic swiglu_start;
  logic geglu_start;
  logic unknown_opcode_start;

  logic add_busy;
  logic add_valid;
  logic [FLIT_W-1:0] add_flit;
  flit_meta_t add_meta;
  logic signed [31:0] add_scalar;

  logic swiglu_busy;
  logic swiglu_valid;
  logic [FLIT_W-1:0] swiglu_flit;
  flit_meta_t swiglu_meta;
  logic signed [31:0] swiglu_scalar;

  logic geglu_busy;
  logic geglu_valid;
  logic [FLIT_W-1:0] geglu_flit;
  flit_meta_t geglu_meta;
  logic signed [31:0] geglu_scalar;

  logic unknown_valid_q;
  logic [FLIT_W-1:0] unknown_flit_q;
  flit_meta_t unknown_meta_q;

  logic matmul_busy;
  logic matmul_valid;
  logic [FLIT_W-1:0] matmul_flit;
  flit_meta_t matmul_meta;
  logic signed [31:0] matmul_scalar;
  matmul_ctx_status_t matmul_ctx_status;

  logic attention_busy;
  logic attention_valid;
  logic [FLIT_W-1:0] attention_result_flit;
  flit_meta_t attention_result_meta;
  logic attention_ctx_valid;
  logic [2:0] attention_ctx_stream_port;
  logic [VC_ID_W-1:0] attention_ctx_stream_vc;

  logic [FLIT_W-1:0] result_flit_comb;
  flit_meta_t result_meta_comb;
  logic signed [31:0] result_scalar_comb;
  logic result_valid_comb;
  logic result_busy_comb;

  logic matmul_mul_req;
  logic swiglu_mul_req;
  logic geglu_mul_req;
  logic attention_mul_req;
  logic matmul_mul_rsp_valid;
  logic swiglu_mul_rsp_valid;
  logic geglu_mul_rsp_valid;
  logic attention_mul_rsp_valid;
  logic signed [7:0] matmul_mul_lhs [0:MUL_LANES-1];
  logic signed [7:0] matmul_mul_rhs [0:MUL_LANES-1];
  logic signed [7:0] swiglu_mul_lhs [0:MUL_LANES-1];
  logic signed [7:0] swiglu_mul_rhs [0:MUL_LANES-1];
  logic signed [7:0] geglu_mul_lhs [0:MUL_LANES-1];
  logic signed [7:0] geglu_mul_rhs [0:MUL_LANES-1];
  logic signed [7:0] attention_mul_lhs [0:MUL_LANES-1];
  logic signed [7:0] attention_mul_rhs [0:MUL_LANES-1];
  logic signed [7:0] shared_mul_lhs [0:MUL_LANES-1];
  logic signed [7:0] shared_mul_rhs [0:MUL_LANES-1];
  logic signed [15:0] shared_mul_product [0:MUL_LANES-1];
  logic shared_mul_valid;
  logic [3:0] mul_req_src_d;
  logic [3:0] mul_req_src_q;
  logic [3:0] mul_rsp_src_q;

  int unsigned mul_lane_idx;

  assign scalar_a = {{24{flit_i[{meta_i.data_offset[4:0], 3'b000} + 7]}},
                     flit_i[{meta_i.data_offset[4:0], 3'b000} +: 8]};
  assign scalar_b = {{24{data_rsp_i.rdata[7]}}, data_rsp_i.rdata[7:0]};

  assign add_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                     (op_i.opcode == OP_ADD);
  assign swiglu_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                        (op_i.opcode == OP_SWIGLU);
  assign geglu_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                       (op_i.opcode == OP_GEGLU);
  assign unknown_opcode_start =
      ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
      (op_i.opcode != OP_ADD) && (op_i.opcode != OP_SWIGLU) &&
      (op_i.opcode != OP_GEGLU);
  mfu_alu_matmul #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W),
      .DATA_W(DATA_W),
      .DATA_ELEM_W(DATA_ELEM_W),
      .ACC_W(DATA_ACC_W),
      .MATMUL_CTX_SLOTS(MATMUL_CTX_SLOTS)
  ) matmul_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .fetch_en_i(ctrl_i.fetch_en),
      .compute_en_i(ctrl_i.compute_en),
      .release_state_i(ctrl_i.state_release),
      .op_i(op_i),
      .pkt_flit_i(flit_i),
      .pkt_meta_i(meta_i),
      .ctx_i(ctx_i),
      .data_rsp_i(data_rsp_i),
      .data_req_o(matmul_data_req),
      .busy_o(matmul_busy),
      .result_valid_o(matmul_valid),
      .result_flit_o(matmul_flit),
      .result_meta_o(matmul_meta),
      .result_scalar_o(matmul_scalar),
      .matmul_ctx_o(matmul_ctx_status),
      .mul_req_o(matmul_mul_req),
      .mul_lhs_o(matmul_mul_lhs),
      .mul_rhs_o(matmul_mul_rhs),
      .mul_rsp_valid_i(matmul_mul_rsp_valid),
      .mul_product_i(shared_mul_product)
  );

  mfu_alu_add #(
      .FLIT_W(FLIT_W)
  ) add_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(add_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
      .ctx_i(ctx_i),
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(add_busy),
      .valid_o(add_valid),
      .result_flit_o(add_flit),
      .result_meta_o(add_meta),
      .result_scalar_o(add_scalar)
  );

  mfu_alu_swiglu #(
      .FLIT_W(FLIT_W)
  ) swiglu_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(swiglu_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
      .ctx_i(ctx_i),
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(swiglu_busy),
      .valid_o(swiglu_valid),
      .result_flit_o(swiglu_flit),
      .result_meta_o(swiglu_meta),
      .result_scalar_o(swiglu_scalar),
      .mul_req_o(swiglu_mul_req),
      .mul_lhs_o(swiglu_mul_lhs),
      .mul_rhs_o(swiglu_mul_rhs),
      .mul_rsp_valid_i(swiglu_mul_rsp_valid),
      .mul_product_i(shared_mul_product)
  );

  mfu_alu_geglu #(
      .FLIT_W(FLIT_W)
  ) geglu_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(geglu_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
      .ctx_i(ctx_i),
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(geglu_busy),
      .valid_o(geglu_valid),
      .result_flit_o(geglu_flit),
      .result_meta_o(geglu_meta),
      .result_scalar_o(geglu_scalar),
      .mul_req_o(geglu_mul_req),
      .mul_lhs_o(geglu_mul_lhs),
      .mul_rhs_o(geglu_mul_rhs),
      .mul_rsp_valid_i(geglu_mul_rsp_valid),
      .mul_product_i(shared_mul_product)
  );

  mfu_alu_attention #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W),
      .DATA_W(DATA_W)
  ) attention_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .fetch_en_i(ctrl_i.fetch_en),
      .compute_en_i(ctrl_i.compute_en),
      .op_i(op_i),
      .ctx_i(ctx_i),
      .release_state_i(ctrl_i.state_release),
      .pkt_flit_i(flit_i),
      .pkt_meta_i(meta_i),
      .data_rsp_i(data_rsp_i),
      .data_req_o(attention_data_req),
      .busy_o(attention_busy),
      .result_valid_o(attention_valid),
      .result_flit_o(attention_result_flit),
      .result_meta_o(attention_result_meta),
      .attention_ctx_valid_o(attention_ctx_valid),
      .attention_ctx_stream_port_o(attention_ctx_stream_port),
      .attention_ctx_stream_vc_o(attention_ctx_stream_vc),
      .mul_req_o(attention_mul_req),
      .mul_lhs_o(attention_mul_lhs),
      .mul_rhs_o(attention_mul_rhs),
      .mul_rsp_valid_i(attention_mul_rsp_valid),
      .mul_product_i(shared_mul_product)
  );

  assign mul_req_src_d = {
      attention_mul_req,
      geglu_mul_req,
      swiglu_mul_req,
      matmul_mul_req
  };
  assign matmul_mul_rsp_valid =
      shared_mul_valid && mul_rsp_src_q[MUL_SRC_MATMUL];
  assign swiglu_mul_rsp_valid =
      shared_mul_valid && mul_rsp_src_q[MUL_SRC_SWIGLU];
  assign geglu_mul_rsp_valid =
      shared_mul_valid && mul_rsp_src_q[MUL_SRC_GEGLU];
  assign attention_mul_rsp_valid =
      shared_mul_valid && mul_rsp_src_q[MUL_SRC_ATTENTION];

  always_comb begin : proc_shared_mul_mux
    for (mul_lane_idx = 0; mul_lane_idx < MUL_LANES; mul_lane_idx = mul_lane_idx + 1) begin
      shared_mul_lhs[mul_lane_idx] = '0;
      shared_mul_rhs[mul_lane_idx] = '0;
      if (matmul_mul_req) begin
        shared_mul_lhs[mul_lane_idx] = matmul_mul_lhs[mul_lane_idx];
        shared_mul_rhs[mul_lane_idx] = matmul_mul_rhs[mul_lane_idx];
      end else if (swiglu_mul_req) begin
        shared_mul_lhs[mul_lane_idx] = swiglu_mul_lhs[mul_lane_idx];
        shared_mul_rhs[mul_lane_idx] = swiglu_mul_rhs[mul_lane_idx];
      end else if (geglu_mul_req) begin
        shared_mul_lhs[mul_lane_idx] = geglu_mul_lhs[mul_lane_idx];
        shared_mul_rhs[mul_lane_idx] = geglu_mul_rhs[mul_lane_idx];
      end else if (attention_mul_req) begin
        shared_mul_lhs[mul_lane_idx] = attention_mul_lhs[mul_lane_idx];
        shared_mul_rhs[mul_lane_idx] = attention_mul_rhs[mul_lane_idx];
      end
    end
  end

  mfu_int8_mul8 shared_mul8_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .valid_i(|mul_req_src_d),
      .valid_o(shared_mul_valid),
      .lhs_i(shared_mul_lhs),
      .rhs_i(shared_mul_rhs),
      .product_o(shared_mul_product)
  );

  always_ff @(posedge clk_i) begin : proc_shared_mul_source_pipeline
    if (reset_i) begin
      mul_req_src_q <= 4'b0000;
      mul_rsp_src_q <= 4'b0000;
    end else begin
      mul_req_src_q <= mul_req_src_d;
      mul_rsp_src_q <= mul_req_src_q;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : proc_mul_req_onehot_check
    if (!reset_i) begin
      assert ($onehot0(mul_req_src_d))
        else $error("mfu_alu shared multiplier requested by multiple leaves");
    end
  end

  always_ff @(posedge clk_i) begin : proc_unknown_opcode_notice
    if (!reset_i && unknown_opcode_start) begin
      $warning("mfu_alu: ignoring unsupported opcode=%0d msg_type=%0d data_offset=%0d flit_kind=%0d stream_port=%0d stream_vc=%0d",
               op_i.opcode, op_i.msg_type, meta_i.data_offset, meta_i.flit_kind,
               ctx_i.stream_port, ctx_i.stream_vc);
    end
  end
`endif

  always_ff @(posedge clk_i) begin : proc_unknown_passthrough_registers
    if (reset_i) begin
      unknown_valid_q <= 1'b0;
      unknown_flit_q <= '0;
      unknown_meta_q <= '0;
    end else begin
      unknown_valid_q <= unknown_opcode_start;
      if (unknown_opcode_start) begin
        unknown_flit_q <= flit_i;
        unknown_meta_q <= meta_i;
      end
    end
  end

  always_comb begin : proc_result_mux
    result_flit_comb = '0;
    result_meta_comb = '0;
    result_scalar_comb = 32'sd0;
    result_valid_comb = 1'b0;
    result_busy_comb = matmul_busy || attention_busy || add_busy ||
                       swiglu_busy || geglu_busy;

    if (op_i.is_data_op) begin
      result_flit_comb = matmul_flit;
      result_meta_comb = matmul_meta;
      result_scalar_comb = matmul_scalar;
      result_valid_comb = matmul_valid;
    end else if (op_i.is_attention) begin
      result_flit_comb = attention_result_flit;
      result_meta_comb = attention_result_meta;
      result_scalar_comb = 32'sd0;
      result_valid_comb = attention_valid;
    end else begin
      unique case (op_i.opcode)
        OP_ADD: begin
          result_flit_comb = add_flit;
          result_meta_comb = add_meta;
          result_scalar_comb = add_scalar;
          result_valid_comb = add_valid;
        end
        OP_SWIGLU: begin
          result_flit_comb = swiglu_flit;
          result_meta_comb = swiglu_meta;
          result_scalar_comb = swiglu_scalar;
          result_valid_comb = swiglu_valid;
        end
        OP_GEGLU: begin
          result_flit_comb = geglu_flit;
          result_meta_comb = geglu_meta;
          result_scalar_comb = geglu_scalar;
          result_valid_comb = geglu_valid;
        end
        default: begin
          result_flit_comb = unknown_flit_q;
          result_meta_comb = unknown_meta_q;
          result_scalar_comb = 32'sd0;
          result_valid_comb = unknown_valid_q;
        end
      endcase
    end
  end

  assign result_o.busy = result_busy_comb;
  assign result_o.valid = result_valid_comb;
  assign result_o.flit = result_flit_comb;
  assign result_o.meta = result_meta_comb;
  assign result_o.scalar = result_scalar_comb;
  assign attention_ctx_o.valid = attention_ctx_valid;
  assign attention_ctx_o.stream_port = attention_ctx_stream_port;
  assign attention_ctx_o.stream_vc = attention_ctx_stream_vc;
  assign matmul_ctx_o = matmul_ctx_status;

  always_comb begin : proc_data_request_mux
    data_req_o = '0;
    if (op_i.is_data_op) begin
      data_req_o = matmul_data_req;
    end else if (op_i.is_attention) begin
      data_req_o = attention_data_req;
    end
  end

endmodule
