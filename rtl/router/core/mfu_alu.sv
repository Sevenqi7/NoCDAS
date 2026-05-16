// Description: Unified MFU ALU wrapper.
//              The wrapper only decodes the predecoded op struct from
//              cnoc_mfu, starts the matching leaf/backend, and muxes the
//              resulting busy/valid/result bundle.

module mfu_alu #(
    parameter int NUM_PORTS = router_ports_pkg::PORT_NUM,
    parameter int VC_NUM = router_ports_pkg::VC_NUM,
    parameter int VC_ID_W = router_ports_pkg::VC_ID_W,
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int SRAM_DEPTH = 2048,
    parameter int SRAM_ADDR_W = router_ports_pkg::CNOC_SRAM_ADDR_W,
    parameter int DATA_W = router_ports_pkg::CNOC_SRAM_DATA_W,
    parameter int DATA_ELEM_W = 8,
    parameter int DATA_ACC_W = 32
) (
    input  logic clk_i,
    input  logic reset_i,
    input  router_ports_pkg::mfu_alu_ctrl_t ctrl_i,
    input  router_ports_pkg::mfu_alu_op_t op_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  router_ports_pkg::mfu_alu_data_wr_t data_wr_i,
    input  router_ports_pkg::mfu_alu_data_rsp_t data_rsp_i,
    output router_ports_pkg::mfu_alu_data_req_t data_req_o,
    output router_ports_pkg::mfu_alu_result_t result_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_ADD       = 5'd18;
  localparam logic [4:0] OP_SWIGLU    = 5'd21;
  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_GEGLU     = 5'd24;

  logic signed [31:0] scalar_a;
  logic signed [31:0] scalar_b;

  logic add_start;
  logic swiglu_start;
  logic geglu_start;
  logic default_start;
  logic attention_start;

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

  logic default_busy;
  logic default_valid;
  logic [FLIT_W-1:0] default_flit;
  flit_meta_t default_meta;
  logic signed [31:0] default_scalar;

  logic matmul_busy;
  logic matmul_valid;
  logic [FLIT_W-1:0] matmul_flit;
  flit_meta_t matmul_meta;
  logic signed [31:0] matmul_scalar;

  logic attention_busy;
  logic attention_valid;
  logic attention_active;
  logic [FLIT_W-1:0] attention_flit;
  flit_meta_t attention_meta;

  logic [FLIT_W-1:0] result_flit_comb;
  flit_meta_t result_meta_comb;
  logic signed [31:0] result_scalar_comb;
  logic result_valid_comb;
  logic result_busy_comb;

  assign scalar_a = {{24{flit_i[{meta_i.data_offset[4:0], 3'b000} + 7]}},
                     flit_i[{meta_i.data_offset[4:0], 3'b000} +: 8]};
  assign scalar_b = {{24{data_rsp_i.rdata[7]}}, data_rsp_i.rdata[7:0]};

  assign add_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                     (op_i.opcode == OP_ADD);
  assign swiglu_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                        (op_i.opcode == OP_SWIGLU);
  assign geglu_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                       (op_i.opcode == OP_GEGLU);
  assign default_start = ctrl_i.start && !op_i.is_data_op && !op_i.is_attention &&
                         (op_i.opcode != OP_ADD) &&
                         (op_i.opcode != OP_SWIGLU) &&
                         (op_i.opcode != OP_GEGLU);
  assign attention_start = ctrl_i.start && op_i.is_attention;

  mfu_alu_matmul #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W),
      .DATA_W(DATA_W),
      .DATA_ELEM_W(DATA_ELEM_W),
      .ACC_W(DATA_ACC_W)
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
      .data_req_o(data_req_o),
      .busy_o(matmul_busy),
      .result_valid_o(matmul_valid),
      .result_flit_o(matmul_flit),
      .result_meta_o(matmul_meta),
      .result_scalar_o(matmul_scalar)
  );

  mfu_alu_add #(
      .FLIT_W(FLIT_W)
  ) add_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(add_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
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
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(swiglu_busy),
      .valid_o(swiglu_valid),
      .result_flit_o(swiglu_flit),
      .result_meta_o(swiglu_meta),
      .result_scalar_o(swiglu_scalar)
  );

  mfu_alu_geglu #(
      .FLIT_W(FLIT_W)
  ) geglu_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(geglu_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(geglu_busy),
      .valid_o(geglu_valid),
      .result_flit_o(geglu_flit),
      .result_meta_o(geglu_meta),
      .result_scalar_o(geglu_scalar)
  );

  mfu_alu_attention #(
      .NUM_PORTS(NUM_PORTS),
      .VC_NUM(VC_NUM),
      .VC_ID_W(VC_ID_W),
      .FLIT_W(FLIT_W),
      .SRAM_DEPTH(SRAM_DEPTH),
      .SRAM_ADDR_W(SRAM_ADDR_W)
  ) attention_i (
      .clk(clk_i),
      .reset(reset_i),
      .op_i(op_i),
      .data_wr_i(data_wr_i),
      .ctx_i(ctx_i),
      .start_i(attention_start),
      .release_state_i(ctrl_i.state_release),
      .pkt_flit_i(flit_i),
      .pkt_meta_i(meta_i),
      .busy_o(attention_busy),
      .valid_o(attention_valid),
      .attention_active_o(attention_active),
      .attention_flit_o(attention_flit),
      .attention_meta_o(attention_meta)
  );

  mfu_alu_default #(
      .FLIT_W(FLIT_W)
  ) default_i (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .start_i(default_start),
      .flit_i(flit_i),
      .meta_i(meta_i),
      .scalar_a_i(scalar_a),
      .scalar_b_i(scalar_b),
      .busy_o(default_busy),
      .valid_o(default_valid),
      .result_flit_o(default_flit),
      .result_meta_o(default_meta),
      .result_scalar_o(default_scalar)
  );

  always_comb begin : proc_result_mux
    result_flit_comb = '0;
    result_meta_comb = '0;
    result_scalar_comb = 32'sd0;
    result_valid_comb = 1'b0;
    result_busy_comb = matmul_busy || attention_busy || add_busy ||
                       swiglu_busy || geglu_busy || default_busy;

    if (op_i.is_data_op) begin
      result_flit_comb = matmul_flit;
      result_meta_comb = matmul_meta;
      result_scalar_comb = matmul_scalar;
      result_valid_comb = matmul_valid;
    end else if (op_i.is_attention) begin
      result_flit_comb = attention_flit;
      result_meta_comb = attention_meta;
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
          result_flit_comb = default_flit;
          result_meta_comb = default_meta;
          result_scalar_comb = default_scalar;
          result_valid_comb = default_valid;
        end
      endcase
    end
  end

  assign result_o.busy = result_busy_comb;
  assign result_o.valid = result_valid_comb;
  assign result_o.flit = result_flit_comb;
  assign result_o.meta = result_meta_comb;
  assign result_o.scalar = result_scalar_comb;
  assign result_o.attention_active = attention_active;
  assign result_o.attention_flit = attention_flit;
  assign result_o.attention_meta = attention_meta;

endmodule
