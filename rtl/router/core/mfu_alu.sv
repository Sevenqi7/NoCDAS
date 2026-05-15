// Description: Latency-aware functional MFU ALU shell.
//              Opcode-specific arithmetic is split into leaf modules so each
//              operation can later be replaced by a dedicated RTL pipeline.

module mfu_alu #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W,
    parameter int LAT_LINEAR = 0,
    parameter int LAT_MATMUL = 0,
    parameter int LAT_ADD = 0,
    parameter int LAT_SWIGLU = 0,
    parameter int LAT_GEGLU = 0,
    parameter int LAT_ATTENTION = 8,
    parameter int LAT_DEFAULT = 0,
    parameter int LAT_TYPE4_STORE = 0
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic start_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  logic signed [31:0] op_a_i,
    input  logic signed [31:0] op_b_i,
    output logic busy_o,
    output logic result_valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_LINEAR    = 5'd0;
  localparam logic [4:0] OP_MATMUL    = 5'd15;
  localparam logic [4:0] OP_ADD       = 5'd18;
  localparam logic [4:0] OP_SWIGLU    = 5'd21;
  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_GEGLU     = 5'd24;

  logic active_q;
  logic [15:0] wait_cnt_q;
  logic [FLIT_W-1:0] result_flit_q;
  flit_meta_t result_meta_q;
  logic signed [31:0] result_scalar_q;
  logic result_valid_q;

  logic [4:0] opcode;
  logic [2:0] msg_type;
  logic [15:0] selected_latency;

  logic [FLIT_W-1:0] type4_flit;
  flit_meta_t type4_meta;
  logic signed [31:0] type4_scalar;
  logic [FLIT_W-1:0] linear_flit;
  flit_meta_t linear_meta;
  logic signed [31:0] linear_scalar;
  logic [FLIT_W-1:0] matmul_flit;
  flit_meta_t matmul_meta;
  logic signed [31:0] matmul_scalar;
  logic [FLIT_W-1:0] add_flit;
  flit_meta_t add_meta;
  logic signed [31:0] add_scalar;
  logic [FLIT_W-1:0] swiglu_flit;
  flit_meta_t swiglu_meta;
  logic signed [31:0] swiglu_scalar;
  logic [FLIT_W-1:0] geglu_flit;
  flit_meta_t geglu_meta;
  logic signed [31:0] geglu_scalar;
  logic [FLIT_W-1:0] attention_flit;
  flit_meta_t attention_meta;
  logic signed [31:0] attention_scalar;
  logic [FLIT_W-1:0] default_flit;
  flit_meta_t default_meta;
  logic signed [31:0] default_scalar;

  logic [FLIT_W-1:0] selected_flit;
  flit_meta_t selected_meta;
  logic signed [31:0] selected_scalar;

  assign opcode = meta_i.opcode;
  assign msg_type = meta_i.msg_type;

  always_comb begin : proc_selected_latency
    selected_latency = 16'(LAT_DEFAULT);
    if (msg_type == ROUTER_MSG_DIST) begin
      selected_latency = 16'(LAT_TYPE4_STORE);
    end else begin
      unique case (opcode)
        OP_LINEAR:    selected_latency = 16'(LAT_LINEAR);
        OP_MATMUL:    selected_latency = 16'(LAT_MATMUL);
        OP_ADD:       selected_latency = 16'(LAT_ADD);
        OP_SWIGLU:    selected_latency = 16'(LAT_SWIGLU);
        OP_GEGLU:     selected_latency = 16'(LAT_GEGLU);
        OP_ATTENTION: selected_latency = 16'(LAT_ATTENTION);
        default:      selected_latency = 16'(LAT_DEFAULT);
      endcase
    end
  end

  mfu_alu_type4_store #(
      .FLIT_W(FLIT_W)
  ) type4_store_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(type4_flit),
      .result_meta_o(type4_meta),
      .result_scalar_o(type4_scalar)
  );

  mfu_alu_linear #(
      .FLIT_W(FLIT_W)
  ) linear_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(linear_flit),
      .result_meta_o(linear_meta),
      .result_scalar_o(linear_scalar)
  );

  mfu_alu_matmul #(
      .FLIT_W(FLIT_W)
  ) matmul_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(matmul_flit),
      .result_meta_o(matmul_meta),
      .result_scalar_o(matmul_scalar)
  );

  mfu_alu_add #(
      .FLIT_W(FLIT_W)
  ) add_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(add_flit),
      .result_meta_o(add_meta),
      .result_scalar_o(add_scalar)
  );

  mfu_alu_swiglu #(
      .FLIT_W(FLIT_W)
  ) swiglu_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(swiglu_flit),
      .result_meta_o(swiglu_meta),
      .result_scalar_o(swiglu_scalar)
  );

  mfu_alu_geglu #(
      .FLIT_W(FLIT_W)
  ) geglu_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(geglu_flit),
      .result_meta_o(geglu_meta),
      .result_scalar_o(geglu_scalar)
  );

  mfu_alu_attention #(
      .FLIT_W(FLIT_W)
  ) attention_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(attention_flit),
      .result_meta_o(attention_meta),
      .result_scalar_o(attention_scalar)
  );

  mfu_alu_default #(
      .FLIT_W(FLIT_W)
  ) default_i (
      .flit_i(flit_i),
      .meta_i(meta_i),
      .op_a_i(op_a_i),
      .op_b_i(op_b_i),
      .result_flit_o(default_flit),
      .result_meta_o(default_meta),
      .result_scalar_o(default_scalar)
  );

  always_comb begin : proc_selected_result
    selected_flit = type4_flit;
    selected_meta = type4_meta;
    selected_scalar = type4_scalar;

    if (msg_type != ROUTER_MSG_DIST) begin
      unique case (opcode)
        OP_LINEAR: begin
          selected_flit = linear_flit;
          selected_meta = linear_meta;
          selected_scalar = linear_scalar;
        end
        OP_MATMUL: begin
          selected_flit = matmul_flit;
          selected_meta = matmul_meta;
          selected_scalar = matmul_scalar;
        end
        OP_ADD: begin
          selected_flit = add_flit;
          selected_meta = add_meta;
          selected_scalar = add_scalar;
        end
        OP_SWIGLU: begin
          selected_flit = swiglu_flit;
          selected_meta = swiglu_meta;
          selected_scalar = swiglu_scalar;
        end
        OP_GEGLU: begin
          selected_flit = geglu_flit;
          selected_meta = geglu_meta;
          selected_scalar = geglu_scalar;
        end
        OP_ATTENTION: begin
          selected_flit = attention_flit;
          selected_meta = attention_meta;
          selected_scalar = attention_scalar;
        end
        default: begin
          selected_flit = default_flit;
          selected_meta = default_meta;
          selected_scalar = default_scalar;
        end
      endcase
    end
  end

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      active_q <= 1'b0;
      wait_cnt_q <= 16'd0;
      result_flit_q <= '0;
      result_meta_q <= '0;
      result_scalar_q <= 32'sd0;
      result_valid_q <= 1'b0;
    end else begin
      result_valid_q <= 1'b0;

      if (start_i && !active_q) begin
        result_flit_q <= selected_flit;
        result_meta_q <= selected_meta;
        result_scalar_q <= selected_scalar;
        if (selected_latency == 16'd0) begin
          active_q <= 1'b0;
          wait_cnt_q <= 16'd0;
          result_valid_q <= 1'b1;
        end else begin
          active_q <= 1'b1;
          wait_cnt_q <= selected_latency;
        end
      end else if (active_q) begin
        if (wait_cnt_q <= 16'd1) begin
          active_q <= 1'b0;
          wait_cnt_q <= 16'd0;
          result_valid_q <= 1'b1;
        end else begin
          wait_cnt_q <= wait_cnt_q - 16'd1;
        end
      end
    end
  end

  assign busy_o = active_q;
  assign result_valid_o = result_valid_q;
  assign result_flit_o = result_flit_q;
  assign result_meta_o = result_meta_q;
  assign result_scalar_o = result_scalar_q;

endmodule
