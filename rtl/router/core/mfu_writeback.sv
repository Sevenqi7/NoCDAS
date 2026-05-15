// Copyright (c) 2026
//
// Description: MFU writeback formatting.
//              Chooses between the latency-aware ALU payload, the MatMul psum
//              payload, or the original type4 passthrough payload.  All arithmetic
//              is completed before this module observes the data.

module mfu_writeback #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic [FLIT_W-1:0] pkt_flit_i,
    input  router_ports_pkg::flit_meta_t pkt_meta_i,
    input  logic             pkt_is_type5_i,
    input  logic [FLIT_W-1:0] alu_flit_i,
    input  router_ports_pkg::flit_meta_t alu_meta_i,
    input  logic             matmul_active_i,
    input  logic [FLIT_W-1:0] matmul_flit_i,
    input  logic             attention_active_i,
    input  logic [FLIT_W-1:0] attention_flit_i,
    input  router_ports_pkg::flit_meta_t attention_meta_i,
    output logic [FLIT_W-1:0] emit_flit_o,
    output router_ports_pkg::flit_meta_t emit_meta_o
);
  import router_ports_pkg::*;

  localparam logic [4:0] OP_LINEAR = 5'd0;
  localparam logic [4:0] OP_MATMUL = 5'd15;
  localparam logic [4:0] OP_ADD    = 5'd18;
  localparam logic [4:0] OP_SWIGLU = 5'd21;
  localparam logic [4:0] OP_ATTENTION = 5'd23;
  localparam logic [4:0] OP_GEGLU  = 5'd24;

  logic [4:0] opcode;
  logic [4:0] data_lane;

  always_comb begin
    emit_flit_o = pkt_flit_i;
    emit_meta_o = pkt_meta_i;
    opcode = pkt_meta_i.opcode;
    data_lane = pkt_meta_i.data_offset[4:0];

    // Only type5 compute packets modify outgoing payload data.  Type4 packets
    // have already committed storage side effects and forward their payload
    // unchanged.
    if (pkt_is_type5_i) begin
      unique case (opcode)
        // ADD treats the payload as pairs [lhs, rhs].  Only task-owned pairs
        // marked in pair_update_mask update their lhs lane.
        OP_ADD: begin
          emit_flit_o = alu_flit_i;
          emit_meta_o = alu_meta_i;
        end
        // SwiGLU uses the same pair layout as ADD but computes SiLU(lhs)*rhs.
        OP_SWIGLU: begin
          emit_flit_o = alu_flit_i;
          emit_meta_o = alu_meta_i;
        end
        // GeGLU mirrors the SwiGLU payload contract with GELU(lhs)*rhs.
        OP_GEGLU: begin
          emit_flit_o = alu_flit_i;
          emit_meta_o = alu_meta_i;
        end
        OP_LINEAR,
        OP_MATMUL: begin
          // MatMul/Linear update psum slots when their psum flit passes the
          // MFU.  The per-task accumulation state is owned by mfu_matmul.
          // If this flit carries a psum lane owned by the router, mfu_matmul
          // returns an edited payload; otherwise this branch forwards unchanged.
          if (matmul_active_i) begin
            emit_flit_o = matmul_flit_i;
          end
`ifdef ROUTER_ENABLE_COSIM
          emit_meta_o.cosim.data_q = emit_flit_o[{data_lane, 3'b000} +: 8];
`endif
        end
        OP_ATTENTION: begin
          // Attention owns a separate control path because it consumes query
          // state and the local KV cache.  The arithmetic is functional, but the
          // flit update still comes from RTL-owned storage/state.
          if (attention_active_i) begin
            emit_flit_o = attention_flit_i;
            emit_meta_o = attention_meta_i;
          end
`ifdef ROUTER_ENABLE_COSIM
          emit_meta_o.cosim.data_q = emit_flit_o[{data_lane, 3'b000} +: 8];
`endif
        end
        // Fallback single-lane writeback for simple scalar/experimental opcodes.
        default: begin
          emit_flit_o = alu_flit_i;
          emit_meta_o = alu_meta_i;
        end
      endcase
    end
  end

endmodule
