// Copyright (c) 2026
//
// Description: MFU local-storage address/control generation.
//              Type4 distribution flits write the router-local weight/KV
//              storage. Type5 compute flits read the same storage and forward
//              operands into the ALU path.  The storage array itself is kept in
//              mfu_sram so this module remains pure control/datapath glue.

module mfu_sram_if #(
    parameter int ADDR_W = 11
) (
    input  logic        emit_valid_i,
    input  logic        type4_valid_i,
    input  logic        type4_bank_sel_i,
    input  logic [ADDR_W-1:0] type4_addr_i,
    input  logic [5:0]  type4_store_bytes_i,
    input  logic [255:0] type4_payload_i,
    input  logic [4:0]  scalar_opcode_i,
    input  logic [9:0]  scalar_data_idx_i,

    output logic        sram_wr_en_o,
    output logic        sram_wr_bank_sel_o,
    output logic [ADDR_W-1:0] sram_wr_addr_o,
    output logic [31:0] sram_wr_byte_en_o,
    output logic [255:0] sram_wr_data_o,
    output logic        sram_rd_bank_sel_o,
    output logic [ADDR_W-1:0] sram_rd_addr_o
);
  localparam logic [4:0] OPCODE_ATTENTION = 5'd23;
  integer byte_idx;
  logic [5:0] effective_store_bytes;

  always_comb begin
    // Clamp the write byte count to the maximum number of byte lanes physically
    // present in a 256-bit flit.
    effective_store_bytes = (type4_store_bytes_i > 6'd32) ? 6'd32 : type4_store_bytes_i;
    // Only committed type4 distribution flits write local storage.  Type5 flits
    // read storage but never write through this interface.
    sram_wr_en_o = emit_valid_i && type4_valid_i && (effective_store_bytes != 6'd0);
    // Bank 0 is the weight SRAM.  Bank 1 is the KV cache, selected by Attention
    // distribution packets.
    sram_wr_bank_sel_o = type4_valid_i && type4_bank_sel_i;
    sram_wr_addr_o = type4_addr_i;
    sram_wr_data_o = type4_payload_i;
    sram_wr_byte_en_o = '0;
    for (byte_idx = 0; byte_idx < 32; byte_idx = byte_idx + 1) begin
      // Enable only the valid lanes for this flit.  Boundary clipping is already
      // computed in cnoc_mfu as type4_store_bytes_i.
      if (byte_idx < effective_store_bytes) begin
        sram_wr_byte_en_o[byte_idx] = 1'b1;
      end
    end
    // Type5 reads use the same bank convention: Attention reads KV, other
    // operations read the weight bank.
    sram_rd_bank_sel_o = (scalar_opcode_i == OPCODE_ATTENTION);
    // The simple ALU path reads a scalar byte at data_idx.  cnoc_mfu muxes the
    // MatMul streaming weight reads around this scalar path during COMPUTE.
    sram_rd_addr_o = {{(ADDR_W-10){1'b0}}, scalar_data_idx_i};
  end

endmodule
