// Description: Local MFU storage.
//              Two banks model router-local weight and KV-cache storage used
//              by the functional cNoC datapath.

module mfu_sram #(
    parameter integer DATA_W = 64,
    parameter integer DEPTH = 2048,
    parameter integer ADDR_W = 11
)(
    input clk,
    input reset,
    input wr_en,
    input wr_bank_sel, // 0: weight bank, 1: kv bank
    input [ADDR_W-1:0] wr_addr,
    input [31:0] wr_byte_en,
    input [255:0] wr_data,
    input rd_bank_sel, // 0: weight bank, 1: kv bank
    input [ADDR_W-1:0] rd_addr,
    output [DATA_W-1:0] rd_data
);
  localparam integer DATA_BYTES = DATA_W / 8;

  reg [7:0] weight_bank [0:DEPTH-1];
  reg [7:0] kv_bank     [0:DEPTH-1];
  reg [DATA_W-1:0] rd_data_comb;

  integer i;
  integer lane;
  integer lane_addr;
  integer read_byte;
  integer read_addr;
  always @(posedge clk) begin
    // Reset clears both banks to make unit tests and trace regressions
    // deterministic.
    if (reset) begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        weight_bank[i] <= 8'd0;
        kv_bank[i] <= 8'd0;
      end
    // Type4 storage writes one or more byte lanes into the selected bank.
    end else if (wr_en) begin
      for (lane = 0; lane < 32; lane = lane + 1) begin
        lane_addr = wr_addr + lane;
        // Byte-enable plus bounds check prevents partial flits from corrupting
        // neighboring data or wrapping past SRAM end.
        if (wr_byte_en[lane] && lane_addr < DEPTH) begin
          // Bank select 1 is KV cache for Attention.
          if (wr_bank_sel)
            kv_bank[lane_addr] <= wr_data[lane*8 +: 8];
          // Bank select 0 is local weight storage for MatMul/Linear.
          else
            weight_bank[lane_addr] <= wr_data[lane*8 +: 8];
        end
      end
    end
  end

  always @* begin
    rd_data_comb = {DATA_W{1'b0}};
    for (read_byte = 0; read_byte < DATA_BYTES; read_byte = read_byte + 1) begin
      read_addr = rd_addr + read_byte;
      if (read_addr < DEPTH) begin
        rd_data_comb[read_byte*8 +: 8] =
            rd_bank_sel ? kv_bank[read_addr] : weight_bank[read_addr];
      end
    end
  end

  assign rd_data = rd_data_comb;
endmodule
