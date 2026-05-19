// Description: Local MFU storage.
//              Two banks model router-local weight and KV-cache storage used
//              by the functional cNoC datapath.

module mfu_sram #(
    parameter int DATA_W = 64,
    parameter int DEPTH = 2048,
    parameter int ADDR_W = 11
) (
    input  logic clk,
    input  logic reset,
    input  logic wr_en,
    input  logic wr_bank_sel,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [31:0] wr_byte_en,
    input  logic [255:0] wr_data,
    input  logic rd_bank_sel,
    input  logic [ADDR_W-1:0] rd_addr,
    output logic [DATA_W-1:0] rd_data
);
  localparam int DATA_BYTES = DATA_W / 8;

  logic [7:0] weight_bank [0:DEPTH-1];
  logic [7:0] kv_bank [0:DEPTH-1];

  always_ff @(posedge clk) begin : proc_storage
    if (reset) begin
      for (int bank_idx = 0; bank_idx < DEPTH; bank_idx = bank_idx + 1) begin
        weight_bank[bank_idx] <= 8'd0;
        kv_bank[bank_idx] <= 8'd0;
      end
    end else if (wr_en) begin
      for (int lane_idx = 0; lane_idx < 32; lane_idx = lane_idx + 1) begin
        if (wr_byte_en[lane_idx] &&
            (({21'd0, wr_addr} + 32'(lane_idx)) < 32'(DEPTH))) begin
          if (wr_bank_sel) begin
            kv_bank[wr_addr + ADDR_W'(lane_idx)] <= wr_data[lane_idx*8 +: 8];
          end else begin
            weight_bank[wr_addr + ADDR_W'(lane_idx)] <= wr_data[lane_idx*8 +: 8];
          end
        end
      end
    end
  end

  always_comb begin : proc_read
    rd_data = '0;
    for (int byte_idx = 0; byte_idx < DATA_BYTES; byte_idx = byte_idx + 1) begin
      if (({21'd0, rd_addr} + 32'(byte_idx)) < 32'(DEPTH)) begin
        rd_data[byte_idx*8 +: 8] =
            rd_bank_sel ? kv_bank[rd_addr + ADDR_W'(byte_idx)] :
                          weight_bank[rd_addr + ADDR_W'(byte_idx)];
      end
    end
  end
endmodule
