// Description: Pipelined shared 8-lane signed int8 multiplier datapath for
//              MFU ALU leaves.

module mfu_int8_mul8 (
    input  logic clk_i,
    input  logic reset_i,
    input  logic valid_i,
    output logic valid_o,
    input  logic signed [7:0]  lhs_i [0:7],
    input  logic signed [7:0]  rhs_i [0:7],
    output logic signed [15:0] product_o [0:7]
);
  logic valid_q;
  logic signed [7:0] lhs_q [0:7];
  logic signed [7:0] rhs_q [0:7];

  int unsigned lane_idx;

  always_ff @(posedge clk_i) begin : proc_mul_pipeline
    if (reset_i) begin
      valid_q <= 1'b0;
      valid_o <= 1'b0;
      for (lane_idx = 0; lane_idx < 8; lane_idx = lane_idx + 1) begin
        lhs_q[lane_idx] <= '0;
        rhs_q[lane_idx] <= '0;
        product_o[lane_idx] <= '0;
      end
    end else begin
      valid_q <= valid_i;
      valid_o <= valid_q;

      if (valid_i) begin
        for (lane_idx = 0; lane_idx < 8; lane_idx = lane_idx + 1) begin
          lhs_q[lane_idx] <= lhs_i[lane_idx];
          rhs_q[lane_idx] <= rhs_i[lane_idx];
        end
      end

      if (valid_q) begin
        for (lane_idx = 0; lane_idx < 8; lane_idx = lane_idx + 1) begin
          product_o[lane_idx] <= lhs_q[lane_idx] * rhs_q[lane_idx];
        end
      end
    end
  end

endmodule
