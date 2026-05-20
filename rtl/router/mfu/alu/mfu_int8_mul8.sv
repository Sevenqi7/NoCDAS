// Description: Shared 8-lane signed int8 multiplier datapath for MFU scalar
//              nonlinear leaves.

module mfu_int8_mul8 (
    input  logic signed [7:0]  lhs_i     [0:7],
    input  logic signed [7:0]  rhs_i     [0:7],
    output logic signed [15:0] product_o [0:7]
);

  for (genvar lane = 0; lane < 8; lane = lane + 1) begin : gen_lane_mul
    assign product_o[lane] = lhs_i[lane] * rhs_i[lane];
  end

endmodule
