// Description: Functional SwiGLU leaf for the MFU ALU.
//              The current implementation keeps the DPI-C contract so RTL can
//              later swap in a pipeline or approximation without changing the
//              parent ALU shell.

module mfu_alu_swiglu #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic start_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  logic signed [31:0] scalar_a_i,
    input  logic signed [31:0] scalar_b_i,
    output logic busy_o,
    output logic valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

  import "DPI-C" function int dpi_swiglu(input int gate_q, input int up_q);

  integer lane_idx;
  logic signed [31:0] lhs_q;
  logic signed [31:0] rhs_q;
  logic signed [31:0] scalar_tmp;
  logic [7:0] sat_byte_tmp;
  logic [7:0] scalar_byte_tmp;
  logic [FLIT_W-1:0] flit_tmp;
  flit_meta_t meta_tmp;
  int dpi_tmp;
  logic [4:0] data_lane;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [15:0] pair_update_mask;
  logic [FLIT_W-1:0] result_flit;
  flit_meta_t result_meta;
  logic signed [31:0] result_scalar;

  assign data_lane = meta_i.data_offset[4:0];
  assign payload_len = meta_i.payload_len;
  assign pair_update_mask = meta_i.header_reserved;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign busy_o = 1'b0;

  always_comb begin : proc_swiglu_result
    flit_tmp = flit_i;
    meta_tmp = meta_i;
    lhs_q = 32'sd0;
    rhs_q = 32'sd0;
    scalar_tmp = 32'sd0;
    sat_byte_tmp = 8'd0;
    scalar_byte_tmp = 8'd0;
    dpi_tmp = 0;

    for (lane_idx = 0; lane_idx < 32; lane_idx = lane_idx + 2) begin
      if (((lane_idx + 1) < effective_payload_len) &&
          pair_update_mask[lane_idx / 2]) begin
        lhs_q = {{24{flit_i[(lane_idx * 8) + 7]}}, flit_i[lane_idx * 8 +: 8]};
        rhs_q = {{24{flit_i[((lane_idx + 1) * 8) + 7]}},
                 flit_i[(lane_idx + 1) * 8 +: 8]};
        dpi_tmp = dpi_swiglu(lhs_q, rhs_q);
        if (dpi_tmp > 127) begin
          sat_byte_tmp = 8'h7f;
        end else if (dpi_tmp < -128) begin
          sat_byte_tmp = 8'h80;
        end else begin
          sat_byte_tmp = dpi_tmp[7:0];
        end
        flit_tmp[lane_idx * 8 +: 8] = sat_byte_tmp;
      end
    end

    scalar_byte_tmp = flit_tmp[{data_lane, 3'b000} +: 8];
    scalar_tmp = {{24{scalar_byte_tmp[7]}}, scalar_byte_tmp} +
                      ((scalar_a_i ^ scalar_b_i) & 32'sd0);
    result_flit = flit_tmp;
    result_meta = meta_tmp;
    result_scalar = scalar_tmp;

`ifdef ROUTER_ENABLE_COSIM
    result_meta.cosim.data_q = result_flit[{data_lane, 3'b000} +: 8];
`endif
  end

  always_ff @(posedge clk_i) begin : proc_swiglu_registers
    if (reset_i) begin
      valid_o <= 1'b0;
      result_flit_o <= '0;
      result_meta_o <= '0;
      result_scalar_o <= 32'sd0;
    end else begin
      valid_o <= start_i;
      if (start_i) begin
        result_flit_o <= result_flit;
        result_meta_o <= result_meta;
        result_scalar_o <= result_scalar;
      end
    end
  end

endmodule
