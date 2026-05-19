// Description: Functional ADD leaf for the MFU ALU.
//              Pairwise INT8 addition stays combinational for now, but this
//              leaf isolates the opcode-specific path for later RTL work.

module mfu_alu_add #(
    parameter int FLIT_W = router_ports_pkg::FLIT_W
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic start_i,
    input  logic [FLIT_W-1:0] flit_i,
    input  router_ports_pkg::flit_meta_t meta_i,
    input  router_ports_pkg::mfu_alu_ctx_t ctx_i,
    input  logic signed [31:0] scalar_a_i,
    input  logic signed [31:0] scalar_b_i,
    output logic busy_o,
    output logic valid_o,
    output logic [FLIT_W-1:0] result_flit_o,
    output router_ports_pkg::flit_meta_t result_meta_o,
    output logic signed [31:0] result_scalar_o
);
  import router_ports_pkg::*;

  integer task_idx;
  integer lane_idx;
  logic signed [31:0] lhs_q;
  logic signed [31:0] rhs_q;
  logic signed [31:0] lane_value_tmp;
  logic [7:0] sat_byte_tmp;
  logic [7:0] scalar_byte_tmp;
  logic [FLIT_W-1:0] flit_tmp;
  flit_meta_t meta_tmp;
  logic [4:0] data_lane;
  logic [5:0] payload_len;
  logic [5:0] effective_payload_len;
  logic [5:0] task_count_limited;
  logic [CNOC_TASK_ID_W-1:0] task_id_current;
  logic [16:0] required_flit_offset;
  logic [16:0] flit_start_idx;
  logic [16:0] flit_end_idx;
  logic [5:0] local_lane;
  logic [15:0] pair_update_mask;
  logic [FLIT_W-1:0] result_flit;
  flit_meta_t result_meta;
  logic signed [31:0] result_scalar;

  assign data_lane = meta_i.data_offset[4:0];
  assign payload_len = meta_i.payload_len;
  assign task_count_limited =
      (ctx_i.task_count > 6'(CNOC_MAX_TASKS)) ? 6'(CNOC_MAX_TASKS) : ctx_i.task_count;
  assign effective_payload_len =
      (payload_len == 6'd0) ? 6'd1 :
      (payload_len > 6'd32) ? 6'd32 :
                              payload_len;
  assign busy_o = 1'b0;

  always_comb begin : proc_pair_update_mask
    pair_update_mask = '0;
    task_id_current = '0;
    required_flit_offset = '0;
    flit_start_idx = {7'd0, meta_i.data_offset};
    flit_end_idx = {7'd0, meta_i.data_offset} + {11'd0, effective_payload_len};
    local_lane = 6'd0;

    for (task_idx = 0; task_idx < CNOC_MAX_TASKS; task_idx = task_idx + 1) begin
      if (6'(task_idx) < task_count_limited) begin
        task_id_current = ctx_i.task_ids_flat[task_idx * CNOC_TASK_ID_W +: CNOC_TASK_ID_W];
        required_flit_offset = {1'b0, task_id_current} << 1;
        local_lane = required_flit_offset[5:0] - meta_i.data_offset[5:0];
        if ((required_flit_offset >= flit_start_idx) &&
            ((required_flit_offset + 17'd1) < flit_end_idx) &&
            ((local_lane + 6'd1) < 6'd32)) begin
          pair_update_mask[local_lane[4:1]] = 1'b1;
        end
      end
    end
  end

  always_comb begin : proc_add_result
    flit_tmp = flit_i;
    meta_tmp = meta_i;
    lhs_q = 32'sd0;
    rhs_q = 32'sd0;
    lane_value_tmp = 32'sd0;
    sat_byte_tmp = 8'd0;
    scalar_byte_tmp = 8'd0;

    for (lane_idx = 0; lane_idx < 32; lane_idx = lane_idx + 2) begin
      if (((lane_idx + 1) < effective_payload_len) &&
          pair_update_mask[lane_idx / 2]) begin
        lhs_q = {{24{flit_i[(lane_idx * 8) + 7]}}, flit_i[lane_idx * 8 +: 8]};
        rhs_q = {{24{flit_i[((lane_idx + 1) * 8) + 7]}},
                 flit_i[(lane_idx + 1) * 8 +: 8]};
        lane_value_tmp = lhs_q + rhs_q;
        if (lane_value_tmp > 32'sd127) begin
          sat_byte_tmp = 8'h7f;
        end else if (lane_value_tmp < -32'sd128) begin
          sat_byte_tmp = 8'h80;
        end else begin
          sat_byte_tmp = lane_value_tmp[7:0];
        end
        flit_tmp[lane_idx * 8 +: 8] = sat_byte_tmp;
      end
    end

    scalar_byte_tmp = flit_tmp[{data_lane, 3'b000} +: 8];
    result_flit = flit_tmp;
    result_meta = meta_tmp;
    result_scalar = {{24{scalar_byte_tmp[7]}}, scalar_byte_tmp} +
                    ((scalar_a_i ^ scalar_b_i) & 32'sd0);

`ifdef ROUTER_ENABLE_COSIM
    result_meta.cosim.data_q = result_flit[{data_lane, 3'b000} +: 8];
`endif
  end

  always_ff @(posedge clk_i) begin : proc_add_registers
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
