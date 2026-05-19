// Description: MFU control FSM.
//              Models the IDLE -> DECODE -> FETCH -> COMPUTE -> WRITE_BACK
//              sequence used by cNoC type4/type5 flits.

module mfu_ctrl_fsm (
    input  logic clk,
    input  logic reset,
    input  logic start,
    input  logic compute_done,
    input  logic write_back_ready,
    output logic busy,
    output logic decode_en,
    output logic fetch_en,
    output logic compute_en,
    output logic write_back_en
);
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_DECODE,
    ST_FETCH,
    ST_COMPUTE,
    ST_WRITE_BACK
  } mfu_state_e;

  mfu_state_e state_q;
  mfu_state_e state_d;

  always_comb begin : proc_next_state
    state_d = state_q;
    case (state_q)
      ST_IDLE: begin
        if (start) begin
          state_d = ST_DECODE;
        end
      end
      ST_DECODE: begin
        state_d = ST_FETCH;
      end
      ST_FETCH: begin
        state_d = ST_COMPUTE;
      end
      ST_COMPUTE: begin
        if (compute_done) begin
          state_d = ST_WRITE_BACK;
        end
      end
      ST_WRITE_BACK: begin
        if (write_back_ready) begin
          state_d = ST_IDLE;
        end
      end
      default: begin
        state_d = ST_IDLE;
      end
    endcase
  end

  always_comb begin : proc_outputs
    decode_en = (state_q == ST_DECODE);
    fetch_en = (state_q == ST_FETCH);
    compute_en = (state_q == ST_COMPUTE);
    write_back_en = (state_q == ST_WRITE_BACK);
    busy = (state_q != ST_IDLE);
  end

  always_ff @(posedge clk) begin : proc_registers
    if (reset) begin
      state_q <= ST_IDLE;
    end else begin
      state_q <= state_d;
    end
  end
endmodule
