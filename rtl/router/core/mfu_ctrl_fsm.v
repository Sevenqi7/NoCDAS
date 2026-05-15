// Description: MFU control FSM.
//              Models the IDLE -> DECODE -> FETCH -> COMPUTE -> WRITE_BACK
//              sequence used by cNoC type4/type5 flits.

module mfu_ctrl_fsm (
    input  clk,
    input  reset,
    input  start,
    input  compute_done,
    input  write_back_ready,
    output busy,
    output decode_en,
    output fetch_en,
    output compute_en,
    output write_back_en
);
  localparam [2:0] ST_IDLE       = 3'd0;
  localparam [2:0] ST_DECODE     = 3'd1;
  localparam [2:0] ST_FETCH      = 3'd2;
  localparam [2:0] ST_COMPUTE    = 3'd3;
  localparam [2:0] ST_WRITE_BACK = 3'd4;

  reg [2:0] state;
  reg [2:0] next_state;

  always @(posedge clk) begin
    // Synchronous reset returns the MFU to an idle, non-busy state.
    if (reset)
      state <= ST_IDLE;
    // Otherwise advance according to the combinational state machine below.
    else
      state <= next_state;
  end

  always @(*) begin
    next_state = state;
    case (state)
      ST_IDLE: begin
        // IDLE only observes that the side pipeline captured one selected flit.
        if (start)
          next_state = ST_DECODE;
      end
      ST_DECODE: begin
        // Decode is modeled as a dedicated stage to match the proposed MFU
        // controller structure, even though decoding is combinational today.
        next_state = ST_FETCH;
      end
      ST_FETCH: begin
        // Fetch represents SRAM/KV operand access before arithmetic begins.
        next_state = ST_COMPUTE;
      end
      ST_COMPUTE: begin
        // Remain in COMPUTE until the operation-specific wait counter says the
        // functional result is available.
        if (compute_done)
          next_state = ST_WRITE_BACK;
      end
      ST_WRITE_BACK: begin
        // Hold writeback when the originally selected output lacks credit; this
        // backpressures the single-entry MFU pipeline.
        if (write_back_ready)
          next_state = ST_IDLE;
      end
      default: begin
        // Illegal state recovery keeps the router from wedging during simulation
        // or after X-propagation in early RTL bring-up.
        next_state = ST_IDLE;
      end
    endcase
  end

  assign decode_en     = (state == ST_DECODE);
  assign fetch_en      = (state == ST_FETCH);
  assign compute_en    = (state == ST_COMPUTE);
  assign write_back_en = (state == ST_WRITE_BACK);
  assign busy          = (state != ST_IDLE);
endmodule
