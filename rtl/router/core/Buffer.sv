// Description: Four-entry FIFO used as one virtual-channel input buffer.
//              Stores the flit payload and typed metadata in lockstep.

module Buffer #(
    parameter integer FLIT_W = router_ports_pkg::FLIT_W
) (
    output logic [FLIT_W-1:0] bf_out,
    output router_ports_pkg::flit_meta_t bf_meta_out,
    output logic [2:0] em_pl,
    input  logic clk,
    input  logic reset,
    input  logic pop,
    input  logic push,
    input  logic [FLIT_W-1:0] bf_in,
    input  router_ports_pkg::flit_meta_t bf_meta_in
);
  import router_ports_pkg::*;

  logic [FLIT_W-1:0] bf [0:3];
  flit_meta_t bf_meta [0:3];
  logic [1:0] add_wr;
  logic [1:0] add_rd;

  assign bf_out = bf[add_rd];
  assign bf_meta_out = bf_meta[add_rd];

  always_ff @(posedge clk) begin
    if (reset) begin
      bf[0]  <= {FLIT_W{1'b0}};
      bf[1]  <= {FLIT_W{1'b0}};
      bf[2]  <= {FLIT_W{1'b0}};
      bf[3]  <= {FLIT_W{1'b0}};
      bf_meta[0] <= '0;
      bf_meta[1] <= '0;
      bf_meta[2] <= '0;
      bf_meta[3] <= '0;
      em_pl  <= 3'd4;
      add_wr <= 2'b0;
      add_rd <= 2'b0;
    end else begin
      if (push == 1'b1 && pop == 1'b0 && em_pl > 3'd0) begin
        bf[add_wr] <= bf_in;
        bf_meta[add_wr] <= bf_meta_in;
        em_pl      <= em_pl - 3'd1;
        add_wr     <= add_wr + 2'd1;
      end else if (push == 1'b0 && pop == 1'b1 && em_pl < 3'd4) begin
        em_pl      <= em_pl + 3'd1;
        add_rd     <= add_rd + 2'd1;
      end else if (push == 1'b1 && pop == 1'b1) begin
        if (em_pl == 3'd4) begin
          bf[add_wr] <= bf_in;
          bf_meta[add_wr] <= bf_meta_in;
          add_wr     <= add_wr + 2'd1;
          em_pl      <= em_pl - 3'd1;
        end else begin
          bf[add_wr] <= bf_in;
          bf_meta[add_wr] <= bf_meta_in;
          add_wr     <= add_wr + 2'd1;
          add_rd     <= add_rd + 2'd1;
        end
      end
    end
  end
endmodule
