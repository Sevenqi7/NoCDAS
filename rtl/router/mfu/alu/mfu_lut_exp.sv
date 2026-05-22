// Copyright (c) 2026
//
// Description: Quantized Q4 exponential lookup used by the cNoC ALU leaves.
//              The table values intentionally match the previous inline
//              Attention exp approximation exactly.

module mfu_lut_exp (
    input  logic signed [7:0] exp_idx_i,
    output logic signed [7:0] exp_q4_o
);

  logic [7:0] exp_idx_u;

  assign exp_idx_u = exp_idx_i;

  always_comb begin : proc_exp_lut
    exp_q4_o = 8'sd0;

    unique case (exp_idx_u)
      8'h00: exp_q4_o = 8'sd16;
      8'h01: exp_q4_o = 8'sd17;
      8'h02: exp_q4_o = 8'sd18;
      8'h03: exp_q4_o = 8'sd19;
      8'h04: exp_q4_o = 8'sd21;
      8'h05: exp_q4_o = 8'sd22;
      8'h06: exp_q4_o = 8'sd23;
      8'h07: exp_q4_o = 8'sd25;
      8'h08: exp_q4_o = 8'sd26;
      8'h09: exp_q4_o = 8'sd28;
      8'h0a: exp_q4_o = 8'sd30;
      8'h0b: exp_q4_o = 8'sd32;
      8'h0c: exp_q4_o = 8'sd34;
      8'h0d: exp_q4_o = 8'sd36;
      8'h0e: exp_q4_o = 8'sd38;
      8'h0f: exp_q4_o = 8'sd41;
      8'h10: exp_q4_o = 8'sd43;
      8'h11: exp_q4_o = 8'sd46;
      8'h12: exp_q4_o = 8'sd49;
      8'h13: exp_q4_o = 8'sd52;
      8'h14: exp_q4_o = 8'sd56;
      8'h15: exp_q4_o = 8'sd59;
      8'h16: exp_q4_o = 8'sd63;
      8'h17: exp_q4_o = 8'sd67;
      8'h18: exp_q4_o = 8'sd72;
      8'h19: exp_q4_o = 8'sd76;
      8'h1a: exp_q4_o = 8'sd81;
      8'h1b: exp_q4_o = 8'sd86;
      8'h1c: exp_q4_o = 8'sd92;
      8'h1d: exp_q4_o = 8'sd98;
      8'h1e: exp_q4_o = 8'sd104;
      8'h1f: exp_q4_o = 8'sd111;
      8'h20: exp_q4_o = 8'sd118;
      8'h21: exp_q4_o = 8'sd126;
      default: begin
        if (exp_idx_u >= 8'h22 && exp_idx_u <= 8'h7f) begin
          exp_q4_o = 8'sd127;
        end else if (exp_idx_u >= 8'hc9 && exp_idx_u <= 8'hda) begin
          exp_q4_o = 8'sd1;
        end else if (exp_idx_u >= 8'hdb && exp_idx_u <= 8'he2) begin
          exp_q4_o = 8'sd2;
        end else if (exp_idx_u >= 8'he3 && exp_idx_u <= 8'he7) begin
          exp_q4_o = 8'sd3;
        end else if (exp_idx_u >= 8'he8 && exp_idx_u <= 8'heb) begin
          exp_q4_o = 8'sd4;
        end else if (exp_idx_u >= 8'hec && exp_idx_u <= 8'hee) begin
          exp_q4_o = 8'sd5;
        end else if (exp_idx_u >= 8'hef && exp_idx_u <= 8'hf1) begin
          exp_q4_o = 8'sd6;
        end else if (exp_idx_u >= 8'hf2 && exp_idx_u <= 8'hf3) begin
          exp_q4_o = 8'sd7;
        end else if (exp_idx_u >= 8'hf4 && exp_idx_u <= 8'hf5) begin
          exp_q4_o = 8'sd8;
        end else if (exp_idx_u >= 8'hf6 && exp_idx_u <= 8'hf7) begin
          exp_q4_o = 8'sd9;
        end else if (exp_idx_u >= 8'hf8 && exp_idx_u <= 8'hf9) begin
          exp_q4_o = 8'sd10;
        end else if (exp_idx_u == 8'hfa) begin
          exp_q4_o = 8'sd11;
        end else if (exp_idx_u >= 8'hfb && exp_idx_u <= 8'hfc) begin
          exp_q4_o = 8'sd12;
        end else if (exp_idx_u == 8'hfd) begin
          exp_q4_o = 8'sd13;
        end else if (exp_idx_u == 8'hfe) begin
          exp_q4_o = 8'sd14;
        end else if (exp_idx_u == 8'hff) begin
          exp_q4_o = 8'sd15;
        end
      end
    endcase
  end

endmodule
