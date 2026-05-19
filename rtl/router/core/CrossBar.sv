// Description: Five-by-five combinational crossbar.
//              Selection values identify the input port driving each output
//              port.  Typed metadata follows the payload through the same mux.
//              The current pipelined Router top performs output staging
//              internally; this module remains as a standalone reference block
//              for legacy/unit coverage and is not on Router.sv's main path.

module CrossBar #(
    parameter integer FLIT_W = router_ports_pkg::FLIT_W
) (
    output logic [FLIT_W-1:0] OE,
    output logic [FLIT_W-1:0] OW,
    output logic [FLIT_W-1:0] ON,
    output logic [FLIT_W-1:0] OS,
    output logic [FLIT_W-1:0] Eject,
    output router_ports_pkg::flit_meta_t OE_META,
    output router_ports_pkg::flit_meta_t OW_META,
    output router_ports_pkg::flit_meta_t ON_META,
    output router_ports_pkg::flit_meta_t OS_META,
    output router_ports_pkg::flit_meta_t Eject_META,
    input  logic [2:0] S_E,
    input  logic [2:0] S_W,
    input  logic [2:0] S_N,
    input  logic [2:0] S_S,
    input  logic [2:0] S_Ejec,
    input  logic [FLIT_W-1:0] IE,
    input  logic [FLIT_W-1:0] IW,
    input  logic [FLIT_W-1:0] IN,
    input  logic [FLIT_W-1:0] IS,
    input  logic [FLIT_W-1:0] Inject,
    input  router_ports_pkg::flit_meta_t IE_META,
    input  router_ports_pkg::flit_meta_t IW_META,
    input  router_ports_pkg::flit_meta_t IN_META,
    input  router_ports_pkg::flit_meta_t IS_META,
    input  router_ports_pkg::flit_meta_t Inject_META
);
  import router_ports_pkg::*;

  always_comb begin
    unique case (S_E)
      3'd0: begin OE = IE;     OE_META = IE_META;     end
      3'd1: begin OE = IW;     OE_META = IW_META;     end
      3'd2: begin OE = IN;     OE_META = IN_META;     end
      3'd3: begin OE = IS;     OE_META = IS_META;     end
      3'd4: begin OE = Inject; OE_META = Inject_META; end
      default: begin OE = {FLIT_W{1'b0}}; OE_META = '0; end
    endcase

    unique case (S_W)
      3'd0: begin OW = IE;     OW_META = IE_META;     end
      3'd1: begin OW = IW;     OW_META = IW_META;     end
      3'd2: begin OW = IN;     OW_META = IN_META;     end
      3'd3: begin OW = IS;     OW_META = IS_META;     end
      3'd4: begin OW = Inject; OW_META = Inject_META; end
      default: begin OW = {FLIT_W{1'b0}}; OW_META = '0; end
    endcase

    unique case (S_N)
      3'd0: begin ON = IE;     ON_META = IE_META;     end
      3'd1: begin ON = IW;     ON_META = IW_META;     end
      3'd2: begin ON = IN;     ON_META = IN_META;     end
      3'd3: begin ON = IS;     ON_META = IS_META;     end
      3'd4: begin ON = Inject; ON_META = Inject_META; end
      default: begin ON = {FLIT_W{1'b0}}; ON_META = '0; end
    endcase

    unique case (S_S)
      3'd0: begin OS = IE;     OS_META = IE_META;     end
      3'd1: begin OS = IW;     OS_META = IW_META;     end
      3'd2: begin OS = IN;     OS_META = IN_META;     end
      3'd3: begin OS = IS;     OS_META = IS_META;     end
      3'd4: begin OS = Inject; OS_META = Inject_META; end
      default: begin OS = {FLIT_W{1'b0}}; OS_META = '0; end
    endcase

    unique case (S_Ejec)
      3'd0: begin Eject = IE;     Eject_META = IE_META;     end
      3'd1: begin Eject = IW;     Eject_META = IW_META;     end
      3'd2: begin Eject = IN;     Eject_META = IN_META;     end
      3'd3: begin Eject = IS;     Eject_META = IS_META;     end
      3'd4: begin Eject = Inject; Eject_META = Inject_META; end
      default: begin Eject = {FLIT_W{1'b0}}; Eject_META = '0; end
    endcase
  end
endmodule
