// Description: Decode cNoC sideband metadata.
//              The payload data bus remains separate; this module exposes the
//              typed control fields required by the MFU helpers.

module cnoc_decode (
    input  router_ports_pkg::flit_meta_t flit_meta,
    output router_ports_pkg::router_flit_kind_e flit_type,
    output router_ports_pkg::router_msg_type_e msg_type,
    output logic [4:0] opcode,
    output logic [9:0] data_idx,
    output logic is_type4,
    output logic is_type5,
    output logic is_tail
);
  import router_ports_pkg::*;

  assign flit_type = flit_meta.flit_kind;
  assign msg_type  = flit_meta.msg_type;
  assign opcode    = flit_meta.opcode;
  assign data_idx  = flit_meta.data_offset;

  // Type4 is the cNoC distribution/loading packet class.
  assign is_type4 = (msg_type == ROUTER_MSG_DIST);
  // Type5 is the cNoC in-transit compute packet class.
  assign is_type5 = (msg_type == ROUTER_MSG_COMP);
  // Head-tail is treated as tail-like for state release.
  assign is_tail = flit_is_tail_like(flit_meta.flit_kind);
endmodule
