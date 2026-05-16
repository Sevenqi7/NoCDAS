// Description: Shared router package.
//              Defines the flattened flit/meta widths, direction encoding, and
//              packed flow/data bundles used by Router and its submodules.

package router_ports_pkg;

  // Direction coding shared by Router and its wrappers.
  typedef enum logic [2:0] {
    ROUTER_PORT_EAST  = 3'd0,
    ROUTER_PORT_WEST  = 3'd1,
    ROUTER_PORT_NORTH = 3'd2,
    ROUTER_PORT_SOUTH = 3'd3,
    ROUTER_PORT_LOCAL = 3'd4,
    ROUTER_PORT_INV   = 3'b111
  } router_port_sel_e;

  // Flit kind coding follows NoCDAS Flit::type.
  typedef enum logic [3:0] {
    ROUTER_FLIT_HEAD      = 4'd0,
    ROUTER_FLIT_TAIL      = 4'd1,
    ROUTER_FLIT_BODY      = 4'd2,
    ROUTER_FLIT_HEAD_TAIL = 4'd10
  } router_flit_kind_e;

  // Message type coding follows NoCDAS Message::type.
  typedef enum logic [2:0] {
    ROUTER_MSG_MEM_READ  = 3'd0,
    ROUTER_MSG_MEM_WRITE = 3'd1,
    ROUTER_MSG_REGULAR_2 = 3'd2,
    ROUTER_MSG_REGULAR_3 = 3'd3,
    ROUTER_MSG_DIST      = 3'd4,
    ROUTER_MSG_COMP      = 3'd5
  } router_msg_type_e;

  // Switch/VC QoS classes. Higher numeric value wins arbitration.
  typedef enum logic [1:0] {
    ROUTER_TRAFFIC_REGULAR = 2'd0,
    ROUTER_TRAFFIC_DIST    = 2'd1,
    ROUTER_TRAFFIC_COMP    = 2'd2
  } router_traffic_class_e;

  localparam int FLIT_W = 256;
  localparam int PORT_NUM = 5;
  localparam int ROUTE_MAX_HOPS = 128;
  localparam int ROUTE_PORT_W = 3;
  localparam int ROUTE_SEQ_W = ROUTE_MAX_HOPS * ROUTE_PORT_W;
  localparam int ROUTE_PROCESS_SEQ_W = ROUTE_MAX_HOPS;
  localparam int VC_ID_W = 3;
  localparam int VC_NUM = 8;
  localparam int CNOC_MAX_TASKS = 32;
  localparam int CNOC_TASK_ID_W = 16;
  localparam int CNOC_SRAM_DATA_W = 64;
  localparam int CNOC_SRAM_ADDR_W = 11;

  // Semantic field widths carried by flit_meta_t.  The 256-bit flit payload is
  // treated as data; routing/compute control lives in the hardware fields below,
  // while C++/NoCDAS co-sim fields are isolated in flit_cosim_meta_t.  Packed bit
  // offsets stay out of RTL logic and are owned by the C++ wrapper ABI boundary.
  localparam int META_FLIT_KIND_W = 4;
  localparam int META_MSG_TYPE_W = 3;
  localparam int META_OPCODE_W = 5;
  localparam int META_COSIM_TOKEN_W = 24;
  localparam int META_DATA_OFFSET_W = 10;
  localparam int META_DATA_Q_W = 8;
  localparam int META_COORD_W = 3;
  localparam int META_ROUTE_LEN_W = 8;
  localparam int META_ROUTE_PTR_W = 8;
  localparam int ROUTE_PATH_W =
      1 + META_ROUTE_LEN_W + META_ROUTE_PTR_W + ROUTE_SEQ_W + ROUTE_PROCESS_SEQ_W;
  localparam int META_VNET_W = 2;
  localparam int META_PAYLOAD_LEN_W = 6;
  localparam int META_TRAFFIC_CLASS_W = 2;
  localparam int META_PSUM_OFFSET_W = 16;
  localparam int META_K_DIM_W = 16;
  localparam int META_HEADER_RESERVED_W = 16;
  localparam int META_PACKET_UID_W = 32;
  localparam int META_FLIT_ID_W = 16;

  typedef struct packed {
    // Source-route sideband carried only at router input/output boundaries.
    // It is captured once per input VC and must not travel through the common
    // flit_meta_t FIFO/crossbar datapath.
    logic [ROUTE_PROCESS_SEQ_W-1:0] route_process_seq;
    logic [ROUTE_SEQ_W-1:0] route_seq;
    logic [META_ROUTE_PTR_W-1:0] route_ptr;
    logic [META_ROUTE_LEN_W-1:0] route_len;
    logic valid;
  } route_path_t;

  typedef struct packed {
    // C++/NoCDAS co-sim-only bookkeeping.  These fields are excluded from
    // deliverable RTL when ROUTER_ENABLE_COSIM is not defined.
    logic [META_COSIM_TOKEN_W-1:0] cosim_token;
    logic [META_PACKET_UID_W-1:0] packet_uid;
    logic [META_FLIT_ID_W-1:0] flit_id;
    logic [META_DATA_Q_W-1:0] data_q;
  } flit_cosim_meta_t;

  typedef struct packed {
`ifdef ROUTER_ENABLE_COSIM
    flit_cosim_meta_t cosim;
`endif
    logic [META_HEADER_RESERVED_W-1:0] header_reserved;
    logic [META_K_DIM_W-1:0] k_dim;
    logic [META_PSUM_OFFSET_W-1:0] psum_offset;
    router_traffic_class_e        traffic_class;
    logic [META_PAYLOAD_LEN_W-1:0] payload_len;
    logic [META_VNET_W-1:0] vnet;
    logic [META_ROUTE_PTR_W-1:0] route_ptr;
    logic                         route_valid;
    logic                         process;
    logic [META_COORD_W-1:0] dst_x;
    logic [META_COORD_W-1:0] dst_y;
    logic [META_DATA_OFFSET_W-1:0] data_offset;
    logic [META_OPCODE_W-1:0] opcode;
    router_msg_type_e             msg_type;
    router_flit_kind_e            flit_kind;
    logic                         valid;
  } flit_meta_t;

  typedef struct packed {
    logic start;
    logic fetch_en;
    logic compute_en;
    logic state_release;
  } mfu_alu_ctrl_t;

  typedef struct packed {
    logic [META_OPCODE_W-1:0] opcode;
    router_msg_type_e         msg_type;
    logic                     is_type5;
    logic                     is_data_op;
    logic                     is_attention;
  } mfu_alu_op_t;

  typedef struct packed {
    logic [VC_ID_W-1:0] vc_id;
    logic [2:0] out_sel;
    logic [15:0] kv_token_count;
    logic [5:0] task_count;
    logic [CNOC_MAX_TASKS*CNOC_TASK_ID_W-1:0] task_ids_flat;
    logic [15:0] weight_row_size;
  } mfu_alu_ctx_t;

  typedef struct packed {
    logic valid;
    logic bank_sel;
    logic [CNOC_SRAM_ADDR_W-1:0] addr;
  } mfu_alu_data_req_t;

  typedef struct packed {
    logic [CNOC_SRAM_DATA_W-1:0] rdata;
  } mfu_alu_data_rsp_t;

  typedef struct packed {
    logic valid;
    logic bank_sel;
    logic [CNOC_SRAM_ADDR_W-1:0] addr;
    logic [31:0] byte_en;
    logic [FLIT_W-1:0] data;
  } mfu_alu_data_wr_t;

  typedef struct packed {
    logic busy;
    logic valid;
    logic [FLIT_W-1:0] flit;
    flit_meta_t meta;
    logic signed [31:0] scalar;
    logic attention_active;
    logic [FLIT_W-1:0] attention_flit;
    flit_meta_t attention_meta;
  } mfu_alu_result_t;

  // Transitional alias for older code/comments.  New RTL should use
  // flit_meta_t so the type name reflects the NoC object it describes.
  typedef flit_meta_t router_header_t;

  localparam int META_HW_W =
      META_HEADER_RESERVED_W +
      META_K_DIM_W +
      META_PSUM_OFFSET_W +
      META_TRAFFIC_CLASS_W +
      META_PAYLOAD_LEN_W +
      META_VNET_W +
      META_ROUTE_PTR_W +
      1 + // route_valid
      1 + // process
      META_COORD_W + // dst_x
      META_COORD_W + // dst_y
      META_DATA_OFFSET_W +
      META_OPCODE_W +
      META_MSG_TYPE_W +
      META_FLIT_KIND_W +
      1; // valid

`ifdef ROUTER_ENABLE_COSIM
  localparam int META_COSIM_W = $bits(flit_cosim_meta_t);
`else
  localparam int META_COSIM_W = 0;
`endif
  localparam int META_W = META_HW_W + META_COSIM_W;
  localparam int META_STRUCT_W = $bits(flit_meta_t);
  localparam int META_WIDTH_GUARD = 1 / (META_STRUCT_W == META_W);

  // Pure combinational predicates.  These helpers intentionally stay tiny:
  // enum in, one boolean expression out, no state, no route/VC/MFU policy.
  function logic flit_is_head_like(input router_flit_kind_e flit_kind);
    flit_is_head_like = (flit_kind == ROUTER_FLIT_HEAD) ||
                        (flit_kind == ROUTER_FLIT_HEAD_TAIL);
  endfunction

  function logic flit_is_tail_like(input router_flit_kind_e flit_kind);
    flit_is_tail_like = (flit_kind == ROUTER_FLIT_TAIL) ||
                        (flit_kind == ROUTER_FLIT_HEAD_TAIL);
  endfunction

  // Router ingress data bundle (environment -> router).
  typedef struct packed {
    logic [FLIT_W-1:0] flit;
    flit_meta_t   meta;
    route_path_t  route_path;
    logic         push;
    logic [VC_ID_W-1:0] vc_id;
  } rinport_data_t;

  // Router ingress flow-control bundle (router -> environment).
  typedef struct packed {
    logic [VC_NUM-1:0] push_ack;
    logic                     in_accept;
  } rinport_flow_t;

  // Router ingress control sideband consumed by rinport(input_unit).
  typedef struct packed {
    logic                     vc_grant;
    logic [2:0]               x_cur;
    logic [2:0]               y_cur;
    logic [2:0]               in_channel;
  } rinport_ctrl_t;

  // Router ingress issue bundle produced by rinport(input_unit).
  typedef struct packed {
    logic [FLIT_W-1:0] flit;
    flit_meta_t   meta;
    logic                     valid;
    logic [2:0]               route_sel;
    logic                     pop_fire;
    logic [VC_ID_W-1:0] sel_vc;
  } rinport_issue_t;

  // Router egress flow-control bundle (environment -> router).
  typedef struct packed {
    logic                     state_ready;
    logic [VC_NUM-1:0] downstream_vc_idle_mask;
    logic [VC_NUM-1:0] downstream_vc_credit_mask;
  } routport_flow_t;

  // Router egress data bundle (router -> environment).
  typedef struct packed {
    logic [FLIT_W-1:0] flit;
    flit_meta_t   meta;
    route_path_t  route_path;
    logic         write_req;
    logic [VC_ID_W-1:0] vc_id;
  } routport_data_t;

endpackage
