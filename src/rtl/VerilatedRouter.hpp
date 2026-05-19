/*
 * VerilatedRouter.hpp
 *
 */

#ifndef VERILATEDROUTER_HPP_
#define VERILATEDROUTER_HPP_

#include "../parameters.hpp"

#include <array>
#include <cstdint>

class Flit;
class FlitBuffer;
class VCRouter;

namespace VerilatedRouterDetail {

constexpr int RTL_PORTS = 5;
constexpr int RTL_VC_NUM = VN_NUM * (VC_PER_VN + VC_PRIORITY_PER_VN);
constexpr int RTL_FLIT_BITS = 256;
constexpr int RTL_FLIT_WORDS = RTL_FLIT_BITS / 32;
constexpr int RTL_ROUTE_MAX_HOPS = 128;
constexpr int RTL_ROUTE_PORT_BITS = 3;
constexpr int RTL_ROUTE_PATH_BITS =
    1 + 8 + 8 + RTL_ROUTE_MAX_HOPS * RTL_ROUTE_PORT_BITS + RTL_ROUTE_MAX_HOPS;
constexpr int RTL_ROUTE_PATH_WORDS = (RTL_ROUTE_PATH_BITS + 31) / 32;
constexpr int RTL_META_BITS = 177;
constexpr int RTL_META_WORDS = (RTL_META_BITS + 31) / 32;
constexpr int RTL_CNOC_MAX_TASKS = 32;
constexpr int RTL_CNOC_TASK_ID_BITS = 16;
constexpr int RTL_CNOC_TASK_IDS_BITS = RTL_CNOC_MAX_TASKS * RTL_CNOC_TASK_ID_BITS;
constexpr int RTL_CNOC_TASK_IDS_WORDS = (RTL_CNOC_TASK_IDS_BITS + 31) / 32;
constexpr int RTL_PORT_DATA_BITS =
    RTL_FLIT_BITS + RTL_META_BITS + RTL_ROUTE_PATH_BITS + 1 + 3;
constexpr int RTL_PORT_DATA_WORDS = (RTL_PORT_DATA_BITS + 31) / 32;
constexpr uint32_t TOKEN_MASK_24 = ((1u << 24) - 1u);
constexpr uint32_t RTL_VC_MASK =
    (RTL_VC_NUM >= 32) ? 0xffffffffu : static_cast<uint32_t>((uint64_t{1} << RTL_VC_NUM) - 1u);
constexpr int CNOC_TYPE_DIST = 4;
constexpr int CNOC_TYPE_COMP = 5;
constexpr int CNOC_OPCODE_ADD = 18;
constexpr int CNOC_OPCODE_SWIGLU = 21;
constexpr int CNOC_OPCODE_ATTENTION = 23;
constexpr int CNOC_OPCODE_GEGLU = 24;
constexpr int RTL_COORD_MAX = 7;

constexpr uint8_t RTL_PORT_EAST = 0u;
constexpr uint8_t RTL_PORT_WEST = 1u;
constexpr uint8_t RTL_PORT_NORTH = 2u;
constexpr uint8_t RTL_PORT_SOUTH = 3u;
constexpr uint8_t RTL_PORT_LOCAL = 4u;
constexpr int RTL_PORT_ROW_DELTA[RTL_PORTS] = {0, 0, -1, 1, 0};
constexpr int RTL_PORT_COL_DELTA[RTL_PORTS] = {1, -1, 0, 0, 0};

constexpr uint32_t RTL_TRAFFIC_REGULAR = 0u;
constexpr uint32_t RTL_TRAFFIC_DIST = 1u;
constexpr uint32_t RTL_TRAFFIC_COMP = 2u;

static_assert(RTL_VC_NUM > 0, "RTL_VC_NUM must be positive");
static_assert(RTL_VC_NUM <= 32, "Wrapper push_ack mask assumes RTL_VC_NUM <= 32");

// RTL port order [E, W, N, S, J] -> NoCDAS port order [Right, Left, Up, Down, Local]
constexpr int RTL_TO_CPP_PORT[RTL_PORTS] = {1, 3, 0, 2, 4};

// Verilator flattens flit_meta_t into WData[]; these offsets are the C++/RTL ABI
// and must stay aligned with the packed field order in router_ports_pkg.sv.
constexpr int META_VALID_BIT = 0;
constexpr int META_FLIT_TYPE_LSB = 1;
constexpr int META_FLIT_TYPE_W = 4;
constexpr int META_MSG_TYPE_LSB = 5;
constexpr int META_OPCODE_LSB = 8;
constexpr int META_DATA_IDX_LSB = 13;
constexpr int META_DEST_Y_LSB = 23;
constexpr int META_DEST_X_LSB = 26;
constexpr int META_PROCESS_BIT = 29;
constexpr int META_ROUTE_VALID_BIT = 30;
constexpr int META_ROUTE_PTR_LSB = 31;
constexpr int META_ROUTE_PTR_W = 8;
constexpr int META_VNET_LSB = 39;
constexpr int META_VNET_W = 2;
constexpr int META_PAYLOAD_LEN_LSB = 41;
constexpr int META_PAYLOAD_LEN_W = 6;
constexpr int META_TRAFFIC_CLASS_LSB = 47;
constexpr int META_TRAFFIC_CLASS_W = 2;
constexpr int META_PSUM_OFFSET_LSB = 49;
constexpr int META_PSUM_OFFSET_W = 16;
constexpr int META_K_DIM_LSB = 65;
constexpr int META_K_DIM_W = 16;
constexpr int META_CNOC_PAIR_MASK_LSB = 81;
constexpr int META_CNOC_PAIR_MASK_W = 16;
constexpr int META_ATTENTION_RM_LSB = META_CNOC_PAIR_MASK_LSB;
constexpr int META_ATTENTION_RS_LSB = META_CNOC_PAIR_MASK_LSB + 8;
constexpr int META_ATTENTION_STATE_W = 8;
constexpr int META_COSIM_DATA_Q_LSB = 97;
constexpr int META_COSIM_FLIT_ID_LSB = 105;
constexpr int META_COSIM_PACKET_UID_LSB = 121;
constexpr int META_COSIM_TOKEN_LSB = 153;

static_assert(META_COSIM_TOKEN_LSB + 24 <= RTL_META_BITS,
              "RTL cosim token exceeds meta width");

constexpr int ROUTE_PATH_VALID_BIT = 0;
constexpr int ROUTE_PATH_LEN_LSB = 1;
constexpr int ROUTE_PATH_LEN_W = 8;
constexpr int ROUTE_PATH_PTR_LSB = 9;
constexpr int ROUTE_PATH_PTR_W = 8;
constexpr int ROUTE_PATH_SEQ_LSB = 17;
constexpr int ROUTE_PATH_PROCESS_SEQ_LSB =
    ROUTE_PATH_SEQ_LSB + RTL_ROUTE_MAX_HOPS * RTL_ROUTE_PORT_BITS;

static_assert(ROUTE_PATH_PROCESS_SEQ_LSB + RTL_ROUTE_MAX_HOPS <= RTL_ROUTE_PATH_BITS,
              "RTL source-route path fields exceed route_path_t width");

// Packed struct bit layout after Verilator flattening.
// rinport_data_t  = { flit[255:0], meta[META_W-1:0], route_path, push, vc_id[2:0] }
// routport_data_t = { flit[255:0], meta[META_W-1:0], route_path, write_req, vc_id[2:0] }
constexpr int PORT_DATA_VC_ID_LSB = 0;
constexpr int PORT_DATA_VC_ID_W = 3;
constexpr int PORT_DATA_VALID_BIT = 3;   // push / write_req
constexpr int PORT_DATA_ROUTE_PATH_LSB = 4;
constexpr int PORT_DATA_META_LSB = PORT_DATA_ROUTE_PATH_LSB + RTL_ROUTE_PATH_BITS;
constexpr int PORT_DATA_FLIT_LSB = PORT_DATA_META_LSB + RTL_META_BITS;

// rinport_flow_t = { push_ack[VC_NUM-1:0], in_accept }
constexpr int INPUT_FLOW_IN_ACCEPT_BIT = 0;
constexpr int INPUT_FLOW_PUSH_ACK_LSB = 1;

// routport_flow_t = { state_ready, downstream_vc_idle_mask, downstream_vc_credit_mask }
constexpr int OUTPUT_FLOW_CREDIT_MASK_LSB = 0;
constexpr int OUTPUT_FLOW_IDLE_MASK_LSB = RTL_VC_NUM;
constexpr int OUTPUT_FLOW_STATE_READY_BIT = 2 * RTL_VC_NUM;

using FlitWord = std::array<uint32_t, RTL_FLIT_WORDS>;
using MetaWord = std::array<uint32_t, RTL_META_WORDS>;
using RoutePathWord = std::array<uint32_t, RTL_ROUTE_PATH_WORDS>;
using TaskIdsWord = std::array<uint32_t, RTL_CNOC_TASK_IDS_WORDS>;
using PortDataWord = std::array<uint32_t, RTL_PORT_DATA_WORDS>;

struct PendingOutput {
  Flit* flit = nullptr;
  int cpp_out_port = -1;
  int target_vc = -1;
  int src_cpp_port = -1;
  int src_vc = -1;
};

struct TokenRecord {
  Flit* flit = nullptr;
  int src_cpp_port = -1;
  int src_vc = -1;
};

struct InjectCandidate {
  bool valid = false;
  int cpp_port = -1;
  int src_vc = -1;
  Flit* flit = nullptr;
  FlitBuffer* src_buf = nullptr;
  uint32_t token = 0;
  FlitWord payload{};
  MetaWord meta{};
  RoutePathWord route_path{};
};

}  // namespace VerilatedRouterDetail

class VerilatedRouter {
public:
  explicit VerilatedRouter(VCRouter* owner);
  ~VerilatedRouter();

  void runOneStep();
  void resetCnocState();
  bool ownsCnocMfu() const;
  unsigned int cnocWeightBytesStored() const;
  unsigned int cnocKvBytesStored() const;
  unsigned int cnocKvTokenCount() const;
  bool cnocLastType4StoreValid() const;

private:
  VerilatedRouter(const VerilatedRouter&) = delete;
  VerilatedRouter& operator=(const VerilatedRouter&) = delete;

  class Impl;
  Impl* impl_;
};

#endif /* VERILATEDROUTER_HPP_ */
