/*
 * VerilatedRouter.cpp
 *
 */

#include "VerilatedRouter.hpp"

#include "../NoC/Flit.hpp"
#include "../NoC/FlitBuffer.hpp"
#include "../NoC/Link.hpp"
#include "../NoC/RInPort.hpp"
#include "../NoC/ROutPort.hpp"
#include "../NoC/VCRouter.hpp"
#include "../CNoCQuant.hpp"
#include "../parameters.hpp"

#include "VNoCRouter.h"
#include "verilated.h"

#ifndef ROUTER_ENABLE_COSIM
#error "VerilatedRouter requires ROUTER_ENABLE_COSIM because the C++ mirror uses cosim tokens."
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <unordered_map>
#include <vector>

extern unsigned int cycles;

using namespace VerilatedRouterDetail;

namespace {

template <typename WordT>
void setBit(WordT& word, int bit_idx, bool value) {
  const int word_idx = bit_idx / 32;
  const int bit_off = bit_idx % 32;
  const uint32_t mask = (1u << bit_off);
  if (value) {
    word[word_idx] |= mask;
  } else {
    word[word_idx] &= ~mask;
  }
}

template <typename WordT>
bool getBit(const WordT& word, int bit_idx) {
  const int word_idx = bit_idx / 32;
  const int bit_off = bit_idx % 32;
  return ((word[word_idx] >> bit_off) & 0x1u) != 0u;
}

template <typename WordT>
void setField(WordT& word, int lsb, int width, uint32_t value) {
  for (int i = 0; i < width; ++i) {
    const bool bit = ((value >> i) & 0x1u) != 0u;
    setBit(word, lsb + i, bit);
  }
}

template <typename WordT>
uint32_t getField(const WordT& word, int lsb, int width) {
  uint32_t value = 0u;
  for (int i = 0; i < width; ++i) {
    if (getBit(word, lsb + i)) {
      value |= (1u << i);
    }
  }
  return value;
}

template <typename DstWordT, typename SrcWordT>
void copyBits(DstWordT& dst, int dst_lsb, const SrcWordT& src, int src_lsb, int width) {
  for (int i = 0; i < width; ++i) {
    setBit(dst, dst_lsb + i, getBit(src, src_lsb + i));
  }
}

template <size_t N>
void writeWideWord(WData* dst, const std::array<uint32_t, N>& src) {
  for (size_t i = 0; i < N; ++i) {
    dst[i] = src[i];
  }
}

template <size_t N>
std::array<uint32_t, N> readWideWord(const WData* src) {
  std::array<uint32_t, N> word{};
  for (size_t i = 0; i < N; ++i) {
    word[i] = src[i];
  }
  return word;
}

}  // namespace

class VerilatedRouter::Impl {
public:
  explicit Impl(VCRouter* owner) : owner_(owner), model_(&ctx_) {
    rr_vc_.fill(0);
    input_flow_ = {&model_.east_input_flow,
                   &model_.west_input_flow,
                   &model_.north_input_flow,
                   &model_.south_input_flow,
                   &model_.local_input_flow};
    input_data_ = {model_.east_input_data,
                   model_.west_input_data,
                   model_.north_input_data,
                   model_.south_input_data,
                   model_.local_input_data};
    output_data_ = {model_.east_output_data,
                    model_.west_output_data,
                    model_.north_output_data,
                    model_.south_output_data,
                    model_.local_output_data};
    output_flow_ = {&model_.east_output_flow,
                    &model_.west_output_flow,
                    &model_.north_output_flow,
                    &model_.south_output_flow,
                    &model_.local_output_flow};
    resetModel();
  }

  void runOneStep() {
    model_.X_cur = static_cast<unsigned char>(owner_->id[1]);  // RTL X = NoCDAS y (column)
    model_.Y_cur = static_cast<unsigned char>(owner_->id[0]);  // RTL Y = NoCDAS x (row)
    driveCnocTaskConfig();

    // Update NoCDAS state based on RTL outputs.
    drainPendingOutputs();

    // Drive RTL downstream flow control based on NoCDAS next-hop mirror state.
    for (int rtl_port = 0; rtl_port < RTL_PORTS; ++rtl_port) {
      *output_flow_[rtl_port] = downstreamFlowForOutput(RTL_TO_CPP_PORT[rtl_port]);
      writeWideWord(input_data_[rtl_port], PortDataWord{});
    }

    // Update combinational outputs (push_ack/in_accept/write_req, etc.) under current cycle settings.
    model_.clk = 0;
    model_.eval();

    // Check each port for accepted injections and prepare candidates for the next cycle.
    std::array<uint32_t, RTL_PORTS> push_ack{};
    for (int rtl_port = 0; rtl_port < RTL_PORTS; ++rtl_port) {
      push_ack[rtl_port] =
          static_cast<uint32_t>((*input_flow_[rtl_port] >> INPUT_FLOW_PUSH_ACK_LSB) & RTL_VC_MASK);
    }

    std::array<InjectCandidate, RTL_PORTS> cands;
    for (int off = 0; off < RTL_PORTS; ++off) {
      const int rtl_port = (rr_port_ + off) % RTL_PORTS;
      const int cpp_port = RTL_TO_CPP_PORT[rtl_port];
      if (cpp_port >= owner_->port_num) {
        continue;
      }
      InjectCandidate cand;
      if (!selectCandidate(rtl_port, cpp_port, push_ack[rtl_port], &cand)) {
        continue;
      }

      const uint32_t token = [&]() -> uint32_t {
        for (int tries = 0; tries < (1 << 24); ++tries) {
          const uint32_t candidate = next_token_++ & TOKEN_MASK_24;
          if (candidate != 0u && token_to_flit_.find(candidate) == token_to_flit_.end()) {
            return candidate;
          }
        }
        return 0u;
      }();
      if (token == 0u) {
        continue;
      }
      cand.token = token;
      {
        const Message& message = cand.flit->packet->message;
        if (message.type == CNOC_TYPE_DIST &&
            message.compute_op != CNOC_OPCODE_ATTENTION &&
            !owner_->assigned_tasks.empty()) {
          const bool owner_is_destination =
              cand.flit->packet->destination[0] == owner_->id[0] &&
              cand.flit->packet->destination[1] == owner_->id[1];
          const int this_router_id = owner_->id[0] * X_NUM + owner_->id[1];
          const std::vector<int>& path = message.routing_path;
          const bool owner_is_in_source_route =
              std::find(path.begin(), path.end(), this_router_id) != path.end();
          if (owner_is_destination || owner_is_in_source_route) {
            const int row_size =
                static_cast<int>(message.data.size()) /
                static_cast<int>(owner_->assigned_tasks.size());
            if (row_size > 0) {
              cnoc_weight_row_size_ =
                  static_cast<uint16_t>(std::clamp(row_size, 0, 0xffff));
            }
          }
        }
      }
      cand.payload = encodePayload(cand.flit);
      cand.meta = encodeMeta(cand.flit, cand.token);
      cand.route_path = encodeRoutePath(cand.flit);
      PortDataWord packed{};
      setField(packed, PORT_DATA_VC_ID_LSB, PORT_DATA_VC_ID_W, static_cast<uint32_t>(cand.src_vc));
      setBit(packed, PORT_DATA_VALID_BIT, true);
      copyBits(packed, PORT_DATA_ROUTE_PATH_LSB, cand.route_path, 0, RTL_ROUTE_PATH_BITS);
      copyBits(packed, PORT_DATA_META_LSB, cand.meta, 0, RTL_META_BITS);
      copyBits(packed, PORT_DATA_FLIT_LSB, cand.payload, 0, RTL_FLIT_BITS);
      writeWideWord(input_data_[rtl_port], packed);
      cands[rtl_port] = cand;
    }

    driveCnocTaskConfig();

    rr_port_ = (rr_port_ + 1) % RTL_PORTS;

    model_.clk = 0;
    model_.eval();
    bool cleared_rejected = false;
    for (int rtl_port = 0; rtl_port < RTL_PORTS; ++rtl_port) {
      if (!cands[rtl_port].valid) {
        continue;
      }
      if (((*input_flow_[rtl_port] >> INPUT_FLOW_IN_ACCEPT_BIT) & 0x1u) == 0u) {
        writeWideWord(input_data_[rtl_port], PortDataWord{});
        cands[rtl_port].valid = false;
        cleared_rejected = true;
      }
    }
    if (cleared_rejected) {
      model_.clk = 0;
      model_.eval();
    }
    for (const InjectCandidate& cand : cands) {
      if (cand.valid) {
        commitAcceptedInjection(cand);
      }
    }

    stepClockAndConsumeOutputs();
    drainPendingOutputs();
  }

  // Required: NoCDAS layer/chunk reset 时同步清 RTL storage 和 wrapper transient state。
  void resetCnocState() {
    pending_outputs_.clear();
    token_to_flit_.clear();
    next_token_ = 1;
    rr_vc_.fill(0);
    rr_port_ = 0;
    cnoc_weight_row_size_ = 0;
    resetModel();
  }

  bool ownsCnocMfu() const {
#if ENABLE_CNOC_MFU
    return true;
#else
    return false;
#endif
  }

  // Required: MACnet phase gating reads RTL-owned weight storage progress.
  unsigned int cnocWeightBytesStored() const {
#if ENABLE_CNOC_MFU
    return static_cast<unsigned int>(model_.cnoc_weight_bytes_stored);
#else
    return 0u;
#endif
  }

  // Required: MACnet phase gating reads RTL-owned KV storage progress.
  unsigned int cnocKvBytesStored() const {
#if ENABLE_CNOC_MFU
    return static_cast<unsigned int>(model_.cnoc_kv_bytes_stored);
#else
    return 0u;
#endif
  }

  // Required: MACnet phase gating reads RTL-owned KV token progress.
  unsigned int cnocKvTokenCount() const {
#if ENABLE_CNOC_MFU
    return static_cast<unsigned int>(model_.cnoc_kv_token_count);
#else
    return 0u;
#endif
  }

  // Required: MACnet phase gating observes type4 store events.
  bool cnocLastType4StoreValid() const {
#if ENABLE_CNOC_MFU
    return model_.cnoc_last_type4_store_valid != 0;
#else
    return false;
#endif
  }

private:
  VCRouter* owner_;
  VerilatedContext ctx_;
  VNoCRouter model_;
  std::array<SData*, RTL_PORTS> input_flow_{};
  std::array<WData*, RTL_PORTS> input_data_{};
  std::array<const WData*, RTL_PORTS> output_data_{};
  std::array<IData*, RTL_PORTS> output_flow_{};
  std::array<int, RTL_PORTS> rr_vc_;
  std::deque<PendingOutput> pending_outputs_;
  std::unordered_map<uint32_t, TokenRecord> token_to_flit_;
  uint32_t next_token_ = 1;
  int rr_port_ = 0;
  uint16_t cnoc_weight_row_size_ = 0;

  // Required: converts NoCDAS assigned tasks and weight row size into RTL config inputs.
  void driveCnocTaskConfig() {
#if ENABLE_CNOC_MFU
    const int task_count =
        std::min(static_cast<int>(owner_->assigned_tasks.size()), RTL_CNOC_MAX_TASKS);
    model_.cnoc_task_count = static_cast<unsigned char>(task_count);
    model_.cnoc_weight_row_size = cnoc_weight_row_size_;

    TaskIdsWord packed_task_ids{};
    for (int slot = 0; slot < task_count; ++slot) {
      const uint32_t task_id =
          static_cast<uint32_t>(std::clamp(owner_->assigned_tasks[slot], 0, 0xffff));
      setField(packed_task_ids, slot * RTL_CNOC_TASK_ID_BITS, RTL_CNOC_TASK_ID_BITS, task_id);
    }
    writeWideWord(model_.cnoc_task_ids_flat, packed_task_ids);
#endif
  }

  // Required: reset sequence must put the Verilated router in a reproducible idle state.
  void resetModel() {
    model_.clk = 0;
    model_.reset = 1;
    model_.X_cur = static_cast<unsigned char>(owner_->id[1]);
    model_.Y_cur = static_cast<unsigned char>(owner_->id[0]);
    driveCnocTaskConfig();

    for (int p = 0; p < RTL_PORTS; ++p) {
      writeWideWord(input_data_[p], PortDataWord{});
      *output_flow_[p] = ((0u & RTL_VC_MASK) << OUTPUT_FLOW_CREDIT_MASK_LSB) |
                         ((0u & RTL_VC_MASK) << OUTPUT_FLOW_IDLE_MASK_LSB) |
                         (1u << OUTPUT_FLOW_STATE_READY_BIT);
    }

    model_.eval();
    model_.clk = 1;
    model_.eval();
    model_.clk = 0;
    model_.eval();

    model_.reset = 0;
    model_.eval();
  }

  // Required: accepted C++ mirror flits are dequeued only after RTL input acceptance.
  void commitAcceptedInjection(const InjectCandidate& cand) {
    assert(cand.valid);
    assert(cand.src_buf != nullptr);
    assert(cand.flit != nullptr);

    Flit* popped = cand.src_buf->dequeue();
    assert(popped == cand.flit);

    cand.flit->sched_time = cycles;

    RInPort* src_in = (cand.cpp_port >= 0 && cand.cpp_port < owner_->port_num)
                          ? owner_->in_port_list[cand.cpp_port]
                          : nullptr;
    const bool head_like = (cand.flit->type == 0 || cand.flit->type == 10);
    const bool tail_like = (cand.flit->type == 1 || cand.flit->type == 10);
    if (src_in != nullptr &&
        cand.src_vc >= 0 &&
        cand.src_vc < static_cast<int>(src_in->state.size()) &&
        head_like &&
        !tail_like) {
      src_in->state[cand.src_vc] = 3;
      if (cand.src_vc < static_cast<int>(src_in->out_vc.size())) {
        src_in->out_vc[cand.src_vc] = -1;
      }
    }

    token_to_flit_[cand.token] = TokenRecord{
        cand.flit,
        cand.cpp_port,
        cand.src_vc};
  }

  // Required: chooses one ready NoCDAS flit per RTL input port without doing route/VC decisions.
  bool selectCandidate(int rtl_port, int cpp_port, uint32_t push_ack_mask, InjectCandidate* out) {
    out->valid = false;

    RInPort* in_port = owner_->in_port_list[cpp_port];
    const int vc_count = static_cast<int>(in_port->buffer_list.size());
    if (vc_count <= 0) {
      return false;
    }

    int start = rr_vc_[rtl_port];
    for (int k = 0; k < vc_count; ++k) {
      int vc_idx = (start + k) % vc_count;
      if (vc_idx >= RTL_VC_NUM) {
        continue;
      }
      if (((push_ack_mask >> vc_idx) & 0x1u) == 0u) {
        continue;
      }

      FlitBuffer* buf = in_port->buffer_list[vc_idx];
      if (buf == nullptr || buf->cur_flit_num == 0) {
        continue;
      }

      Flit* cand = buf->read();
      if (cand == nullptr || cand->packet == nullptr) {
        continue;
      }
      if (cand->sched_time >= cycles) {
        continue;
      }
      const bool head_like = (cand->type == 0 || cand->type == 10);
      const int required_state = head_like ? 2 : 3;
      if (vc_idx >= static_cast<int>(in_port->state.size()) ||
          in_port->state[vc_idx] != required_state) {
        continue;
      }
      if (!head_like &&
          (vc_idx >= static_cast<int>(in_port->out_vc.size()) || in_port->out_vc[vc_idx] < 0)) {
        continue;
      }

      rr_vc_[rtl_port] = (vc_idx + 1) % vc_count;
      out->valid = true;
      out->cpp_port = cpp_port;
      out->src_vc = vc_idx;
      out->flit = cand;
      out->src_buf = buf;
      return true;
    }
    return false;
  }

  // Step the clock and pack RTL outputs into NoCDAS state updates
  void stepClockAndConsumeOutputs() {
    model_.clk = 0;
    model_.eval();
    for (int port = 0; port < RTL_PORTS; ++port) {
      consumeOutput(port, readWideWord<RTL_PORT_DATA_WORDS>(output_data_[port]));
    }
    model_.clk = 1;
    model_.eval();

    model_.clk = 0;
    model_.eval();
  }

  // Generate downstream flow control signals for VC allocation in the RTL router.
  uint32_t downstreamFlowForOutput(int cpp_out_port) {
    if (cpp_out_port >= owner_->port_num ||
        owner_->out_port_list[cpp_out_port]->out_link == nullptr) {
      return (0u << OUTPUT_FLOW_CREDIT_MASK_LSB) |
             (0u << OUTPUT_FLOW_IDLE_MASK_LSB) |
             (0u << OUTPUT_FLOW_STATE_READY_BIT);
    }

    RInPort* next = owner_->out_port_list[cpp_out_port]->out_link->rInPort;
    if (next == nullptr) {
      return (0u << OUTPUT_FLOW_CREDIT_MASK_LSB) |
             (0u << OUTPUT_FLOW_IDLE_MASK_LSB) |
             (0u << OUTPUT_FLOW_STATE_READY_BIT);
    }
    uint32_t idle_mask = 0u;
    uint32_t credit_mask = 0u;
    const int total_vc = static_cast<int>(next->buffer_list.size());
    for (int vc = 0; vc < total_vc && vc < RTL_VC_NUM; ++vc) {
      if (vc < static_cast<int>(next->state.size()) && next->state[vc] == 0) {
        idle_mask |= (1u << vc);
      }
      if (next->buffer_list[vc] != nullptr && !next->buffer_list[vc]->isFull()) {
        credit_mask |= (1u << vc);
      }
    }
    return ((credit_mask & RTL_VC_MASK) << OUTPUT_FLOW_CREDIT_MASK_LSB) |
           ((idle_mask & RTL_VC_MASK) << OUTPUT_FLOW_IDLE_MASK_LSB) |
           (1u << OUTPUT_FLOW_STATE_READY_BIT);
  }

  // Pack RTL cNoC payload fields back into the NoCDAS message for type5 compute packets
  void mirrorRtlCnocPayload(Flit* flit, const MetaWord& meta, const FlitWord& payload) {
#if CNOC_QUANT_GOLDEN
    if (flit == nullptr ||
        flit->packet == nullptr) {
      return;
    }

    const int msg_type = static_cast<int>(getField(meta, META_MSG_TYPE_LSB, 3));
    const int opcode = static_cast<int>(getField(meta, META_OPCODE_LSB, 5));
    if (msg_type != CNOC_TYPE_COMP) {
      return;
    }

    Message& message = flit->packet->message;
    if (message.cnoc_qdata.size() != message.data.size()) {
      CNoCQuant::quantizeVector(message.data, message.cnoc_qdata);
    }

    const int payload_size =
        std::clamp(flit->get_payload_size(), 0, FLIT_LENGTH / CNOC_QUANT_DATA_BYTES);
    const int base_idx = flit->global_data_offset;
    for (int lane = 0; lane < payload_size; ++lane) {
      const int qdata_idx = base_idx + lane;
      if (qdata_idx < 0 || qdata_idx >= static_cast<int>(message.cnoc_qdata.size())) {
        continue;
      }
      const uint32_t raw = getField(payload, lane * 8, 8);
      const int32_t qvalue = static_cast<int32_t>(static_cast<int8_t>(raw & 0xffu));
      message.cnoc_qdata[qdata_idx] = qvalue;
      if (qdata_idx < static_cast<int>(message.data.size())) {
        message.data[qdata_idx] = CNoCQuant::dequantize(qvalue);
      }
    }
    if (opcode == CNOC_OPCODE_ATTENTION) {
      const uint32_t flit_kind = getField(meta, META_FLIT_TYPE_LSB, META_FLIT_TYPE_W);
      const bool tail_like = (flit_kind == 1u || flit_kind == 10u);
      // Attention running state is committed at the tail control point.  Head
      // and body flits carry stale sideband values while the wormhole packet is
      // still streaming; mirroring those values would erase the previous
      // router's online-softmax update before the tail reaches the next router.
      if (tail_like) {
        const uint32_t rm_raw = getField(meta, META_ATTENTION_RM_LSB, META_ATTENTION_STATE_W);
        const uint32_t rs_raw = getField(meta, META_ATTENTION_RS_LSB, META_ATTENTION_STATE_W);
        const int32_t rm_q = static_cast<int32_t>(static_cast<int8_t>(rm_raw & 0xffu));
        const int32_t rs_q = static_cast<int32_t>(static_cast<int8_t>(rs_raw & 0xffu));
        message.running_max = CNoCQuant::dequantize(rm_q);
        message.running_sum = CNoCQuant::dequantize(rs_q);
      }
    }
#else
    (void)flit;
    (void)meta;
    (void)payload;
#endif
  }

  // Pack RTL output port data into NoCDAS state updates
  void consumeOutput(int rtl_out_port, const PortDataWord& out_data) {
    const bool write_req = getBit(out_data, PORT_DATA_VALID_BIT);
    if (!write_req) {
      return;
    }
    FlitWord payload{};
    MetaWord meta{};
    RoutePathWord route_path{};
    copyBits(payload, 0, out_data, PORT_DATA_FLIT_LSB, RTL_FLIT_BITS);
    copyBits(meta, 0, out_data, PORT_DATA_META_LSB, RTL_META_BITS);
    copyBits(route_path, 0, out_data, PORT_DATA_ROUTE_PATH_LSB, RTL_ROUTE_PATH_BITS);
    const uint32_t vc_id = getField(out_data, PORT_DATA_VC_ID_LSB, PORT_DATA_VC_ID_W);

    if (!getBit(meta, META_VALID_BIT)) {
      return;
    }
    const uint32_t token = getField(meta, META_COSIM_TOKEN_LSB, 24) & TOKEN_MASK_24;
    if (token == 0u) {
      return;
    }

    auto it = token_to_flit_.find(token);
    if (it == token_to_flit_.end()) {
      return;
    }

    TokenRecord rec = it->second;
#if ENABLE_CNOC_MFU
    mirrorRtlCnocPayload(rec.flit, meta, payload);
#else
    applyCppCnocSideEffects(rec.flit, rtl_out_port);
#endif
    const int cpp_out_port = RTL_TO_CPP_PORT[rtl_out_port];
    if (getBit(route_path, ROUTE_PATH_VALID_BIT) && rec.flit != nullptr) {
      const uint32_t route_len = getField(route_path, ROUTE_PATH_LEN_LSB, ROUTE_PATH_LEN_W);
      const uint32_t route_ptr = getField(route_path, ROUTE_PATH_PTR_LSB, ROUTE_PATH_PTR_W);
      rec.flit->rtl_route_ptr = static_cast<int>(std::min(route_ptr, route_len));
    }
    pending_outputs_.push_back(PendingOutput{
        rec.flit,
        cpp_out_port,
        static_cast<int>(vc_id),
        rec.src_cpp_port,
        rec.src_vc});
    if (rec.src_cpp_port >= 0 && rec.src_cpp_port < owner_->port_num) {
      RInPort* src_in = owner_->in_port_list[rec.src_cpp_port];
      if (src_in != nullptr &&
          rec.src_vc >= 0 &&
          rec.src_vc < static_cast<int>(src_in->state.size()) &&
          rec.flit != nullptr) {
        const bool head_like = (rec.flit->type == 0 || rec.flit->type == 10);
        const bool tail_like = (rec.flit->type == 1 || rec.flit->type == 10);
        if (head_like && !tail_like) {
          if (rec.src_vc < static_cast<int>(src_in->out_vc.size())) {
            src_in->out_vc[rec.src_vc] = static_cast<int>(vc_id);
          }
        } else if (tail_like) {
          src_in->state[rec.src_vc] = 0;
          if (rec.src_vc < static_cast<int>(src_in->out_vc.size())) {
            src_in->out_vc[rec.src_vc] = -1;
          }
        }
      }
    }
    token_to_flit_.erase(it);
  }

  void applyCppCnocSideEffects(Flit* flit, int rtl_out_port) {
    if (flit == nullptr || flit->packet == nullptr) {
      return;
    }

    const int msg_type = flit->packet->message.type;
    const int cpp_out_port = RTL_TO_CPP_PORT[rtl_out_port];
    if (msg_type == CNOC_TYPE_DIST) {
      const int this_router = owner_->id[0] * X_NUM + owner_->id[1];
      bool is_target = false;
      for (int router_id : flit->packet->message.routing_path) {
        if (router_id == this_router) {
          is_target = true;
        }
      }
      if (flit->packet->destination[0] == owner_->id[0] &&
          flit->packet->destination[1] == owner_->id[1]) {
        is_target = true;
      }
      if (is_target) {
        owner_->processDistributionPacket(flit);
      }
    } else if (msg_type == CNOC_TYPE_COMP) {
      owner_->computeInTransit(flit, cpp_out_port);
    }
  }

  // Moves a pending output flit into the next hop if the target C++ RInPort state allows it.
  bool enqueueToNextHop(const PendingOutput& pending) {
    if (pending.cpp_out_port >= owner_->port_num ||
        owner_->out_port_list[pending.cpp_out_port]->out_link == nullptr) {
      return false;
    }
    RInPort* next = owner_->out_port_list[pending.cpp_out_port]->out_link->rInPort;
    // Output may point off-chip/border in malformed cases; keep the flit pending.
    if (next == nullptr) {
      return false;
    }
    const int total_vc = static_cast<int>(next->buffer_list.size());
    if (pending.target_vc < 0 || pending.target_vc >= total_vc) {
      return false;
    }
    if (next->buffer_list[pending.target_vc]->isFull()) {
      return false;
    }

    // Enqueue the flit into the next hop and update C++ mirror state accordingly.
    Flit* flit = pending.flit;
    const bool head_like = (flit->type == 0 || flit->type == 10);
    const bool tail_like = (flit->type == 1 || flit->type == 10);

    if (head_like) {
      if (pending.target_vc >= static_cast<int>(next->state.size()) ||
          next->state[pending.target_vc] != 0) {
        return false;
      }
    }

    flit->vc = pending.target_vc;
    next->buffer_list[pending.target_vc]->get_credit();
    next->buffer_list[pending.target_vc]->enqueue(flit);

    flit->sched_time = cycles + LINK_TIME - 1;
    flit->trace_node.push_back(next->rid[0] * X_NUM + next->rid[1]);
    flit->trace_time.push_back(cycles + LINK_TIME - 1);

    // Head flits claim the target VC and tail flits release the source VC; body flits do neither.
    if (head_like) {
      if (pending.target_vc < static_cast<int>(next->out_port.size())) {
        next->out_port[pending.target_vc] = -1;
      }
      next->state[pending.target_vc] = 2;
    }

    // Successfully enqueued to the next hop; update utilization stats and remove from pending.
    owner_->port_total_utilization++;
    if (pending.cpp_out_port <= 3) {
      owner_->port_utilization_innet++;
    }
    return true;
  }

  // Checks pending output flits for next-hop readiness and enqueues them if possible.
  // Preserve ordering within each source VC, but allow unrelated source VCs to
  // pass a blocked new head so older tails can release their destination VCs.
  void drainPendingOutputs() {
    std::array<std::array<bool, RTL_VC_NUM>, RTL_PORTS> blocked_source_vc{};
    for (auto it = pending_outputs_.begin(); it != pending_outputs_.end();) {
      if (it->src_cpp_port >= 0 &&
          it->src_cpp_port < RTL_PORTS &&
          it->src_vc >= 0 &&
          it->src_vc < RTL_VC_NUM &&
          blocked_source_vc[it->src_cpp_port][it->src_vc]) {
        ++it;
        continue;
      }
      if (enqueueToNextHop(*it)) {
        it = pending_outputs_.erase(it);
      } else {
        if (it->src_cpp_port >= 0 &&
            it->src_cpp_port < RTL_PORTS &&
            it->src_vc >= 0 &&
            it->src_vc < RTL_VC_NUM) {
          blocked_source_vc[it->src_cpp_port][it->src_vc] = true;
        }
        ++it;
      }
    }
  }

  // Translate NoCDAS source-route path into RTL head flit route sequence + process mask.
  void ensureSourceRoute(Flit* flit) const {
    if (flit == nullptr ||
        flit->packet == nullptr ||
        flit->rtl_route_initialized ||
        (flit->packet->message.type != CNOC_TYPE_DIST &&
         flit->packet->message.type != CNOC_TYPE_COMP)) {
      return;
    }

    flit->rtl_route_initialized = true;
    flit->rtl_route_ptr = 0;
    flit->rtl_route_ports.clear();
    flit->rtl_route_process.clear();

    const std::vector<int>& path = flit->packet->message.routing_path;
    if (path.empty()) {
      return;
    }

    int row = owner_->id[0];
    int col = owner_->id[1];
    const int msg_type = flit->packet->message.type;
    std::bitset<TOT_NUM> processed_router;
    auto append_hop = [&](uint8_t rtl_port) -> bool {
      if (flit->rtl_route_ports.size() >= RTL_ROUTE_MAX_HOPS) {
        return false;
      }
      const int current_router = row * X_NUM + col;
      const bool process_here =
          msg_type == CNOC_TYPE_COMP &&
          current_router >= 0 &&
          current_router < TOT_NUM &&
          !processed_router.test(static_cast<size_t>(current_router));
      if (process_here) {
        processed_router.set(static_cast<size_t>(current_router));
      }
      flit->rtl_route_ports.push_back(rtl_port);
      flit->rtl_route_process.push_back(process_here ? 1u : 0u);
      if (rtl_port < RTL_PORTS) {
        row += RTL_PORT_ROW_DELTA[rtl_port];
        col += RTL_PORT_COL_DELTA[rtl_port];
      }
      return true;
    };

    for (int target_router : path) {
      if (flit->rtl_route_ports.size() >= RTL_ROUTE_MAX_HOPS) {
        break;
      }
      while (flit->rtl_route_ports.size() < RTL_ROUTE_MAX_HOPS &&
             target_router >= 0 &&
             target_router < TOT_NUM &&
             (row * X_NUM + col) != target_router) {
        const int target_row = target_router / X_NUM;
        const int target_col = target_router % X_NUM;
        const uint8_t next_port =
            (col < target_col) ? RTL_PORT_EAST :
            (col > target_col) ? RTL_PORT_WEST :
            (row < target_row) ? RTL_PORT_SOUTH :
            (row > target_row) ? RTL_PORT_NORTH : RTL_PORT_LOCAL;
        if (next_port == RTL_PORT_LOCAL ||
            !append_hop(next_port)) {
          break;
        }
      }
    }

    const int dest_router =
        flit->packet->destination[0] * X_NUM + flit->packet->destination[1];
    if (flit->rtl_route_ports.size() < RTL_ROUTE_MAX_HOPS &&
        (row * X_NUM + col) == dest_router) {
      (void)append_hop(RTL_PORT_LOCAL);
    } else {
      while (flit->rtl_route_ports.size() < RTL_ROUTE_MAX_HOPS &&
             dest_router >= 0 &&
             dest_router < TOT_NUM &&
             (row * X_NUM + col) != dest_router) {
        const int target_row = dest_router / X_NUM;
        const int target_col = dest_router % X_NUM;
        const uint8_t next_port =
            (col < target_col) ? RTL_PORT_EAST :
            (col > target_col) ? RTL_PORT_WEST :
            (row < target_row) ? RTL_PORT_SOUTH :
            (row > target_row) ? RTL_PORT_NORTH : RTL_PORT_LOCAL;
        if (next_port == RTL_PORT_LOCAL ||
            !append_hop(next_port)) {
          break;
        }
      }
      if (flit->rtl_route_ports.size() < RTL_ROUTE_MAX_HOPS &&
          (row * X_NUM + col) == dest_router) {
        (void)append_hop(RTL_PORT_LOCAL);
      }
    }
  }

  // Encode NoCDAS flit payload into RTL cNoC payload fields for type5 compute packets
  FlitWord encodePayload(const Flit* flit) const {
    FlitWord payload{};
    if (flit == nullptr || flit->packet == nullptr) {
      return payload;
    }

    const int start_idx = (flit->global_data_offset >= 0) ? flit->global_data_offset : 0;
    // The payload is quantized in the C++ model for golden comparison
#if CNOC_QUANT_GOLDEN
    if (flit != nullptr &&
        flit->packet != nullptr &&
        (flit->packet->message.type == CNOC_TYPE_DIST ||
         flit->packet->message.type == CNOC_TYPE_COMP) &&
        !flit->packet->message.cnoc_qdata.empty()) {
      const std::vector<int32_t>& qdata = flit->packet->message.cnoc_qdata;
      const int elems_per_flit = FLIT_LENGTH / CNOC_QUANT_DATA_BYTES;
      for (int i = 0; i < elems_per_flit; ++i) {
        const int idx = start_idx + i;
        if (idx < 0 || idx >= static_cast<int>(qdata.size())) {
          break;
        }
        const int32_t q = CNoCQuant::sat8(qdata[idx]);
        setField(payload, i * 8, 8, static_cast<uint32_t>(static_cast<uint8_t>(q)));
      }
      return payload;
    }
#endif
    const std::vector<float>& data = flit->packet->message.data;
#if DATA_BYTES == 1
    const int elems_per_flit = FLIT_LENGTH / DATA_BYTES;
    for (int i = 0; i < elems_per_flit; ++i) {
      const int idx = start_idx + i;
      if (idx < 0 || idx >= static_cast<int>(data.size())) {
        break;
      }
      const int8_t q =
          static_cast<int8_t>(std::clamp(static_cast<int>(std::lround(data[idx])), -128, 127));
      setField(payload, i * 8, 8, static_cast<uint32_t>(static_cast<uint8_t>(q)));
    }
#else
    const int elems_per_flit = FLIT_LENGTH / DATA_BYTES;
    for (int i = 0; i < elems_per_flit; ++i) {
      const int idx = start_idx + i;
      if (idx < 0 || idx >= static_cast<int>(data.size())) {
        break;
      }
      uint32_t bits = 0u;
      float v = data[idx];
      std::memcpy(&bits, &v, sizeof(uint32_t));
      setField(payload, i * 32, 32, bits);
    }
#endif
    return payload;
  }

  RoutePathWord encodeRoutePath(Flit* flit) const {
    RoutePathWord route_path{};
    if (flit == nullptr || flit->packet == nullptr) {
      return route_path;
    }

    const bool head_like = (flit->type == 0 || flit->type == 10);
    if (!head_like ||
        (flit->packet->message.type != CNOC_TYPE_DIST &&
         flit->packet->message.type != CNOC_TYPE_COMP)) {
      return route_path;
    }

    ensureSourceRoute(flit);
    if (flit->rtl_route_ports.empty()) {
      return route_path;
    }

    const int route_len =
        std::min(static_cast<int>(flit->rtl_route_ports.size()), RTL_ROUTE_MAX_HOPS);
    const int route_ptr = std::clamp(flit->rtl_route_ptr, 0, route_len);
    setBit(route_path, ROUTE_PATH_VALID_BIT, true);
    setField(route_path, ROUTE_PATH_LEN_LSB, ROUTE_PATH_LEN_W, static_cast<uint32_t>(route_len));
    setField(route_path, ROUTE_PATH_PTR_LSB, ROUTE_PATH_PTR_W, static_cast<uint32_t>(route_ptr));
    for (int hop = 0; hop < route_len; ++hop) {
      setField(route_path,
               ROUTE_PATH_SEQ_LSB + hop * RTL_ROUTE_PORT_BITS,
               RTL_ROUTE_PORT_BITS,
               static_cast<uint32_t>(flit->rtl_route_ports[hop] & 0x7u));
      if (hop < static_cast<int>(flit->rtl_route_process.size()) &&
          flit->rtl_route_process[hop] != 0u) {
        setBit(route_path, ROUTE_PATH_PROCESS_SEQ_LSB + hop, true);
      }
    }
    return route_path;
  }

  // Encode metadata fields from NoCDAS flit and message into RTL input ports.
  MetaWord encodeMeta(Flit* flit, uint32_t token24) const {
    MetaWord meta{};
    if (flit == nullptr || flit->packet == nullptr) {
      return meta;
    }

    const int dest_x = std::clamp(flit->packet->destination[1], 0, std::min(std::max(X_NUM - 1, 0), RTL_COORD_MAX));
    const int dest_y = std::clamp(flit->packet->destination[0], 0, std::min(std::max(Y_NUM - 1, 0), RTL_COORD_MAX));

    const int msg_type = flit->packet->message.type & 0x7;
    const int opcode = flit->packet->message.compute_op & 0x1f;
    const uint8_t flit_type =
        (flit->type == 1) ? 1u :
        (flit->type == 2) ? 2u :
        (flit->type == 10) ? 10u : 0u;
    const bool head_like = (flit->type == 0 || flit->type == 10);
    const bool process_in_this_router = (msg_type == CNOC_TYPE_COMP);
    const uint32_t traffic_class =
        (msg_type == CNOC_TYPE_COMP) ? RTL_TRAFFIC_COMP :
        (msg_type == CNOC_TYPE_DIST) ? RTL_TRAFFIC_DIST :
                                      RTL_TRAFFIC_REGULAR;
    uint32_t cnoc_reserved = 0u;
    if (msg_type == CNOC_TYPE_COMP &&
        (opcode == CNOC_OPCODE_ADD || opcode == CNOC_OPCODE_SWIGLU || opcode == CNOC_OPCODE_GEGLU)) {
      const int payload_size =
          std::clamp(flit->get_payload_size(), 0, FLIT_LENGTH / CNOC_QUANT_DATA_BYTES);
      for (int lane = 0; lane + 1 < payload_size; lane += 2) {
        const int task_id = (flit->global_data_offset + lane) / 2;
        if (std::find(owner_->assigned_tasks.begin(),
                      owner_->assigned_tasks.end(),
                      task_id) != owner_->assigned_tasks.end()) {
          cnoc_reserved |= (1u << (lane / 2));
        }
      }
    }
    if (msg_type == CNOC_TYPE_COMP && opcode == CNOC_OPCODE_ATTENTION) {
#if CNOC_QUANT_GOLDEN
      const uint32_t rm_q = static_cast<uint32_t>(static_cast<uint8_t>(
          CNoCQuant::sat8(CNoCQuant::quantize(static_cast<float>(flit->packet->message.running_max)))));
      const uint32_t rs_q = static_cast<uint32_t>(static_cast<uint8_t>(
          CNoCQuant::sat8(CNoCQuant::quantize(static_cast<float>(flit->packet->message.running_sum)))));
      cnoc_reserved = rm_q | (rs_q << 8);
#endif
    }
    const uint32_t vnet =
        (msg_type == CNOC_TYPE_DIST || msg_type == CNOC_TYPE_COMP)
            ? 1u
            : static_cast<uint32_t>(std::max(flit->vnet, 0) & 0x3);

    const int data_idx =
        ((msg_type == CNOC_TYPE_DIST || msg_type == CNOC_TYPE_COMP) &&
         flit->global_data_offset >= 0 &&
         flit->global_data_offset < static_cast<int>(flit->packet->message.data.size()))
            ? (flit->global_data_offset & 0x3ff)
            : 0;

    setBit(meta, META_VALID_BIT, true);
    setField(meta, META_FLIT_TYPE_LSB, META_FLIT_TYPE_W, static_cast<uint32_t>(flit_type));
    setField(meta, META_MSG_TYPE_LSB, 3, static_cast<uint32_t>(msg_type));
    setField(meta, META_OPCODE_LSB, 5, static_cast<uint32_t>(opcode));
    setField(meta, META_COSIM_TOKEN_LSB, 24, token24 & TOKEN_MASK_24);
    setField(meta, META_DATA_IDX_LSB, 10, static_cast<uint32_t>(data_idx));
    setField(meta, META_DEST_Y_LSB, 3, static_cast<uint32_t>(dest_y));
    setField(meta, META_DEST_X_LSB, 3, static_cast<uint32_t>(dest_x));
    setBit(meta, META_PROCESS_BIT, process_in_this_router);
    setField(meta, META_VNET_LSB, META_VNET_W, vnet);
    setField(meta,
             META_PAYLOAD_LEN_LSB,
             META_PAYLOAD_LEN_W,
             static_cast<uint32_t>(std::clamp(flit->get_payload_size(), 0, FLIT_LENGTH)));
    setField(meta, META_TRAFFIC_CLASS_LSB, META_TRAFFIC_CLASS_W, traffic_class);
    setField(meta,
             META_PSUM_OFFSET_LSB,
             META_PSUM_OFFSET_W,
             static_cast<uint32_t>(std::clamp(flit->packet->message.psum_offset, 0, 0xffff)));
    setField(meta,
             META_K_DIM_LSB,
             META_K_DIM_W,
             static_cast<uint32_t>(std::clamp(flit->packet->message.k_dim, 0, 0xffff)));
    setField(meta,
             META_CNOC_PAIR_MASK_LSB,
             META_CNOC_PAIR_MASK_W,
             cnoc_reserved);
    setField(meta,
             META_COSIM_FLIT_ID_LSB,
             16,
             static_cast<uint32_t>(std::clamp(flit->id, 0, 0xffff)));
    setField(meta,
             META_COSIM_PACKET_UID_LSB,
             32,
             static_cast<uint32_t>(std::max(flit->packet->message.signal_id, 0)));

    if (head_like &&
        (flit->packet->message.type == CNOC_TYPE_DIST ||
         flit->packet->message.type == CNOC_TYPE_COMP)) {
      ensureSourceRoute(flit);
    }

    if (head_like &&
        (flit->packet->message.type == CNOC_TYPE_DIST ||
         flit->packet->message.type == CNOC_TYPE_COMP) &&
        !flit->rtl_route_ports.empty()) {
      const int route_len =
          std::min(static_cast<int>(flit->rtl_route_ports.size()), RTL_ROUTE_MAX_HOPS);
      const int route_ptr = std::clamp(flit->rtl_route_ptr, 0, route_len);
      setBit(meta, META_ROUTE_VALID_BIT, true);
      setField(meta, META_ROUTE_PTR_LSB, META_ROUTE_PTR_W, static_cast<uint32_t>(route_ptr));
    }

    return meta;
  }
};

VerilatedRouter::VerilatedRouter(VCRouter* owner) {
  impl_ = new Impl(owner);
}

VerilatedRouter::~VerilatedRouter() {
  delete impl_;
}

void VerilatedRouter::runOneStep() {
  impl_->runOneStep();
}

void VerilatedRouter::resetCnocState() {
  impl_->resetCnocState();
}

unsigned int VerilatedRouter::cnocWeightBytesStored() const {
  return impl_->cnocWeightBytesStored();
}

bool VerilatedRouter::ownsCnocMfu() const {
  return impl_->ownsCnocMfu();
}

unsigned int VerilatedRouter::cnocKvBytesStored() const {
  return impl_->cnocKvBytesStored();
}

unsigned int VerilatedRouter::cnocKvTokenCount() const {
  return impl_->cnocKvTokenCount();
}

bool VerilatedRouter::cnocLastType4StoreValid() const {
  return impl_->cnocLastType4StoreValid();
}
