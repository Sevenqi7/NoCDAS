/*
 * TraceLogger.cpp
 *
 */

#include "TraceLogger.hpp"
#include "CNoCQuant.hpp"
#include "NoC/Packet.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace {

uint64_t mixHash64(uint64_t hash, uint64_t value) {
  hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  return hash;
}

uint64_t hashFloatVector(const std::vector<float>& data) {
  uint64_t hash = 1469598103934665603ULL;
  hash = mixHash64(hash, static_cast<uint64_t>(data.size()));
  for (size_t i = 0; i < data.size(); ++i) {
    uint32_t bits = 0;
    std::memcpy(&bits, &data[i], sizeof(bits));
    uint64_t payload = (static_cast<uint64_t>(i) << 32) | static_cast<uint64_t>(bits);
    hash = mixHash64(hash, payload);
  }
  return hash;
}

std::string formatHash(uint64_t value) {
  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << value;
  return oss.str();
}

std::string formatSample(const Packet& packet, size_t sample_count) {
  const std::vector<float>& data = packet.message.data;
  if (data.empty()) {
    return "-";
  }

  size_t start = 0;
  if (packet.message.psum_offset > 0 &&
      static_cast<size_t>(packet.message.psum_offset) < data.size()) {
    start = static_cast<size_t>(packet.message.psum_offset);
  }

  size_t end = std::min(data.size(), start + sample_count);
  std::ostringstream oss;
  oss << "[";
  for (size_t i = start; i < end; ++i) {
    if (i != start) {
      oss << "|";
    }
    oss << std::setprecision(6) << std::fixed << data[i];
  }
  oss << "]";
  return oss.str();
}

std::string formatQuantSample(const Packet& packet, size_t sample_count) {
  const std::vector<int32_t>& data = packet.message.cnoc_qdata;
  if (data.empty()) {
    return "-";
  }

  size_t start = 0;
  if (packet.message.psum_offset > 0 &&
      static_cast<size_t>(packet.message.psum_offset) < data.size()) {
    start = static_cast<size_t>(packet.message.psum_offset);
  }

  return CNoCQuant::formatIntSample(data, start, sample_count);
}

}  // namespace

TraceLogger::TraceLogger() : enabled(false) {}

TraceLogger::~TraceLogger() {
  close();
}

TraceLogger& TraceLogger::instance() {
  static TraceLogger logger;
  return logger;
}

void TraceLogger::enable(const std::string& path) {
  out_file.open(path, std::ios::out);
  if (!out_file.is_open()) {
    enabled = false;
    return;
  }

  enabled = true;
  out_file << "# packet-level routing trace\n";
#if CNOC_QUANT_GOLDEN
  out_file << "# cnoc_quant_golden: int8_q4_4 integer_hash_sample\n";
#endif
  out_file << "# fields: global_pid,msg_type,signal_id,source_id,destination,path_nodes,path_times,compute_op,cnoc_in_hash,cnoc_out_hash,cnoc_rm,cnoc_rs,cnoc_sample\n";
}

bool TraceLogger::isEnabled() const {
  return enabled;
}

void TraceLogger::logPacket(const Packet& packet) {
  if (!enabled) {
    return;
  }

  std::ostringstream node_stream;
  for (size_t i = 0; i < packet.trace_nodes.size(); ++i) {
    if (i != 0) {
      node_stream << ">";
    }
    node_stream << packet.trace_nodes[i];
  }

  std::ostringstream time_stream;
  for (size_t i = 0; i < packet.trace_times.size(); ++i) {
    if (i != 0) {
      time_stream << ">";
    }
    time_stream << packet.trace_times[i];
  }

  std::string compute_op = "-";
  std::string in_hash = "-";
  std::string out_hash = "-";
  std::string running_max = "-";
  std::string running_sum = "-";
  std::string cnoc_sample = "-";

  if (packet.message.type == 4 || packet.message.type == 5) {
    compute_op = std::to_string(packet.message.compute_op);
    in_hash = formatHash(packet.cnoc_input_hash);
#if CNOC_QUANT_GOLDEN
    out_hash = formatHash(CNoCQuant::hashIntVector(packet.message.cnoc_qdata));
#else
    out_hash = formatHash(hashFloatVector(packet.message.data));
#endif

    if (packet.message.type == 5) {
      std::ostringstream rm_oss;
      rm_oss << std::setprecision(6) << std::fixed << packet.message.running_max;
      running_max = rm_oss.str();

      std::ostringstream rs_oss;
      rs_oss << std::setprecision(6) << std::fixed << packet.message.running_sum;
      running_sum = rs_oss.str();

#if CNOC_QUANT_GOLDEN
      cnoc_sample = formatQuantSample(packet, 8);
#else
      cnoc_sample = formatSample(packet, 8);
#endif
    } else {
#if CNOC_QUANT_GOLDEN
      cnoc_sample = formatQuantSample(packet, 8);
#else
      std::ostringstream dist_oss;
      dist_oss << "dist_len=" << packet.message.data.size()
               << ";path_len=" << packet.message.routing_path.size();
      cnoc_sample = dist_oss.str();
#endif
    }
  }

  out_file
      << packet.global_pid << ","
      << packet.message.type << ","
      << packet.message.signal_id << ","
      << packet.message.source_id << ","
      << packet.message.destination << ","
      << node_stream.str() << ","
      << time_stream.str() << ","
      << compute_op << ","
      << in_hash << ","
      << out_hash << ","
      << running_max << ","
      << running_sum << ","
      << cnoc_sample
      << "\n";
}

void TraceLogger::close() {
  if (out_file.is_open()) {
    out_file.close();
  }
  enabled = false;
}
