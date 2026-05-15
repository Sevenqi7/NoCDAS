#ifndef CNOCQUANT_HPP_
#define CNOCQUANT_HPP_

#include "parameters.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace CNoCQuant {

constexpr int kFracBits = CNOC_QUANT_FRAC_BITS;
constexpr int kScale = CNOC_QUANT_SCALE;

inline int32_t sat8(int64_t value) {
  if (value > 127) {
    return 127;
  }
  if (value < -128) {
    return -128;
  }
  return static_cast<int32_t>(value);
}

inline int32_t quantize(float value) {
  return sat8(static_cast<int64_t>(std::lround(value * static_cast<float>(kScale))));
}

inline float dequantize(int32_t qvalue) {
  return static_cast<float>(qvalue) / static_cast<float>(kScale);
}

inline int32_t roundShift(int64_t value, int shift) {
  if (shift <= 0) {
    return static_cast<int32_t>(value);
  }
  const int64_t bias = int64_t{1} << (shift - 1);
  if (value >= 0) {
    return static_cast<int32_t>((value + bias) >> shift);
  }
  return -static_cast<int32_t>(((-value) + bias) >> shift);
}

inline int32_t mulQ4(int32_t lhs_q4, int32_t rhs_q4) {
  return roundShift(static_cast<int64_t>(lhs_q4) * static_cast<int64_t>(rhs_q4), kFracBits);
}

inline int32_t addSatQ4(int32_t lhs_q4, int32_t rhs_q4) {
  return sat8(static_cast<int64_t>(lhs_q4) + static_cast<int64_t>(rhs_q4));
}

inline int32_t siluQ4(int32_t gate_q4, int32_t up_q4) {
  const float gate = dequantize(gate_q4);
  const float up = dequantize(up_q4);
  const float silu = gate * (1.0f / (1.0f + std::exp(-gate)));
  return quantize(silu * up);
}

inline int32_t geluQ4(int32_t gate_q4, int32_t up_q4) {
  const float gate = dequantize(gate_q4);
  const float up = dequantize(up_q4);
  const float gelu = 0.5f * gate * (1.0f + std::erf(gate / 1.41421356f));
  return quantize(gelu * up);
}

inline int32_t expQ4(int32_t x_q4) {
  return quantize(std::exp(dequantize(x_q4)));
}

inline int32_t divQ4(int32_t num_q4, int32_t den_q4) {
  if (den_q4 == 0) {
    return 0;
  }
  return quantize(dequantize(num_q4) / dequantize(den_q4));
}

inline void quantizeVector(const std::vector<float>& data, std::vector<int32_t>& qdata) {
  qdata.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    qdata[i] = quantize(data[i]);
  }
}

inline uint64_t mixHash64(uint64_t hash, uint64_t value) {
  hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  return hash;
}

inline uint64_t hashIntVector(const std::vector<int32_t>& data) {
  uint64_t hash = 1469598103934665603ULL;
  hash = mixHash64(hash, static_cast<uint64_t>(data.size()));
  for (size_t i = 0; i < data.size(); ++i) {
    const uint64_t payload =
        (static_cast<uint64_t>(i) << 32) |
        static_cast<uint32_t>(data[i]);
    hash = mixHash64(hash, payload);
  }
  return hash;
}

inline std::string formatIntSample(const std::vector<int32_t>& data,
                                   size_t start,
                                   size_t sample_count) {
  if (data.empty()) {
    return "-";
  }

  start = std::min(start, data.size());
  const size_t end = std::min(data.size(), start + sample_count);
  std::ostringstream oss;
  oss << "[";
  for (size_t i = start; i < end; ++i) {
    if (i != start) {
      oss << "|";
    }
    oss << data[i];
  }
  oss << "]";
  return oss.str();
}

}  // namespace CNoCQuant

#endif  // CNOCQUANT_HPP_
