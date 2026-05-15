#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../CNoCQuant.hpp"

inline int clamp_i8(int64_t v) {
  return static_cast<int>(CNoCQuant::sat8(v));
}

extern "C" int dpi_silu(int x_q) {
  const float x = CNoCQuant::dequantize(x_q);
  const float s = 1.0f / (1.0f + std::exp(-x));
  return clamp_i8(CNoCQuant::quantize(x * s));
}

extern "C" int dpi_gelu(int x_q) {
  const float x = CNoCQuant::dequantize(x_q);
  const float k = 0.7978845608f;  // sqrt(2/pi)
  const float t = k * (x + 0.044715f * x * x * x);
  const float y = 0.5f * x * (1.0f + std::tanh(t));
  return clamp_i8(CNoCQuant::quantize(y));
}

extern "C" int dpi_swiglu(int gate_q, int up_q) {
  return clamp_i8(CNoCQuant::siluQ4(gate_q, up_q));
}

extern "C" int dpi_geglu(int gate_q, int up_q) {
  return clamp_i8(CNoCQuant::geluQ4(gate_q, up_q));
}

extern "C" int dpi_exp(int x_q) {
  const float x = CNoCQuant::dequantize(x_q);
  return clamp_i8(CNoCQuant::quantize(std::exp(x)));
}

extern "C" int dpi_div(int num_q, int den_q) {
  if (den_q == 0) {
    return 0;
  }
  const float y = CNoCQuant::dequantize(num_q) / CNoCQuant::dequantize(den_q);
  return clamp_i8(CNoCQuant::quantize(y));
}

extern "C" int dpi_attention_mix(int q_q, int kv_q) {
  // Functional-only placeholder for attention MAC + normalization behavior.
  return clamp_i8(CNoCQuant::mulQ4(q_q, kv_q));
}
