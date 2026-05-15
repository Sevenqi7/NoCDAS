#include <cmath>
#include <cstdint>

namespace {

int sat8(int value) {
  if (value > 127) {
    return 127;
  }
  if (value < -128) {
    return -128;
  }
  return value;
}

int quantize(float value) {
  return sat8(static_cast<int>(std::lround(value * 16.0f)));
}

float dequantize(int value) {
  return static_cast<float>(value) / 16.0f;
}

}  // namespace

extern "C" int dpi_swiglu(int gate_q, int up_q) {
  const float gate = dequantize(gate_q);
  const float up = dequantize(up_q);
  const float silu = gate / (1.0f + std::exp(-gate));
  return quantize(silu * up);
}

extern "C" int dpi_geglu(int gate_q, int up_q) {
  const float gate = dequantize(gate_q);
  const float up = dequantize(up_q);
  const float gelu = 0.5f * gate * (1.0f + std::erf(gate / 1.41421356f));
  return quantize(gelu * up);
}

extern "C" int dpi_exp(int x_q) {
  return quantize(std::exp(dequantize(x_q)));
}

extern "C" int dpi_div(int num_q, int den_q) {
  if (den_q == 0) {
    return sat8(num_q);
  }
  return quantize(dequantize(num_q) / dequantize(den_q));
}

extern "C" int dpi_attention_mix(int q_q, int kv_q) {
  return sat8(q_q + kv_q);
}
