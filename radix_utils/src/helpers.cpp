#include "radix_utils/helpers.hpp"

float bayesianUpdate(float prior, float likelihood) {
  return prior * likelihood /
         (prior * likelihood + (1 - prior) * (1 - likelihood));
}

uint32_t pack_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (r << 16) | (g << 8) | b;
}

float rgb_to_float(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t packed = pack_rgb(r, g, b);
  float result;
  std::memcpy(&result, &packed, sizeof(result));
  return result;
}

std::tuple<int32_t, float> get_max_label(const std::vector<float> label_probs) {
  int32_t max_label = 0;
  float max_label_prob = 0;
  for (int i = 0; i < label_probs.size(); ++i) {
    if (label_probs[i] > max_label_prob) {
      max_label_prob = label_probs[i];
      max_label = i;
    }
  }
  return std::make_tuple(max_label, max_label_prob);
}

std::chrono::high_resolution_clock::time_point getTime() {
  return std::chrono::high_resolution_clock::now();
}
