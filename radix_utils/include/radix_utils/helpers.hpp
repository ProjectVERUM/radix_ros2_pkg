#pragma once
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include "radix_utils/logger.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;

[[nodiscard]] static constexpr int32_t logods(float prob) {
  return int32_t(1e6 * std::log(prob / (1.0 - prob)));
}

[[nodiscard]] static constexpr float prob(int32_t logods_fixed) {
  float logods = float(logods_fixed) * 1e-6;
  return (1.0 - 1.0 / (1.0 + std::exp(logods)));
}

float bayesianUpdate(float prior, float likelihood);

uint32_t pack_rgb(uint8_t r, uint8_t g, uint8_t b);

float rgb_to_float(uint8_t r, uint8_t g, uint8_t b);

struct Color {
  int r;
  int g;
  int b;
};

std::tuple<int32_t, float> get_max_label(std::vector<float> label_probs);

std::chrono::high_resolution_clock::time_point getTime();

struct Imgs {
  Image semantic_img;
  Image semantic_prob_img;
  Image depth_img;
  Image depth_prob_img;
};

struct RayHit {
  bool hit;
  float x;
  float y;
  float z;
};
