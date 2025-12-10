#pragma once
#include <string>
#include "radix_utils/logger.hpp"
#include "radix_utils/validation.hpp"

struct BirdsEyeViewConfig {
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float min_height;
  float max_height;

  float occupancy_threshold;
  std::string level;

  ValidationResult validate();
  void print();
};

inline ValidationResult BirdsEyeViewConfig::validate() {
  if (x_min > x_max)
    return {false, "x_min > x_max"};
  if (y_min > y_max)
    return {false, "y_min > y_max"};
  if (min_height > max_height)
    return {false, "min_height > max_height"};
  return {true, ""};
}

inline void BirdsEyeViewConfig::print() {
  RCLCPP_INFO(Logging::logger, "Chunk config:");
  RCLCPP_INFO(Logging::logger, "x_min: %f", x_min);
  RCLCPP_INFO(Logging::logger, "x_max: %f", x_max);
  RCLCPP_INFO(Logging::logger, "y_min: %f", y_min);
  RCLCPP_INFO(Logging::logger, "y_max: %f", y_max);
  RCLCPP_INFO(Logging::logger, "min_height: %f", min_height);
  RCLCPP_INFO(Logging::logger, "max_height: %f", max_height);
  RCLCPP_INFO(Logging::logger, "occupancy_threshold: %f", occupancy_threshold);
  RCLCPP_INFO(Logging::logger, "level: %s", level.c_str());
}
