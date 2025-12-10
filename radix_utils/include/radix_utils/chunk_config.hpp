#pragma once
#include <string>
#include "radix_utils/logger.hpp"
#include "radix_utils/validation.hpp"

struct Region {
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;

  ValidationResult validate() const;
  void print() const;
};

inline ValidationResult Region::validate() const {
  if (x_min > x_max)
    return {false, "x_min > x_max"};
  if (y_min > y_max)
    return {false, "y_min > y_max"};
  if (z_min > z_max)
    return {false, "z_min > z_max"};
  return {true, ""};
}

inline void Region::print() const {
  RCLCPP_INFO(Logging::logger, "Region:");
  RCLCPP_INFO(Logging::logger, "x_min: %f", x_min);
  RCLCPP_INFO(Logging::logger, "x_max: %f", x_max);
  RCLCPP_INFO(Logging::logger, "y_min: %f", y_min);
  RCLCPP_INFO(Logging::logger, "y_max: %f", y_max);
  RCLCPP_INFO(Logging::logger, "z_min: %f", z_min);
  RCLCPP_INFO(Logging::logger, "z_max: %f", z_max);
}

struct ChunkConfig : public Region {
  bool free;
  bool delete_rest;
  bool publish;

  float occupancy_threshold;
  std::string level;

  void print();
};

inline void ChunkConfig::print() {
  RCLCPP_INFO(Logging::logger, "Chunk config:");
  RCLCPP_INFO(Logging::logger, "x_min: %f", x_min);
  RCLCPP_INFO(Logging::logger, "x_max: %f", x_max);
  RCLCPP_INFO(Logging::logger, "y_min: %f", y_min);
  RCLCPP_INFO(Logging::logger, "y_max: %f", y_max);
  RCLCPP_INFO(Logging::logger, "z_min: %f", z_min);
  RCLCPP_INFO(Logging::logger, "z_max: %f", z_max);
  RCLCPP_INFO(Logging::logger, "free: %s", free ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "delete_rest: %s",
              delete_rest ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "publish: %s", publish ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "occupancy_threshold: %f", occupancy_threshold);
  RCLCPP_INFO(Logging::logger, "level: %s", level.c_str());
}
