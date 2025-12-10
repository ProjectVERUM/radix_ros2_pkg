#pragma once
#include "radix_utils/logger.hpp"

struct RayConfig {
  float x_start;
  float y_start;
  float z_start;
  float x_end;
  float y_end;
  float z_end;
  float occupancy_threshold;

  void print();
};

inline void RayConfig::print() {

  RCLCPP_INFO(Logging::logger, "Ray Config:");
  RCLCPP_INFO(Logging::logger, "x_start: %f", x_start);
  RCLCPP_INFO(Logging::logger, "y_start: %f", y_start);
  RCLCPP_INFO(Logging::logger, "z_start: %f", z_start);
  RCLCPP_INFO(Logging::logger, "x_end: %f", x_end);
  RCLCPP_INFO(Logging::logger, "y_end: %f", y_end);
  RCLCPP_INFO(Logging::logger, "z_end: %f", z_end);
  RCLCPP_INFO(Logging::logger, "occupancy_threshold: %f", occupancy_threshold);
}
