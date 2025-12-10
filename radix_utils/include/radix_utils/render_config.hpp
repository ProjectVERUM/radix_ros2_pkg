#pragma once
#include <string>
#include "radix_utils/logger.hpp"
#include "radix_utils/validation.hpp"

struct RenderConfig {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  float max_range;
  int width;
  int height;
  float fov_x;
  float fov_y;
  std::string level;
  bool live;
  bool publish;
  float occupancy_threshold;
  bool use_rgb;

  ValidationResult validate();
  void print();
};

inline ValidationResult RenderConfig::validate() {
  return {true, ""};
}

inline void RenderConfig::print() {
  RCLCPP_INFO(Logging::logger, "Render config:");
  RCLCPP_INFO(Logging::logger, "x: %f", x);
  RCLCPP_INFO(Logging::logger, "y: %f", y);
  RCLCPP_INFO(Logging::logger, "z: %f", z);
  RCLCPP_INFO(Logging::logger, "roll: %f", roll);
  RCLCPP_INFO(Logging::logger, "pitch: %f", pitch);
  RCLCPP_INFO(Logging::logger, "yaw: %f", yaw);
  RCLCPP_INFO(Logging::logger, "max_range: %f", max_range);
  RCLCPP_INFO(Logging::logger, "width: %d", width);
  RCLCPP_INFO(Logging::logger, "height: %d", height);
  RCLCPP_INFO(Logging::logger, "yaw_angle: %f", fov_x);
  RCLCPP_INFO(Logging::logger, "pitch_angle: %f", fov_y);
  RCLCPP_INFO(Logging::logger, "level: %s", level.c_str());
  RCLCPP_INFO(Logging::logger, "live: %s", live ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "publish: %s", publish ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "occupancy_threshold: %f", occupancy_threshold);
  RCLCPP_INFO(Logging::logger, "use_rgb: %s", use_rgb ? "true" : "false");
}
