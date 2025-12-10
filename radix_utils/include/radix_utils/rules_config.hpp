#pragma once
#include <string>
#include "radix_utils/logger.hpp"

struct HistoryLimitConfig {
  int limit;

  void print() const;
};

struct ZLimitConfig {
  std::string label;
  int limit;

  void print() const;
};

struct DilationConfig {
  std::vector<int> kernel_size;
  int update_thres;

  void print() const;
};

struct RulesConfig {
  std::vector<ZLimitConfig> z_limits;
  std::vector<HistoryLimitConfig> h_limits;
  DilationConfig dilation;

  void print() const;
};

inline void ZLimitConfig::print() const {
  RCLCPP_INFO(Logging::logger, "ZLimitConfig: limit=%d, label=%s", limit,
              label.c_str());
}

inline void HistoryLimitConfig::print() const {
  RCLCPP_INFO(Logging::logger, "HistoryLimitConfig: limit=%d", limit);
}

inline void DilationConfig::print() const {
  std::string kernel_str = "[";
  for (size_t i = 0; i < kernel_size.size(); ++i) {
    kernel_str += std::to_string(kernel_size[i]);
    if (i + 1 < kernel_size.size())
      kernel_str += ", ";
  }
  kernel_str += "]";

  RCLCPP_INFO(Logging::logger, "DilationConfig:");
  RCLCPP_INFO(Logging::logger, "list of kernel_sizes: %s", kernel_str.c_str());
  RCLCPP_INFO(Logging::logger, "update_threshold: %d", update_thres);

  // for (auto [id, thres] : Config.label) {
  //   RCLCPP_INFO(Logging::logger, "Label: %d --> thres: %d", id, update_thres);
  // }
}

inline void RulesConfig::print() const {
  RCLCPP_INFO(Logging::logger, "RulesConfig:");

  for (const auto& z : z_limits) {
    z.print();
  }

  for (const auto& h : h_limits) {
    h.print();
  }

  dilation.print();
}

RulesConfig load_rules(const std::string& path) {

  RulesConfig config;
  YAML::Node rule_params = YAML::LoadFile(path);
  auto rules = rule_params["radix_server"]["ros__parameters"]["rules"];

  if (rules["z-limit"]) {
    for (const auto& node : rules["z-limit"]) {
      config.z_limits.push_back(
          {node["label"].as<std::string>(), node["limit"].as<int>()});
    }
  }

  if (rules["history_limit"]) {
    for (const auto& node : rules["history_limit"]) {
      config.h_limits.push_back({node["limit"].as<int>()});
    }
  }

  if (rules["dilation"]) {
    const auto& node = rules["dilation"];
    if (node["kernel_size"]) {
      for (const auto& k : node["kernel_size"]) {
        config.dilation.kernel_size.push_back(k.as<int>());
      }
    }
    config.dilation.update_thres = node["update_thres"].as<int>();
  }

  return config;
}
