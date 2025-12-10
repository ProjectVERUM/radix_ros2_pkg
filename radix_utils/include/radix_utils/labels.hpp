#pragma once
#include <optional>
#include "radix_utils/helpers.hpp"
#include "radix_utils/logger.hpp"

struct LabelEntry {
  std::string name;
  Color color;

  // Optional per-label probabilities (stored as log-odds, scaled by 1e6)
  std::optional<int32_t> prob_hit_log;
  std::optional<int32_t> prob_miss_log;

  std::optional<int32_t> dilation_update_thres;

  void print();
};

inline std::unordered_map<size_t, LabelEntry> load_labels(
    std::string label_params_path) {

  std::unordered_map<size_t, LabelEntry> labels;
  YAML::Node class_params = YAML::LoadFile(label_params_path);

  for (const auto& label : class_params["labels"]) {
    // label is a key-value pair
    size_t key = label.first.as<int>();
    const auto& value = label.second;

    LabelEntry entry;
    entry.name = value["name"].as<std::string>();
    entry.color.r = value["color"]["r"].as<int>();
    entry.color.g = value["color"]["g"].as<int>();
    entry.color.b = value["color"]["b"].as<int>();

    // Optional per-label hit/miss probabilities in [0,1]
    if (value["prob_hit"]) {
      const double ph = value["prob_hit"].as<double>();
      entry.prob_hit_log = logods(static_cast<float>(ph));
    }
    if (value["prob_miss"]) {
      const double pm = value["prob_miss"].as<double>();
      entry.prob_miss_log = logods(static_cast<float>(pm));
    }

    if (value["dilation_update_thres"]) {
      const int thres = value["dilation_update_thres"].as<int>();
      entry.dilation_update_thres = thres;
    }
    labels[key] = entry;
  }

  return labels;
}

inline std::vector<std::string> format_labels(
    std::unordered_map<size_t, LabelEntry> labels) {

  std::vector<std::string> formatted_lines;
  for (auto& [id, label] : labels) {
    std::string bg_color = "\033[48;2;" + std::to_string(label.color.r) + ";" +
                           std::to_string(label.color.g) + ";" +
                           std::to_string(label.color.b) + "m";
    std::string reset_code = "\033[0m";

    // Large highlight characters (█ blocks) for visibility
    std::string highlight = bg_color + "      " + reset_code;

    // Format the log message
    std::string log_message = highlight + " " + "id: " + std::to_string(id) +
                              " " + label.name + " [" +
                              std::to_string(label.color.r) + ", " +
                              std::to_string(label.color.g) + ", " +
                              std::to_string(label.color.b) + "] ";
    formatted_lines.push_back(log_message);
  }
  return formatted_lines;
}

inline void LabelEntry::print() {
  RCLCPP_INFO(Logging::logger, "Label name: %s", name.c_str());
  RCLCPP_INFO(Logging::logger, "Label color: %d, %d, %d", color.r, color.g,
              color.b);
}
