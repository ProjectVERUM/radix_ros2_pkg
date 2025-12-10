#pragma once

#include "radix_utils/config.hpp"

// Returns label-specific hit log-odds if defined for the label; otherwise the default
inline int32_t get_label_specific_hit_log(const Config& config, int label_id) {
  const LabelEntry label_entry = config.getLabelEntry(label_id);
  if (label_entry.prob_hit_log.has_value()) {
    return *(label_entry.prob_hit_log);
  }
  return config.prob_hit_log;
}

// Returns label-specific miss log-odds if defined for the label; otherwise the default
inline int32_t get_label_specific_miss_log(const Config& config, int label_id) {
  const LabelEntry label_entry = config.getLabelEntry(label_id);
  if (label_entry.prob_miss_log.has_value()) {
    return *(label_entry.prob_miss_log);
  }
  return config.prob_miss_log;
}
