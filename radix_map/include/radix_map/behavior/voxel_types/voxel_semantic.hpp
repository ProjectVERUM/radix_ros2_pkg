#pragma once
#include <cmath>
#include <iostream>
#include <numeric>

#include "radix_map/behavior/voxel_types/voxel_base.hpp"
#include "radix_map/serialize.hpp"
#include "radix_map/utils/queue.hpp"

class VoxelSemantic : public VoxelBase {
 public:
  // bitfields: cannot be default-initialized before C++20
  int32_t update_id : 4;
  int32_t probability_log : 28;

  int instanceID;
  Queue<int, 5> label_history;
  Queue<float, 5> uncertainty_history;
  Queue<float, 5> semantic_conf_history;
  Queue<float, 5> topological_conf_history;

  inline static uint32_t num_labels;

  std::vector<float> label_probs;  // probability for each class label
  int32_t label;                   // label with currently highest probability

  // Proper initialization for C++17 ----
  VoxelSemantic() : update_id(0), probability_log(0), instanceID(0), label(0) {
    // Initialize label_probs uniformly
    if (num_labels > 0) {
      label_probs.resize(num_labels, 1.0f / static_cast<float>(num_labels));
    }
  }

  size_t customMemUsage() const { return sizeof(VoxelSemantic); }

  /**
   * @brief Updates the label probabilities using an exponential moving average based on observed labels.
   *
   * This function updates the internal label probability distribution (`label_probs`) by applying an exponential
   * moving average update for each label observed in the input vector. Only labels present in `observed_labels`
   * are updated, and all label probabilities are decayed by a factor of alpha. The function also optionally
   * normalizes the probabilities to ensure they sum to 1, correcting for floating-point drift.
   *
   * @param observed_labels Vector of observed label indices. Assumes that observed_labels is not empty.
   * @param alpha Smoothing factor for the distribution's exponential moving average. Higher alpha = more weight
   *              on old distribution (default: 0.9f).
   * @param tolerance Tolerance for normalization to handle floating-point drift (default: 1e-4f).
   */
  void updateLabels(const std::vector<int32_t>& observed_labels,
                    float alpha = 0.8f, float tolerance = 1e-4f) {
    if (observed_labels.empty())
      return;

    // Count label occurrences
    std::vector<int32_t> counts(num_labels, 0);
    for (int32_t label_id : observed_labels) {
      if (static_cast<unsigned>(label_id) < static_cast<unsigned>(num_labels))
        counts[label_id]++;
    }

    // EMA update: decay all label probabilities
    const float inv_n = 1.0f / static_cast<float>(observed_labels.size());
    const float one_minus_alpha = 1.0f - alpha;
    for (float& p : label_probs)
      p *= alpha;

    // update probabilities for observed labels
    for (uint32_t i = 0; i < num_labels; ++i) {
      if (counts[i] > 0) {
        label_probs[i] += one_minus_alpha * counts[i] * inv_n;
      }
    }

    // Normalize probabilities only if drift is significant
    float sum = std::accumulate(label_probs.begin(), label_probs.end(), 0.0f);
    if (std::fabs(sum - 1.0f) > tolerance && sum > 0.0f) {
      float inv_sum = 1.0f / sum;
      for (float& p : label_probs)
        p *= inv_sum;
    }

    // update label with max probability
    int32_t best = 0;
    float best_prob = 0.0f;

    for (uint32_t i = 0; i < num_labels; ++i) {
      if (label_probs[i] > best_prob) {
        best_prob = label_probs[i];
        best = i;
      }
    }

    // // when unlabeled (= 0) classifications are incoming to a voxel that already has a label, then keep the existing label
    if (best != 0)
      label = best;
  }
};

template <>
inline void Write(std::ostream& out, const VoxelSemantic& voxel) {
  Write(out, voxel.update_id);
  Write(out, voxel.probability_log);
  Write(out, voxel.instanceID);
  Write(out, voxel.label_history);
  Write(out, voxel.uncertainty_history);
  Write(out, voxel.semantic_conf_history);
  Write(out, voxel.topological_conf_history);
  Write(out, voxel.label_probs);
}

template <>
inline VoxelSemantic Read(std::istream& input) {
  VoxelSemantic voxel;

  voxel.update_id = Read<int32_t>(input);
  voxel.probability_log = Read<int32_t>(input);
  voxel.instanceID = Read<int>(input);

  voxel.label_history = Read<Queue<int, 5>>(input);
  voxel.uncertainty_history = Read<Queue<float, 5>>(input);
  voxel.semantic_conf_history = Read<Queue<float, 5>>(input);
  voxel.topological_conf_history = Read<Queue<float, 5>>(input);

  voxel.label_probs = Read<std::vector<float>>(input);

  return voxel;
}
