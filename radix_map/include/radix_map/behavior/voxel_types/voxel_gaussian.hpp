#pragma once
#include <iostream>
#include "radix_map/behavior/voxel_types/voxel_base.hpp"
#include "radix_map/serialize.hpp"
#include "radix_map/utils/queue.hpp"

class VoxelGaussian : public VoxelBase {
 public:
  // bitfields: cannot be default-initialized before C++20
  int32_t update_id : 4;
  int32_t probability_log : 28;

  Queue<int, 5> label_history;

  // Gaussian distribution
  float cov_xx = 0.0f, cov_xy = 0.0f, cov_xz = 0.0f, cov_yy = 0.0f,
        cov_yz = 0.0f, cov_zz = 0.0f;
  float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
  float weight = 0.0f;     // effective number of points
  int32_t num_points = 0;  // total number of points inserted

  // latest or first inserted point into voxel
  float x0 = std::numeric_limits<float>::quiet_NaN();
  float y0 = std::numeric_limits<float>::quiet_NaN();
  float z0 = std::numeric_limits<float>::quiet_NaN();

  // voxel label distribution
  inline static uint32_t num_labels;
  std::vector<float> label_probs;  // probability for each class label
  int32_t label;                   // label with currently highest probability

  // Proper initialization for C++17 ----
  VoxelGaussian() : update_id(0), probability_log(0), label(0) {
    // Initialize label_probs uniformly
    if (num_labels > 0) {
      label_probs.resize(num_labels, 1.0f / static_cast<float>(num_labels));
    }
  }
  size_t customMemUsage() const { return sizeof(VoxelGaussian); }

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
inline void Write(std::ostream& out, const VoxelGaussian& voxel) {
  Write(out, voxel.update_id);
  Write(out, voxel.probability_log);
  Write(out, voxel.label_history);
  Write(out, voxel.cov_xx);
  Write(out, voxel.cov_xy);
  Write(out, voxel.cov_xz);
  Write(out, voxel.cov_yy);
  Write(out, voxel.cov_yz);
  Write(out, voxel.cov_zz);
  Write(out, voxel.mean_x);
  Write(out, voxel.mean_y);
  Write(out, voxel.mean_z);
  Write(out, voxel.num_points);
  Write(out, voxel.x0);
  Write(out, voxel.y0);
  Write(out, voxel.z0);
  Write(out, voxel.label_probs);
}

template <>
inline VoxelGaussian Read(std::istream& input) {
  VoxelGaussian voxel;
  voxel.update_id = Read<int32_t>(input);
  voxel.probability_log = Read<int32_t>(input);
  voxel.label_history = Read<Queue<int, 5>>(input);

  voxel.cov_xx = Read<float>(input);
  voxel.cov_xy = Read<float>(input);
  voxel.cov_xz = Read<float>(input);
  voxel.cov_yy = Read<float>(input);
  voxel.cov_yz = Read<float>(input);
  voxel.cov_zz = Read<float>(input);
  voxel.mean_x = Read<float>(input);
  voxel.mean_y = Read<float>(input);
  voxel.mean_z = Read<float>(input);
  voxel.num_points = Read<int32_t>(input);
  voxel.x0 = Read<float>(input);
  voxel.y0 = Read<float>(input);
  voxel.z0 = Read<float>(input);
  voxel.label_probs = Read<std::vector<float>>(input);

  return voxel;
}
