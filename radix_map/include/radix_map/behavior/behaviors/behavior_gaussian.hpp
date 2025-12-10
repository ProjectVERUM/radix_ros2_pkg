#pragma once
#include <pcl/point_types.h>
#include "bonxai/grid_coord.hpp"
#include "radix_map/behavior/behaviors/behavior_base.hpp"
#include "radix_map/behavior/point_types/carry_semantic.hpp"
#include "radix_map/behavior/point_types/point_gaussian.hpp"
#include "radix_map/behavior/voxel_types/voxel_gaussian.hpp"
#include "radix_utils/config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/labels.hpp"
#include "radix_utils/probabilities.hpp"

using PointXYZL = pcl::PointXYZL;
using PointXYZRGBA = pcl::PointXYZRGBA;
using Vector3D = Eigen::Vector3d;
using Point3D = Bonxai::Point3D;

class BehaviorGaussian : public BehaviorBase {
 public:
  using PointIn = PointXYZL;
  using PointOut = PointGaussian;
  using PointChunk = PointGaussian;
  using Voxel = VoxelGaussian;
  using CarryPoint = CustomPointSemantic;

  BehaviorGaussian(const Config& config)
      : BehaviorBase(config), config(config) {
    Voxel::num_labels = config.num_labels;
  }

  void updateMiss(Voxel* cell, const Vector3D& point, uint8_t _update_count);

  void updateHitBatch(Voxel* voxel, const std::vector<PointIn>& points,
                      const Vector3D& pos, uint8_t _update_count);

  void supplyCarryPoint(std::vector<CarryPoint>& carry_points,
                        const Point3D& pos, Voxel* voxel);
  PointChunk createChunkPoint(const Point3D& center, const Voxel& voxel);

  void updateGaussian(Voxel* voxel, const Vector3D& point);
  void regular_update_voxel(Voxel* voxel, const Vector3D& center,
                            float prob_reset, float decay_rate);

  void correctVoxelLabel(Voxel* voxel, int32_t label);

 private:
  const Config& config;
};

inline void BehaviorGaussian::updateMiss(
    Voxel* voxel, [[maybe_unused]] const Vector3D& point,
    [[maybe_unused]] uint8_t _update_count) {

  // check whether voxel should be updated
  if (voxel->update_id != _update_count || !config.update_id) {

    // get label-specific miss prob if available
    int32_t delta_miss_log = get_label_specific_miss_log(config, voxel->label);
    voxel->probability_log =
        std::max(voxel->probability_log + delta_miss_log, config.prob_min_log);

    // reset gaussian statistics of voxel when probability falls below threshold
    // keep label information
    if (voxel->probability_log < config.occupancy_threshold_log) {

      int32_t probability_log_save = voxel->probability_log;
      uint8_t update_id_save = voxel->update_id;
      auto label_probs_saved = voxel->label_probs;

      // new voxel
      *voxel = VoxelGaussian{};
      voxel->probability_log = probability_log_save;
      voxel->update_id = update_id_save;
      voxel->label_probs = label_probs_saved;
    }

    voxel->update_id = _update_count;
  }
};

inline void BehaviorGaussian::updateHitBatch(
    Voxel* voxel, const std::vector<PointIn>& points,
    [[maybe_unused]] const Vector3D& pos, uint8_t _update_count) {
  if (points.empty())
    return;

  // Early out if no update needed
  if (config.update_id && voxel->update_id == _update_count)
    return;

  // Pre-size vector ONCE to avoid dynamic growth
  std::vector<int32_t> observed_labels;
  observed_labels.reserve(points.size());

  // Single pass through points: collect labels and update Gaussian
  for (const auto& p : points) {
    if (p.label < Voxel::num_labels) {
      observed_labels.push_back(p.label);
    }

    // cheaper temporary init
    updateGaussian(voxel, Vector3D{p.x, p.y, p.z});
  }

  // Apply label update (if any)
  if (!observed_labels.empty()) {
    voxel->updateLabels(observed_labels);
  }

  // Update prob using label-specific hit log
  const int32_t delta_hit_log =
      get_label_specific_hit_log(config, voxel->label);

  voxel->probability_log =
      std::min(voxel->probability_log + delta_hit_log, config.prob_max_log);

  voxel->update_id = _update_count;
}

inline void BehaviorGaussian::supplyCarryPoint(
    std::vector<CarryPoint>& carry_points, const Point3D& pos, Voxel* voxel) {
  int32_t prob_log = voxel->probability_log;

  carry_points.emplace_back(pos.x, pos.y, pos.z, prob_log, voxel->label,
                            voxel->label_probs[voxel->label]);
};

inline BehaviorGaussian::PointChunk BehaviorGaussian::createChunkPoint(
    const Point3D& center, const Voxel& voxel) {
  PointChunk point;

  // Position
  point.x = center.x;
  point.y = center.y;
  point.z = center.z;

  // Probability and label
  point.prob = prob(voxel.probability_log);

  const int32_t label = voxel.label;
  point.label = label;
  point.label_prob = voxel.label_probs[label];

  // Gaussian statistics
  point.mean_x = voxel.mean_x;
  point.mean_y = voxel.mean_y;
  point.mean_z = voxel.mean_z;

  point.cov_xx = voxel.cov_xx;
  point.cov_xy = voxel.cov_xy;
  point.cov_xz = voxel.cov_xz;
  point.cov_yy = voxel.cov_yy;
  point.cov_yz = voxel.cov_yz;
  point.cov_zz = voxel.cov_zz;

  point.num_points = voxel.num_points;

  point.x0 = voxel.x0;
  point.y0 = voxel.y0;
  point.z0 = voxel.z0;

  return point;
}

inline void BehaviorGaussian::updateGaussian(Voxel* voxel,
                                             const Vector3D& point) {

  // store either the latest or the first inserted point based on config
  if (config.store_last || voxel->num_points == 0) {
    voxel->x0 = point.x();
    voxel->y0 = point.y();
    voxel->z0 = point.z();
  }

  if (voxel->num_points == 0) {
    assert(voxel->weight <= 0.0f);
    voxel->num_points = 1;

    // first point: initialize statistics and exit
    voxel->mean_x = point.x();
    voxel->mean_y = point.y();
    voxel->mean_z = point.z();
    voxel->cov_xx = voxel->cov_yy = voxel->cov_zz = 0.0f;
    voxel->cov_xy = voxel->cov_xz = voxel->cov_yz = 0.0f;
    voxel->weight = 1.0f;

    return;
  }

  // subsequent points
  assert(voxel->weight > 0.0f);
  voxel->num_points += 1;

  // EMA-like update based on effective weight
  voxel->weight += 1.0f;
  float alpha = 1.0f / voxel->weight;

  // update mean
  float dx = point.x() - voxel->mean_x;
  float dy = point.y() - voxel->mean_y;
  float dz = point.z() - voxel->mean_z;

  voxel->mean_x += alpha * dx;
  voxel->mean_y += alpha * dy;
  voxel->mean_z += alpha * dz;

  // update covariance
  float dx2 = point.x() - voxel->mean_x;
  float dy2 = point.y() - voxel->mean_y;
  float dz2 = point.z() - voxel->mean_z;

  voxel->cov_xx += alpha * (dx * dx2 - voxel->cov_xx);
  voxel->cov_xy += alpha * (dx * dy2 - voxel->cov_xy);
  voxel->cov_xz += alpha * (dx * dz2 - voxel->cov_xz);
  voxel->cov_yy += alpha * (dy * dy2 - voxel->cov_yy);
  voxel->cov_yz += alpha * (dy * dz2 - voxel->cov_yz);
  voxel->cov_zz += alpha * (dz * dz2 - voxel->cov_zz);
}

inline void BehaviorGaussian::regular_update_voxel(Voxel* voxel,
                                                   const Vector3D& center,
                                                   float prob_reset = 0.00f,
                                                   float decay_rate = 0.00f) {

  // with probability prob_reset, reset mean and covariance
  if (prob_reset > 0.0f && static_cast<float>(rand()) / RAND_MAX < prob_reset) {
    voxel->mean_x = center.x();
    voxel->mean_y = center.y();
    voxel->mean_z = center.z();
    voxel->cov_xx = voxel->cov_yy = voxel->cov_zz = 0.0f;
    voxel->cov_xy = voxel->cov_xz = voxel->cov_yz = 0.0f;
    voxel->weight = 1.0f;
    return;
  }

  if (voxel->weight <= 0.0f)
    return;

  // decay mean and covariance toward prior with decay_rate
  if (decay_rate > 0.0f) {
    float keep = 1.0f - decay_rate;

    // decay weight (effective information)
    voxel->weight *= keep;

    // decay mean toward voxel center
    voxel->mean_x = keep * voxel->mean_x + decay_rate * center.x();
    voxel->mean_y = keep * voxel->mean_y + decay_rate * center.y();
    voxel->mean_z = keep * voxel->mean_z + decay_rate * center.z();

    // decay covariance toward prior
    float prior_cov_xx = 0.0f;
    float prior_cov_yy = 0.0f;
    float prior_cov_zz = 0.0f;

    voxel->cov_xx = keep * voxel->cov_xx + decay_rate * prior_cov_xx;
    voxel->cov_xy = keep * voxel->cov_xy;  // prior has no correlation
    voxel->cov_xz = keep * voxel->cov_xz;  // prior has no correlation
    voxel->cov_yy = keep * voxel->cov_yy + decay_rate * prior_cov_yy;
    voxel->cov_yz = keep * voxel->cov_yz;  // prior has no correlation
    voxel->cov_zz = keep * voxel->cov_zz + decay_rate * prior_cov_zz;
  }
}

inline void BehaviorGaussian::correctVoxelLabel(
    [[maybe_unused]] Voxel* voxel, [[maybe_unused]] int32_t label) {
  // do nothing
}
