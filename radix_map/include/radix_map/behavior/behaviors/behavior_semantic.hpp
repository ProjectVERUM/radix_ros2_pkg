#pragma once
#include <pcl/point_types.h>
#include "bonxai/grid_coord.hpp"
#include "radix_map/behavior/behaviors/behavior_base.hpp"
#include "radix_map/behavior/point_types/carry_semantic.hpp"
#include "radix_map/behavior/point_types/point_semantic.hpp"
#include "radix_map/behavior/voxel_types/voxel_semantic.hpp"
#include "radix_utils/config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/probabilities.hpp"

using PointXYZL = pcl::PointXYZL;
using PointXYZRGB = pcl::PointXYZRGB;
using PointXYZRGBL = pcl::PointXYZRGBL;
using Vector3D = Eigen::Vector3d;
using Point3D = Bonxai::Point3D;

class BehaviorSemantic : public BehaviorBase {

 public:
  using PointIn = PointXYZL;
  using PointOut = PointXYZL;
  using PointChunk = PointSemantic;
  using Voxel = VoxelSemantic;
  using CarryPoint = CustomPointSemantic;

  BehaviorSemantic(const Config& config)
      : BehaviorBase(config), config(config) {
    Voxel::num_labels = config.num_labels;
  }

  void updateMiss(Voxel* voxel, const Vector3D& point, uint8_t _update_count);

  void updateHitBatch(Voxel* voxel, const std::vector<PointIn>& points,
                      const Vector3D& pos, uint8_t _update_count);

  void supplyCarryPoint(std::vector<CarryPoint>& carry_points,
                        const Point3D& pos, Voxel* voxel);

  PointChunk createChunkPoint(const Point3D& center, const Voxel& voxel);

  void regular_update_voxel(Voxel* voxel, const Vector3D& center);

  void correctVoxelLabel(Voxel* voxel, int32_t label);

 private:
  const Config& config;
};

inline void BehaviorSemantic::updateMiss(Voxel* voxel,
                                         [[maybe_unused]] const Vector3D& point,
                                         uint8_t _update_count) {

  // check whether voxel should be updated
  if (voxel->update_id != _update_count || !config.update_id) {

    // get label-specific miss prob if available
    int32_t delta_miss_log = get_label_specific_miss_log(config, voxel->label);
    voxel->probability_log =
        std::max(voxel->probability_log + delta_miss_log, config.prob_min_log);

    voxel->update_id = _update_count;
  }
}

inline void BehaviorSemantic::updateHitBatch(
    Voxel* voxel, const std::vector<PointIn>& points,
    [[maybe_unused]] const Vector3D& pos, uint8_t _update_count) {
  if (points.empty())
    return;

  // check whether voxel should be updated
  if (voxel->update_id != _update_count || !config.update_id) {

    // voxel label and label probabilities update
    std::vector<int32_t> observed_labels;
    observed_labels.reserve(points.size());

    for (const auto& point : points) {
      if (point.label < Voxel::num_labels) {
        observed_labels.push_back(point.label);
      }
    }

    if (!observed_labels.empty()) {
      voxel->updateLabels(observed_labels);
    }

    // update occupied probability using label-specific hit prob if provided
    int32_t delta_hit_log = get_label_specific_hit_log(config, voxel->label);
    voxel->probability_log =
        std::min(voxel->probability_log + delta_hit_log, config.prob_max_log);

    voxel->update_id = _update_count;
  }
}

inline void BehaviorSemantic::supplyCarryPoint(
    std::vector<CarryPoint>& carry_points, const Point3D& pos, Voxel* voxel) {
  int32_t prob_log = voxel->probability_log;

  carry_points.emplace_back(pos.x, pos.y, pos.z, prob_log, voxel->label,
                            voxel->label_probs[voxel->label]);
}

inline BehaviorSemantic::PointChunk BehaviorSemantic::createChunkPoint(
    const Point3D& center, const Voxel& voxel) {
  PointChunk point;

  // Position
  point.x = center.x;
  point.y = center.y;
  point.z = center.z;

  // Probability
  point.prob = prob(voxel.probability_log);

  // Label info
  const int32_t label = voxel.label;
  point.label = label;
  point.label_prob = voxel.label_probs[label];

  // Color (packed ARGB)
  const Color& c = config.getLabelEntry(label).color;
  point.rgb = (255u << 24) | (c.r << 16) | (c.g << 8) | c.b;

  return point;
}

inline void BehaviorSemantic::regular_update_voxel(
    [[maybe_unused]] Voxel* voxel, [[maybe_unused]] const Vector3D& center) {
  // do nothing
}

inline void BehaviorSemantic::correctVoxelLabel(
    [[maybe_unused]] Voxel* voxel, [[maybe_unused]] int32_t label) {
  // do nothing
}
