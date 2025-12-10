#pragma once
#include <pcl/point_types.h>
#include "bonxai/grid_coord.hpp"
#include "radix_map/behavior/behaviors/behavior_base.hpp"
#include "radix_map/behavior/point_types/carry.hpp"
#include "radix_map/behavior/point_types/point_basic.hpp"
#include "radix_map/behavior/voxel_types/voxel_basic.hpp"
#include "radix_utils/config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/probabilities.hpp"

using PointXYZL = pcl::PointXYZL;
using PointXYZRGB = pcl::PointXYZRGB;
using PointXYZRGBL = pcl::PointXYZRGBL;
using Vector3D = Eigen::Vector3d;
using Point3D = Bonxai::Point3D;

class BehaviorBasic : public BehaviorBase {

 public:
  using PointIn = PointXYZL;
  using PointOut = PointXYZL;
  using PointChunk = PointBasic;
  using Voxel = VoxelBasic;
  using CarryPoint = CustomPoint;

  BehaviorBasic(const Config& config) : BehaviorBase(config), config(config) {}

  void updateHitBatch(Voxel* voxel, const std::vector<PointIn>& points,
                      const Vector3D& pos, uint8_t _update_count);
  void updateMiss(Voxel* voxel, const Vector3D& point, uint8_t _update_count);

  void supplyCarryPoint(std::vector<CarryPoint>& carry_points,
                        const Point3D& pos, Voxel* voxel);
  PointChunk createChunkPoint(const Point3D& center, const Voxel& voxel);

  void regular_update_voxel(Voxel* voxel, const Vector3D& center);

  void correctVoxelLabel(Voxel* voxel, int32_t label);

 private:
  const Config& config;
};

inline void BehaviorBasic::updateMiss(Voxel* voxel,
                                      [[maybe_unused]] const Vector3D& point,
                                      [[maybe_unused]] uint8_t _update_count) {
  if (voxel->update_id != _update_count || !config.update_id) {
    // Use label-specific prob_miss if provided based on last seen label
    int32_t delta_miss_log = config.prob_miss_log;
    if (!voxel->label_history.empty()) {

      delta_miss_log = get_label_specific_miss_log(config, voxel->label);
    }
    voxel->probability_log =
        std::max(voxel->probability_log + delta_miss_log, config.prob_min_log);

    voxel->update_id = _update_count;
  }
};

inline void BehaviorBasic::supplyCarryPoint(
    std::vector<CarryPoint>& carry_points, const Point3D& pos, Voxel* voxel) {
  int32_t prob_log = voxel->probability_log;

  carry_points.emplace_back(pos.x, pos.y, pos.z, prob_log, voxel->label);
};

inline BehaviorBasic::PointChunk BehaviorBasic::createChunkPoint(
    const Point3D& center, const Voxel& voxel) {

  PointChunk point;
  point.x = center.x;
  point.y = center.y;
  point.z = center.z;
  point.prob = prob(voxel.probability_log);

  point.label = voxel.label;

  Color color = config.getLabelEntry(voxel.label).color;
  uint8_t alpha = 255;
  uint8_t r = color.r;
  uint8_t g = color.g;
  uint8_t b = color.b;
  point.rgb = (alpha << 24) | (r << 16) | (g << 8) | b;

  return point;
};

inline void BehaviorBasic::regular_update_voxel(
    [[maybe_unused]] Voxel* voxel, [[maybe_unused]] const Vector3D& center) {
  // intentionally empty
}
inline void BehaviorBasic::updateHitBatch(Voxel* voxel,
                                          const std::vector<PointIn>& points,
                                          [[maybe_unused]] const Vector3D& pos,
                                          uint8_t _update_count) {

  if (points.empty())
    return;

  // Update label history
  for (const auto& point : points) {
    voxel->label_history.push(point.label);
    voxel->label = voxel->label_history.back();
  }

  if (voxel->update_id != _update_count || !config.update_id) {

    voxel->update_id = _update_count;

    // Use the last point's label
    const PointIn& last_point = points.back();

    // 1. Topological Probability Update
    int32_t delta_hit_log =
        get_label_specific_hit_log(config, last_point.label);
    voxel->probability_log =
        std::min(voxel->probability_log + delta_hit_log, config.prob_max_log);
  }
};

inline void BehaviorBasic::correctVoxelLabel(Voxel* voxel, int32_t label) {

  voxel->label_history.push(label);
  voxel->label = voxel->label_history.back();
}
