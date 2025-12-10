#pragma once
#include "bonxai/grid_coord.hpp"
#include "radix_map/map.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/logger.hpp"
#include "radix_utils/ray_config.hpp"

template <typename B>
RayHit MapImpl<B>::checkRayHit(const RayConfig& ray_config) {

  CoordT start = grid.posToCoord(ray_config.x_start, ray_config.y_start,
                                 ray_config.z_start);
  CoordT end =
      grid.posToCoord(ray_config.x_end, ray_config.y_end, ray_config.z_end);
  float occupancy_threshold = ray_config.occupancy_threshold;
  Bonxai::CoordT hit_coord;
  RayHit ray_hit;
  ray_hit.x = NAN;
  ray_hit.y = NAN;
  ray_hit.y = NAN;

  auto accessor = grid.createAccessor();
  float res_half = grid.getResolution() / 2.0;

  // find the first hit
  auto fun = [&accessor, &occupancy_threshold, &hit_coord,
              &ray_hit](const Bonxai::CoordT& coord) {
    if (typename B::Voxel* cell = accessor.value(coord, false);
        cell && cell->probability_log > logods(occupancy_threshold)) {

      ray_hit.hit = true;
      hit_coord = coord;

      return false;
    }

    return true;
  };

  RayIterator(start, end, fun);

  if (!ray_hit.hit) {
    fun(end);
  }

  if (ray_hit.hit) {

    Point3D pos = grid.coordToPos(hit_coord);

    // voxel center
    ray_hit.x = pos.x + res_half;
    ray_hit.y = pos.y + res_half;
    ray_hit.z = pos.z + res_half;

    RCLCPP_INFO(Logging::logger, "Voxel hit at (%.2f, %.2f, %.2f)", ray_hit.x,
                ray_hit.y, ray_hit.z);
  }

  else {

    RCLCPP_INFO(Logging::logger, "no Voxel hit along the ray");
  }

  return ray_hit;
}
