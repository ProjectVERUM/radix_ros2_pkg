#pragma once
#include "radix_map/map.hpp"
#include "radix_utils/chunk_config.hpp"

#include <cstdint>
#include <set>

template <typename B>
PointCloud2 MapImpl<B>::chunk(const ChunkConfig& chunk_config) {
  PointCloud2 cloud_msg;

  pcl::PointCloud<typename B::PointChunk> cloud;

  getChunkPoints(chunk_config, cloud);

  pcl::toROSMsg(cloud, cloud_msg);

  auto total_volume = (chunk_config.x_max - chunk_config.x_min) *
                      (chunk_config.y_max - chunk_config.y_min) *
                      (chunk_config.z_max - chunk_config.z_min);
  auto total_points = pow(grid.getInvResolution(), 3) * total_volume;
  auto points_percent = cloud.points.size() * 100.0 / total_points;

  RCLCPP_INFO(Logging::logger,
              "Returning CHUNK with %ld voxels (%% of total: %.2f)",
              cloud.points.size(), points_percent);

  RCLCPP_INFO(Logging::logger, "Total possible voxels in CHUNK: %.2f",
              total_points);
  return cloud_msg;
}

template <typename B>
void MapImpl<B>::getChunkPoints(
    const ChunkConfig& chunk_config,
    pcl::PointCloud<typename B::PointChunk>& cloud) {
  size_t step = getChunkEdgeLength();

  auto coordComparator = [](const Bonxai::CoordT& a, const Bonxai::CoordT& b) {
    if (a.x != b.x)
      return a.x < b.x;
    if (a.y != b.y)
      return a.y < b.y;
    return a.z < b.z;
  };
  auto chunk_coords_visited =
      std::set<CoordT, decltype(coordComparator)>(coordComparator);

  for (float x = chunk_config.x_min; x < chunk_config.x_max + step; x += step) {
    for (float y = chunk_config.y_min; y < chunk_config.y_max + step;
         y += step) {
      for (float z = chunk_config.z_min; z < chunk_config.z_max + step;
           z += step) {
        CoordT coord = grid.getRootKey(Bonxai::PosToCoord(
            Bonxai::Point3D(x, y, z), 1.0 / grid.getResolution()));
        chunk_coords_visited.insert(coord);
        getSingleChunkPoints(x, y, z, chunk_config, cloud);
      }
    }
  }

  if (chunk_config.delete_rest) {
    RCLCPP_INFO(Logging::logger, "Deleting rest...");
    RCLCPP_INFO(Logging::logger, "Chunks in root map: %zu",
                grid.root_map.size());
    RCLCPP_INFO(Logging::logger, "Chunks in chunk (some empty): %zu",
                chunk_coords_visited.size());
    auto it = grid.root_map.begin();
    while (it != grid.root_map.end()) {
      auto key = it->first;
      if (chunk_coords_visited.find(key) == chunk_coords_visited.end()) {
        it = grid.root_map.erase(it);
      } else {
        ++it;
      }
    }
    RCLCPP_INFO(Logging::logger, "Chunks left in root map: %zu",
                grid.root_map.size());
    grid.releaseUnusedMemory();
  }
}

template <typename B>
void MapImpl<B>::getSingleChunkPoints(
    float chunkx, float chunky, float chunkz, const ChunkConfig& chunk_config,
    pcl::PointCloud<typename B::PointChunk>& cloud) {

  float res_half = grid.getResolution() / 2;

  const int32_t log_occupancy_threshold =
      logods(chunk_config.occupancy_threshold);

  if (chunk_config.level == "cell") {
    auto visitor = [&](typename B::Voxel& voxel, const CoordT& coord) {
      if (chunk_config.free ||
          voxel.probability_log > log_occupancy_threshold) {
        Point3D p = grid.coordToPos(coord);
        Point3D center =
            Point3D{p.x + res_half, p.y + res_half, p.z + res_half};
        // only retrieve points inside the requested range even if the chunk is larger
        if (center.x < chunk_config.x_min || center.x > chunk_config.x_max ||
            center.y < chunk_config.y_min || center.y > chunk_config.y_max ||
            center.z < chunk_config.z_min || center.z > chunk_config.z_max) {
          return;
        }

        typename B::PointChunk point = behavior.createChunkPoint(center, voxel);
        cloud.push_back(point);
      }
    };

    grid.forEachCellInInner(visitor, chunkx, chunky, chunkz);
  } else {
    throw std::runtime_error(
        "Metadata levels leaf and inner not implemented yet.");
  }
}
