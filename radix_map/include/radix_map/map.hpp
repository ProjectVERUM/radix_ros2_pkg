#pragma once

#include "bonxai/bonxai.hpp"

#include "radix_map/behavior/behaviors/behavior_base.hpp"
#include "radix_utils/birds_eye_view_config.hpp"
#include "radix_utils/chunk_config.hpp"
#include "radix_utils/config.hpp"
#include "radix_utils/ray_config.hpp"
#include "radix_utils/render_config.hpp"
#include "radix_utils/rules_config.hpp"

#include "radix_utils/helpers.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <cstdint>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_set>

using namespace Bonxai;

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;
using Vector3D = Eigen::Vector3d;

class Map {
 public:
  virtual ~Map() = default;
  virtual void insertCloud(const PointCloud2::ConstSharedPtr cloud,
                           std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) = 0;
  virtual PointCloud2 publishAll() = 0;
  virtual PointCloud2 chunk(const ChunkConfig& chunk_config) = 0;
  virtual Imgs render(const RenderConfig& render_config) = 0;
  virtual Imgs birds_eye_view(
      const BirdsEyeViewConfig& birds_eye_view_config) = 0;
  virtual void save_map(std::string filename) = 0;
  virtual RayHit checkRayHit(const RayConfig& ray_config) = 0;
  virtual void traverseChunk(const int& ksize, const int& update_thres,
                             const Region& region) = 0;
  virtual void printConfig() = 0;
  virtual void load_grid(std::string path) = 0;
  virtual void log_map_stats() = 0;
  size_t getVoxelCount();
  size_t getLeafGridCount();
  size_t getInnerGridCount();
  size_t customMemUsage();
  size_t rootMapSize();
};

template <typename B>
class MapImpl : public Map {
 public:
  // members
  VoxelGrid<typename B::Voxel> grid;
  mutable typename VoxelGrid<typename B::Voxel>::Accessor _accessor;

  // constructor
  MapImpl(double resolution, const Config& config)
      : grid(VoxelGrid<typename B::Voxel>(resolution)),
        _accessor(grid.createAccessor()),
        behavior(config),
        config(config) {}

  // services
  void insertCloud(const PointCloud2::ConstSharedPtr cloud,
                   std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);
  PointCloud2 publishAll();

  template <typename Allocator>
  void insertPointCloudBatch(
      const std::vector<typename B::PointIn, Allocator>& points,
      const pcl::PointXYZ& origin, float max_range);

  PointCloud2 chunk(const ChunkConfig& chunk_config);
  Imgs render(const RenderConfig& render_config);
  Imgs birds_eye_view(const BirdsEyeViewConfig& birds_eye_view_config);

  void save_map(std::string filename);
  void log_map_stats();

  // helpers
  void updateFreeCells(const Vector3D& origin);
  void getOccupiedVoxels(std::vector<CoordT>& coords);
  size_t updateVoxels();
  size_t getVoxelCount();
  size_t getLeafGridCount();
  size_t getInnerGridCount();
  size_t customMemUsage();
  size_t rootMapSize() { return grid.rootMap().size(); }
  float getChunkEdgeLength() {
    return grid.getResolution() * pow(2, grid.innerBits() + grid.leafBits());
  }
  void getChunkPoints(const ChunkConfig& chunk_config,
                      pcl::PointCloud<typename B::PointChunk>& cloud);
  void getSingleChunkPoints(float chunkx, float chunky, float chunkz,
                            const ChunkConfig& chunk_config,
                            pcl::PointCloud<typename B::PointChunk>& cloud);
  void getOccupiedVoxels(std::vector<typename B::CarryPoint>& points) {
    thread_local std::vector<Bonxai::CoordT> coords;
    coords.clear();
    getOccupiedVoxels(coords);

    auto acc = grid.createAccessor();

    for (const auto& coord : coords) {
      const auto p = grid.coordToPos(coord);
      typename B::Voxel* voxel = acc.value(coord);

      behavior.supplyCarryPoint(points, p, voxel);
    }
  }

  void printConfig() { config.print(); }
  void load_grid(std::string path);
  void applyRules(typename B::Voxel& cell, const CoordT& coord,
                  const RulesConfig& rules_config);
  void applyZLimit(typename B::Voxel& cell, const CoordT& coord,
                   const RulesConfig& config);
  void applyHistoryLimit(typename B::Voxel& cell, const CoordT& coord,
                         const HistoryLimitConfig& config);
  RayHit checkRayHit(const RayConfig& ray_config);
  void traverseChunk(const int& ksize, const int& update_thres,
                     const Region& region);
  void getMajorityLabel(const int& ksize, const int& update_thres,
                        const CoordT& coord,
                        std::unordered_map<CoordT, int>& voxel_set);
  void applyDilation(std::unordered_map<CoordT, int>& voxel_set);

 private:
  B behavior;
  const Config& config;

  uint8_t _update_count = 1;

  std::unordered_set<CoordT> _miss_coords;
  std::unordered_set<CoordT> _hit_coords;
};

template <typename B>
PointCloud2 MapImpl<B>::publishAll() {

  thread_local std::vector<typename B::CarryPoint> radix_result;
  radix_result.clear();
  getOccupiedVoxels(radix_result);

  if (radix_result.size() <= 1) {
    RCLCPP_INFO(Logging::logger, "Nothing to publish, radix is empty");
    return PointCloud2();
  }

  float res_half = grid.getResolution() / 2.0;

  // init pointcloud for occupied space:
  if (true) {
    PointCloud2 cloud;
    thread_local pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl_cloud.clear();

    for (const auto& voxel : radix_result) {

      pcl::PointXYZRGB point(static_cast<float>(voxel.x + res_half),
                             static_cast<float>(voxel.y + res_half),
                             static_cast<float>(voxel.z + res_half));

      Color color = config.getLabelEntry(voxel.label).color;

      uint8_t r = color.r;
      uint8_t g = color.g;
      uint8_t b = color.b;
      uint8_t alpha = 255;

      // Combine into rgba
      point.rgba = (alpha << 24) | (r << 16) | (g << 8) | b;
      pcl_cloud.push_back(point);
    }
    pcl::toROSMsg(pcl_cloud, cloud);

    RCLCPP_INFO(Logging::logger, "Published occupancy grid with %ld voxels.",
                pcl_cloud.points.size());
    return cloud;
  }

  return PointCloud2();
}

template <typename B>
void MapImpl<B>::insertCloud(const PointCloud2::ConstSharedPtr cloud,
                             std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {

  using PCLPointCloud = pcl::PointCloud<typename B::PointIn>;
  PCLPointCloud pc;

  pcl::fromROSMsg(*cloud, pc);

  // remove NaN and Inf values
  size_t filtered_index = 0;
  for (const auto& point : pc.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      pc.points[filtered_index++] = point;
    }
  }
  pc.resize(filtered_index);

  // Sensor In Global Frames Coordinates
  geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
  try {
    sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
        config.world_frame_id, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException& ex) {
    std::clog << ex.what() << std::endl;
    return;
  }

  Eigen::Matrix4f sensor_to_world =
      tf2::transformToEigen(sensor_to_world_transform_stamped.transform)
          .matrix()
          .cast<float>();

  // Transforming Points to Global Reference Frame
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  // Getting the Translation from the sensor to the Global Reference Frame
  const auto& t = sensor_to_world_transform_stamped.transform.translation;

  const pcl::PointXYZ sensor_to_world_vec3(t.x, t.y, t.z);

  if (config.sensor_max_range >= 0) {
    insertPointCloudBatch(pc.points, sensor_to_world_vec3,
                          config.sensor_max_range);
  } else {
    insertPointCloudBatch(pc.points, sensor_to_world_vec3,
                          std::numeric_limits<double>::infinity());
  }

  // updateVoxels();
}

template <typename B>
template <typename Allocator>
inline void MapImpl<B>::insertPointCloudBatch(
    const std::vector<typename B::PointIn, Allocator>& points,
    const pcl::PointXYZ& origin, float max_range) {

  const Vector3D origin_vec = ConvertPoint<Vector3D, pcl::PointXYZ>(origin);
  const float max_range_sqr = max_range * max_range;

  // Reserve an approximate number of buckets (fewer rehashes)
  std::unordered_map<CoordT, std::vector<typename B::PointIn>> voxel_points;
  voxel_points.reserve(points.size() / 4 + 1);

  Vector3D vect;
  vect.setZero();  // ensures no uninitialized memory

  for (const typename B::PointIn& point : points) {
    const Vector3D point_vec =
        ConvertPoint<Vector3D, typename B::PointIn>(point);

    vect = point_vec - origin_vec;
    const float squared_norm = vect.squaredNorm();

    // MISS case → cleaning ray
    if (squared_norm >= max_range_sqr) {
      const float dist = std::sqrt(squared_norm);

      // normalized() is faster than vect / dist manually
      const Vector3D new_point = origin_vec + vect * (max_range / dist);

      const CoordT coord = grid.posToCoord(new_point);
      typename B::Voxel* voxel = _accessor.value(coord, true);

      behavior.updateMiss(voxel, new_point, _update_count);
      _miss_coords.insert(coord);

    } else {
      // HIT case -> batch addition
      const CoordT coord = grid.posToCoord(point_vec);
      voxel_points.try_emplace(coord).first->second.push_back(point);
      _hit_coords.insert(coord);
    }
  }

  // process each voxel's accumulated hits
  const float res_half = grid.getResolution() * 0.5f;

  for (auto& pair : voxel_points) {
    const CoordT& coord = pair.first;
    auto& voxel_p = pair.second;

    typename B::Voxel* voxel = _accessor.value(coord, true);

    // compute voxel center position
    const Point3D p = grid.coordToPos(coord);
    const Vector3D pos(p.x + res_half, p.y + res_half, p.z + res_half);

    behavior.updateHitBatch(voxel, voxel_p, pos, _update_count);
  }

  // Ray clearing, if enabled
  if (config.cleaning_ray) {
    updateFreeCells(origin_vec);
  }

  // Commit changes to voxels
  updateVoxels();

  // Always clear coord tracking and handle update counter
  _miss_coords.clear();
  _hit_coords.clear();

  if (++_update_count == 4) {
    _update_count = 1;
  }
}

template <typename B>
size_t MapImpl<B>::updateVoxels() {

  float res_half = grid.getResolution() / 2.0;

  auto visitor = [&](typename B::Voxel& voxel, const CoordT& coord) {
    Point3D p = grid.coordToPos(coord);
    Vector3D pos(p.x + res_half, p.y + res_half, p.z + res_half);

    behavior.regular_update_voxel(&voxel, pos);
  };

  // apply regular update to all cells
  grid.forEachCell(visitor);
  return true;
}

template <typename B>
void MapImpl<B>::updateFreeCells(const Vector3D& origin) {
  auto accessor = grid.createAccessor();

  float res_half = grid.getResolution() / 2.0;

  // same as updateMiss, but using lambda will force inlining
  auto clearPoint = [this, &accessor, res_half](
                        const CoordT& coord,
                        [[maybe_unused]] auto&&... /*args*/) -> bool {
    // Do not reduce if this voxel was hit during this same insertion
    if (_hit_coords.find(coord) != _hit_coords.end()) {
      return true;
    }

    typename B::Voxel* voxel = accessor.value(coord, true);

    // forward center as "actual" position
    Point3D p = grid.coordToPos(coord);
    Vector3D pos(p.x + res_half, p.y + res_half, p.z + res_half);

    behavior.updateMiss(voxel, pos, _update_count);

    return true;
  };

  const auto coord_origin = grid.posToCoord(origin);

  for (const auto& coord_end : _hit_coords) {
    RayIterator(coord_origin, coord_end, clearPoint, coord_end);
  }

  for (const auto& coord_end : _miss_coords) {
    RayIterator(coord_origin, coord_end, clearPoint, coord_end);
  }
}

template <class Functor, typename... Args>
inline void RayIterator(const CoordT& key_origin, const CoordT& key_end,
                        const Functor& func, Args&&... args) {
  if (key_origin == key_end) {
    return;
  }
  if (!func(key_origin, std::forward<Args>(args)...)) {
    return;
  }

  CoordT error = {0, 0, 0};
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = {delta.x < 0 ? -1 : 1, delta.y < 0 ? -1 : 1,
                       delta.z < 0 ? -1 : 1};

  delta = {delta.x < 0 ? -delta.x : delta.x, delta.y < 0 ? -delta.y : delta.y,
           delta.z < 0 ? -delta.z : delta.z};

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

  // maximum change of any coordinate
  for (int i = 0; i < max - 1; ++i) {
    // update errors
    error = error + delta;
    // manual loop unrolling
    if ((error.x << 1) >= max) {
      coord.x += step.x;
      error.x -= max;
    }
    if ((error.y << 1) >= max) {
      coord.y += step.y;
      error.y -= max;
    }
    if ((error.z << 1) >= max) {
      coord.z += step.z;
      error.z -= max;
    }
    if (!func(coord, std::forward<Args>(args)...)) {
      return;
    }
  }
}

template <typename B>
void MapImpl<B>::getOccupiedVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&]([[maybe_unused]] typename B::Voxel& cell,
                     const CoordT& coord) {
    if (cell.probability_log > config.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  grid.forEachCell(visitor);
}

template <typename B>
size_t MapImpl<B>::getVoxelCount() {
  int32_t count = 0;
  auto visitor = [&]([[maybe_unused]] typename B::Voxel& cell,
                     [[maybe_unused]] const CoordT& coord) {
    count++;
  };
  grid.forEachCell(visitor);
  return count;
}

template <typename B>
size_t MapImpl<B>::getLeafGridCount() {
  int32_t count = 0;
  auto visitor = [&]([[maybe_unused]] auto& leaf_grid,
                     [[maybe_unused]] const CoordT& coord) {
    count++;
  };
  grid.forEachLeafGrid(visitor);
  return count;
}

template <typename B>
size_t MapImpl<B>::customMemUsage() {
  // return getVoxelCount() * (sizeof(CoordT) + sizeof(void*) + sizeof(typename B::Voxel));
  return grid.customMemUsage();
}

template <typename B>
void MapImpl<B>::log_map_stats() {
  size_t memory_usage = customMemUsage();
  RCLCPP_INFO(Logging::logger, "Memory usage: %ld MB",
              memory_usage / 1024 / 1024);
  RCLCPP_INFO(Logging::logger, "Total Voxel count: %ld", getVoxelCount());
  RCLCPP_INFO(Logging::logger, "Leaf grid count: %ld", getLeafGridCount());
  RCLCPP_INFO(Logging::logger, "Inner grid count: %ld", rootMapSize());
}

#include "radix_map/birds_eye_view.hpp"
#include "radix_map/chunking.hpp"
#include "radix_map/ray_hit.hpp"
#include "radix_map/rendering.hpp"
#include "radix_map/rules.hpp"
#include "radix_map/serialize.hpp"
