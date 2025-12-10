#pragma once
#include <cv_bridge/cv_bridge.h>
#include "bonxai/grid_coord.hpp"
#include "radix_map/map.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/render_config.hpp"

using Image = sensor_msgs::msg::Image;

Eigen::Vector3d computeRayDirectionCam(int x, int y, int width, int height,
                                       float fov_x, float fov_y, float aspect) {
  float ndc_x = (x + 0.5f) / width * 2.0f - 1.0f;
  float ndc_y = (y + 0.5f) / height * 2.0f - 1.0f;
  float px = ndc_x * std::tan(fov_x / 2.0f) * aspect;
  float py = ndc_y * std::tan(fov_y / 2.0f);
  float pz = 1.0f;
  // -X forward, like rviz
  Eigen::Vector3d dir(-pz, px, -py);
  dir.normalize();
  return dir;
}

Eigen::Vector3d applyCameraRotation(const Eigen::Vector3d& dir,
                                    const RenderConfig& c) {
  // Create quaternions for each rotation
  Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_pitch(
      Eigen::AngleAxisd(c.pitch, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond q_roll(
      Eigen::AngleAxisd(c.roll, Eigen::Vector3d::UnitX()));

  // Compose them: (order matters! usually yaw * pitch * roll)
  Eigen::Quaterniond q = q_yaw * q_pitch * q_roll;

  return q * dir;
}

Point3D computeEndpoint(const RenderConfig& c,
                        const Eigen::Vector3d& ray_dir_world) {
  return Point3D{c.x + c.max_range * ray_dir_world.x(),
                 c.y + c.max_range * ray_dir_world.y(),
                 c.z + c.max_range * ray_dir_world.z()};
}

struct PixelValue {
  size_t label_id = 0;
  float semantic_prob = 0.0;
  float depth_prob = 0.0;
  CoordT hit_coord = CoordT{0, 0, 0};
};

template <typename B>
inline PixelValue getPixelValue(
    typename VoxelGrid<typename B::Voxel>::Accessor accessor, CoordT start,
    CoordT end, float occupancy_threshold, std::string level) {

  PixelValue pixel_value;
  pixel_value.hit_coord = end;

  auto fun = [&accessor, &occupancy_threshold, &pixel_value,
              &level](const Bonxai::CoordT& coord) {
    if (typename B::Voxel* cell = accessor.value(coord, false);
        cell && cell->probability_log > logods(occupancy_threshold)) {

      // auto tmp = get_max_class(cell->classmap);
      if (level == "cell") {
        pixel_value.label_id = cell->label;
      } else {
        pixel_value.label_id = accessor.getMetaData(coord, level)->max_class;
      }

      //TODO: semantic prob
      pixel_value.semantic_prob = 0.5;
      pixel_value.hit_coord = coord;
      pixel_value.depth_prob = prob(cell->probability_log);
      return false;
    }

    return true;
  };

  RayIterator(start, end, fun);

  return pixel_value;
}

template <typename B>
Imgs MapImpl<B>::render(const RenderConfig& render_config) {

  CoordT origin =
      grid.posToCoord(render_config.x, render_config.y, render_config.z);
  float def = 0.0;

  cv::Mat img_sem;
  if (render_config.use_rgb) {
    img_sem = cv::Mat(render_config.height, render_config.width, CV_8UC3,
                      cv::Scalar(def, def, def));
  } else {
    img_sem = cv::Mat(render_config.height, render_config.width, CV_8UC1,
                      cv::Scalar(def));
  }
  cv::Mat img_sem_prob(render_config.height, render_config.width, CV_32FC1,
                       cv::Scalar(def));
  cv::Mat img_depth(render_config.height, render_config.width, CV_32FC1,
                    cv::Scalar(def));
  cv::Mat img_depth_prob(render_config.height, render_config.width, CV_32FC1,
                         cv::Scalar(def));

  float aspect = render_config.width / float(render_config.height);

  cv::parallel_for_(
      cv::Range(0, render_config.height * render_config.width),
      [&](const cv::Range& range) {
        for (int idx = range.start; idx < range.end; ++idx) {
          int y = idx / render_config.width;
          int x = idx % render_config.width;

          Eigen::Vector3d ray_dir_cam = computeRayDirectionCam(
              x, y, render_config.width, render_config.height,
              render_config.fov_x, render_config.fov_y, aspect);
          Eigen::Vector3d ray_dir_world =
              applyCameraRotation(ray_dir_cam, render_config);
          Point3D end_point = computeEndpoint(render_config, ray_dir_world);

          CoordT end = grid.posToCoord(end_point);

          PixelValue pixel_value = getPixelValue<B>(
              grid.createAccessor(), origin, end,
              render_config.occupancy_threshold, render_config.level);

          Point3D real_end_pos = grid.coordToPos(pixel_value.hit_coord);
          float depth = sqrt(pow(real_end_pos.x - render_config.x, 2) +
                             pow(real_end_pos.y - render_config.y, 2) +
                             pow(real_end_pos.z - render_config.z, 2));

          LabelEntry label = config.getLabelEntry(pixel_value.label_id);

          if (render_config.use_rgb) {
            img_sem.at<cv::Vec3b>(y, x)[0] = label.color.r;
            img_sem.at<cv::Vec3b>(y, x)[1] = label.color.g;
            img_sem.at<cv::Vec3b>(y, x)[2] = label.color.b;
          } else {
            img_sem.at<uint8_t>(y, x) = pixel_value.label_id;
          }

          img_sem_prob.at<float>(y, x) = pixel_value.semantic_prob;
          img_depth.at<float>(y, x) = depth;
          img_depth_prob.at<float>(y, x) = pixel_value.depth_prob;
        }
      });

  // create 4 channel image
  cv_bridge::CvImage bridge_sem;
  if (render_config.use_rgb) {
    bridge_sem = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img_sem);
  } else {
    bridge_sem = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_sem);
  }
  cv_bridge::CvImage bridge_sem_prob(std_msgs::msg::Header(), "32FC1",
                                     img_sem_prob);
  cv_bridge::CvImage bridge_depth(std_msgs::msg::Header(), "32FC1", img_depth);
  cv_bridge::CvImage bridge_depth_prob(std_msgs::msg::Header(), "32FC1",
                                       img_depth_prob);

  Image msg_sem, msg_sem_prob, msg_depth, msg_depth_prob;
  bridge_sem.toImageMsg(msg_sem);
  bridge_sem_prob.toImageMsg(msg_sem_prob);
  bridge_depth.toImageMsg(msg_depth);
  bridge_depth_prob.toImageMsg(msg_depth_prob);
  return Imgs{msg_sem, msg_sem_prob, msg_depth, msg_depth_prob};
}
