#pragma once
#include <cv_bridge/cv_bridge.h>
#include "bonxai/grid_coord.hpp"
#include "radix_map/map.hpp"
#include "radix_map/rendering.hpp"
#include "radix_utils/birds_eye_view_config.hpp"

using Image = sensor_msgs::msg::Image;

template <typename B>
Imgs MapImpl<B>::birds_eye_view(
    const BirdsEyeViewConfig& birds_eye_view_config) {
  float step_size = grid.getResolution();

  int width =
      std::abs(birds_eye_view_config.x_max - birds_eye_view_config.x_min) *
      (1 / step_size);
  int height =
      std::abs(birds_eye_view_config.y_max - birds_eye_view_config.y_min) *
      (1 / step_size);
  float def = 0.0;

  // Create 4 separate images
  cv::Mat img_sem(height, width, CV_8UC3, cv::Scalar(def, def, def));
  cv::Mat img_sem_prob(height, width, CV_32FC1, cv::Scalar(def));
  cv::Mat img_depth(height, width, CV_32FC1, cv::Scalar(def));
  cv::Mat img_depth_prob(height, width, CV_32FC1, cv::Scalar(def));

  cv::parallel_for_(cv::Range(0, height * width), [&](const cv::Range& range) {
    for (int idx = range.start; idx < range.end; ++idx) {
      int y = idx / width;
      int x = idx % width;

      float world_x = birds_eye_view_config.x_min + x * step_size;
      float world_y = birds_eye_view_config.y_min + y * step_size;

      CoordT origin =
          grid.posToCoord(world_x, world_y, birds_eye_view_config.max_height);
      CoordT end =
          grid.posToCoord(world_x, world_y, birds_eye_view_config.min_height);

      PixelValue pixel_value =
          getPixelValue<B>(grid.createAccessor(), origin, end,
                           birds_eye_view_config.occupancy_threshold,
                           birds_eye_view_config.level);

      Point3D real_end_pos = grid.coordToPos(pixel_value.hit_coord);
      float depth = real_end_pos.z - birds_eye_view_config.max_height;

      const LabelEntry& label = config.getLabelEntry(pixel_value.label_id);

      // Set color
      img_sem.at<cv::Vec3b>(y, x)[0] = label.color.r;
      img_sem.at<cv::Vec3b>(y, x)[1] = label.color.g;
      img_sem.at<cv::Vec3b>(y, x)[2] = label.color.b;
      img_sem_prob.at<float>(y, x) = pixel_value.semantic_prob;
      img_depth.at<float>(y, x) = depth;
      img_depth_prob.at<float>(y, x) = pixel_value.depth_prob;
    }
  });

  cv_bridge::CvImage bridge_sem(std_msgs::msg::Header(), "rgb8", img_sem);
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
