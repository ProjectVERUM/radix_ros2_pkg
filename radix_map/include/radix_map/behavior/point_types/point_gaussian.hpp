#ifndef POINT_GAUSSIAN_HPP
#define POINT_GAUSSIAN_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

struct PointGaussian {
  static constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float prob = 0.0f;
  int32_t label = 0;
  float label_prob = 0.0f;

  float mean_x = 0.0f;
  float mean_y = 0.0f;
  float mean_z = 0.0f;
  float cov_xx = 0.0f;
  float cov_xy = 0.0f;
  float cov_xz = 0.0f;
  float cov_yy = 0.0f;
  float cov_yz = 0.0f;
  float cov_zz = 0.0f;
  int32_t num_points = 0;

  float x0 = NaN;
  float y0 = NaN;
  float z0 = NaN;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointGaussian,
    (float, x, x)(float, y, y)(float, z, z)(float, prob, prob)(
        int32_t, label, label)(float, label_prob, label_prob)(
        float, mean_x, mean_x)(float, mean_y, mean_y)(float, mean_z, mean_z)(
        float, cov_xx, cov_xx)(float, cov_xy, cov_xy)(float, cov_xz, cov_xz)(
        float, cov_yy, cov_yy)(float, cov_yz, cov_yz)(float, cov_zz, cov_zz)(
        int32_t, num_points, num_points)(float, x0, x0)(float, y0, y0)(float,
                                                                       z0, z0))

#endif
