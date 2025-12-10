#ifndef POINT_BASIC_HPP
#define POINT_BASIC_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>

struct PointBasic {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float prob = 0.0f;
  int32_t label = 0;
  PCL_ADD_RGB;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointBasic, (float, x, x)(float, y, y)(float, z, z)(float, prob, prob)(
                    int32_t, label, label)(float, rgb, rgb))

#endif  // POINT_BASIC_HPP
