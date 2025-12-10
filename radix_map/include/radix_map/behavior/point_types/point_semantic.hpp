#ifndef POINT_SEMANTIC_HPP
#define POINT_SEMANTIC_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>

struct PointSemantic {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float prob = 0.0f;
  int32_t label = 0;
  float label_prob = 0.0f;
  PCL_ADD_RGB;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointSemantic,
    (float, x, x)(float, y, y)(float, z, z)(float, prob, prob)(
        int32_t, label, label)(float, label_prob, label_prob)(float, rgb, rgb))

#endif  // POINT_SEMANTIC_HPP
