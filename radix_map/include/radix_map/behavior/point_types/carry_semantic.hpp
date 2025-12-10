#pragma once
#include <cstdint>

struct CustomPointSemantic {
  double x;
  double y;
  double z;
  // TODO define an instance of CellT here instead of reimplementing all the fileds
  int32_t probability_log;
  int32_t label;
  float label_prob;
  CustomPointSemantic(double x_, double y_, double z_, int probability_log_,
                      int label_, double label_prob_)
      : x(x_),
        y(y_),
        z(z_),
        probability_log(probability_log_),
        label(label_),
        label_prob(label_prob_) {}
};
