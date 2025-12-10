#pragma once

#include <pcl/point_types.h>
#include "behavior/behaviors/behavior_basic.hpp"
#include "behavior/behaviors/behavior_gaussian.hpp"
#include "behavior/behaviors/behavior_semantic.hpp"
#include "map.hpp"

inline std::shared_ptr<Map> createMapFromConfig(double res,
                                                std::string configType,
                                                Config& config) {
  if (configType == "basic") {
    return std::make_shared<MapImpl<BehaviorBasic>>(res, config);
  } else if (configType == "gaussian") {
    return std::make_shared<MapImpl<BehaviorGaussian>>(res, config);
  } else if (configType == "semantic") {
    return std::make_shared<MapImpl<BehaviorSemantic>>(res, config);
  } else {
    throw std::runtime_error("Invalid config_type parameter");
  }
}
