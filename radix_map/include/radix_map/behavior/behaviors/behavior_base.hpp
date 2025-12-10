#pragma once
#include "radix_utils/config.hpp"

class BehaviorBase {
 public:
  BehaviorBase(const Config& config) : config(config) {}

 private:
  const Config& config;
};
