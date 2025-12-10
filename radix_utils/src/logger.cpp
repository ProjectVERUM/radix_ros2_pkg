#include "radix_utils/logger.hpp"
#include <chrono>
#include "radix_utils/helpers.hpp"

namespace Logging {
rclcpp::Logger logger = rclcpp::get_logger("radix_server");

void logTime(std::chrono::high_resolution_clock::time_point start_time,
             std::string message) {
  double total_elapsed =
      std::chrono::duration<double>(getTime() - start_time).count();
  RCLCPP_INFO(logger, "%s took: %f s", message.c_str(), total_elapsed);
};
}  // namespace Logging
