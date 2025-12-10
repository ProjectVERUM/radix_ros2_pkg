#include <rclcpp/rclcpp.hpp>

namespace Logging {
extern rclcpp::Logger logger;
void logTime(std::chrono::high_resolution_clock::time_point start_time,
             std::string message);
}  // namespace Logging
