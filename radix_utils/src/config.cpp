#include "radix_utils/config.hpp"
#include <fstream>
#include "radix_utils/helpers.hpp"
#include "radix_utils/logger.hpp"

void Config::print() const {
  using namespace std;

  RCLCPP_INFO(Logging::logger, "  -------General Config-------");
  RCLCPP_INFO(Logging::logger, "  behavior:               %s",
              behavior.c_str());
  RCLCPP_INFO(Logging::logger, "  resolution:             %f", resolution);

  RCLCPP_INFO(Logging::logger, "  -------Topics-------");
  RCLCPP_INFO(Logging::logger, "  cloud_in_topic:         %s",
              cloud_in_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  cloud_in_apriori_topic: %s",
              cloud_in_apriori_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  point_cloud_centers_topic: %s",
              point_cloud_centers_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  chunk_topic: %s", chunk_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  chunk_free_topic: %s",
              chunk_free_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  image_topic: %s", image_topic.c_str());
  RCLCPP_INFO(Logging::logger, "  ack_topic: %s", ack_topic.c_str());

  RCLCPP_INFO(Logging::logger, "  -------Sensor-------");
  RCLCPP_INFO(Logging::logger, "  world_frame_id:         %s",
              world_frame_id.c_str());
  RCLCPP_INFO(Logging::logger, "  sensor_max_range:        %f",
              sensor_max_range);
  RCLCPP_INFO(Logging::logger, "  monodepth_bias:          %s",
              monodepth_bias ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "  monodepth_bias_thresh:   %f",
              monodepth_bias_thresh);
  RCLCPP_INFO(Logging::logger, "  cleaning_ray:   %s",
              cleaning_ray ? "true" : "false");

  RCLCPP_INFO(Logging::logger, "  -------Load-------");
  RCLCPP_INFO(Logging::logger, "  load_dir:               %s", map_dir.c_str());
  RCLCPP_INFO(Logging::logger, "  load_name:              %s",
              load_map.c_str());
  RCLCPP_INFO(Logging::logger, "  save_extension:         %s",
              save_extension.c_str());

  RCLCPP_INFO(Logging::logger, "  -------Render-------");
  RCLCPP_INFO(Logging::logger, "  render_frame_id: %s",
              render_frame_id.c_str());

  RCLCPP_INFO(Logging::logger, "  -------Other-------");
  RCLCPP_INFO(Logging::logger, "  apply_rules: %s",
              apply_rules ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "  rules_path: %s", rules_path.c_str());
  RCLCPP_INFO(Logging::logger, "  continuous_publish: %s",
              continuous_publish ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "  publish_on_insert: %s",
              publish_on_insert ? "true" : "false");
  RCLCPP_INFO(Logging::logger, "  occupancy_threshold: %f",
              occupancy_threshold);
  RCLCPP_INFO(Logging::logger, "  latched_topics: %s",
              latched_topics ? "true" : "false");

  RCLCPP_INFO(Logging::logger, "  -------Probabilities-------");
  RCLCPP_INFO(Logging::logger, "  prob_init: %f", prob_init);
  RCLCPP_INFO(Logging::logger, "  prob_hit: %f", prob_hit);
  RCLCPP_INFO(Logging::logger, "  prob_miss: %f", prob_miss);
  RCLCPP_INFO(Logging::logger, "  prob_min: %f", prob_min);
  RCLCPP_INFO(Logging::logger, "  prob_max: %f", prob_max);

  RCLCPP_INFO(Logging::logger, "  -------Labels-------");

  std::vector<std::string> formatted_labels = format_labels(labels);

  for (auto& line : formatted_labels) {
    RCLCPP_INFO(Logging::logger, "%s", line.c_str());
  }

  RCLCPP_INFO(Logging::logger, "\033[1;32mREADY!\033[0m");
}

//get config parameters
void Config::getParams(const std::shared_ptr<ConfigSrv::Response>& resp) {
  resp->resolution = resolution;
  resp->behavior = behavior;
  resp->sensor_max_range = sensor_max_range;
  resp->cleaning_ray = cleaning_ray;
  resp->occupancy_threshold = occupancy_threshold;
}
