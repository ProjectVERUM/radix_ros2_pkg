#pragma once

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include "radix_msgs/srv/config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/labels.hpp"
#include "radix_utils/logger.hpp"

using ConfigSrv = radix_msgs::srv::Config;

class Config {
 public:
  // Constructor
  Config(rclcpp::Node& node) {
    behavior = node.declare_parameter("behavior", std::string("default"));
    resolution = node.declare_parameter("resolution", 0.5);

    // Pointcloud input topics
    cloud_in_topic = node.declare_parameter("cloud_in_topic",
                                            std::string("/fus/pc_label_id"));
    cloud_in_apriori_topic =
        node.declare_parameter("cloud_in_apriori_topic", std::string("/osm"));

    // Frame ids
    world_frame_id =
        node.declare_parameter("world_frame_id", std::string("map"));

    // Sensor model
    sensor_max_range = node.declare_parameter("sensor_model.max_range", -1.0);

    // Monodepth
    monodepth_bias = node.declare_parameter("monodepth_bias", false);
    monodepth_bias_thresh =
        node.declare_parameter("monodepth_bias_thresh", 1.1);
    cleaning_ray = node.declare_parameter("cleaning_ray", true);
    update_id = node.declare_parameter("update_id", false);

    // Loading
    map_dir = node.declare_parameter("map_dir", std::string("maps"));
    load_map = node.declare_parameter("load_map", std::string(""));
    save_extension =
        node.declare_parameter("savefile_extension", std::string(".radix"));

    // Rendering
    render_frame_id = node.declare_parameter("render_frame_id",
                                             std::string("octane_camera0"));

    // Other
    apply_rules = node.declare_parameter("apply_rules", false);
    continuous_publish = node.declare_parameter("cont_pub", false);
    publish_on_insert = node.declare_parameter("publish_on_insert", false);
    occupancy_threshold = node.declare_parameter("occupancy_threshold", 0.5);
    occupancy_threshold_log = logods(occupancy_threshold);
    latched_topics = node.declare_parameter("latched_topics", true);
    delete_rest = node.declare_parameter("delete_rest", false);
    tf_queue_size = node.declare_parameter("tf_queue_size", 20);
    tf_timeout = node.declare_parameter("tf_timeout", 20);
    publish_timer = node.declare_parameter("publish_timer", 10);

    // behavior specific
    store_last = node.declare_parameter("store_last", false);

    prob_init = node.declare_parameter("prob_init", 0.5);
    prob_hit = node.declare_parameter("prob_hit", 0.7);
    prob_miss = node.declare_parameter("prob_miss", 0.4);
    prob_min = node.declare_parameter("prob_min", 0.12);
    prob_max = node.declare_parameter("prob_max", 0.97);

    prob_init_log = logods(prob_init);
    prob_miss_log = logods(prob_miss);
    prob_hit_log = logods(prob_hit);
    prob_min_log = logods(prob_min);
    prob_max_log = logods(prob_max);

    // labels/classes
    label_params_path =
        node.declare_parameter("label_params_path", std::string(""));

    labels = load_labels(label_params_path);

    num_labels = labels.size();

    // rules
    rules_path = node.declare_parameter("rules_path", std::string(""));

    // Publisher topics
    point_cloud_centers_topic = node.declare_parameter(
        "point_cloud_centers_topic", std::string("radix_point_cloud_centers"));
    chunk_topic =
        node.declare_parameter("chunk_topic", std::string("radix_chunk"));
    chunk_free_topic = node.declare_parameter("chunk_free_topic",
                                              std::string("radix_chunk_free"));
    image_topic =
        node.declare_parameter("image_topic", std::string("radix_image"));
    ack_topic = node.declare_parameter("ack_topic", std::string("radix_ack"));
  }

  // Parameters as members
  std::string behavior;
  double resolution;

  std::string cloud_in_topic;
  std::string cloud_in_apriori_topic;

  std::string world_frame_id;

  double sensor_max_range;

  bool monodepth_bias;
  double monodepth_bias_thresh;
  bool cleaning_ray;
  bool update_id;

  std::string map_dir;
  std::string load_map;
  std::string save_extension;

  std::string render_frame_id;

  bool apply_rules;
  bool continuous_publish;
  bool publish_on_insert;
  float occupancy_threshold;
  int32_t occupancy_threshold_log;
  bool latched_topics;
  bool delete_rest;
  bool store_last;
  int32_t tf_queue_size;
  int32_t tf_timeout;
  int32_t publish_timer;

  double prob_init;
  double prob_hit;
  double prob_miss;
  double prob_min;
  double prob_max;

  int32_t prob_init_log;
  int32_t prob_miss_log;
  int32_t prob_hit_log;
  int32_t prob_min_log;
  int32_t prob_max_log;

  std::string label_params_path;
  std::string rules_path;

  uint32_t num_labels;

  std::unordered_map<size_t, LabelEntry> labels;

  std::string point_cloud_centers_topic;
  std::string chunk_topic;
  std::string chunk_free_topic;
  std::string image_topic;
  std::string ack_topic;

  void print() const;

  rcl_interfaces::msg::SetParametersResult updateConfig(
      const std::vector<rclcpp::Parameter>& parameters);
  using ParamUpdater = std::function<void(const rclcpp::Parameter&)>;
  std::unordered_map<std::string, ParamUpdater> param_updaters;
  void getParams(const std::shared_ptr<ConfigSrv::Response>& resp);
  const LabelEntry getLabelEntry(int id) const;

 private:
  bool updateParam(const rclcpp::Parameter& param);
};

inline rcl_interfaces::msg::SetParametersResult Config::updateConfig(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& param : parameters) {
    if (!updateParam(param)) {
      result.successful = false;
    }
  }

  return result;
}

inline const LabelEntry Config::getLabelEntry(int id) const {
  auto it = labels.find(id);
  if (it == labels.end()) {
    RCLCPP_ERROR(Logging::logger, "Label id %d not found", id);
    throw std::runtime_error("Label id " + std::to_string(id) +
                             " not found in Config::getLabelEntry");
  }
  return it->second;
}

inline bool Config::updateParam(const rclcpp::Parameter& param) {
  const std::string& param_name = param.get_name();

  if (param_name == "behavior") {
    behavior = param.as_string();
    return true;
  } else if (param_name == "resolution") {
    resolution = param.as_double();
    return true;
  } else if (param_name == "cloud_in_topic") {
    cloud_in_topic = param.as_string();
    return true;
  } else if (param_name == "cloud_in_apriori_topic") {
    cloud_in_apriori_topic = param.as_string();
    return true;
  } else if (param_name == "world_frame_id") {
    world_frame_id = param.as_string();
    return true;
  } else if (param_name == "sensor_model.max_range") {
    sensor_max_range = param.as_double();
    return true;
  } else if (param_name == "monodepth_bias") {
    monodepth_bias = param.as_bool();
    return true;
  } else if (param_name == "monodepth_bias_thresh") {
    monodepth_bias_thresh = param.as_double();
    return true;
  } else if (param_name == "cleaning_ray") {
    cleaning_ray = param.as_bool();
    return true;
  } else if (param_name == "update_id") {
    update_id = param.as_bool();
    return true;
  } else if (param_name == "map_dir") {
    map_dir = param.as_string();
    return true;
  } else if (param_name == "load_map") {
    load_map = param.as_string();
    return true;
  } else if (param_name == "savefile_extension") {
    save_extension = param.as_string();
    return true;
  } else if (param_name == "render_frame_id") {
    render_frame_id = param.as_string();
    return true;
  } else if (param_name == "apply_rules") {
    apply_rules = param.as_bool();
    return true;
  } else if (param_name == "cont_pub") {
    continuous_publish = param.as_bool();
    return true;
  } else if (param_name == "occupancy_threshold") {
    occupancy_threshold = param.as_double();
    return true;
  } else if (param_name == "latched_topics") {
    latched_topics = param.as_bool();
    return true;
  } else if (param_name == "prob_init") {
    prob_init = param.as_double();
    return true;
  } else if (param_name == "prob_hit") {
    prob_hit = param.as_double();
    return true;
  } else if (param_name == "prob_miss") {
    prob_miss = param.as_double();
    return true;
  } else if (param_name == "prob_min") {
    prob_min = param.as_double();
    return true;
  } else if (param_name == "prob_max") {
    prob_max = param.as_double();
    return true;
  } else if (param_name == "label_params_path") {
    label_params_path = param.as_string();
    return true;
  } else if (param_name == "rules_path") {
    rules_path = param.as_string();
    return true;
  } else if (param_name == "delete_rest") {
    rules_path = param.as_bool();
    return true;
  } else if (param_name == "point_cloud_centers_topic") {
    point_cloud_centers_topic = param.as_string();
    return true;
  } else if (param_name == "chunk_topic") {
    chunk_topic = param.as_string();
    return true;
  } else if (param_name == "chunk_free_topic") {
    chunk_free_topic = param.as_string();
    return true;
  } else if (param_name == "image_topic") {
    image_topic = param.as_string();
    return true;
  } else if (param_name == "ack_topic") {
    ack_topic = param.as_string();
    return true;
  } else {
    return false;
  }
}
