#include "server.hpp"
#include "radix_map/map.hpp"
#include "radix_map/rendering.hpp"
#include "radix_map/serialize.hpp"
#include "radix_utils/birds_eye_view_config.hpp"
#include "radix_utils/chunk_config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/logger.hpp"
#include "radix_utils/ray_config.hpp"
#include "radix_utils/render_config.hpp"
#include "radix_utils/rules_config.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

Server::Server(const rclcpp::NodeOptions& options)
    : rclcpp::Node("MapServer", options), config(*this) {

  config.print();

  initPublishers();
  initTF();
  initSubscribers();

  initTimers();
  initServices();

  initCallbacks();

  initMap();
}

void Server::initPublishers() {
  rclcpp::QoS qos =
      config.latched_topics ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
  point_cloud_pub_ =
      create_publisher<PointCloud2>(config.point_cloud_centers_topic, qos);
  chunk_pub_ = create_publisher<PointCloud2>(config.chunk_topic, qos);
  chunk_free_pub_ = create_publisher<PointCloud2>(config.chunk_free_topic, qos);
  image_pub_ =
      create_publisher<sensor_msgs::msg::Image>(config.image_topic, qos);
  ack_pub_ = create_publisher<Ack>(config.ack_topic, qos);
}

void Server::initSubscribers() {
  using std::chrono_literals::operator""s;
  point_cloud_sub_.subscribe(this, config.cloud_in_topic,
                             rmw_qos_profile_sensor_data);

  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      point_cloud_sub_, *tf2_buffer_, config.world_frame_id,
      config.tf_queue_size, this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      std::chrono::seconds(config.tf_timeout));

  tf_point_cloud_sub_->registerCallback(&Server::insertCloudCallback, this);
}

void Server::initTimers() {
  pub_timer_ = create_wall_timer(std::chrono::seconds(config.publish_timer),
                                 std::bind(&Server::publishCont, this));
}

void Server::initCallbacks() {
  on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&Server::paramCallback, this, _1));
}

rcl_interfaces::msg::SetParametersResult Server::paramCallback(
    const std::vector<rclcpp::Parameter>& parameters) {

  rcl_interfaces::msg::SetParametersResult resp =
      config.updateConfig(parameters);
  RCLCPP_INFO(Logging::logger, "Config Updated");
  config.print();
  return resp;
}

void Server::initServices() {

  reset_srv_ = create_service<EmptySrv>(
      "~/reset", std::bind(&Server::resetSrv, this, _1, _2));

  // exit_srv_ = create_service<EmptySrv>(
  //     "~/exit", std::bind(&Server::exitSrv, this, _1, _2));

  save_map_srv_ = create_service<SaveSrv>(
      "~/save", std::bind(&Server::saveMapSrv, this, _1, _2));

  load_map_srv_ = create_service<LoadSrv>(
      "~/load", std::bind(&Server::loadMapSrv, this, _1, _2));

  render_srv_ = create_service<RenderSrv>(
      "~/render", std::bind(&Server::renderSrv, this, _1, _2));

  birds_eye_view_srv_ = create_service<BirdsEyeViewSrv>(
      "~/birds_eye_view", std::bind(&Server::birdsEyeViewSrv, this, _1, _2));

  chunk_srv_ = create_service<ChunkSrv>(
      "~/chunk", std::bind(&Server::chunkSrv, this, _1, _2));

  get_config_srv_ = create_service<ConfigSrv>(
      "~/config", std::bind(&Server::getConfigSrv, this, _1, _2));
  publish_srv_ = create_service<EmptySrv>(
      "~/publish", std::bind(&Server::publishSrv, this, _1, _2));

  rules_srv_ = create_service<RulesSrv>(
      "~/rules", std::bind(&Server::rulesSrv, this, _1, _2));

  check_ray_hit_srv_ = create_service<RayHitSrv>(
      "~/ray_hit", std::bind(&Server::checkRayHitSrv, this, _1, _2));

  // check_map_srv_ = create_service<CheckSrv>(
  //   "~/check", std::bind(&Server::checkMapSrv, this, _1, _2));
}

void Server::initTF() {
  std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface());

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_buffer_->setCreateTimerInterface(timer_interface);

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void Server::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud) {

  auto start_time = getTime();

  map->insertCloud(cloud, tf2_buffer_);

  Logging::logTime(start_time, "INSERTING");

  Ack ack;
  ack.stamp = cloud->header.stamp;
  ack_pub_->publish(ack);

  if (config.publish_on_insert) {
    auto start_pub_time = getTime();
    publishAll(cloud->header.stamp);
    Logging::logTime(start_pub_time, "PUBLISHING");
  }
}

inline void Server::publishAll(const rclcpp::Time& rostime) {

  bool publish_point_cloud =
      (config.latched_topics ||
       point_cloud_pub_->get_subscription_count() +
               point_cloud_pub_->get_intra_process_subscription_count() >
           0);
  if (publish_point_cloud) {
    PointCloud2 cloud = map->publishAll();

    cloud.header.frame_id = config.world_frame_id;
    cloud.header.stamp = rostime;
    point_cloud_pub_->publish(cloud);
  }
}

bool Server::resetSrv(const std::shared_ptr<EmptySrv::Request>,
                      const std::shared_ptr<EmptySrv::Response>) {
  const auto rostime = now();
  map = createMapFromConfig(config.resolution, config.behavior, config);

  tf_point_cloud_sub_.reset();
  tf2_listener_.reset();
  tf2_buffer_.reset();

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  using std::chrono_literals::operator""s;
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      point_cloud_sub_, *tf2_buffer_, config.world_frame_id,
      config.tf_queue_size, this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      std::chrono::seconds(config.tf_timeout));

  tf_point_cloud_sub_->registerCallback(&Server::insertCloudCallback, this);

  RCLCPP_INFO(get_logger(),
              "Full reset complete - Bonxai, TF buffer, and message filter "
              "reinitialized");
  publishAll(rostime);

  return true;
}

void Server::renderSrv(const std::shared_ptr<RenderSrv::Request> req,
                       const std::shared_ptr<RenderSrv::Response> resp) {

  RCLCPP_INFO(Logging::logger, "Rendering requested");

  RenderConfig render_config = RenderConfig{req->x,
                                            req->y,
                                            req->z,
                                            req->roll,
                                            req->pitch,
                                            req->yaw,
                                            req->max_range,
                                            static_cast<int>(req->width),
                                            static_cast<int>(req->height),
                                            req->yaw_angle,
                                            req->pitch_angle,
                                            req->level,
                                            req->live,
                                            req->publish,
                                            req->occupancy_threshold,
                                            req->use_rgb};

  ValidationResult val = render_config.validate();
  if (!val.success) {
    resp->success = val.success;
    resp->reason = val.reason;
    return;
  }

  render_config.print();

  auto start_time = getTime();
  Imgs imgs = map->render(render_config);

  resp->semantic_img = imgs.semantic_img;
  set_header(resp->semantic_img, config.world_frame_id);
  resp->semantic_prob_img = imgs.semantic_prob_img;
  set_header(resp->semantic_prob_img, config.world_frame_id);
  resp->depth_img = imgs.depth_img;
  set_header(resp->depth_img, config.world_frame_id);
  resp->depth_prob_img = imgs.depth_prob_img;
  set_header(resp->depth_prob_img, config.world_frame_id);

  resp->success = true;
  Logging::logTime(start_time, "RENDERING");

  if (req->publish) {
    RCLCPP_INFO(Logging::logger, "Image publishing not implemented yet");
  }
}

void Server::chunkSrv(const std::shared_ptr<ChunkSrv::Request> req,
                      const std::shared_ptr<ChunkSrv::Response> resp) {

  ChunkConfig chunk_config = ChunkConfig{req->x_min,   req->x_max,
                                         req->y_min,   req->y_max,
                                         req->z_min,   req->z_max,
                                         req->free,    req->delete_rest,
                                         req->publish, req->occupancy_threshold,
                                         req->level};

  ValidationResult val = chunk_config.validate();
  if (!val.success) {
    resp->success = val.success;
    resp->reason = val.reason;
    return;
  }

  chunk_config.print();

  auto start_time = getTime();
  PointCloud2 chunk_cloud = map->chunk(chunk_config);

  chunk_cloud.header.frame_id = config.world_frame_id;
  chunk_cloud.header.stamp = now();
  if (chunk_config.publish) {
    chunk_pub_->publish(chunk_cloud);
  }
  resp->chunk = chunk_cloud;
  resp->success = true;

  Logging::logTime(start_time, "CHUNKING");
}

void Server::birdsEyeViewSrv(
    const std::shared_ptr<BirdsEyeViewSrv::Request> req,
    const std::shared_ptr<BirdsEyeViewSrv::Response> resp) {

  BirdsEyeViewConfig birds_eye_view_config =
      BirdsEyeViewConfig{req->x_min,
                         req->x_max,
                         req->y_min,
                         req->y_max,
                         req->min_height,
                         req->max_height,
                         req->occupancy_threshold,
                         req->level};

  ValidationResult val = birds_eye_view_config.validate();
  if (!val.success) {
    resp->success = val.success;
    resp->reason = val.reason;
    return;
  }

  birds_eye_view_config.print();

  auto start_time = getTime();
  Imgs imgs = map->birds_eye_view(birds_eye_view_config);

  resp->semantic_img = imgs.semantic_img;
  set_header(resp->semantic_img, config.world_frame_id);
  resp->semantic_prob_img = imgs.semantic_prob_img;
  set_header(resp->semantic_prob_img, config.world_frame_id);
  resp->depth_img = imgs.depth_img;
  set_header(resp->depth_img, config.world_frame_id);
  resp->depth_prob_img = imgs.depth_prob_img;
  set_header(resp->depth_prob_img, config.world_frame_id);

  resp->success = true;
  Logging::logTime(start_time, "BIRDS_EYE_VIEW RENDERING");
}

void Server::rulesSrv(const std::shared_ptr<RulesSrv::Request> req,
                      const std::shared_ptr<RulesSrv::Response> resp) {

  RCLCPP_INFO(Logging::logger, "Applying Rules from loaded config");

  RulesConfig rules_config = load_rules(config.rules_path);
  Region region = Region{req->x_min, req->x_max, req->y_min,
                         req->y_max, req->z_min, req->z_max};

  rules_config.print();
  region.print();
  RCLCPP_INFO(Logging::logger, "Applying Label Dilation");
  auto start_time = getTime();

  for (const auto& ksize : rules_config.dilation.kernel_size) {
    map->traverseChunk(ksize, rules_config.dilation.update_thres, region);
    RCLCPP_INFO(Logging::logger, "a %dx%d kernel applied successfully", ksize,
                ksize);
    // map->save_map("");
  }

  Logging::logTime(start_time, "Label Dilation");

  RCLCPP_INFO(Logging::logger, "All rules are applied!");

  resp->success = true;
}

void Server::publishSrv(
    [[maybe_unused]] const std::shared_ptr<EmptySrv::Request> req,
    [[maybe_unused]] const std::shared_ptr<EmptySrv::Response> resp) {
  RCLCPP_INFO(Logging::logger, "Publishing requested");
  auto start_pub_time = getTime();

  publishAll(now());

  Logging::logTime(start_pub_time, "PUBLISHING");
}

void Server::publishCont() {
  if (config.continuous_publish) {
    const auto rostime = now();
    map->log_map_stats();

    publishAll(rostime);
  }
}

void Server::saveMapSrv(const std::shared_ptr<SaveSrv::Request> req,
                        const std::shared_ptr<SaveSrv::Response>) {
  if (req->filename == "") {
    RCLCPP_WARN(Logging::logger, "No path provided, not saving map!!!");
  } else {

    map->save_map(req->filename);
  }
}

void Server::load_map_from_file(std::string filename) {
  if (filename == "") {
    RCLCPP_WARN(Logging::logger, "No path provided, not loading map!!!");
  } else {
    auto path = std::filesystem::path(config.map_dir) /
                (filename + config.save_extension);
    HeaderInfo info = get_header_info(path);
    config.behavior = info.behavior_name;
    config.resolution = info.resolution;
    RCLCPP_INFO(Logging::logger, "Set behavior to %s", config.behavior.c_str());
    RCLCPP_INFO(Logging::logger, "Set resolution to %f", config.resolution);
    map = createMapFromConfig(config.resolution, config.behavior, config);
    RCLCPP_INFO(Logging::logger, "Loading map from %s ...", path.c_str());
    map->load_grid(path);
    RCLCPP_INFO(Logging::logger, "Loaded map from %s", path.c_str());
    map->log_map_stats();
  }
}

void Server::loadMapSrv(const std::shared_ptr<LoadSrv::Request> req,
                        const std::shared_ptr<LoadSrv::Response>) {
  load_map_from_file(req->filename);
}

void Server::initMap() {

  if (config.load_map == "") {
    map = createMapFromConfig(config.resolution, config.behavior, config);
  } else {
    load_map_from_file(config.load_map);
  }
}

void Server::getConfigSrv(const std::shared_ptr<ConfigSrv::Request>,
                          const std::shared_ptr<ConfigSrv::Response> resp) {

  RCLCPP_INFO(Logging::logger, "Config parameters requested");
  config.getParams(resp);
}

void Server::checkRayHitSrv(const std::shared_ptr<RayHitSrv::Request> req,
                            const std::shared_ptr<RayHitSrv::Response> resp) {

  RCLCPP_INFO(Logging::logger, "CheckRayHit requested");
  RayConfig ray_config = RayConfig{req->x_start,
                                   req->y_start,
                                   req->z_start,
                                   req->x_end,
                                   req->y_end,
                                   req->z_end,
                                   req->occupancy_threshold};

  ray_config.print();

  RayHit ray_hit = map->checkRayHit(ray_config);
  resp->hit = ray_hit.hit;
  resp->x = ray_hit.x;
  resp->y = ray_hit.y;
  resp->z = ray_hit.z;
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(Server)
