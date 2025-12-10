#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "message_filters/subscriber.h"
#include "radix_map/map_factory.hpp"
#include "radix_utils/config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"

#include "radix_msgs/msg/ack.hpp"
#include "radix_msgs/srv/birds_eye_view.hpp"
#include "radix_msgs/srv/check.hpp"
#include "radix_msgs/srv/chunk.hpp"
#include "radix_msgs/srv/config.hpp"
#include "radix_msgs/srv/load.hpp"
#include "radix_msgs/srv/ray_hit.hpp"
#include "radix_msgs/srv/render.hpp"
#include "radix_msgs/srv/rules.hpp"
#include "radix_msgs/srv/save.hpp"

using RenderSrv = radix_msgs::srv::Render;
using BirdsEyeViewSrv = radix_msgs::srv::BirdsEyeView;
using ChunkSrv = radix_msgs::srv::Chunk;
using SaveSrv = radix_msgs::srv::Save;
using LoadSrv = radix_msgs::srv::Load;
using CheckSrv = radix_msgs::srv::Check;
using ConfigSrv = radix_msgs::srv::Config;
using RulesSrv = radix_msgs::srv::Rules;
using RayHitSrv = radix_msgs::srv::RayHit;

using Ack = radix_msgs::msg::Ack;

using EmptySrv = std_srvs::srv::Empty;
using sensor_msgs::msg::PointCloud2;

class Server : public rclcpp::Node {
 public:
  Server(const rclcpp::NodeOptions& options);

  // Publishers
  void initPublishers();
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr chunk_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr chunk_free_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<Ack>::SharedPtr ack_pub_;

  // Subscribers
  void initSubscribers();
  message_filters::Subscriber<PointCloud2> point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;

  // TF
  void initTF();
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Features
  void insertCloudCallback(const PointCloud2::ConstSharedPtr cloud);
  void publishAll(const rclcpp::Time& rostime);

  // Services
  void initServices();
  rclcpp::Service<EmptySrv>::SharedPtr reset_srv_;
  bool resetSrv(const std::shared_ptr<EmptySrv::Request> req,
                const std::shared_ptr<EmptySrv::Response> resp);
  rclcpp::Service<EmptySrv>::SharedPtr exit_srv_;
  void exitSrv(const std::shared_ptr<EmptySrv::Request> req,
               const std::shared_ptr<EmptySrv::Response> resp);
  rclcpp::Service<RenderSrv>::SharedPtr render_srv_;
  void renderSrv(const std::shared_ptr<RenderSrv::Request> req,
                 const std::shared_ptr<RenderSrv::Response> resp);
  rclcpp::Service<BirdsEyeViewSrv>::SharedPtr birds_eye_view_srv_;
  void birdsEyeViewSrv(const std::shared_ptr<BirdsEyeViewSrv::Request> req,
                       const std::shared_ptr<BirdsEyeViewSrv::Response> resp);
  rclcpp::Service<SaveSrv>::SharedPtr save_map_srv_;
  void saveMapSrv(const std::shared_ptr<SaveSrv::Request> req,
                  const std::shared_ptr<SaveSrv::Response> resp);
  void saveMapHandler();
  rclcpp::Service<LoadSrv>::SharedPtr load_map_srv_;
  void loadMapSrv(const std::shared_ptr<LoadSrv::Request> req,
                  const std::shared_ptr<LoadSrv::Response> resp);
  rclcpp::Service<ChunkSrv>::SharedPtr chunk_srv_;
  void chunkSrv(const std::shared_ptr<ChunkSrv::Request> req,
                const std::shared_ptr<ChunkSrv::Response> resp);
  rclcpp::Service<CheckSrv>::SharedPtr check_map_srv_;
  void checkMapSrv(const std::shared_ptr<CheckSrv::Request> req,
                   const std::shared_ptr<CheckSrv::Response> resp);
  rclcpp::Service<ConfigSrv>::SharedPtr get_config_srv_;
  void getConfigSrv(const std::shared_ptr<ConfigSrv::Request> req,
                    const std::shared_ptr<ConfigSrv::Response> resp);
  rclcpp::Service<EmptySrv>::SharedPtr publish_srv_;
  void publishSrv(const std::shared_ptr<EmptySrv::Request> req,
                  const std::shared_ptr<EmptySrv::Response> resp);
  rclcpp::Service<RulesSrv>::SharedPtr rules_srv_;
  void rulesSrv(const std::shared_ptr<RulesSrv::Request> req,
                const std::shared_ptr<RulesSrv::Response> resp);

  rclcpp::Service<RayHitSrv>::SharedPtr check_ray_hit_srv_;
  void checkRayHitSrv(const std::shared_ptr<RayHitSrv::Request> req,
                      const std::shared_ptr<RayHitSrv::Response> resp);

  // timers
  void initTimers();
  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void publishCont();

  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  void initCallbacks();

  template <typename T>
  void set_header(T& msg, std::string frame_id = "map") {
    msg.header.frame_id = frame_id;
    msg.header.stamp = now();
  }

  //other
  void initMap();
  void load_map_from_file(std::string filename);

  Config config;
  std::shared_ptr<Map> map;
};
