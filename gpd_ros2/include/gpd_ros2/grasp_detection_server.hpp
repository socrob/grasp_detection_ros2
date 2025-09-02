#pragma once

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <ctime>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

#include "gpd_ros2/grasp_plotter.hpp"
#include "gpd_ros2/grasp_messages.hpp"

#include "gpd_ros2_interfaces/msg/grasp_config_list.hpp"
#include "gpd_ros2_interfaces/msg/cloud_indexed.hpp"
#include "gpd_ros2_interfaces/msg/cloud_sources.hpp"
#include "gpd_ros2_interfaces/srv/detect_grasps.hpp"

namespace gpd_ros2 {

using PointCloudRGBA        = pcl::PointCloud<pcl::PointXYZRGBA>;
using PointCloudPointNormal = pcl::PointCloud<pcl::PointNormal>;

class GraspDetectionServer : public rclcpp::Node {
public:
  GraspDetectionServer();
  ~GraspDetectionServer() override = default;

private:
  // service callback
  void handle_detect_grasps(
    const std::shared_ptr<rmw_request_id_t> /*header*/,
    const std::shared_ptr<gpd_ros2_interfaces::srv::DetectGrasps::Request> req,
    std::shared_ptr<gpd_ros2_interfaces::srv::DetectGrasps::Response> res);

  // helper: init Cloud from CloudSources msg
  void initCloudCamera(const gpd_ros2_interfaces::msg::CloudSources& msg, const std_msgs::msg::Header& header);

  // pubs/srvs
  rclcpp::Publisher<gpd_ros2_interfaces::msg::GraspConfigList>::SharedPtr grasps_pub_;
  rclcpp::Service<gpd_ros2_interfaces::srv::DetectGrasps>::SharedPtr service_;

  // state
  std_msgs::msg::Header cloud_camera_header_;
  std::string frame_;
  std::unique_ptr<gpd::GraspDetector> grasp_detector_;
  
  // Active cloud ownership pointer and raw convenience alias
  std::unique_ptr<gpd::util::Cloud> cloud_camera_unique_;
  gpd::util::Cloud* cloud_camera_ = nullptr;
  
  // Queue/history to keep clouds alive (shared ownership) to avoid premature deletion while other subsystems may still hold references.
  static constexpr std::size_t MAX_CLOUD_HISTORY = 3;
  std::deque<std::shared_ptr<gpd::util::Cloud>> cloud_history_;
  
  std::unique_ptr<gpd_ros2::GraspPlotter> rviz_plotter_;
  bool use_rviz_{false};
  std::vector<double> workspace_;
  Eigen::Vector3d view_point_{0.0, 0.0, 0.0};
};

} // namespace gpd_ros2