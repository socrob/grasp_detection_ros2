#pragma once

// std
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <ctime>

// Eigen
#include <Eigen/Core>

// rclcpp / msgs
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// our msgs / helpers
#include "gpd_ros2_interfaces/msg/cloud_indexed.hpp"
#include "gpd_ros2_interfaces/msg/cloud_samples.hpp"
#include "gpd_ros2_interfaces/msg/cloud_sources.hpp"
#include "gpd_ros2_interfaces/msg/grasp_config_list.hpp"
#include "gpd_ros2_interfaces/msg/samples_msg.hpp"

#include "gpd_ros2/grasp_messages.hpp"
#include "gpd_ros2/grasp_plotter.hpp"

namespace gpd_ros2 {

using PointCloudRGBA        = pcl::PointCloud<pcl::PointXYZRGBA>;
using PointCloudPointNormal = pcl::PointCloud<pcl::PointNormal>;

class GraspDetectionNode : public rclcpp::Node {
public:
  GraspDetectionNode();

  void run();  // spins the node (matches ROS1 style)

  // Detect grasp poses in the current cloud
  std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPoses();

private:
  // helpers
  std::vector<int> getSamplesInBall(const PointCloudRGBA::Ptr& cloud,
                                    const pcl::PointXYZRGBA& centroid,
                                    float radius);

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cloudIndexedCallback(const gpd_ros2_interfaces::msg::CloudIndexed::SharedPtr msg);
  void cloudSamplesCallback(const gpd_ros2_interfaces::msg::CloudSamples::SharedPtr msg);
  void samplesCallback(const gpd_ros2_interfaces::msg::SamplesMsg::SharedPtr msg);

  // Build a gpd::util::Cloud from our CloudSources wrapper
  std::unique_ptr<gpd::util::Cloud> createCloudFromSources(const gpd_ros2_interfaces::msg::CloudSources& msg);

  void timerTick();  // replaces ros::Rate loop

  // params / state
  Eigen::Vector3d view_point_{0.0, 0.0, 0.0};

  // Header of the currently processed/last published cloud
  std_msgs::msg::Header cloud_camera_header_;

  // Active cloud ownership pointer and raw convenience alias (used by existing code paths)
  std::unique_ptr<gpd::util::Cloud> cloud_camera_unique_;
  gpd::util::Cloud* cloud_camera_ = nullptr;

  // Queue/history to keep clouds alive (shared ownership) to avoid premature deletion while other subsystems may still hold references.
  static constexpr std::size_t MAX_CLOUD_HISTORY = 3;
  std::deque<std::shared_ptr<gpd::util::Cloud>> cloud_history_;

  // Hand-off buffer between callbacks and processing tick
  std::unique_ptr<gpd::util::Cloud> pending_cloud_;
  std_msgs::msg::Header pending_header_;

  // Concurrency guards
  std::atomic<bool> is_processing_{false};
  std::mutex cloud_mutex_;

  std::unique_ptr<gpd::GraspDetector> grasp_detector_;
  std::unique_ptr<gpd_ros2::GraspPlotter> rviz_plotter_;

  int  size_left_cloud_{0};
  bool has_cloud_{false};
  bool has_normals_{false};
  bool has_samples_{true};
  bool use_importance_sampling_{false};
  bool use_rviz_{false};
  std::vector<double> workspace_;
  std::string frame_;

  // ROS pipes
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_pc2_;
  rclcpp::Subscription<gpd_ros2_interfaces::msg::CloudIndexed>::SharedPtr cloud_sub_indexed_;
  rclcpp::Subscription<gpd_ros2_interfaces::msg::CloudSamples>::SharedPtr cloud_sub_samples_;
  rclcpp::Subscription<gpd_ros2_interfaces::msg::SamplesMsg>::SharedPtr samples_sub_;

  rclcpp::Publisher<gpd_ros2_interfaces::msg::GraspConfigList>::SharedPtr grasps_pub_;

  rclcpp::TimerBase::SharedPtr timer_; // 100 Hz like ROS1

  // input type constants
  static constexpr int POINT_CLOUD_2 = 0; // sensor_msgs/PointCloud2
  static constexpr int CLOUD_INDEXED = 1; // cloud with indices
  static constexpr int CLOUD_SAMPLES = 2; // cloud with (x,y,z) samples
};

} // namespace gpd_ros2