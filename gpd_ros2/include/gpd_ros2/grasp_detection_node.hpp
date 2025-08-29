#pragma once

// std
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <ctime>
#include <mutex>
#include <atomic>
#include <utility>
#include <deque>

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspDetectionNode();
  ~GraspDetectionNode() override {
    // No manual delete needed - unique_ptr handles it automatically
    // cloud_camera_ is just a non-owning pointer now
  }

  // Non-copyable (raw pointer member)
  GraspDetectionNode(const GraspDetectionNode&) = delete;
  GraspDetectionNode& operator=(const GraspDetectionNode&) = delete;

  // spins the node (ROS1-style)
  void run();

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
  
  // Helper function to create Cloud from CloudSources
  std::unique_ptr<gpd::util::Cloud> createCloudFromSources(const gpd_ros2_interfaces::msg::CloudSources& msg);

  void timerTick();  // replaces ros::Rate loop

  // params / state
  Eigen::Vector3d view_point_{0.0, 0.0, 0.0};

  std_msgs::msg::Header cloud_camera_header_;
  // FIXED: Use smart pointer for proper memory management
  std::unique_ptr<gpd::util::Cloud> cloud_camera_unique_;
  // Keep raw pointer for compatibility with existing detectGraspPoses method
  gpd::util::Cloud* cloud_camera_ = nullptr;

  // FIXED: Member variables for pending cloud (no global staging)
  std::unique_ptr<gpd::util::Cloud> pending_cloud_;
  std_msgs::msg::Header pending_header_;
  std::mutex cloud_mutex_; // Thread safety for cloud operations
  
  // CRITICAL FIX: Flag to prevent processing while detection is running
  std::atomic<bool> is_processing_{false};
  
  // CRITICAL FIX: Keep a history of clouds to prevent premature deletion
  // GPD may keep internal references beyond the detection phase
  std::deque<std::unique_ptr<gpd::util::Cloud>> cloud_history_;
  static constexpr size_t MAX_CLOUD_HISTORY = 10;  // Increased from 3

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