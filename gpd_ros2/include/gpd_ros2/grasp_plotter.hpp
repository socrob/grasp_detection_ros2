#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

namespace gpd_ros2 {

class GraspPlotter {
public:
  // Pass rviz_topic directly; this class does NOT declare parameters.
  GraspPlotter(rclcpp::Node& node,
               const gpd::candidate::HandGeometry& params,
               const std::string& rviz_topic);

  // Publish the markers to RViz (no-op if rviz_topic was empty).
  void drawGrasps(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
                  const std::string& frame);

  // Convert a set of grasps into a MarkerArray (no publish).
  visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
      const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
      const std::string& frame_id);

  visualization_msgs::msg::Marker createFingerMarker(const Eigen::Vector3d& center,
      const Eigen::Matrix3d& frame, const Eigen::Vector3d& lwh, int id,
      const std::string& frame_id);

  visualization_msgs::msg::Marker createHandBaseMarker(const Eigen::Vector3d& start,
      const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length,
      double height, int id, const std::string& frame_id);

private:
  rclcpp::Node* node_{nullptr};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;

  double outer_diameter_{0.0};
  double hand_depth_{0.0};
  double finger_width_{0.0};
  double hand_height_{0.0};
};

} // namespace gpd_ros2
