#include "gpd_ros2/grasp_plotter.hpp"
#include <geometry_msgs/msg/point.hpp>

namespace gpd_ros2 {

GraspPlotter::GraspPlotter(rclcpp::Node& node,
                           const gpd::candidate::HandGeometry& params,
                           const std::string& rviz_topic)
: node_{&node}
{
  if (!rviz_topic.empty()) {
    rviz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        rviz_topic, rclcpp::QoS(1));
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "RViz topic is empty; markers will not be published.");
  }

  hand_depth_     = params.depth_;
  hand_height_    = params.height_;
  outer_diameter_ = params.outer_diameter_;
  finger_width_   = params.finger_width_;
}

void GraspPlotter::drawGrasps(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std::string& frame)
{
  if (!rviz_pub_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "RViz publisher not created; set a non-empty rviz_topic.");
    return;
  }
  auto markers = convertToVisualGraspMsg(hands, frame);
  rviz_pub_->publish(markers);
}

namespace {
inline geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  return p;
}
} // anon

visualization_msgs::msg::MarkerArray GraspPlotter::convertToVisualGraspMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std::string& frame_id)
{
  const double hw = 0.5 * outer_diameter_ - 0.5 * finger_width_;
  visualization_msgs::msg::MarkerArray marker_array;

  for (int i = 0; i < static_cast<int>(hands.size()); i++) {
    const auto& H = *hands[i];

    const Eigen::Vector3d left_bottom  = H.getPosition() - hw * H.getBinormal();
    const Eigen::Vector3d right_bottom = H.getPosition() + hw * H.getBinormal();
    const Eigen::Vector3d left_top     = left_bottom  + hand_depth_ * H.getApproach();
    const Eigen::Vector3d right_top    = right_bottom + hand_depth_ * H.getApproach();
    const Eigen::Vector3d left_center  = left_bottom  + 0.5 * (left_top  - left_bottom);
    const Eigen::Vector3d right_center = right_bottom + 0.5 * (right_top - right_bottom);
    const Eigen::Vector3d base_center  = left_bottom  + 0.5 * (right_bottom - left_bottom) - 0.01 * H.getApproach();
    const Eigen::Vector3d approach_center = base_center - 0.04 * H.getApproach();

    Eigen::Vector3d finger_lwh, approach_lwh;
    finger_lwh   << hand_depth_, finger_width_, hand_height_;
    approach_lwh << 0.08,        finger_width_, hand_height_;

    auto base     = createHandBaseMarker(left_bottom, right_bottom, H.getFrame(), 0.02, hand_height_, i, frame_id);
    auto left_f   = createFingerMarker(left_center,  H.getFrame(), finger_lwh,   i*3,   frame_id);
    auto right_f  = createFingerMarker(right_center, H.getFrame(), finger_lwh,   i*3+1, frame_id);
    auto approach = createFingerMarker(approach_center, H.getFrame(), approach_lwh, i*3+2, frame_id);

    marker_array.markers.push_back(left_f);
    marker_array.markers.push_back(right_f);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}

visualization_msgs::msg::Marker GraspPlotter::createFingerMarker(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& frame,
    const Eigen::Vector3d& lwh,
    int id,
    const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp    = node_->get_clock()->now();

  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = toPoint(center);

  marker.lifetime.sec = 10;
  marker.lifetime.nanosec = 0;

  const Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  marker.scale.x = lwh.x();
  marker.scale.y = lwh.y();
  marker.scale.z = lwh.z();

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  return marker;
}

visualization_msgs::msg::Marker GraspPlotter::createHandBaseMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Matrix3d& frame,
    double length,
    double height,
    int id,
    const std::string& frame_id)
{
  const Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp    = node_->get_clock()->now();

  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = toPoint(center);

  marker.lifetime.sec = 10;
  marker.lifetime.nanosec = 0;

  const Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  marker.scale.x = length;
  marker.scale.y = (end - start).norm();
  marker.scale.z = height;

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  return marker;
}

} // namespace gpd_ros2
