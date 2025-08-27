#include "gpd_ros2/grasp_messages.hpp"

namespace {

// tiny helpers to copy Eigen -> ROS 2 msgs (no tf needed)
inline geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  return p;
}

inline geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Vector3 vv;
  vv.x = v.x(); vv.y = v.y(); vv.z = v.z();
  return vv;
}

} // anon

namespace gpd_ros2::grasp_messages
{

gpd_ros2_interfaces::msg::GraspConfigList createGraspListMsg(
  const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
  const std_msgs::msg::Header& header)
{
  gpd_ros2_interfaces::msg::GraspConfigList msg;
  msg.header = header;
  msg.grasps.reserve(hands.size());

  for (const auto& h : hands) {
    msg.grasps.emplace_back(convertToGraspMsg(*h));
  }
  return msg;
}

gpd_ros2_interfaces::msg::GraspConfig convertToGraspMsg(
  const gpd::candidate::Hand& hand)
{
  gpd_ros2_interfaces::msg::GraspConfig msg;
  msg.position = toPoint(hand.getPosition());
  msg.approach = toVector3(hand.getApproach());
  msg.binormal = toVector3(hand.getBinormal());
  msg.axis     = toVector3(hand.getAxis());
  msg.width    = static_cast<float>(hand.getGraspWidth());
  msg.score    = static_cast<float>(hand.getScore());
  msg.sample   = toPoint(hand.getSample());
  return msg;
}

} // namespace gpd_ros2::grasp_messages
