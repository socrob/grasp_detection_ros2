#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

#include <gpd/candidate/hand.h>

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gpd_ros2_interfaces/msg/grasp_config.hpp"
#include "gpd_ros2_interfaces/msg/grasp_config_list.hpp"

namespace gpd_ros2::grasp_messages
{

gpd_ros2_interfaces::msg::GraspConfigList createGraspListMsg(
  const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
  const std_msgs::msg::Header& header);

gpd_ros2_interfaces::msg::GraspConfig convertToGraspMsg(
  const gpd::candidate::Hand& hand);

} // namespace gpd_ros2::grasp_messages
