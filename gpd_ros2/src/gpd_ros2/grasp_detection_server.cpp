#include "gpd_ros2/grasp_detection_server.hpp"

namespace gpd_ros2 {

GraspDetectionServer::GraspDetectionServer()
: rclcpp::Node("detect_grasps_server")
{
  // Initialize cloud pointers
  cloud_camera_unique_ = nullptr;
  cloud_camera_ = nullptr;

  // Declare & get parameters (mirrors ROS1)
  auto cfg_file = this->declare_parameter<std::string>("config_file", "");
  auto rviz_topic = this->declare_parameter<std::string>("rviz_topic", "");
  this->declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
  this->get_parameter("workspace", workspace_);

  // Optional camera position param (default origin)
  if (this->has_parameter("camera_position")) {
    std::vector<double> camera_position;
    if (this->get_parameter("camera_position", camera_position) && camera_position.size() >= 3) {
      view_point_ << camera_position[0], camera_position[1], camera_position[2];
    }
  }

  // GPD detector
  grasp_detector_ = std::make_unique<gpd::GraspDetector>(cfg_file);
  RCLCPP_INFO(this->get_logger(), "Created GPD with config: '%s'", cfg_file.c_str());

  // RViz plotter
  rviz_plotter_ = std::make_unique<gpd_ros2::GraspPlotter>(
  *this, grasp_detector_->getHandSearchParameters().hand_geometry_, rviz_topic);
  use_rviz_ = !rviz_topic.empty();

  // Publisher for detected grasps
  grasps_pub_ = this->create_publisher<gpd_ros2_interfaces::msg::GraspConfigList>("clustered_grasps", rclcpp::QoS(10));

  // Service
  service_ = this->create_service<gpd_ros2_interfaces::srv::DetectGrasps>(
    "detect_grasps",
    std::bind(&GraspDetectionServer::handle_detect_grasps, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(this->get_logger(), "Grasp detection service ready: '%s/detect_grasps'", this->get_fully_qualified_name());
}

void GraspDetectionServer::handle_detect_grasps(
  const std::shared_ptr<rmw_request_id_t> /*header*/,
  const std::shared_ptr<gpd_ros2_interfaces::srv::DetectGrasps::Request> req,
  std::shared_ptr<gpd_ros2_interfaces::srv::DetectGrasps::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Received service request ...");

  // 1) Initialize Cloud with memory management
  initCloudCamera(req->cloud_indexed.cloud_sources, req->cloud_indexed.cloud_sources.cloud.header);
  frame_ = req->cloud_indexed.cloud_sources.cloud.header.frame_id;

  // Set the indices to sample at (int64[] -> int)
  std::vector<int> indices(req->cloud_indexed.indices.size());
  for (size_t i = 0; i < req->cloud_indexed.indices.size(); ++i) {
    indices[i] = static_cast<int>(req->cloud_indexed.indices[i]);
  }
  cloud_camera_->setSampleIndices(indices);

  RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points, and %zu samples",
              cloud_camera_->getCloudProcessed()->size(), req->cloud_indexed.indices.size());

  // 2) Preprocess
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3) Detect grasps
  auto grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (!grasps.empty()) {
    // Visualize
    if (use_rviz_) {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }
    // Build response and publish same list on topic
    auto selected = gpd_ros2::grasp_messages::createGraspListMsg(grasps, cloud_camera_header_);
    res->grasp_configs = selected;
    grasps_pub_->publish(selected);

    RCLCPP_INFO(this->get_logger(), "Detected %zu highest-scoring grasps.", selected.grasps.size());
  } else {
    RCLCPP_WARN(this->get_logger(), "No grasps detected!");
  }
}

void GraspDetectionServer::initCloudCamera(
  const gpd_ros2_interfaces::msg::CloudSources& msg,
  const std_msgs::msg::Header& header)
{
  cloud_camera_header_ = header;

  // CRITICAL FIX: Keep a history of clouds to prevent premature deletion
  // Move current cloud to history before replacing it
  if (cloud_camera_unique_) {
    // Transfer ownership from unique_ptr -> shared_ptr so the object stays alive
    auto keep = std::shared_ptr<gpd::util::Cloud>(std::move(cloud_camera_unique_));
    
    // Keep a non-owning raw pointer for convenience.
    // This remains valid as long as `keep` (now in history) stays alive.
    cloud_camera_ = keep.get();
    
    // Retain a few recent clouds to guarantee lifetime across service calls.
    cloud_history_.push_back(std::move(keep));
    while (cloud_history_.size() > MAX_CLOUD_HISTORY) {
      cloud_history_.pop_front();
    }
  }

  // View points
  Eigen::Matrix3Xd view_points(3, msg.view_points.size());
  for (size_t i = 0; i < msg.view_points.size(); ++i) {
    view_points.col(i) << msg.view_points[i].x, msg.view_points[i].y, msg.view_points[i].z;
  }

  // With normals?
  const bool has_normals =
      (msg.cloud.fields.size() == 6) &&
      (msg.cloud.fields[3].name == "normal_x") &&
      (msg.cloud.fields[4].name == "normal_y") &&
      (msg.cloud.fields[5].name == "normal_z");

  std::unique_ptr<gpd::util::Cloud> new_cloud;

  if (has_normals) {
    auto cloud = std::make_shared<PointCloudPointNormal>();
    pcl::fromROSMsg(msg.cloud, *cloud);

    // Create a completely new copy of the cloud to avoid sharing
    auto cloud_copy = std::make_shared<PointCloudPointNormal>();
    pcl::copyPointCloud(*cloud, *cloud_copy);

    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), static_cast<int>(cloud_copy->size()));
    for (size_t i = 0; i < msg.camera_source.size(); ++i) {
      const int cam_idx = static_cast<int>(msg.camera_source[i]);
      if (cam_idx >= 0 && cam_idx < camera_source.rows() && static_cast<int>(i) < camera_source.cols()) {
        camera_source(cam_idx, static_cast<int>(i)) = 1;
      }
    }

    new_cloud = std::make_unique<gpd::util::Cloud>(cloud_copy, camera_source, view_points);
  } else {
    auto cloud = std::make_shared<PointCloudRGBA>();
    pcl::fromROSMsg(msg.cloud, *cloud);

    // Create a completely new copy of the cloud to avoid sharing
    auto cloud_copy = std::make_shared<PointCloudRGBA>();
    pcl::copyPointCloud(*cloud, *cloud_copy);

    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), static_cast<int>(cloud_copy->size()));
    for (size_t i = 0; i < msg.camera_source.size(); ++i) {
      const int cam_idx = static_cast<int>(msg.camera_source[i]);
      if (cam_idx >= 0 && cam_idx < camera_source.rows() && static_cast<int>(i) < camera_source.cols()) {
        camera_source(cam_idx, static_cast<int>(i)) = 1;
      }
    }

    new_cloud = std::make_unique<gpd::util::Cloud>(cloud_copy, camera_source, view_points);
  }

  // Now safely set the new cloud
  cloud_camera_unique_ = std::move(new_cloud);
  cloud_camera_ = cloud_camera_unique_.get();
}

} // namespace gpd_ros2

// ---------------- main ----------------
int main(int argc, char** argv)
{
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gpd_ros2::GraspDetectionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}