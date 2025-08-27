#include "gpd_ros2/grasp_detection_node.hpp"

namespace gpd_ros2
{

GraspDetectionNode::GraspDetectionNode()
: rclcpp::Node("detect_grasps")
{
  RCLCPP_INFO(this->get_logger(), "Init ...");
  cloud_camera_.reset();

  // Parameters (declared with defaults)
  auto cfg_file    = this->declare_parameter<std::string>("config_file", "");
  auto cloud_type  = this->declare_parameter<int>("cloud_type", POINT_CLOUD_2);
  auto cloud_topic = this->declare_parameter<std::string>("cloud_topic", "/camera/depth_registered/points");
  auto samples_topic = this->declare_parameter<std::string>("samples_topic", "");
  auto rviz_topic  = this->declare_parameter<std::string>("rviz_topic", "plot_grasps");
  this->declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
  this->get_parameter("workspace", workspace_);
  this->declare_parameter<bool>("use_importance_sampling", false);
  this->get_parameter("use_importance_sampling", use_importance_sampling_);

  // (Optional) camera position param (kept as in ROS1, default origin)
  if (this->has_parameter("camera_position")) {
    std::vector<double> camera_position;
    if (this->get_parameter("camera_position", camera_position) && camera_position.size() >= 3) {
      view_point_ << camera_position[0], camera_position[1], camera_position[2];
    }
  }

  // GPD detector
  grasp_detector_ = std::make_unique<gpd::GraspDetector>(cfg_file);
  RCLCPP_INFO(this->get_logger(), "Created GPD with config: '%s'", cfg_file.c_str());

  // RViz plotter (publishes if rviz_topic non-empty)
  rviz_plotter_ = std::make_unique<gpd_ros2::GraspPlotter>(*this, grasp_detector_->getHandSearchParameters().hand_geometry_, rviz_topic);
  use_rviz_ = !rviz_topic.empty();

  // Subscriptions (by cloud_type)
  if (cloud_type == POINT_CLOUD_2) {
    cloud_sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(1),
      std::bind(&GraspDetectionNode::cloudCallback, this, std::placeholders::_1));
  } else if (cloud_type == CLOUD_INDEXED) {
    cloud_sub_indexed_ = this->create_subscription<gpd_ros2_interfaces::msg::CloudIndexed>(
      cloud_topic, rclcpp::QoS(1),
      std::bind(&GraspDetectionNode::cloudIndexedCallback, this, std::placeholders::_1));
  } else if (cloud_type == CLOUD_SAMPLES) {
    cloud_sub_samples_ = this->create_subscription<gpd_ros2_interfaces::msg::CloudSamples>(
      cloud_topic, rclcpp::QoS(1),
      std::bind(&GraspDetectionNode::cloudSamplesCallback, this, std::placeholders::_1));
    has_samples_ = false; // as in ROS1
  }

  // Optional separate samples topic
  if (!samples_topic.empty()) {
    samples_sub_ = this->create_subscription<gpd_ros2_interfaces::msg::SamplesMsg>(
      samples_topic, rclcpp::QoS(1),
      std::bind(&GraspDetectionNode::samplesCallback, this, std::placeholders::_1));
    has_samples_ = false;
  }

  // Publisher for clustered grasps
  grasps_pub_ = this->create_publisher<gpd_ros2_interfaces::msg::GraspConfigList>("clustered_grasps", rclcpp::QoS(10));

  // 100 Hz timer (like ros::Rate(100) loop)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&GraspDetectionNode::timerTick, this)
  );
}

void GraspDetectionNode::run()
{
  RCLCPP_INFO(this->get_logger(), "Waiting for point cloud to arrive ...");
  rclcpp::spin(shared_from_this());
}

std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspDetectionNode::detectGraspPoses()
{
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

  if (use_importance_sampling_) {
    RCLCPP_ERROR(this->get_logger(), "Importance sampling is not supported yet.");
  } else {
    // preprocess & detect
    grasp_detector_->preprocessPointCloud(*cloud_camera_);
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  }

  // Publish selected grasps
  auto selected = gpd_ros2::grasp_messages::createGraspListMsg(grasps, cloud_camera_header_);
  grasps_pub_->publish(selected);
  RCLCPP_INFO(this->get_logger(), "Published %zu highest-scoring grasps.", selected.grasps.size());

  return grasps;
}

std::vector<int> GraspDetectionNode::getSamplesInBall(const PointCloudRGBA::Ptr& cloud,
                                                      const pcl::PointXYZRGBA& centroid,
                                                      float radius)
{
  std::vector<int> indices;
  std::vector<float> dists;
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);
  kdtree.radiusSearch(centroid, radius, indices, dists);
  return indices;
}

void GraspDetectionNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (has_cloud_) return;

  cloud_camera_.reset();

  Eigen::Matrix3Xd view_points(3, 1);
  view_points.col(0) = view_point_;

  // If normals present
  if (msg->fields.size() == 6 &&
      msg->fields[3].name == "normal_x" &&
      msg->fields[4].name == "normal_y" &&
      msg->fields[5].name == "normal_z")
  {
    auto cloud = std::make_shared<PointCloudPointNormal>();
    pcl::fromROSMsg(*msg, *cloud);
    cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, 0, view_points);
    cloud_camera_header_ = msg->header;
    RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points and normals.",
                cloud_camera_->getCloudProcessed()->size());
  }
  else
  {
    auto cloud = std::make_shared<PointCloudRGBA>();
    pcl::fromROSMsg(*msg, *cloud);
    cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, 0, view_points);
    cloud_camera_header_ = msg->header;
    RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points.",
                cloud_camera_->getCloudProcessed()->size());
  }

  has_cloud_ = true;
  frame_ = msg->header.frame_id;
}

void GraspDetectionNode::cloudIndexedCallback(const gpd_ros2_interfaces::msg::CloudIndexed::SharedPtr msg)
{
  if (has_cloud_) return;

  initCloudCamera(msg->cloud_sources);

  // indices are int64[] in our ROS2 msg; convert to int
  std::vector<int> indices(msg->indices.size());
  for (size_t i = 0; i < msg->indices.size(); ++i) {
    indices[i] = static_cast<int>(msg->indices[i]);
  }
  cloud_camera_->setSampleIndices(indices);

  has_cloud_ = true;
  frame_ = msg->cloud_sources.cloud.header.frame_id;

  RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points, and %zu samples",
              cloud_camera_->getCloudProcessed()->size(), msg->indices.size());
}

void GraspDetectionNode::cloudSamplesCallback(const gpd_ros2_interfaces::msg::CloudSamples::SharedPtr msg)
{
  if (has_cloud_) return;

  initCloudCamera(msg->cloud_sources);

  // Set sample points
  Eigen::Matrix3Xd samples(3, msg->samples.size());
  for (size_t i = 0; i < msg->samples.size(); ++i) {
    samples.col(i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
  }
  cloud_camera_->setSamples(samples);

  has_cloud_ = true;
  has_samples_ = true;
  frame_ = msg->cloud_sources.cloud.header.frame_id;

  RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points, and %ld samples",
              cloud_camera_->getCloudProcessed()->size(),
              static_cast<long>(cloud_camera_->getSamples().cols()));
}

void GraspDetectionNode::samplesCallback(const gpd_ros2_interfaces::msg::SamplesMsg::SharedPtr msg)
{
  if (has_samples_) return;

  Eigen::Matrix3Xd samples(3, msg->samples.size());
  for (size_t i = 0; i < msg->samples.size(); ++i) {
    samples.col(i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
  }

  cloud_camera_->setSamples(samples);
  has_samples_ = true;

  RCLCPP_INFO(this->get_logger(), "Received grasp samples message with %zu samples",
              msg->samples.size());
}

void GraspDetectionNode::initCloudCamera(const gpd_ros2_interfaces::msg::CloudSources& msg)
{
  cloud_camera_.reset();

  // view points matrix
  Eigen::Matrix3Xd view_points(3, msg.view_points.size());
  for (size_t i = 0; i < msg.view_points.size(); ++i) {
    view_points.col(i) << msg.view_points[i].x, msg.view_points[i].y, msg.view_points[i].z;
  }

  // cloud with or without normals
  if (msg.cloud.fields.size() == 6 &&
      msg.cloud.fields[3].name == "normal_x" &&
      msg.cloud.fields[4].name == "normal_y" &&
      msg.cloud.fields[5].name == "normal_z")
  {
    auto cloud = std::make_shared<PointCloudPointNormal>();
    pcl::fromROSMsg(msg.cloud, *cloud);

    // one-hot camera source matrix [num_views x num_points]
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), static_cast<int>(cloud->size()));
    for (size_t i = 0; i < msg.camera_source.size(); ++i) {
      int cam_idx = static_cast<int>(msg.camera_source[i]);
      if (cam_idx >= 0 && cam_idx < camera_source.rows() && static_cast<int>(i) < camera_source.cols()) {
        camera_source(cam_idx, static_cast<int>(i)) = 1;
      }
    }

    cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
  }
  else
  {
    auto cloud = std::make_shared<PointCloudRGBA>();
    pcl::fromROSMsg(msg.cloud, *cloud);

    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), static_cast<int>(cloud->size()));
    for (size_t i = 0; i < msg.camera_source.size(); ++i) {
      int cam_idx = static_cast<int>(msg.camera_source[i]);
      if (cam_idx >= 0 && cam_idx < camera_source.rows() && static_cast<int>(i) < camera_source.cols()) {
        camera_source(cam_idx, static_cast<int>(i)) = 1;
      }
    }

    cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
  }
}

void GraspDetectionNode::timerTick()
{
  if (!has_cloud_) return;

  // detect
  auto grasps = detectGraspPoses();

  // visualize
  if (use_rviz_) {
    rviz_plotter_->drawGrasps(grasps, frame_);
  }

  // reset
  has_cloud_ = false;
  has_samples_ = false;
  has_normals_ = false;

  RCLCPP_INFO(this->get_logger(), "Waiting for point cloud to arrive ...");
}

} // namespace gpd_ros2

// ---------------------- main ----------------------
int main(int argc, char** argv)
{
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gpd_ros2::GraspDetectionNode>();
  node->run();  // spins
  rclcpp::shutdown();
  return 0;
}
