#include "gpd_ros2/grasp_detection_node.hpp"
#include <atomic>
#include <thread>
#include <chrono>

namespace gpd_ros2
{

GraspDetectionNode::GraspDetectionNode()
: rclcpp::Node("detect_grasps")
{
  RCLCPP_INFO(this->get_logger(), "Init ...");
  cloud_camera_ = nullptr;
  cloud_camera_unique_ = nullptr;

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

  // Optional camera position (like ROS1)
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
  rviz_plotter_ = std::make_unique<gpd_ros2::GraspPlotter>(*this, grasp_detector_->getHandSearchParameters().hand_geometry_, rviz_topic);
  use_rviz_ = !rviz_topic.empty();

  // Subscriptions
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
    has_samples_ = false; // matches ROS1 behavior
  }

  // Optional separate samples topic
  if (!samples_topic.empty()) {
    samples_sub_ = this->create_subscription<gpd_ros2_interfaces::msg::SamplesMsg>(
      samples_topic, rclcpp::QoS(1),
      std::bind(&GraspDetectionNode::samplesCallback, this, std::placeholders::_1));
    has_samples_ = false;
  }

  // Publisher
  grasps_pub_ = this->create_publisher<gpd_ros2_interfaces::msg::GraspConfigList>("clustered_grasps", rclcpp::QoS(10));

  // 100 Hz timer (like ros::Rate(100))
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&GraspDetectionNode::timerTick, this)
  );
}

void GraspDetectionNode::run()
{
  RCLCPP_INFO(get_logger(), "Waiting for point cloud to arrive ...");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(get_node_base_interface());
  exec.spin();
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
  // Skip if currently processing
  if (is_processing_.load()) {
    RCLCPP_WARN(this->get_logger(), "Skipping cloud - detection in progress");
    return;
  }
  
  // Build a fresh Cloud directly into member variable (no staging)
  Eigen::Matrix3Xd view_points(3, 1);
  view_points.col(0) = view_point_;

  const bool has_normals =
      (msg->fields.size() == 6 &&
       msg->fields[3].name == "normal_x" &&
       msg->fields[4].name == "normal_y" &&
       msg->fields[5].name == "normal_z");

  // FIXED: Create cloud with DEEP COPIES to avoid shared pointer issues
  std::unique_ptr<gpd::util::Cloud> new_cloud;
  
  if (has_normals)
  {
    auto cloud = std::make_shared<PointCloudPointNormal>();
    pcl::fromROSMsg(*msg, *cloud);
    
    // Create a completely new copy of the cloud to avoid sharing
    auto cloud_copy = std::make_shared<PointCloudPointNormal>();
    pcl::copyPointCloud(*cloud, *cloud_copy);
    
    new_cloud = std::make_unique<gpd::util::Cloud>(cloud_copy, 0, view_points);
    
    RCLCPP_INFO(this->get_logger(),
                "Received cloud with %zu points and normals.",
                new_cloud->getCloudProcessed()->size());
  }
  else
  {
    auto cloud = std::make_shared<PointCloudRGBA>();
    pcl::fromROSMsg(*msg, *cloud);
    
    // Create a completely new copy of the cloud to avoid sharing
    auto cloud_copy = std::make_shared<PointCloudRGBA>();
    pcl::copyPointCloud(*cloud, *cloud_copy);
    
    new_cloud = std::make_unique<gpd::util::Cloud>(cloud_copy, 0, view_points);
    
    RCLCPP_INFO(this->get_logger(),
                "Received cloud with %zu points.",
                new_cloud->getCloudProcessed()->size());
  }

  // FIXED: Directly assign to member variables with proper synchronization
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  pending_cloud_ = std::move(new_cloud);
  pending_header_ = msg->header;
  has_cloud_ = true;
  frame_ = msg->header.frame_id;
}

void GraspDetectionNode::cloudIndexedCallback(const gpd_ros2_interfaces::msg::CloudIndexed::SharedPtr msg)
{
  // Skip if currently processing
  if (is_processing_.load()) {
    RCLCPP_WARN(this->get_logger(), "Skipping cloud - detection in progress");
    return;
  }
  
  // Build a fresh Cloud from sources
  std::unique_ptr<gpd::util::Cloud> new_cloud = createCloudFromSources(msg->cloud_sources);

  // Attach indices to the cloud
  if (new_cloud) {
    std::vector<int> indices(msg->indices.size());
    for (size_t i = 0; i < msg->indices.size(); ++i) {
      indices[i] = static_cast<int>(msg->indices[i]);
    }
    new_cloud->setSampleIndices(indices);
    
    RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points, and %zu samples",
                new_cloud->getCloudProcessed()->size(), msg->indices.size());
  }

  // FIXED: Directly assign to member variables with proper synchronization
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  pending_cloud_ = std::move(new_cloud);
  pending_header_ = msg->cloud_sources.cloud.header;
  has_cloud_ = true;
  frame_ = msg->cloud_sources.cloud.header.frame_id;
}

void GraspDetectionNode::cloudSamplesCallback(const gpd_ros2_interfaces::msg::CloudSamples::SharedPtr msg)
{
  // Skip if currently processing
  if (is_processing_.load()) {
    RCLCPP_WARN(this->get_logger(), "Skipping cloud - detection in progress");
    return;
  }
  
  // Build a fresh Cloud from sources
  std::unique_ptr<gpd::util::Cloud> new_cloud = createCloudFromSources(msg->cloud_sources);

  // Attach sample points
  if (new_cloud) {
    Eigen::Matrix3Xd samples(3, msg->samples.size());
    for (size_t i = 0; i < msg->samples.size(); ++i) {
      samples.col(i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
    }
    new_cloud->setSamples(samples);
    
    RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points, and %ld samples",
                new_cloud->getCloudProcessed()->size(),
                static_cast<long>(new_cloud->getSamples().cols()));
  }

  // FIXED: Directly assign to member variables with proper synchronization
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  pending_cloud_ = std::move(new_cloud);
  pending_header_ = msg->cloud_sources.cloud.header;
  has_cloud_ = true;
  has_samples_ = true;
  frame_ = msg->cloud_sources.cloud.header.frame_id;
}

void GraspDetectionNode::samplesCallback(const gpd_ros2_interfaces::msg::SamplesMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  
  // Only attach to the pending cloud
  if (!pending_cloud_) {
    RCLCPP_WARN(this->get_logger(), "Samples received but no pending cloud available; ignoring samples.");
    return;
  }

  Eigen::Matrix3Xd samples(3, msg->samples.size());
  for (size_t i = 0; i < msg->samples.size(); ++i) {
    samples.col(i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
  }

  pending_cloud_->setSamples(samples);
  has_samples_ = true;

  RCLCPP_INFO(this->get_logger(), "Received grasp samples message with %zu samples", msg->samples.size());
}

std::unique_ptr<gpd::util::Cloud> GraspDetectionNode::createCloudFromSources(const gpd_ros2_interfaces::msg::CloudSources& msg)
{
  // view points matrix
  Eigen::Matrix3Xd view_points(3, msg.view_points.size());
  for (size_t i = 0; i < msg.view_points.size(); ++i) {
    view_points.col(i) << msg.view_points[i].x, msg.view_points[i].y, msg.view_points[i].z;
  }

  // build camera_source one-hot [num_views x num_points] and create Cloud
  if (msg.cloud.fields.size() == 6 &&
      msg.cloud.fields[3].name == "normal_x" &&
      msg.cloud.fields[4].name == "normal_y" &&
      msg.cloud.fields[5].name == "normal_z")
  {
    auto cloud = std::make_shared<PointCloudPointNormal>();
    pcl::fromROSMsg(msg.cloud, *cloud);

    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), static_cast<int>(cloud->size()));
    for (size_t i = 0; i < msg.camera_source.size(); ++i) {
      int cam_idx = static_cast<int>(msg.camera_source[i]);
      if (cam_idx >= 0 && cam_idx < camera_source.rows() && static_cast<int>(i) < camera_source.cols()) {
        camera_source(cam_idx, static_cast<int>(i)) = 1;
      }
    }

    return std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
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

    return std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
  }
}

void GraspDetectionNode::timerTick()
{
  // Skip if already processing
  if (is_processing_.load()) {
    return;
  }
  
  // Check if we have a new cloud to process
  std::unique_ptr<gpd::util::Cloud> cloud_to_process;
  std_msgs::msg::Header header_to_process;
  
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    // Only act when a new cloud is available and has samples (if needed)
    if (!has_cloud_ || !pending_cloud_) {
      return;
    }
    
    // If we're waiting for samples but don't have them yet, return
    if (!has_samples_) {
      return;
    }
    
    // Move the pending cloud to local variable for processing
    cloud_to_process = std::move(pending_cloud_);
    header_to_process = pending_header_;
    has_cloud_ = false; // Reset flag
    // NOTE: has_samples_ stays true unless explicitly set false by callbacks
  }
  
  // Mark as processing BEFORE updating cloud pointers
  is_processing_.store(true);
  
  // CRITICAL FIX: Keep a history of clouds to prevent premature deletion
  // Move current cloud to history before replacing it
  if (cloud_camera_unique_) {
    cloud_history_.push_back(std::move(cloud_camera_unique_));
    
    // Keep only the last few clouds in history
    while (cloud_history_.size() > MAX_CLOUD_HISTORY) {
      cloud_history_.pop_front();
    }
    
    cloud_camera_ = nullptr;
  }
  
  // Now safely set the new cloud
  cloud_camera_unique_ = std::move(cloud_to_process);
  cloud_camera_ = cloud_camera_unique_.get();
  cloud_camera_header_ = header_to_process;

  try {
    // Run detection on this cloud
    auto grasps = detectGraspPoses();
    if (use_rviz_) {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error during grasp detection: %s", e.what());
  }

  // Mark processing as complete
  is_processing_.store(false);
  
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