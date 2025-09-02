#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cctype>
#include <cstdint>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <gpd_ros2_interfaces/msg/cloud_indexed.hpp>
#include <gpd_ros2_interfaces/msg/cloud_sources.hpp>
#include <gpd_ros2_interfaces/srv/detect_grasps.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class PcdServiceClient : public rclcpp::Node {
public:
  PcdServiceClient()
  : Node("pcd_service_client")
  {
    // --- params ---
    pcd_file_     = this->declare_parameter<std::string>("pcd_file", "");
    frame_id_     = this->declare_parameter<std::string>("frame", "map");
    service_name_ = this->declare_parameter<std::string>("service_name", "/detect_grasps");
    repeat_       = this->declare_parameter<bool>("repeat", false);
    period_ms_    = this->declare_parameter<int>("period_ms", 1000);
    num_indices_  = this->declare_parameter<int>("num_indices", 500);
    view_px_      = this->declare_parameter<double>("view_point_x", 0.0);
    view_py_      = this->declare_parameter<double>("view_point_y", 0.0);
    view_pz_      = this->declare_parameter<double>("view_point_z", 0.0);
    
    // NEW: Control how many grasps to show in detail
    show_detailed_ = this->declare_parameter<int>("show_detailed", 3);
    show_summary_  = this->declare_parameter<bool>("show_summary", true);

    if (pcd_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'pcd_file' is empty.");
      throw std::runtime_error("pcd_file is required");
    }

    // Load point cloud once
    if (!loadCloud(pcd_file_, cloud_msg_)) {
      RCLCPP_FATAL(get_logger(), "Failed to load point cloud from '%s'", pcd_file_.c_str());
      throw std::runtime_error("failed to load pcd/ply");
    }
    cloud_msg_.header.frame_id = frame_id_;

    // Build CloudSources
    sources_.cloud = cloud_msg_;
    geometry_msgs::msg::Point vp;
    vp.x = view_px_; vp.y = view_py_; vp.z = view_pz_;
    sources_.view_points.clear();
    sources_.view_points.push_back(vp);

    const size_t N = static_cast<size_t>(cloud_msg_.width) * static_cast<size_t>(cloud_msg_.height);
    sources_.camera_source.assign(N, 0u);  // single camera index = 0

    client_ = this->create_client<gpd_ros2_interfaces::srv::DetectGrasps>(service_name_);

    if (repeat_) {
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        std::bind(&PcdServiceClient::tick, this));
    } else {
      // One-shot: schedule tick after construction (avoids shared_from_this in ctor)
      oneshot_timer_ = this->create_wall_timer(
        10ms,
        [this](){
          if (oneshot_done_) return;
          oneshot_done_ = true;
          oneshot_timer_->cancel();
          tick();  // async; shutdown in response callback
        }
      );
    }
  }

private:
  void tick() {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Service '%s' not available yet...", service_name_.c_str());
      return;
    }

    auto req = std::make_shared<gpd_ros2_interfaces::srv::DetectGrasps::Request>();

    gpd_ros2_interfaces::msg::CloudIndexed ci;
    ci.cloud_sources = sources_;
    ci.indices = makeStrideIndices(
      static_cast<size_t>(cloud_msg_.width) * static_cast<size_t>(cloud_msg_.height),
      static_cast<size_t>(num_indices_)
    );
    req->cloud_indexed = std::move(ci);

    RCLCPP_INFO(get_logger(), "Calling service with %zu indices (cloud has %ux%u = %u points)",
                req->cloud_indexed.indices.size(),
                cloud_msg_.width, cloud_msg_.height,
                cloud_msg_.width * cloud_msg_.height);

    // Always async; handle response in callback
    client_->async_send_request(
      req,
      std::bind(&PcdServiceClient::onResponse, this, std::placeholders::_1)
    );
  }

  using Detect       = gpd_ros2_interfaces::srv::DetectGrasps;
  using SharedFuture = rclcpp::Client<Detect>::SharedFuture;

  void onResponse(SharedFuture f) {
    auto res = f.get();
    handleResponse(res);
    if (!repeat_) {
      // For one-shot mode, exit after first response
      rclcpp::shutdown();
    }
  }

  void handleResponse(const std::shared_ptr<Detect::Response>& res) {
    const auto& list = res->grasp_configs;
    
    RCLCPP_INFO(this->get_logger(),
                "\n========== GRASP DETECTION RESULTS ==========");
    RCLCPP_INFO(this->get_logger(),
                "Detected %zu grasps in frame '%s'",
                list.grasps.size(),
                list.header.frame_id.c_str());
    
    if (list.grasps.empty()) {
      RCLCPP_WARN(this->get_logger(), "No grasps detected!");
      return;
    }

    // Show summary statistics
    if (show_summary_) {
      printSummary(list);
    }

    // Show detailed information for top grasps
    const size_t show_detailed = std::min<size_t>(
      static_cast<size_t>(show_detailed_), 
      list.grasps.size()
    );
    
    if (show_detailed > 0) {
      RCLCPP_INFO(this->get_logger(),
                  "\n--- TOP %zu GRASP DETAILS ---", show_detailed);
      
      for (size_t i = 0; i < show_detailed; ++i) {
        printGraspDetails(list.grasps[i], i + 1);
      }
    }

    RCLCPP_INFO(this->get_logger(), 
                "============================================\n");
  }

  void printSummary(const gpd_ros2_interfaces::msg::GraspConfigList& list) {
    // Calculate statistics
    float min_score = std::numeric_limits<float>::max();
    float max_score = std::numeric_limits<float>::lowest();
    float avg_score = 0.0f;
    
    for (const auto& grasp : list.grasps) {
      min_score = std::min(min_score, grasp.score);
      max_score = std::max(max_score, grasp.score);
      avg_score += grasp.score;
    }
    avg_score /= static_cast<float>(list.grasps.size());

    RCLCPP_INFO(this->get_logger(),
                "\n--- SUMMARY STATISTICS ---");
    RCLCPP_INFO(this->get_logger(),
                "Score range: %.4f to %.4f (avg: %.4f)",
                min_score, max_score, avg_score);
    
    // Count grasps by score ranges
    size_t excellent = 0, good = 0, fair = 0, poor = 0;
    for (const auto& grasp : list.grasps) {
      if (grasp.score >= 0.8) excellent++;
      else if (grasp.score >= 0.6) good++;
      else if (grasp.score >= 0.4) fair++;
      else poor++;
    }
    
    RCLCPP_INFO(this->get_logger(),
                "Quality distribution: %zu excellent (≥0.8), %zu good (≥0.6), %zu fair (≥0.4), %zu poor (<0.4)",
                excellent, good, fair, poor);
  }

  void printGraspDetails(const gpd_ros2_interfaces::msg::GraspConfig& grasp, size_t rank) {
    const auto& pos = grasp.position;
    const auto& approach = grasp.approach;
    const auto& binormal = grasp.binormal;
    const auto& axis = grasp.axis;

    RCLCPP_INFO(this->get_logger(),
                "\n  GRASP #%zu (Score: %.4f):", rank, grasp.score);
    
    // Position
    RCLCPP_INFO(this->get_logger(),
                "    Position:    [%7.4f, %7.4f, %7.4f]",
                pos.x, pos.y, pos.z);
    
    // Direction vectors (R = [approach binormal axis])
    RCLCPP_INFO(this->get_logger(),
                "    Approach:    [%7.4f, %7.4f, %7.4f]",
                approach.x, approach.y, approach.z);
    
    RCLCPP_INFO(this->get_logger(),
                "    Binormal:    [%7.4f, %7.4f, %7.4f]",
                binormal.x, binormal.y, binormal.z);
    
    RCLCPP_INFO(this->get_logger(),
                "    Axis:        [%7.4f, %7.4f, %7.4f]",
                axis.x, axis.y, axis.z);
    
    // Grasp width and sample point
    RCLCPP_INFO(this->get_logger(),
                "    Width:       %.4f m", grasp.width);
    
    RCLCPP_INFO(this->get_logger(),
                "    Sample:      [%7.4f, %7.4f, %7.4f]", 
                grasp.sample.x, grasp.sample.y, grasp.sample.z);

    // Calculate and show derived information
    printDerivedInfo(grasp);
  }

  void printDerivedInfo(const gpd_ros2_interfaces::msg::GraspConfig& grasp) {
    const auto& pos = grasp.position;
    const auto& approach = grasp.approach;
    const auto& binormal = grasp.binormal;
    const auto& axis = grasp.axis;
    
    // Distance from origin
    double distance = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
    RCLCPP_INFO(this->get_logger(),
                "    Distance from origin: %.4f m", distance);
    
    // Calculate approach angle relative to vertical (z-axis)
    double approach_length = std::sqrt(approach.x * approach.x + approach.y * approach.y + approach.z * approach.z);
    if (approach_length > 0.001) {  // Avoid division by zero
      double approach_angle = std::acos(std::abs(approach.z) / approach_length) * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(),
                  "    Approach angle from vertical: %.2f°", approach_angle);
    }
    
    // Calculate if the grasp vectors are orthonormal (they should be)
    double approach_mag = std::sqrt(approach.x * approach.x + approach.y * approach.y + approach.z * approach.z);
    double binormal_mag = std::sqrt(binormal.x * binormal.x + binormal.y * binormal.y + binormal.z * binormal.z);
    double axis_mag = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    
    RCLCPP_INFO(this->get_logger(),
                "    Vector magnitudes: approach=%.4f, binormal=%.4f, axis=%.4f",
                approach_mag, binormal_mag, axis_mag);
    
    // Calculate dot products to check orthogonality
    double dot_ab = approach.x * binormal.x + approach.y * binormal.y + approach.z * binormal.z;
    double dot_aa = approach.x * axis.x + approach.y * axis.y + approach.z * axis.z;
    double dot_ba = binormal.x * axis.x + binormal.y * axis.y + binormal.z * axis.z;
    
    RCLCPP_INFO(this->get_logger(),
                "    Orthogonality check: app·bin=%.4f, app·axis=%.4f, bin·axis=%.4f (should be ~0)",
                dot_ab, dot_aa, dot_ba);
  }

  static std::vector<int64_t> makeStrideIndices(size_t total, size_t target) {
    std::vector<int64_t> out;
    if (total == 0) return out;
    if (target == 0 || target >= total) {
      out.resize(total);
      for (size_t i=0;i<total;++i) out[i] = static_cast<int64_t>(i);
      return out;
    }
    const size_t stride = std::max<size_t>(1, total / target);
    out.reserve(std::min(target, total));
    for (size_t i=0; i<total; i+=stride) {
      out.push_back(static_cast<int64_t>(i));
      if (out.size() >= target) break;
    }
    return out;
  }

  static bool loadCloud(const std::string& path, sensor_msgs::msg::PointCloud2& out) {
    pcl::PCLPointCloud2 pcl_cloud;
    const auto lower = toLower(path);
    int ret = -1;
    if (endsWith(lower, ".pcd")) {
      ret = pcl::io::loadPCDFile(path, pcl_cloud);
    } else if (endsWith(lower, ".ply")) {
      ret = pcl::io::loadPLYFile(path, pcl_cloud);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("pcd_service_client"),
                   "Unsupported file extension in '%s' (use .pcd or .ply)", path.c_str());
      return false;
    }
    if (ret < 0) return false;
    pcl_conversions::moveFromPCL(pcl_cloud, out);
    out.header.stamp = rclcpp::Clock().now();
    return true;
  }

  static std::string toLower(std::string s) {
    for (auto& c: s) c = static_cast<char>(::tolower(c));
    return s;
  }
  static bool endsWith(const std::string& s, const std::string& suf) {
    if (s.size() < suf.size()) return false;
    return std::equal(suf.rbegin(), suf.rend(), s.rbegin());
  }

private:
  // params
  std::string pcd_file_;
  std::string frame_id_;
  std::string service_name_;
  bool repeat_{false};
  int period_ms_{1000};
  int num_indices_{500};
  double view_px_{0.0}, view_py_{0.0}, view_pz_{0.0};
  
  // NEW: Display control parameters
  int show_detailed_{3};   // How many grasps to show in detail
  bool show_summary_{true}; // Whether to show summary statistics

  // cached data
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  gpd_ros2_interfaces::msg::CloudSources sources_;

  // comms
  rclcpp::Client<gpd_ros2_interfaces::srv::DetectGrasps>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr oneshot_timer_;
  bool oneshot_done_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PcdServiceClient>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_service_client"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}