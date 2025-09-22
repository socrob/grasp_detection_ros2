#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <optional>

using namespace std::chrono_literals;

class ObjectCloudToGPD : public rclcpp::Node {
public:
  ObjectCloudToGPD()
  : Node("object_cloud_to_gpd") {
    // Parameters
    input_topic_  = declare_parameter<std::string>("input_topic", "/mask2pcl/object_cloud"); // from mask2pcl
    output_topic_ = declare_parameter<std::string>("output_topic", "/gpd/input_cloud"); // GPD default
    frame_override_ = declare_parameter<std::string>("frame_override", ""); // empty -> keep incoming
    repeat_rate_hz_ = declare_parameter<double>("repeat_rate_hz", 0.0); // 0 -> no repeating

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&ObjectCloudToGPD::onCloud, this, std::placeholders::_1));

    if (repeat_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / repeat_rate_hz_);
      repeat_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ObjectCloudToGPD::repeatPublish, this));
      RCLCPP_INFO(get_logger(), "Repeating enabled at %.2f Hz", repeat_rate_hz_);
    }

    RCLCPP_INFO(get_logger(), "Forwarding '%s' -> '%s'%s",
                input_topic_.c_str(), output_topic_.c_str(),
                frame_override_.empty() ? "" : (std::string{" with frame='"} + frame_override_ + "'").c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_msg_ = *msg; // copy
    // Optionally override frame
    if (!frame_override_.empty()) {
      last_msg_->header.frame_id = frame_override_;
    }
    // Always stamp with current time like pcd_publisher does
    last_msg_->header.stamp = now();

    pub_->publish(*last_msg_);

    if (!first_published_) {
      first_published_ = true;
      RCLCPP_INFO(get_logger(), "Received first cloud (w=%u h=%u, fields=%zu). Forwarded to %s",
                  last_msg_->width, last_msg_->height, last_msg_->fields.size(), output_topic_.c_str());
    }
  }

  void repeatPublish() {
    if (!last_msg_.has_value()) return;
    last_msg_->header.stamp = now();
    pub_->publish(*last_msg_);
  }

  rclcpp::Time now() { return get_clock()->now(); }

  // Params
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_override_;
  double repeat_rate_hz_{};

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr repeat_timer_;

  // State
  std::optional<sensor_msgs::msg::PointCloud2> last_msg_;
  bool first_published_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectCloudToGPD>());
  rclcpp::shutdown();
  return 0;
}
