#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

static void usage(const char* prog) {
  fprintf(stderr,
    "Usage: %s --file <path.pcd> [--topic /cloud_stitched] [--frame camera_link] [--rate 2.0] [--once]\n",
    prog);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string file, topic = "/cloud_stitched", frame = "camera_link";
  double rate = 2.0;
  bool once = false;

  // Tiny arg parser
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if ((a == "--file" || a == "-f") && i+1 < argc) { file = argv[++i]; }
    else if ((a == "--topic" || a == "-t") && i+1 < argc) { topic = argv[++i]; }
    else if ((a == "--frame" || a == "-F") && i+1 < argc) { frame = argv[++i]; }
    else if ((a == "--rate" || a == "-r") && i+1 < argc) { rate = std::stod(argv[++i]); }
    else if (a == "--once") { once = true; }
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
  }

  if (file.empty()) {
    usage(argv[0]);
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("pcd_publisher");
  auto pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::QoS(10));

  // Load into PCLPointCloud2 and convert: supports any field layout
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile(file, cloud_blob) < 0) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load PCD: %s", file.c_str());
    return 2;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(cloud_blob, cloud_msg);
  cloud_msg.header.frame_id = frame;

  RCLCPP_INFO(node->get_logger(), "Loaded %s (width=%u height=%u fields=%zu) -> topic %s",
              file.c_str(), cloud_msg.width, cloud_msg.height, cloud_msg.fields.size(), topic.c_str());

  if (once) {
    // Single publish then exit
    cloud_msg.header.stamp = node->get_clock()->now();
    pub->publish(cloud_msg);
    rclcpp::spin_some(node);
    // give subscribers a moment to receive
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  } else {
    rclcpp::WallRate loop(rate);
    while (rclcpp::ok()) {
      cloud_msg.header.stamp = node->get_clock()->now();
      pub->publish(cloud_msg);
      rclcpp::spin_some(node);
      loop.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}
