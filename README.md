# GPD ROS2 - Grasp Pose Detection for ROS2

A ROS2 implementation of Grasp Pose Detection (GPD) for robotic grasping applications. This package provides both streaming and service-based interfaces for detecting 6-DOF grasp poses from point cloud data.

## Overview

This project ports the [GPD library](https://github.com/atenpas/gpd) to ROS2, providing:
- **Real-time grasp detection** from point cloud streams
- **Service-based grasp detection** for on-demand processing
- **Multiple point cloud input formats** (PointCloud2, indexed clouds, sampled clouds)
- **RViz visualization** of detected grasps
- **Comprehensive testing tools** for development and validation

## Architecture

The project consists of two main packages:

### `gpd_ros2_interfaces`
Custom ROS2 message and service definitions:
- **Messages**: `GraspConfig`, `GraspConfigList`, `CloudSources`, `CloudIndexed`, `CloudSamples`
- **Services**: `DetectGrasps`

### `gpd_ros2` 
Core implementation with multiple nodes and tools:
- **Streaming Node**: Continuous grasp detection from point cloud topics
- **Service Node**: On-demand grasp detection via ROS2 services
- **Testing Tools**: PCD file processing and point cloud publishing utilities

## Prerequisites

### System Dependencies
- **ROS2 Humble** (or compatible distribution)
- **PCL** (Point Cloud Library)
- **Eigen3**
- **OpenCV** (for GPD)

### GPD Library Installation

The GPD library must be installed separately:

```bash
# Clone and build GPD
git clone https://github.com/atenpas/gpd.git
cd gpd
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Ensure library path is available
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
```

### ROS2 Dependencies
```bash
# Install ROS2 dependencies
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-sensor-msgs
```

## Installation

```bash
# Create workspace
mkdir -p ~/grasp_ws/src
cd ~/grasp_ws/src

# Clone this repository
git clone <repository-url> grasp_detection_ros2

# Build the workspace
cd ~/grasp_ws
colcon build --packages-select gpd_ros2_interfaces gpd_ros2

# Source the workspace
source install/setup.bash
```

## Quick Start

### 1. Launch Grasp Detection Service
```bash
# Start the grasp detection server
ros2 launch gpd_ros2 grasp_detection_server.launch.py

# Or use the convenience script
./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection_server.sh
```

### 2. Test with Sample Data
```bash
# Test grasp detection with a sample PCD file
./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_service_client.sh

# Test with custom PCD file
./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_service_client.sh /path/to/your/file.pcd
```

## Available Nodes

### 1. Grasp Detection Node (`detect_grasps`)
**Streaming grasp detection from point cloud topics.**

```bash
ros2 launch gpd_ros2 grasp_detection.launch.py \
  config_file:=/path/to/gpd/config.cfg \
  cloud_topic:=/camera/depth_registered/points
```

**Features:**
- Processes point clouds at ~100Hz
- Publishes detected grasps to `/clustered_grasps`
- Supports multiple input formats (PointCloud2, CloudIndexed, CloudSamples)
- Optional RViz visualization
- Memory-safe processing with automatic cleanup

### 2. Grasp Detection Server (`detect_grasps_server`)
**Service-based grasp detection for on-demand processing.**

A synchronous service interface for grasp detection that processes point clouds on request rather than continuously. Ideal for integration with manipulation pipelines that need precise control over when detection occurs.

#### Quick Start
```bash
# Basic server launch
ros2 launch gpd_ros2 grasp_detection_server.launch.py

# Custom configuration with visualization
ros2 launch gpd_ros2 grasp_detection_server.launch.py \
  config_file:=/path/to/your/config.cfg \
  rviz_topic:=/grasp_markers

# Use convenience script with production settings
./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection_server.sh \
  --auto-restart --config-file /path/to/production.cfg
```

#### Architecture & Features

**Request-Response Pipeline:**
1. **Service Request Reception**: Receives CloudIndexed messages with point cloud and sample indices
2. **Memory-Safe Processing**: Same cloud history buffer as streaming node for stability
3. **GPD Execution**: Runs full preprocessing and detection pipeline synchronously
4. **Dual Output**: Returns results via service response AND publishes to topic

**Key Features:**
- **Synchronous Processing**: Blocks until detection completes, ensuring deterministic behavior
- **CloudIndexed Input**: Accepts pre-processed clouds with specific sample points
- **Dual Publication**: Results available via both service response and topic subscription
- **Memory Management**: Same robust cloud lifecycle management as streaming node
- **RViz Integration**: Optional visualization of detected grasps

#### Service Interface

**Service Definition:**
```yaml
# gpd_ros2_interfaces/srv/DetectGrasps.srv

# Request
gpd_ros2_interfaces/CloudIndexed cloud_indexed
  gpd_ros2_interfaces/CloudSources cloud_sources
    sensor_msgs/PointCloud2 cloud           # Input point cloud
    geometry_msgs/Point[] view_points        # Camera viewpoints
    uint32[] camera_source                   # Camera source mapping
  int64[] indices                            # Sample point indices

---
# Response  
gpd_ros2_interfaces/GraspConfigList grasp_configs
  std_msgs/Header header
  gpd_ros2_interfaces/GraspConfig[] grasps   # Detected grasp poses
```

**Service Usage:**
```bash
# Check service availability
ros2 service list | grep detect_grasps

# Manual service call (for testing)
ros2 service call /detect_grasps gpd_ros2_interfaces/srv/DetectGrasps "{...}"

# Service info
ros2 service type /detect_grasps
```

#### Parameters

**Core Parameters:**
- `config_file` (string): Path to GPD configuration file
- `rviz_topic` (string): RViz marker topic (empty disables visualization)

**Advanced Parameters:**
- `workspace` (double[]): Spatial bounds for detection
- `camera_position` (double[]): Default camera viewpoint `[x,y,z]`
- `use_asan` (bool): Enable AddressSanitizer for memory debugging

#### Topics

**Published Topics:**
- `/clustered_grasps` (`gpd_ros2_interfaces/GraspConfigList`): Detected grasp poses (same as service response)
- `{rviz_topic}` (`visualization_msgs/MarkerArray`): RViz visualization (optional)

**Service:**
- `/detect_grasps` (`gpd_ros2_interfaces/srv/DetectGrasps`): Main detection service

#### Memory Management

The server implements the same memory safety mechanisms as the streaming node:

**Cloud History Buffer:**
- Maintains `MAX_CLOUD_HISTORY = 3` recent clouds in shared ownership
- Prevents premature deletion during GPD processing
- Deep copying of PCL data structures to avoid pointer conflicts

**Service Call Isolation:**
- Each service call processes independently
- No interference between concurrent requests (though processing is sequential)
- Automatic cleanup after each detection cycle

#### Performance Characteristics

- **Latency**: 100-500ms per request depending on cloud size and complexity
- **Throughput**: ~2-10 requests/second depending on point cloud density
- **Memory**: ~3x cloud size overhead for history buffer
- **Concurrency**: Sequential processing (one request at a time)

#### Usage Examples

**Python Service Client:**
```python
import rclpy
from gpd_ros2_interfaces.srv import DetectGrasps
from gpd_ros2_interfaces.msg import CloudIndexed, CloudSources

# Create service client
client = node.create_client(DetectGrasps, '/detect_grasps')
client.wait_for_service()

# Prepare request
request = DetectGrasps.Request()
request.cloud_indexed = CloudIndexed()
request.cloud_indexed.cloud_sources = your_cloud_sources
request.cloud_indexed.indices = your_sample_indices

# Call service
future = client.call_async(request)
response = future.result()

# Process results
if response.grasp_configs.grasps:
    print(f"Detected {len(response.grasp_configs.grasps)} grasps")
    best_grasp = response.grasp_configs.grasps[0]  # Highest scoring
```

**C++ Service Client:**
```cpp
#include "gpd_ros2_interfaces/srv/detect_grasps.hpp"

auto client = node->create_client<gpd_ros2_interfaces::srv::DetectGrasps>("/detect_grasps");
auto request = std::make_shared<gpd_ros2_interfaces::srv::DetectGrasps::Request>();

// Fill request...
request->cloud_indexed = your_cloud_indexed;

// Synchronous call
auto result = client->async_send_request(request);
if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    // Process response->grasp_configs...
}
```

**Integration with Manipulation Pipeline:**
```bash
# 1. Start detection server
ros2 launch gpd_ros2 grasp_detection_server.launch.py

# 2. In your manipulation node, call service when needed
# 3. Use returned grasps for motion planning
# 4. Execute grasp with robot controller
```

#### Production Deployment

**Robust Server Launch:**
```bash
# Auto-restart with memory debugging
./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection_server.sh \
  --auto-restart \
  --max-restarts 10 \
  --use-asan \
  --config-file /opt/gpd/production.cfg
```

**Monitoring and Health Checks:**
```bash
# Check service health
ros2 service call /detect_grasps gpd_ros2_interfaces/srv/DetectGrasps

# Monitor topic output
ros2 topic echo /clustered_grasps

# Check node status
ros2 node info /detect_grasps_server
```

#### Comparison: Server vs Streaming Node

| Aspect | Server Node | Streaming Node |
|--------|-------------|----------------|
| **Processing** | On-demand | Continuous |
| **Latency** | Higher (full pipeline) | Lower (pre-processed) |
| **Control** | Precise timing | Real-time flow |
| **Resource Usage** | Bursty | Steady |
| **Use Case** | Manipulation pipelines | Real-time monitoring |
| **Integration** | Service calls | Topic subscriptions |

### 3. Testing and Utility Tools

#### PCD Publisher (`pcd_publisher`)
**Publishes PCD/PLY files as PointCloud2 messages for testing and development.**

A simple utility that loads point cloud files and publishes them to ROS2 topics, useful for testing grasp detection without live sensors.

**Features:**
- Supports both PCD and PLY file formats
- Configurable publishing rate or single-shot mode
- Automatic field layout detection
- Proper timestamp handling

**Usage:**
```bash
# Publish once and exit
ros2 run gpd_ros2 pcd_publisher \
  --file /path/to/cloud.pcd \
  --topic /cloud_stitched \
  --frame camera_link \
  --once

# Publish continuously at 2 Hz
ros2 run gpd_ros2 pcd_publisher \
  --file /path/to/cloud.pcd \
  --topic /cloud_stitched \
  --frame camera_link \
  --rate 2.0

# Use convenience script
./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_publisher.sh
```

**Command Line Options:**
- `--file/-f`: Path to PCD or PLY file (required)
- `--topic/-t`: Output topic name (default: `/cloud_stitched`)
- `--frame/-F`: Frame ID for the point cloud (default: `camera_link`)
- `--rate/-r`: Publishing rate in Hz (default: `2.0`)
- `--once`: Publish once and exit (overrides rate)
- `--help/-h`: Show usage information

**Environment Variables:**
- `PCD_FILE`: Default file path
- `CLOUD_TOPIC`: Default topic name
- `FRAME`: Default frame ID

#### Mask2GPD Converter (`mask2gpd`)
**Converts segmented object point clouds to GPD-compatible format.**

Bridges segmentation pipelines (like `mask2pcl`) with GPD by forwarding and optionally reformatting point clouds.

**Features:**
- Real-time forwarding of segmented point clouds
- Optional frame ID override for coordinate frame consistency
- Repeating mode for continuous processing
- Automatic timestamping

**Usage:**
```bash
# Basic forwarding from segmentation to GPD
ros2 run gpd_ros2 mask2gpd \
  --ros-args \
  -p input_topic:=object_cloud \
  -p output_topic:=/cloud_stitched

# With frame override and repeating
ros2 run gpd_ros2 mask2gpd \
  --ros-args \
  -p input_topic:=segmented_objects \
  -p output_topic:=/cloud_stitched \
  -p frame_override:=camera_optical_frame \
  -p repeat_rate_hz:=5.0
```

**Parameters:**
- `input_topic` (string): Source topic from segmentation (default: `object_cloud`)
- `output_topic` (string): Destination topic for GPD (default: `/cloud_stitched`)
- `frame_override` (string): Override frame ID (empty = keep original)
- `repeat_rate_hz` (double): Republish rate in Hz (0.0 = no repeating)

**Typical Pipeline:**
```
Camera → Segmentation → mask2gpd → GPD Detection
             (SAM2)    (mask2pcl)   (detect_grasps)
```

#### PCD Service Client (`pcd_service_client`)
**Comprehensive testing tool for the grasp detection service.**

[Already documented above with enhanced features]

#### Integration Examples

**Complete Testing Pipeline:**
```bash
# Terminal 1: Start grasp detection service
ros2 launch gpd_ros2 grasp_detection_server.launch.py

# Terminal 2: Publish test data
./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_publisher.sh

# Terminal 3: Test service with detailed output
./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_service_client.sh \
  --show-detailed 5 --show-summary
```

**Live Segmentation Pipeline:**
```bash
# Terminal 1: Start streaming detection
ros2 launch gpd_ros2 grasp_detection.launch.py \
  cloud_topic:=/cloud_stitched

# Terminal 2: Run segmentation and forwarding
ros2 run your_segmentation_pkg mask2pcl [args] &
ros2 run gpd_ros2 mask2gpd \
  --ros-args \
  -p input_topic:=object_cloud \
  -p output_topic:=/cloud_stitched \
  -p repeat_rate_hz:=2.0
```

## Configuration

### GPD Configuration File
GPD requires a configuration file (typically `ros_eigen_params.cfg`). Key parameters:

```yaml
# Example GPD configuration
workspace: [-1, 1, -1, 1, -1, 1]  # [xmin, xmax, ymin, ymax, zmin, zmax]
camera_position: [0, 0, 0]        # Camera viewpoint
num_samples: 1000                 # Number of grasp candidates to evaluate
num_threads: 4                    # Parallel processing threads
```

### Launch Parameters

#### Common Parameters
- `config_file`: Path to GPD configuration file
- `cloud_topic`: Input point cloud topic
- `rviz_topic`: RViz marker topic (empty to disable)
- `frame`: Point cloud frame ID

#### Advanced Options
- `use_asan`: Enable AddressSanitizer for debugging memory issues
- `auto_restart`: Automatically restart node on crashes
- `workspace`: Spatial limits for grasp detection

## Message Formats

### GraspConfig
Describes a single 6-DOF grasp pose:
```yaml
geometry_msgs/Point position      # Grasp position
geometry_msgs/Vector3 approach    # Approach direction
geometry_msgs/Vector3 binormal    # Hand closing direction  
geometry_msgs/Vector3 axis        # Hand axis
float32 width                     # Required gripper opening
float32 score                     # Grasp quality score
geometry_msgs/Point sample        # Detection point
```

### GraspConfigList
Collection of grasp poses with metadata:
```yaml
std_msgs/Header header
gpd_ros2_interfaces/GraspConfig[] grasps
```

## Development and Debugging

### Memory Management
The implementation includes temporary memory management solutions for GPD stability:
- Cloud history buffers prevent premature memory deallocation
- ASAN-compatible modes for memory debugging
- Automatic restart capabilities for production use

### Debug Mode
```bash
# Launch with debug options
ros2 launch gpd_ros2 grasp_detection.launch.py use_asan:=true

# Enable auto-restart for production
./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection.sh \
  --auto-restart --max-restarts 10
```

### Performance Monitoring
- Monitor `/clustered_grasps` topic rate
- Check node CPU/memory usage with `htop`
- Use RViz markers to visualize detection quality

## Sample Data

The `tutorials/` directory contains sample PCD files:
- `mug.pcd`: Simple mug object
- `table_mug.pcd`: Mug on table surface
- `krylon.pcd`: Spray can object

## Troubleshooting

### Common Issues

1. **GPD Library Not Found**
   ```bash
   export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
   ```

2. **Memory Issues**
   - Use `--use-asan` flag for debugging
   - Enable auto-restart for production: `--auto-restart`

3. **No Grasps Detected**
   - Check point cloud density and quality
   - Verify workspace parameters in config file
   - Ensure proper camera viewpoint configuration

4. **Service Not Available**
   ```bash
   # Check if service is running
   ros2 service list | grep detect_grasps
   
   # Test service availability
   ros2 service call /detect_grasps gpd_ros2_interfaces/srv/DetectGrasps
   ```

### Performance Tuning
- Adjust `num_samples` in GPD config for speed/quality tradeoff
- Use `num_threads` to leverage multiple CPU cores
- Consider point cloud downsampling for faster processing

## Contributing

When contributing to this project:

1. **Test thoroughly** with the provided sample data
2. **Update documentation** for any new features or parameters
3. **Follow ROS2 best practices** for node lifecycle and parameter handling
4. **Consider memory safety** when modifying core detection loops

## License

This project follows the same license as the original GPD library. See LICENSE file for details.

## References

- [GPD: Grasp Pose Detection in Point Clouds](https://github.com/atenpas/gpd)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [PCL Documentation](https://pointclouds.org/documentation/)