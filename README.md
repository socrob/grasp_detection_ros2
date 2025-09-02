BUILD
colcon build --packages-select gpd_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

GRASP DETECTION TEST
./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection.sh

./src/grasp_detection_ros2/gpd_ros2/launch/grasp_detection.sh --use-asan --auto-restart --max-restarts 5

./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_publisher.sh


DEBUG
ros2 launch gpd_ros2 grasp_detection_debug.launch.py debugger:=gdbserver gdb_port:=3333 cloud_type:=0 cloud_topic:=/cloud_stitched rviz_topic:="/grasp_poses"