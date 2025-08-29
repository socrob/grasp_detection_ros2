#!/usr/bin/env bash
set -euo pipefail


# Example usage:
#   ./src/grasp_detection_ros2/gpd_ros2/launch/run_pcd_service_client.sh \
#       /home/socrob/datasets/example.pcd

PCD_FILE="${PCD_FILE:-$HOME/grasp_ws/src/grasp_detection_ros2/tutorials/mug.pcd}"

ros2 launch gpd_ros2 pcd_service_client.launch.py \
  pcd_file:=${PCD_FILE} \
  frame:=camera_link \
  service_name:=/detect_grasps \
  repeat:=false \
  num_indices:=500 \
  view_point_x:=0.0 view_point_y:=0.0 view_point_z:=0.0
