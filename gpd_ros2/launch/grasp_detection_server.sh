#!/usr/bin/env bash
set -euo pipefail


# Defaults (override via env or pass via ros2 launch args)
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg}"
RVIZ_TOPIC="${RVIZ_TOPIC:-/grasp_poses}"

echo "Launching grasp_detection_server..."
echo "  config_file: $CONFIG_FILE"
echo "  rviz_topic : $RVIZ_TOPIC"

ros2 launch gpd_ros2 grasp_detection_server.launch.py \
  config_file:="$CONFIG_FILE" \
  rviz_topic:="$RVIZ_TOPIC"
