#!/usr/bin/env bash
set -euo pipefail

# ---- Edit these if needed ----
PCD_FILE="${PCD_FILE:-$HOME/grasp_ws/src/grasp_detection_ros2/tutorials/mug.pcd}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/gpd/input_cloud}"
FRAME="${FRAME:-camera_link}"

# Publish the PCD once and exit
ros2 run gpd_ros2 pcd_publisher \
  --file "${PCD_FILE}" \
  --topic "${CLOUD_TOPIC}" \
  --frame "${FRAME}" \
  --once
