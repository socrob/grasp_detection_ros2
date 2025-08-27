#!/usr/bin/env bash
set -euo pipefail


# ---- Editable parameters ----
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/Development/gpd/cfg/ros_vino_params.cfg}"
RVIZ_TOPIC="${RVIZ_TOPIC:-plot_grasps}"
WORKSPACE="${WORKSPACE:-[]}"            # YAML array literal
CAMERA_POSITION="${CAMERA_POSITION:-}"   # e.g. "[0.0,0.0,0.0]" or empty

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

ARGS=(--ros-args
  -p config_file:="${CONFIG_FILE}"
  -p rviz_topic:="${RVIZ_TOPIC}"
  -p workspace:=${WORKSPACE}
)

if [[ -n "${CAMERA_POSITION}" ]]; then
  ARGS+=(-p camera_position:=${CAMERA_POSITION})
fi

exec ros2 run gpd_ros2 detect_grasps_server "${ARGS[@]}"
