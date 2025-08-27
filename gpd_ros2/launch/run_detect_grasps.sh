#!/usr/bin/env bash
set -euo pipefail


# ---- Editable env overrides (all optional) ----
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg}"
CLOUD_TYPE="${CLOUD_TYPE:-0}"              # 0,1,2
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_stitched}"
SAMPLES_TOPIC="${SAMPLES_TOPIC:-}"         # if empty, we won't pass it
RVIZ_TOPIC="${RVIZ_TOPIC:-plot_grasps}"

# Ensure GPD runtime path if installed to /usr/local/lib
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

ARGS=(
  "config_file:=${CONFIG_FILE}"
  "cloud_type:=${CLOUD_TYPE}"
  "cloud_topic:=${CLOUD_TOPIC}"
  "rviz_topic:=${RVIZ_TOPIC}"
)

# pass samples_topic only when non-empty
if [[ -n "${SAMPLES_TOPIC}" ]]; then
  ARGS+=("samples_topic:=${SAMPLES_TOPIC}")
fi

exec ros2 launch gpd_ros2 ur5.launch.py "${ARGS[@]}"