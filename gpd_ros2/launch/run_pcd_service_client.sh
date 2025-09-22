#!/usr/bin/env bash
set -euo pipefail

# ---- Configuration (can be overridden by env vars) ----
PCD_FILE="${PCD_FILE:-$HOME/grasp_ws/src/grasp_detection_ros2/tutorials/krylon.pcd}"
FRAME="${FRAME:-camera_link}"
SERVICE_NAME="${SERVICE_NAME:-/gpd/detect_grasps}"
REPEAT="${REPEAT:-false}"
NUM_INDICES="${NUM_INDICES:-500}"
VIEW_POINT_X="${VIEW_POINT_X:-0.0}"
VIEW_POINT_Y="${VIEW_POINT_Y:-0.0}"
VIEW_POINT_Z="${VIEW_POINT_Z:-0.0}"

# NEW: Display control options
SHOW_DETAILED="${SHOW_DETAILED:-3}"    # Number of grasps to show in detail
SHOW_SUMMARY="${SHOW_SUMMARY:-true}"   # Whether to show summary statistics

# Function to print usage
usage() {
    cat << 'EOF'
Usage: $0 [OPTIONS] [PCD_FILE]

Test the grasp detection service with a PCD/PLY file.

Positional Arguments:
    PCD_FILE            Path to .pcd or .ply file (default: mug.pcd in tutorials)

Options:
    --frame FRAME       Frame ID for the point cloud (default: camera_link)
    --service SERVICE   Service name to call (default: /detect_grasps)
    --repeat            Run continuously instead of once
    --num-indices N     Number of sample indices to use (default: 500)
    --show-detailed N   Number of top grasps to show in detail (default: 3, 0 to disable)
    --show-summary      Show summary statistics (default: true)
    --no-summary        Disable summary statistics
    --view-point X Y Z  Camera view point coordinates (default: 0 0 0)
    --help              Show this help

Environment variables:
    PCD_FILE, FRAME, SERVICE_NAME, REPEAT, NUM_INDICES,
    VIEW_POINT_X, VIEW_POINT_Y, VIEW_POINT_Z,
    SHOW_DETAILED, SHOW_SUMMARY

Examples:
    # Basic usage with default mug
    ./run_pcd_service_client.sh
    
    # Test with custom PCD file
    ./run_pcd_service_client.sh /path/to/object.pcd
    
    # Show only top grasp in detail, no summary
    ./run_pcd_service_client.sh --show-detailed 1 --no-summary
    
    # Repeat mode with custom view point
    ./run_pcd_service_client.sh --repeat --view-point 0.5 0.0 1.0
    
    # Show lots of detail
    ./run_pcd_service_client.sh --show-detailed 10 --show-summary
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --frame)
            FRAME="$2"
            shift 2
            ;;
        --service)
            SERVICE_NAME="$2"
            shift 2
            ;;
        --repeat)
            REPEAT="true"
            shift
            ;;
        --num-indices)
            NUM_INDICES="$2"
            shift 2
            ;;
        --show-detailed)
            SHOW_DETAILED="$2"
            shift 2
            ;;
        --show-summary)
            SHOW_SUMMARY="true"
            shift
            ;;
        --no-summary)
            SHOW_SUMMARY="false"
            shift
            ;;
        --view-point)
            VIEW_POINT_X="$2"
            VIEW_POINT_Y="$3"
            VIEW_POINT_Z="$4"
            shift 4
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        --*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            # Positional argument - assume it's the PCD file
            PCD_FILE="$1"
            shift
            ;;
    esac
done

# Verify PCD file exists
if [[ ! -f "$PCD_FILE" ]]; then
    echo "Error: PCD file does not exist: $PCD_FILE"
    echo ""
    echo "Available tutorial files:"
    find "$HOME/grasp_ws/src/grasp_detection_ros2/tutorials" -name "*.pcd" -o -name "*.ply" 2>/dev/null | head -5 || true
    exit 1
fi

echo "Running grasp detection service client..."
echo "  PCD file:     $PCD_FILE"
echo "  Frame:        $FRAME"
echo "  Service:      $SERVICE_NAME"
echo "  Repeat:       $REPEAT"
echo "  Num indices:  $NUM_INDICES"
echo "  View point:   [$VIEW_POINT_X, $VIEW_POINT_Y, $VIEW_POINT_Z]"
echo "  Show detailed: $SHOW_DETAILED grasps"
echo "  Show summary:  $SHOW_SUMMARY"
echo ""

ros2 launch gpd_ros2 pcd_service_client.launch.py \
  pcd_file:="$PCD_FILE" \
  frame:="$FRAME" \
  service_name:="$SERVICE_NAME" \
  repeat:="$REPEAT" \
  num_indices:="$NUM_INDICES" \
  view_point_x:="$VIEW_POINT_X" \
  view_point_y:="$VIEW_POINT_Y" \
  view_point_z:="$VIEW_POINT_Z" \
  show_detailed:="$SHOW_DETAILED" \
  show_summary:="$SHOW_SUMMARY"