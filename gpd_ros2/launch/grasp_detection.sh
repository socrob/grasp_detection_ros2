#!/usr/bin/env bash
set -euo pipefail

# ---- Configuration (can be overridden by env vars) ----
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/grasp_ws/src/grasp_detection_ros2/gpd_ros2/config/ros_eigen_params.cfg}"
CLOUD_TYPE="${CLOUD_TYPE:-0}"              # 0,1,2
CLOUD_TOPIC="${CLOUD_TOPIC:-input_cloud}"
SAMPLES_TOPIC="${SAMPLES_TOPIC:-}"         # if empty, we won't pass it
RVIZ_TOPIC="${RVIZ_TOPIC:-plot_grasps}"
NAMESPACE="${NAMESPACE:-gpd}"                 # NEW: namespace support

# ASAN and restart options
USE_ASAN="${USE_ASAN:-false}"              # true/false
AUTO_RESTART="${AUTO_RESTART:-true}"      # true/false  
MAX_RESTARTS="${MAX_RESTARTS:-10}"         # number
RESTART_DELAY="${RESTART_DELAY:-3}"        # seconds between restarts

# Ensure GPD runtime path if installed to /usr/local/lib
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# Function to print usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
    --namespace NS      Set namespace for the grasp detection node
    --use-asan          Enable ASAN workaround for GPD memory issues
    --auto-restart      Auto-restart node if it crashes
    --max-restarts N    Maximum restarts (default: 10)
    --restart-delay N   Seconds between restarts (default: 3)
    --config-file PATH  GPD config file path
    --cloud-topic TOPIC Input cloud topic
    --help              Show this help

Environment variables:
    NAMESPACE, USE_ASAN, AUTO_RESTART, MAX_RESTARTS, RESTART_DELAY, 
    CONFIG_FILE, CLOUD_TYPE, CLOUD_TOPIC, SAMPLES_TOPIC, RVIZ_TOPIC

Examples:
    # Standard run
    $0
    
    # With namespace
    $0 --namespace grasp_detection
    
    # With ASAN workaround and namespace
    $0 --use-asan --namespace left_arm_grasp
    
    # With auto-restart (up to 5 times) and namespace
    $0 --auto-restart --max-restarts 5 --namespace right_arm_grasp
    
    # Multi-arm setup
    $0 --namespace left_arm --cloud-topic /left_arm/cloud_stitched
    $0 --namespace right_arm --cloud-topic /right_arm/cloud_stitched
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        --use-asan)
            USE_ASAN="true"
            shift
            ;;
        --auto-restart)
            AUTO_RESTART="true"
            shift
            ;;
        --max-restarts)
            MAX_RESTARTS="$2"
            shift 2
            ;;
        --restart-delay)
            RESTART_DELAY="$2"
            shift 2
            ;;
        --config-file)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --cloud-topic)
            CLOUD_TOPIC="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Build launch arguments for launch file approach
LAUNCH_ARGS=(
    "config_file:=${CONFIG_FILE}"
    "cloud_type:=${CLOUD_TYPE}"
    "cloud_topic:=${CLOUD_TOPIC}"
    "rviz_topic:=${RVIZ_TOPIC}"
    "use_asan:=${USE_ASAN}"
    "auto_restart:=${AUTO_RESTART}"
    "max_restarts:=${MAX_RESTARTS}"
)

# Add namespace if provided
if [[ -n "${NAMESPACE}" ]]; then
    LAUNCH_ARGS+=("namespace:=${NAMESPACE}")
fi

# Add samples_topic only when non-empty
if [[ -n "${SAMPLES_TOPIC}" ]]; then
    LAUNCH_ARGS+=("samples_topic:=${SAMPLES_TOPIC}")
fi

# Build direct ros2 run arguments for restart control
ROS_ARGS=(
    "--ros-args"
    "-p" "config_file:=${CONFIG_FILE}"
    "-p" "cloud_type:=${CLOUD_TYPE}"
    "-p" "cloud_topic:=${CLOUD_TOPIC}"
    "-p" "rviz_topic:=${RVIZ_TOPIC}"
)

# Add namespace to ros2 run command if provided
if [[ -n "${NAMESPACE}" ]]; then
    ROS_ARGS+=("-r" "__ns:=/${NAMESPACE}")
fi

# Add samples_topic only when non-empty
if [[ -n "${SAMPLES_TOPIC}" ]]; then
    ROS_ARGS+=("-p" "samples_topic:=${SAMPLES_TOPIC}")
fi

# Function to run the node once (for restart functionality)
run_node() {
    local attempt=$1
    echo "===== Attempt $attempt: Starting GPD node ====="
    
    if [[ -n "${NAMESPACE}" ]]; then
        echo "Using namespace: $NAMESPACE"
    fi
    
    # Set ASAN environment if requested
    if [[ "${USE_ASAN}" == "true" ]]; then
        echo "Using ASAN workaround for memory issues"
        export ASAN_OPTIONS="new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:detect_leaks=0"
        export MALLOC_CHECK_=0
        export MALLOC_PERTURB_=0
    fi
    
    # Run the node directly (not through launch file)
    echo "Command: ros2 run gpd_ros2 detect_grasps ${ROS_ARGS[*]}"
    ros2 run gpd_ros2 detect_grasps "${ROS_ARGS[@]}"
    
    return $?
}

# Function to handle graceful shutdown
cleanup() {
    echo ""
    echo "Received interrupt signal. Cleaning up..."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Show configuration
echo "==============================================="
echo "GPD GRASP DETECTION LAUNCH"
echo "==============================================="
echo "Config file: $CONFIG_FILE"
echo "Cloud topic: $CLOUD_TOPIC"
echo "Cloud type: $CLOUD_TYPE"
if [[ -n "${NAMESPACE}" ]]; then
    echo "Namespace: $NAMESPACE"
else
    echo "Namespace: (none)"
fi
if [[ -n "${SAMPLES_TOPIC}" ]]; then
    echo "Samples topic: $SAMPLES_TOPIC"
fi
echo "RViz topic: $RVIZ_TOPIC"
echo "ASAN workaround: $USE_ASAN"
echo "Auto-restart: $AUTO_RESTART"
if [[ "${AUTO_RESTART}" == "true" ]]; then
    echo "Max restarts: $MAX_RESTARTS"
    echo "Restart delay: ${RESTART_DELAY}s"
fi
echo "==============================================="

# Main execution logic
if [[ "${AUTO_RESTART}" == "true" ]]; then
    echo "Auto-restart enabled (max: $MAX_RESTARTS, delay: ${RESTART_DELAY}s)"
    echo "Press Ctrl+C to stop"
    
    restart_count=0
    
    while [[ $restart_count -lt $MAX_RESTARTS ]]; do
        restart_count=$((restart_count + 1))
        
        # Run the node
        set +e  # Don't exit on command failure
        run_node $restart_count
        exit_code=$?
        set -e  # Re-enable exit on error
        
        if [[ $exit_code -eq 0 ]]; then
            echo "Node exited normally"
            break
        elif [[ $exit_code -eq 130 ]]; then
            # SIGINT (Ctrl+C)
            echo "Interrupted by user"
            break
        else
            echo "Node crashed with exit code $exit_code (restart #$restart_count/$MAX_RESTARTS)"
            
            if [[ $restart_count -lt $MAX_RESTARTS ]]; then
                echo "Restarting in ${RESTART_DELAY} seconds... (Press Ctrl+C to stop)"
                
                # Interruptible sleep
                for ((i=RESTART_DELAY; i>0; i--)); do
                    echo -ne "\rRestarting in $i seconds... (Press Ctrl+C to stop) "
                    sleep 1
                done
                echo ""
            else
                echo "Maximum restarts reached. Giving up."
                exit $exit_code
            fi
        fi
    done
else
    # Use launch file for single run (cleaner parameter passing)
    echo "Using launch file approach"
    echo "Launch arguments: ${LAUNCH_ARGS[*]}"
    ros2 launch your_package_name grasp_detection_with_namespace.launch.py "${LAUNCH_ARGS[@]}"
fi

echo "Script completed."