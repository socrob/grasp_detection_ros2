#!/usr/bin/env bash
set -euo pipefail

# ---- Configuration (can be overridden by env vars) ----
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/grasp_ws/src/grasp_detection_ros2/gpd_ros2/config/ros_eigen_params.cfg}"
RVIZ_TOPIC="${RVIZ_TOPIC:-grasp_poses}"
NAMESPACE="${NAMESPACE:-gpd}"

# ASAN and restart options
USE_ASAN="${USE_ASAN:-false}"
AUTO_RESTART="${AUTO_RESTART:-true}"
MAX_RESTARTS="${MAX_RESTARTS:-10}"
RESTART_DELAY="${RESTART_DELAY:-3}"

# Ensure GPD runtime path if installed to /usr/local/lib
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# Function to print usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
    --namespace NS      Set namespace for the grasp detection server node
    --use-asan          Enable ASAN workaround for GPD memory issues
    --auto-restart      Auto-restart node if it crashes
    --max-restarts N    Maximum restarts (default: 10)
    --restart-delay N   Seconds between restarts (default: 3)
    --config-file PATH  GPD config file path
    --rviz-topic TOPIC  RViz marker topic
    --help              Show this help

Environment variables:
    NAMESPACE, USE_ASAN, AUTO_RESTART, MAX_RESTARTS, RESTART_DELAY, 
    CONFIG_FILE, RVIZ_TOPIC

Examples:
    # Standard run
    $0
    
    # With namespace
    $0 --namespace grasp_server
    
    # With ASAN workaround and namespace
    $0 --use-asan --namespace left_arm_grasp
    
    # Multi-arm setup
    $0 --namespace left_arm --rviz-topic /left_arm/grasp_poses
    $0 --namespace right_arm --rviz-topic /right_arm/grasp_poses
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
        --rviz-topic)
            RVIZ_TOPIC="$2"
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

# Build launch arguments
LAUNCH_ARGS=(
    "config_file:=${CONFIG_FILE}"
    "rviz_topic:=${RVIZ_TOPIC}"
    "use_asan:=${USE_ASAN}"
)

# Add namespace if provided
if [[ -n "${NAMESPACE}" ]]; then
    LAUNCH_ARGS+=("namespace:=${NAMESPACE}")
fi

# Build ros2 run arguments for direct execution
ROS_ARGS=(
    "--ros-args"
    "-p" "config_file:=${CONFIG_FILE}"
    "-p" "rviz_topic:=${RVIZ_TOPIC}"
)

# Add namespace to ros2 run command if provided
if [[ -n "${NAMESPACE}" ]]; then
    ROS_ARGS+=("-r" "__ns:=/${NAMESPACE}")
fi

# Function to run via launch file
run_with_launch() {
    local attempt=$1
    echo "===== Attempt $attempt: Starting GPD server via launch ====="
    
    echo "Command: ros2 launch gpd_ros2 grasp_detection_server_namespace.launch.py ${LAUNCH_ARGS[*]}"
    ros2 launch gpd_ros2 grasp_detection_server_namespace.launch.py "${LAUNCH_ARGS[@]}"
    
    return $?
}

# Function to run directly 
run_direct() {
    local attempt=$1
    echo "===== Attempt $attempt: Starting GPD server directly ====="
    
    # Set ASAN environment if requested
    if [[ "${USE_ASAN}" == "true" ]]; then
        echo "Using ASAN workaround for memory issues"
        export ASAN_OPTIONS="new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:detect_leaks=0"
        export MALLOC_CHECK_=0
        export MALLOC_PERTURB_=0
    fi
    
    echo "Command: ros2 run gpd_ros2 detect_grasps_server ${ROS_ARGS[*]}"
    ros2 run gpd_ros2 detect_grasps_server "${ROS_ARGS[@]}"
    
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
echo "GPD GRASP DETECTION SERVER LAUNCH"
echo "==============================================="
echo "Config file: $CONFIG_FILE"
echo "RViz topic: $RVIZ_TOPIC"
if [[ -n "${NAMESPACE}" ]]; then
    echo "Namespace: $NAMESPACE"
else
    echo "Namespace: (none)"
fi
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
        if [[ "${USE_ASAN}" == "true" ]]; then
            # Use launch file for ASAN support
            run_with_launch $restart_count
        else
            # Use direct execution for simpler cases
            run_direct $restart_count
        fi
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
    # Single run
    echo "Single run mode (no auto-restart)"
    if [[ "${USE_ASAN}" == "true" ]]; then
        run_with_launch 1
    else
        run_direct 1
    fi
fi

echo "Script completed."