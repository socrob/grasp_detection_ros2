#!/usr/bin/env bash
set -euo pipefail

# ---- Configuration (can be overridden by env vars) ----
CONFIG_FILE="${CONFIG_FILE:-/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg}"
RVIZ_TOPIC="${RVIZ_TOPIC:-/grasp_poses}"

# NEW: ASAN and restart options
USE_ASAN="${USE_ASAN:-false}"              # true/false
AUTO_RESTART="${AUTO_RESTART:-false}"      # true/false  
MAX_RESTARTS="${MAX_RESTARTS:-10}"         # number
RESTART_DELAY="${RESTART_DELAY:-3}"        # seconds between restarts

# Ensure GPD runtime path if installed to /usr/local/lib
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# Function to print usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
    --use-asan          Enable ASAN workaround for GPD memory issues
    --auto-restart      Auto-restart node if it crashes
    --max-restarts N    Maximum restarts (default: 10)
    --restart-delay N   Seconds between restarts (default: 3)
    --config-file PATH  GPD config file path
    --rviz-topic TOPIC  RViz marker topic
    --help              Show this help

Environment variables:
    USE_ASAN, AUTO_RESTART, MAX_RESTARTS, RESTART_DELAY, 
    CONFIG_FILE, RVIZ_TOPIC

Examples:
    # Standard run
    $0
    
    # With ASAN workaround
    $0 --use-asan
    
    # With auto-restart (up to 5 times)
    $0 --auto-restart --max-restarts 5
    
    # Both ASAN and auto-restart
    $0 --use-asan --auto-restart
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
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

# Function to run via launch file (for ASAN support)
run_with_launch() {
    local attempt=$1
    echo "===== Attempt $attempt: Starting GPD server via launch ====="
    
    local launch_args=(
        "config_file:=${CONFIG_FILE}"
        "rviz_topic:=${RVIZ_TOPIC}"
    )
    
    if [[ "${USE_ASAN}" == "true" ]]; then
        echo "Using ASAN workaround for memory issues"
        launch_args+=("use_asan:=true")
    fi
    
    echo "Command: ros2 launch gpd_ros2 grasp_detection_server.launch.py ${launch_args[*]}"
    ros2 launch gpd_ros2 grasp_detection_server.launch.py "${launch_args[@]}"
    
    return $?
}

# Function to run directly (for non-ASAN)
run_direct() {
    local attempt=$1
    echo "===== Attempt $attempt: Starting GPD server directly ====="
    
    local ros_args=(
        "--ros-args"
        "-p" "config_file:=${CONFIG_FILE}"
        "-p" "rviz_topic:=${RVIZ_TOPIC}"
    )
    
    echo "Command: ros2 run gpd_ros2 detect_grasps_server ${ros_args[*]}"
    ros2 run gpd_ros2 detect_grasps_server "${ros_args[@]}"
    
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

# Main execution logic
if [[ "${AUTO_RESTART}" == "true" ]]; then
    echo "Auto-restart enabled (max: $MAX_RESTARTS, delay: ${RESTART_DELAY}s)"
    echo "Press Ctrl+C to stop"
    
    restart_count=0
    
    while [[ $restart_count -lt $MAX_RESTARTS ]]; do
        restart_count=$((restart_count + 1))
        
        # Run the node (use launch for ASAN, direct for non-ASAN)
        set +e  # Don't exit on command failure
        if [[ "${USE_ASAN}" == "true" ]]; then
            run_with_launch $restart_count
        else
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