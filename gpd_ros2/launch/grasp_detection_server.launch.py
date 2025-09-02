from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Existing arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg",
        description="Path to GPD config (*.cfg)",
    )
    rviz_topic_arg = DeclareLaunchArgument(
        "rviz_topic",
        default_value="/grasp_poses",
        description="Topic to publish RViz grasp markers",
    )

    # NEW: ASAN options
    use_asan = DeclareLaunchArgument(
        'use_asan', default_value='false',
        description='Enable ASAN workaround for GPD memory issues'
    )

    params = {
        "config_file": LaunchConfiguration("config_file"),
        "rviz_topic":  LaunchConfiguration("rviz_topic"),
        # The server declares these as well; leave to defaults unless you want to set:
        # "workspace": [],                     # e.g. [-1,1,-1,1,-1,1]
        # "camera_position": [0.0, 0.0, 0.0],  # optional
    }

    # Standard node (no ASAN)
    standard_node = Node(
        package="gpd_ros2",
        executable="detect_grasps_server",   
        name="detect_grasps_server",
        output="screen",
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration('use_asan'))
    )

    # ASAN node (with memory workaround)
    asan_node = Node(
        package="gpd_ros2",
        executable="detect_grasps_server",
        name="detect_grasps_server",
        output="screen",
        parameters=[params],
        additional_env={
            'ASAN_OPTIONS': 'new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:detect_leaks=0',
            'MALLOC_CHECK_': '0',
            'MALLOC_PERTURB_': '0'
        },
        condition=IfCondition(LaunchConfiguration('use_asan'))
    )

    return LaunchDescription([
        config_file_arg,
        rviz_topic_arg,
        use_asan,
        standard_node,
        asan_node
    ])