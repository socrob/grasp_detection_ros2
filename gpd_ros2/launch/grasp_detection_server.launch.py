from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='gpd',
        description='Namespace for the grasp detection server node'
    )
    
    # Existing arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="/home/socrob/grasp_ws/src/grasp_detection_ros2/gpd_ros2/config/ros_eigen_params.cfg",
        description="Path to GPD config (*.cfg)",
    )
    rviz_topic_arg = DeclareLaunchArgument(
        "rviz_topic",
        default_value="grasp_poses",
        description="Topic to publish RViz grasp markers",
    )

    # ASAN options
    use_asan = DeclareLaunchArgument(
        'use_asan', default_value='false',
        description='Enable ASAN workaround for GPD memory issues'
    )

    params = {
        "config_file": LaunchConfiguration("config_file"),
        "rviz_topic":  LaunchConfiguration("rviz_topic"),
    }

    # Standard node (no ASAN)
    standard_node = Node(
        package="gpd_ros2",
        executable="detect_grasps_server",   
        name="detect_grasps_server",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration('use_asan'))
    )

    # ASAN node (with memory workaround)
    asan_node = Node(
        package="gpd_ros2",
        executable="detect_grasps_server",
        name="detect_grasps_server",
        namespace=LaunchConfiguration('namespace'),
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
        namespace_arg,
        config_file_arg,
        rviz_topic_arg,
        use_asan,
        standard_node,
        asan_node
    ])