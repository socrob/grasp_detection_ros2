from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Namespace argument
    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='gpd',
        description='Namespace for the grasp detection node'
    )
    
    # Existing arguments
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/home/socrob/grasp_ws/src/grasp_detection_ros2/gpd_ros2/config/ros_eigen_params.cfg',
        description='Path to GPD ros_*_params.cfg'
    )
    cloud_type = DeclareLaunchArgument(
        'cloud_type', default_value='2',
        description='0: PointCloud2, 1: CloudIndexed, 2: CloudSamples'
    )
    cloud_topic = DeclareLaunchArgument(
        'cloud_topic', default_value='input_cloud',
        description='Input cloud topic'
    )
    samples_topic = DeclareLaunchArgument(
        'samples_topic', default_value='',
        description='(Optional) separate samples topic'
    )
    rviz_topic = DeclareLaunchArgument(
        'rviz_topic', default_value='',
        description='MarkerArray topic for RViz (empty disables plotting)'
    )

    # ASAN and restart options
    use_asan = DeclareLaunchArgument(
        'use_asan', default_value='false',
        description='Enable ASAN workaround for GPD memory issues'
    )
    auto_restart = DeclareLaunchArgument(
        'auto_restart', default_value='false',
        description='Auto-restart node if it crashes'
    )
    max_restarts = DeclareLaunchArgument(
        'max_restarts', default_value='10',
        description='Maximum number of restarts (only used with auto_restart=true)'
    )

    params = {
        'config_file':   LaunchConfiguration('config_file'),
        'cloud_type':    ParameterValue(LaunchConfiguration('cloud_type'), value_type=int),
        'cloud_topic':   LaunchConfiguration('cloud_topic'),
        'samples_topic': LaunchConfiguration('samples_topic'),
        'rviz_topic':    LaunchConfiguration('rviz_topic'),
    }

    # Standard node (no ASAN, no restart)
    standard_node = Node(
        package='gpd_ros2',
        executable='detect_grasps',
        name='detect_grasps',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration('use_asan'))
    )

    # ASAN node (with memory workaround)
    asan_node = Node(
        package='gpd_ros2',
        executable='detect_grasps',
        name='detect_grasps',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[params],
        additional_env={
            'ASAN_OPTIONS': 'new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:detect_leaks=0',
            'MALLOC_CHECK_': '0',
            'MALLOC_PERTURB_': '0'
        },
        condition=IfCondition(LaunchConfiguration('use_asan'))
    )

    return LaunchDescription([
        namespace, config_file, cloud_type, cloud_topic, samples_topic, rviz_topic,
        use_asan, auto_restart, max_restarts,
        standard_node,
        asan_node
    ])