from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch-time arguments with safe defaults
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg',
        description='Path to GPD ros_*_params.cfg'
    )
    cloud_type = DeclareLaunchArgument(
        'cloud_type', default_value='2',
        description='0: PointCloud2, 1: CloudIndexed, 2: CloudSamples'
    )
    cloud_topic = DeclareLaunchArgument(
        'cloud_topic', default_value='/cloud_stitched',
        description='Input cloud topic'
    )
    samples_topic = DeclareLaunchArgument(
        'samples_topic', default_value='',
        description='(Optional) separate samples topic'
    )
    rviz_topic = DeclareLaunchArgument(
        'rviz_topic', default_value='plot_grasps',
        description='MarkerArray topic for RViz'
    )

    params = {
        'config_file':   LaunchConfiguration('config_file'),
        # Ensure correct type for integer param
        'cloud_type':    ParameterValue(LaunchConfiguration('cloud_type'), value_type=int),
        'cloud_topic':   LaunchConfiguration('cloud_topic'),
        'samples_topic': LaunchConfiguration('samples_topic'),
        'rviz_topic':    LaunchConfiguration('rviz_topic'),
        # Add more params here later if needed (workspace, camera_position, ...)
    }

    node = Node(
        package='gpd_ros2',
        executable='detect_grasps',
        name='detect_grasps',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        config_file, cloud_type, cloud_topic, samples_topic, rviz_topic,
        node
    ])