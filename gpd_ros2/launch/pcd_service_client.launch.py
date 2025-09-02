from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch-time args (so CLI/sh overrides work)
    args = [
        DeclareLaunchArgument('pcd_file', default_value='/home/socrob/grasp_ws/src/grasp_detection_ros2/tutorials/mug.pcd'),
        DeclareLaunchArgument('frame', default_value='camera_link'),
        DeclareLaunchArgument('service_name', default_value='/detect_grasps'),

        DeclareLaunchArgument('repeat', default_value='false'),
        DeclareLaunchArgument('period_ms', default_value='1000'),

        DeclareLaunchArgument('num_indices', default_value='500'),

        DeclareLaunchArgument('view_point_x', default_value='0.0'),
        DeclareLaunchArgument('view_point_y', default_value='0.0'),
        DeclareLaunchArgument('view_point_z', default_value='0.0'),

        # NEW: Display control parameters
        DeclareLaunchArgument('show_detailed', default_value='3', 
                            description='Number of top grasps to show in detail'),
        DeclareLaunchArgument('show_summary', default_value='true',
                            description='Whether to show summary statistics'),
    ]

    # Use typed ParameterValue to avoid "stringy" params
    params = {
        'pcd_file':     LaunchConfiguration('pcd_file'),
        'frame':        LaunchConfiguration('frame'),
        'service_name': LaunchConfiguration('service_name'),
        'repeat':       ParameterValue(LaunchConfiguration('repeat'), value_type=bool),
        'period_ms':    ParameterValue(LaunchConfiguration('period_ms'), value_type=int),
        'num_indices':  ParameterValue(LaunchConfiguration('num_indices'), value_type=int),
        'view_point_x': ParameterValue(LaunchConfiguration('view_point_x'), value_type=float),
        'view_point_y': ParameterValue(LaunchConfiguration('view_point_y'), value_type=float),
        'view_point_z': ParameterValue(LaunchConfiguration('view_point_z'), value_type=float),
        'show_detailed': ParameterValue(LaunchConfiguration('show_detailed'), value_type=int),
        'show_summary': ParameterValue(LaunchConfiguration('show_summary'), value_type=bool),
    }

    node = Node(
        package='gpd_ros2',
        executable='pcd_service_client',
        name='pcd_service_client',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription(args + [node])