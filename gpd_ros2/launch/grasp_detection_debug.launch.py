from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Same functional args
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/home/socrob/Development/gpd/cfg/ros_eigen_params.cfg',
        description='Path to GPD ros_*_params.cfg'
    )
    cloud_type = DeclareLaunchArgument('cloud_type', default_value='2',
        description='0: PointCloud2, 1: CloudIndexed, 2: CloudSamples')
    cloud_topic = DeclareLaunchArgument('cloud_topic', default_value='/cloud_stitched',
        description='Input cloud topic')
    samples_topic = DeclareLaunchArgument('samples_topic', default_value='',
        description='(Optional) separate samples topic')
    rviz_topic = DeclareLaunchArgument('rviz_topic', default_value='',
        description='MarkerArray topic for RViz (empty disables plotting)')

    # Debug knobs
    debugger = DeclareLaunchArgument(
        'debugger', default_value='gdbserver',
        description='Choose "gdbserver" (default) or "gdb"'
    )
    gdb_port = DeclareLaunchArgument(
        'gdb_port', default_value='3333',
        description='gdbserver TCP port when debugger:=gdbserver'
    )

    params = {
        'config_file':   LaunchConfiguration('config_file'),
        'cloud_type':    ParameterValue(LaunchConfiguration('cloud_type'), value_type=int),
        'cloud_topic':   LaunchConfiguration('cloud_topic'),
        'samples_topic': LaunchConfiguration('samples_topic'),
        'rviz_topic':    LaunchConfiguration('rviz_topic'),
    }

    # Conditions
    use_gdbserver = IfCondition(PythonExpression(["'", LaunchConfiguration('debugger'), "' == 'gdbserver'"]))
    use_gdb       = IfCondition(PythonExpression(["'", LaunchConfiguration('debugger'), "' == 'gdb'"]))

    node_gdbserver = Node(
        package='gpd_ros2',
        executable='detect_grasps',
        name='detect_grasps',
        output='screen',
        parameters=[params],
        prefix=['gdbserver localhost:', LaunchConfiguration('gdb_port'), ' '],
        condition=use_gdbserver
    )

    node_gdb = Node(
        package='gpd_ros2',
        executable='detect_grasps',
        name='detect_grasps',
        output='screen',
        parameters=[params],
        prefix=['gdb -q --args '],
        condition=use_gdb
    )

    return LaunchDescription([
        config_file, cloud_type, cloud_topic, samples_topic, rviz_topic,
        debugger, gdb_port,
        node_gdbserver, node_gdb
    ])
