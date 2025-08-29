from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args (mirror how you ran the other one)
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

    node = Node(
        package="gpd_ros2",
        executable="detect_grasps_server",   # <-- adjust if your target name differs
        name="detect_grasps_server",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "rviz_topic":  LaunchConfiguration("rviz_topic"),
            # The server declares these as well; leave to defaults unless you want to set:
            # "workspace": [],                     # e.g. [-1,1,-1,1,-1,1]
            # "camera_position": [0.0, 0.0, 0.0],  # optional
        }],
    )

    return LaunchDescription([
        config_file_arg,
        rviz_topic_arg,
        node
    ])
