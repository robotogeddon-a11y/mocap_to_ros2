import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    share = get_package_share_directory("mocap_teleop")
    with open(os.path.join(share, "urdf", "simple_humanoid.urdf")) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("mocap_file", description="JSON from export_bone_positions.py"),
        DeclareLaunchArgument("mapping", default_value="unreal",
                              description="Skeleton preset: config/mapping_<name>.yaml (unreal, mixamo)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="mocap_teleop",
            executable="mocap_player",
            parameters=[
                ParameterFile([os.path.join(share, "config", "mapping_"),
                               LaunchConfiguration("mapping"), ".yaml"]),
                {"mocap_file": LaunchConfiguration("mocap_file")},
            ],
            output="screen",
        ),
        # Saved config: Fixed Frame = base_link, RobotModel on /robot_description
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "config", "display.rviz")]),
    ])
