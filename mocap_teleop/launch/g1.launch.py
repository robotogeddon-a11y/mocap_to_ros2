import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("mocap_teleop")
    with open(os.path.join(share, "urdf", "g1", "g1_29dof_rev_1_0.urdf")) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("mocap_file", description="QuickMagic robot export (*_robot_g1.json)"),
        DeclareLaunchArgument("in_place", default_value="false",
                              description="true = dance on the spot (drop the pelvis travel)"),
        DeclareLaunchArgument("speed", default_value="1.0", description="playback multiplier"),
        DeclareLaunchArgument("skip_frames", default_value="2",
                              description="T-pose calibration frames at the start of the export"),
        DeclareLaunchArgument("start_yaw_deg", default_value="-90.0",
                              description="heading the dance starts with (-90 = facing the saved cameras, nan = as recorded)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="mocap_teleop",
            executable="robot_player",
            parameters=[{
                "mocap_file": LaunchConfiguration("mocap_file"),
                "in_place": ParameterValue(LaunchConfiguration("in_place"), value_type=bool),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "skip_frames": ParameterValue(LaunchConfiguration("skip_frames"), value_type=int),
                "start_yaw_deg": ParameterValue(LaunchConfiguration("start_yaw_deg"), value_type=float),
            }],
            output="screen",
        ),
        # Saved config: Fixed Frame = world, RobotModel on /robot_description
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "config", "g1.rviz")]),
    ])
