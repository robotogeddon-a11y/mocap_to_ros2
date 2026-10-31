import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop_share = get_package_share_directory("mocap_teleop")
    replay_share = get_package_share_directory("mocap_gz_replay")
    with open(os.path.join(teleop_share, "urdf", "g1", "g1_29dof_rev_1_0.urdf")) as f:
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
        DeclareLaunchArgument("rviz", default_value="true", description="also open RViz"),
        DeclareLaunchArgument("world", default_value=os.path.join(replay_share, "worlds", "dance_floor.sdf"),
                              description="Gazebo world SDF (must load the MocapReplay plugin)"),
        # Gazebo finds the MocapReplay plugin and the package:// meshes through these
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH",
                                  os.path.join(get_package_prefix("mocap_gz_replay"), "lib")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(teleop_share)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r ", LaunchConfiguration("world")],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "g1", "-z", "0.8"],
            output="screen",
        ),
        # ROS -> Gazebo: joint angles and pelvis pose for the MocapReplay plugin
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/joint_states@sensor_msgs/msg/JointState]gz.msgs.Model",
                       "/root_pose@geometry_msgs/msg/Pose]gz.msgs.Pose"],
            output="screen",
        ),
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
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(teleop_share, "config", "g1.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
