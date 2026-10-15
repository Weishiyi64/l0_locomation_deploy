#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    body_name = LaunchConfiguration("body_name")

    # $(find <robot_type>_description)
    description_path = FindPackageShare(PythonExpression(["'", robot_type, "_description'"]))

    # <description_path>/mjcf/<robot_type>.xml
    legged_robot_scene = PathJoinSubstitution(
        [
            description_path,
            "mjcf",
            PythonExpression(["'", robot_type, ".xml'"]),
        ]
    )

    setup_script = PathJoinSubstitution(
        [
            FindPackageShare("mujoco"),
            "scripts",
            "setup_mujoco_software_rendering.sh",
        ]
    )

    # prefix: source env then exec
    prefix_cmd = PythonExpression(
        ["'bash -c \"source ", setup_script, "; exec \\\\\\\"$0\\\\\\\" \\\\\\\"$@\\\\\\\"\" --'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value=TextSubstitution(text="l0_v3"),
            ),
            DeclareLaunchArgument(
                "body_name",
                default_value=TextSubstitution(text="base_link"),
            ),
            Node(
                package="mujoco",
                executable="mujoco_simulate",
                name="mujoco",
                output="screen",
                parameters=[{"mujoco_scene": legged_robot_scene}],
                arguments=[legged_robot_scene, body_name],
                prefix=prefix_cmd,
            ),
        ]
    )
