from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_file = LaunchConfiguration("mapping_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("system_command_arbiter"), "config", "l0_v3.yaml"]
                ),
            ),
            Node(
                package="system_command_arbiter",
                executable="system_command_arbiter_node",
                name="system_command_arbiter_node",
                namespace="system/command_arbiter",
                output="screen",
                parameters=[{"mapping_file": mapping_file, "pilot_mode": False}],
                remappings=[
                    ("motor_command", "/actuator/motor/command"),
                    ("motor_state", "/actuator/motor/state"),
                    ("set_motor_enabled", "/actuator/motor/set_enabled"),
                    ("emergency_stop", "/system/safety/emergency_stop"),
                    ("hardware_fault", "/system/safety/hardware_fault"),
                ],
            ),
        ]
    )
