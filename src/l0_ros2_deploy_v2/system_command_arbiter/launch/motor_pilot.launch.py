from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_file = LaunchConfiguration("mapping_file")
    allow_actuation = LaunchConfiguration("allow_actuation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("system_command_arbiter"), "config", "l0_v3.yaml"]
                ),
            ),
            DeclareLaunchArgument("allow_actuation", default_value="false"),
            DeclareLaunchArgument("pilot_motor_type", default_value="4"),
            DeclareLaunchArgument("pilot_bus_index", default_value="4"),
            DeclareLaunchArgument("pilot_motor_id", default_value="1"),
            Node(
                package="system_command_arbiter",
                executable="system_command_arbiter_node",
                name="system_command_arbiter_node",
                namespace="system/command_arbiter",
                output="screen",
                parameters=[
                    {
                        "mapping_file": mapping_file,
                        "pilot_mode": True,
                        "allow_actuation": ParameterValue(allow_actuation, value_type=bool),
                        "pilot_motor_type": ParameterValue(LaunchConfiguration("pilot_motor_type"), value_type=int),
                        "pilot_bus_index": ParameterValue(LaunchConfiguration("pilot_bus_index"), value_type=int),
                        "pilot_motor_id": ParameterValue(LaunchConfiguration("pilot_motor_id"), value_type=int),
                    }
                ],
                remappings=[
                    ("motor_command", "/actuator/motor/command"),
                    ("motor_state", "/actuator/motor/state"),
                    ("set_motor_enabled", "/actuator/motor/set_enabled"),
                ],
            ),
        ]
    )
