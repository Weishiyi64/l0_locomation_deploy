from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_file = PathJoinSubstitution([FindPackageShare("system_command_arbiter"), "config", "l0_v3.yaml"])
    return LaunchDescription(
        [
            Node(
                package="system_command_arbiter",
                executable="mock_motor_driver_node",
                name="mock_motor_driver_node",
                namespace="debug/actuator/motor",
                output="screen",
            ),
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
                        "allow_actuation": True,
                        "pilot_motor_type": 4,
                        "pilot_bus_index": 0,
                        "pilot_motor_id": 5,
                    }
                ],
                remappings=[
                    ("motor_command", "/debug/actuator/motor/command"),
                    ("motor_state", "/debug/actuator/motor/state"),
                    ("set_motor_enabled", "/debug/actuator/motor/set_enabled"),
                ],
            ),
        ]
    )
