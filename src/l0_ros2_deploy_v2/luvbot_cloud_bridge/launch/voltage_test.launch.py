from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('device_id', default_value='LUV2026090001'),
            DeclareLaunchArgument('host', default_value='mqtt.luvbotics.com'),
            DeclareLaunchArgument(
                'cert_dir',
                default_value='',
            ),
            Node(
                package='luvbot_cloud_bridge',
                executable='voltage_mqtt_test_node',
                output='screen',
                parameters=[
                    {
                        'device_id': LaunchConfiguration('device_id'),
                        'host': LaunchConfiguration('host'),
                        'cert_dir': ParameterValue(LaunchConfiguration('cert_dir'), value_type=str),
                    }
                ],
            ),
        ]
    )
