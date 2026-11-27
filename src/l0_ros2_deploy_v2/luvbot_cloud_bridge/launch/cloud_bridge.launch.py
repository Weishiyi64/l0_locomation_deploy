from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('device_id', default_value='RUMI-TEST-007'),
            DeclareLaunchArgument('host', default_value='mqtt.luvbotics.com'),
            DeclareLaunchArgument(
                'cert_dir',
                default_value='',
            ),
            DeclareLaunchArgument('ca_file', default_value='/etc/ssl/certs/ca-certificates.crt'),
            DeclareLaunchArgument('wake_timeout_sec', default_value='10.0'),
            DeclareLaunchArgument('health_overheat_c', default_value='85.0'),
            DeclareLaunchArgument('health_recover_c', default_value='80.0'),
            Node(
                package='luvbot_cloud_bridge',
                executable='cloud_bridge_node',
                output='screen',
                parameters=[
                    {
                        'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=str),
                        'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
                        'cert_dir': ParameterValue(LaunchConfiguration('cert_dir'), value_type=str),
                        'ca_file': ParameterValue(LaunchConfiguration('ca_file'), value_type=str),
                        'wake_timeout_sec': ParameterValue(LaunchConfiguration('wake_timeout_sec'), value_type=float),
                        'health.overheat_c': ParameterValue(LaunchConfiguration('health_overheat_c'), value_type=float),
                        'health.recover_c': ParameterValue(LaunchConfiguration('health_recover_c'), value_type=float),
                    }
                ],
            ),
        ]
    )
