from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('device_sn', default_value='RUMI-TEST-007'),
            Node(
                package='luvbot_provisioning',
                executable='luvbot_bridge',
                output='screen',
                parameters=[{'device_sn': ParameterValue(LaunchConfiguration('device_sn'), value_type=str)}],
            ),
        ]
    )
