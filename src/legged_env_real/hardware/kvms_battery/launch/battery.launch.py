import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = context.launch_configurations.get('config_file', '')
    if not config_file:
        pkg_share = get_package_share_directory('kvms_battery')
        config_file = os.path.join(pkg_share, 'config', 'battery_config.yaml')

    battery_node = Node(
        package='kvms_battery',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[config_file],
    )

    return [battery_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value='', description='Path to battery configuration file'),
            OpaqueFunction(function=launch_setup),
        ]
    )
