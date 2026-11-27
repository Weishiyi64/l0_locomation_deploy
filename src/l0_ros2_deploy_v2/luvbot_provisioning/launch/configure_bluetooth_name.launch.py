from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='luvbot_provisioning',
                executable='luvbot_bridge',
                name='luvbot_name_configurator',
                output='screen',
                parameters=[{'configure_name_only': True}],
            ),
        ]
    )
