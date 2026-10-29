#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node_robot',
                output='screen',
                parameters=[
                    {
                        'dev': '/dev/input/js0',
                        'dev_ff': '/dev/input/event8',
                        'deadzone': 1e-2,
                        'autorepeat_rate': 10.0,
                        'coalesce_interval': 0.05,
                    }
                ],
            ),
        ]
    )
