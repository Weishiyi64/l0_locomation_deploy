#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Fixed shared gamepad: joy_node publishes raw input, then the adapter
    # gates /joy for the robot and drives physical head yaw/pitch in head mode.
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
                remappings=[('joy', '/joy_raw')],
            ),
            Node(
                package='legged_controllers',
                executable='shared_gamepad_head_controller.py',
                name='shared_gamepad_head_controller',
                output='screen',
                parameters=[
                    {
                        'input_joy_topic': '/joy_raw',
                        'robot_joy_topic': '/joy',
                        'head_target_topic': '/head_joystick',
                        'toggle_lt_button': 8,
                        'toggle_rt_button': 9,
                        'axis_head_yaw': 0,
                        'axis_head_pitch': 1,
                        'deadzone': 0.12,
                        'update_rate_hz': 10.0,
                        'head_target_span': 0.5,
                    }
                ],
            ),
        ]
    )
