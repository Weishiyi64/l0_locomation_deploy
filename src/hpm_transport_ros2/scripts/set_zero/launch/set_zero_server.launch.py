from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="set_zero", executable="set_zero_server", output="screen"),
    ])
