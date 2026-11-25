#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    def _launch(context, *args, **kwargs):
        share_dir = get_package_share_directory('y1_v3_description')
        urdf_file = os.path.join(share_dir, 'urdf', 'y1_v3.urdf')

        with open(urdf_file, 'r') as f:
            robot_description = f.read()

        rviz_config = os.path.join(share_dir, 'rviz', 'urdf.rviz')

        return [
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                condition=IfCondition(gui),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'publish_frequency': 30.0},
                    {'robot_description': robot_description},
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(rviz),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch)])
