#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('l0_v5_description')
    with open(os.path.join(share_dir, 'urdf', 'l0_v5.urdf'), 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 30.0,
                'robot_description': ParameterValue(robot_description, value_type=str),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(share_dir, 'rviz', 'urdf.rviz')],
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
