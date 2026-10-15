#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    standalone = LaunchConfiguration('standalone')

    declare = [
        DeclareLaunchArgument('robot_type', default_value='l0_v3'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('standalone', default_value='true'),
    ]

    def _launch(context, *args, **kwargs):
        rt = robot_type.perform(context)
        desc_pkg = f"{rt}_description"

        desc_share = get_package_share_directory(desc_pkg)
        urdf_file = os.path.join(desc_share, 'urdf', f'{rt}.urdf')
        rviz_config = os.path.join(desc_share, 'rviz', 'urdf.rviz')

        # 从文件读取 robot_description（ROS2: 直接读文件，避免 Command 拼接导致 cat/path 错误）
        with open(urdf_file, 'r') as f:
            robot_description = f.read()

        nodes = []

        # standalone: joint_state_publisher_gui + robot_state_publisher
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                condition=IfCondition(standalone),
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                condition=IfCondition(standalone),
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'publish_frequency': 30.0},
                    {'robot_description': robot_description},
                ],
            )
        )

        # non-standalone: 只启动 robot_state_publisher
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                condition=UnlessCondition(standalone),
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_description': robot_description},
                ],
            )
        )

        # always launch RViz2
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                # required=true 的效果：rviz 退出则整体结束（ROS2 用 on_exit 机制更精细；这里先不强加）
            )
        )

        return nodes

    return LaunchDescription(declare + [OpaqueFunction(function=_launch)])
