#!/usr/bin/env python3
"""Relative follow-goal bridge and RK3588 planner without replacing one_start."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('planner_model_path'),
        DeclareLaunchArgument('planner_depth_input_enabled', default_value='false'),
        # Compatibility-only: full-resolution preview now runs inside Odin's
        # pointcloud_to_depth node instead of this launch file.
        DeclareLaunchArgument('raw_depth_preview_enabled', default_value='false'),
        DeclareLaunchArgument('raw_depth_topic', default_value='/manifold/ODIN2/device0/pointcloud2depth/depth'),
    ]

    planner = Node(
        package='legged_deploy',
        executable='local_planner_rknn',
        name='local_planner',
        parameters=[
            {
                'model_path': LaunchConfiguration('planner_model_path'),
                'input_timeout': 0.5,
            }
        ],
        output='screen',
    )
    goal_manager = Node(
        package='legged_controllers',
        executable='goal_to_planner',
        name='goal_to_planner',
        output='screen',
    )
    planner_depth = Node(
        package='legged_controllers',
        executable='planner_depth_preprocessor',
        name='planner_depth_preprocessor',
        condition=IfCondition(LaunchConfiguration('planner_depth_input_enabled')),
        output='screen',
    )
    return LaunchDescription(args + [planner, goal_manager, planner_depth])
