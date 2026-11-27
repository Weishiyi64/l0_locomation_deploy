#!/usr/bin/env python3
"""Odin sensors, target following and the RK3588 recurrent depth planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_nodes(context):
    robot = LaunchConfiguration('robot_type').perform(context)
    check_sensor_timestamps = LaunchConfiguration('nav_check_sensor_timestamps').perform(context).lower()
    if check_sensor_timestamps not in ('true', 'false'):
        raise RuntimeError('nav_check_sensor_timestamps must be true or false')
    sensor_parameters = {'nav_check_sensor_timestamps': check_sensor_timestamps == 'true'}
    reference_receive_timeout = LaunchConfiguration('nav_reference_receive_timeout').perform(context).lower()
    if reference_receive_timeout not in ('true', 'false'):
        raise RuntimeError('nav_reference_receive_timeout must be true or false')
    description = os.path.join(get_package_share_directory(robot+'_description'), 'urdf', robot+'.urdf')
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('odin_ros_driver_rev1'), 'launch', 'driver.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_radar_driver')),
        launch_arguments={'post_process': 'true', 'undistort_mapping_only': 'true'}.items(),
    )
    sensors = Node(package='legged_controllers', executable='planner_sensor_bridge',
                   parameters=[{'robot_description_path': description}, sensor_parameters],
                   output='screen')
    goals = Node(package='legged_controllers', executable='goal_to_planner',
                 parameters=[sensor_parameters, {'nav_reference_receive_timeout': reference_receive_timeout == 'true'}],
                 output='screen')
    planner = Node(package='legged_deploy', executable='local_planner',
                   parameters=[{'model_path': LaunchConfiguration('planner_model_path'), 'input_timeout': .5},
                               sensor_parameters],
                   output='screen')
    mode = Node(package='legged_controllers', executable='radar_map_mode', output='screen')
    return [driver, sensors, goals, planner, mode]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='l0_v5'),
        DeclareLaunchArgument('planner_model_path'),
        DeclareLaunchArgument('start_radar_driver', default_value='true'),
        DeclareLaunchArgument('nav_check_sensor_timestamps', default_value='true'),
        DeclareLaunchArgument('nav_reference_receive_timeout', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
