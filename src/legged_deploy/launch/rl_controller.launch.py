#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    rl_config_path = LaunchConfiguration('rl_config_path')
    rl_controller_config_path = LaunchConfiguration('rl_controller_config_path')
    policy_type = LaunchConfiguration('policy_type')

    default_rl_config = PathJoinSubstitution(
        [FindPackageShare('legged_deploy'), 'policy', robot_type, subpolicy, 'rl_config.yaml']
    )
    default_rl_controller_config = PathJoinSubstitution(
        [FindPackageShare('legged_deploy'), 'config', 'rl_controller_config.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_type', default_value='l0_v3'),
            DeclareLaunchArgument('subpolicy', default_value=''),
            DeclareLaunchArgument('rl_config_path', default_value=default_rl_config),
            DeclareLaunchArgument('rl_controller_config_path', default_value=default_rl_controller_config),
            DeclareLaunchArgument(
                'policy_type',
                default_value='',
                description='If non-empty, overrides RobotCfg.policy_type (e.g. rl, rl_imitation)',
            ),
            Node(
                package='legged_deploy',
                executable='rl_controller',
                name='rl_controller',
                output='screen',
                parameters=[
                    {'robot_type': robot_type},
                    {'robot_cfg_path': rl_config_path},
                    {'subpolicy': subpolicy},
                    {'rl_controller_config_path': rl_controller_config_path},
                    {'policy_type': policy_type},
                ],
            ),
        ]
    )
