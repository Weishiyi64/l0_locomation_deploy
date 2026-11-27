#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch_ros.actions import Node


def _legged_mujoco_sim_setup(context, *args, **kwargs):
    cfg = context.launch_configurations

    legged_controllers_share = get_package_share_directory('legged_controllers')
    command_arbiter_share = get_package_share_directory('system_command_arbiter')
    robot_type = cfg['robot_type']
    subpolicy = cfg['subpolicy']
    legged_deploy_share = get_package_share_directory('legged_deploy')

    robot_cfg_path = os.path.abspath(
        os.path.join(legged_deploy_share, 'policy', robot_type, subpolicy, 'rl_config.yaml')
    )

    legged_controller_config_path = os.path.abspath(
        os.path.join(legged_controllers_share, 'config', 'legged_controller_config.yaml')
    )
    joy_cfg_path = os.path.abspath(
        os.path.join(legged_deploy_share, 'policy', robot_type, subpolicy, 'joy_config.yaml')
    )
    mapping_file = os.path.abspath(os.path.join(command_arbiter_share, 'config', f'{robot_type}.yaml'))

    start_delay = float(cfg.get('start_delay', '1.0'))

    adapter_node = Node(
        package='legged_real',
        executable='topic_hardware_adapter',
        name='topic_hardware_adapter',
        output='screen',
        on_exit=Shutdown(reason='topic hardware adapter exited'),
        parameters=[
            {'mapping_file': mapping_file},
            {'robot_type': robot_type},
            {'robot_cfg_path': robot_cfg_path},
            {'legged_controller_config_path': legged_controller_config_path},
            {'joy_cfg_path': joy_cfg_path},
            {'isSim': True},
            {'wbc_enabled': cfg['wbc_enabled'] == 'true'},
            {'/RobotHw/imu_enabled': True},
            {'motor_lifecycle_period_ms': 1},
            {'rviz': cfg['rviz'] == 'true'},
            {'emergency.damping_kd': float(cfg.get('emergency_damping_kd', '5.0'))},
            {'arm_swing.enabled': cfg.get('arm_swing_enabled', 'true') == 'true'},
            {'arm_swing.stop_vx_threshold': float(cfg.get('arm_swing_stop_vx_threshold', '0.05'))},
            {'arm_swing.high_vx_threshold': float(cfg.get('arm_swing_high_vx_threshold', '0.8'))},
            {'arm_swing.vx_hysteresis': float(cfg.get('arm_swing_vx_hysteresis', '0.03'))},
            {'arm_swing.blend_sec': float(cfg.get('arm_swing_blend_sec', '0.35'))},
        ],
        remappings=[
            ('emergency_stop', '/system/safety/emergency_stop'),
            ('hardware_fault', '/system/safety/hardware_fault'),
            ('motors_ready', '/hardware_status/motors_ready'),
        ],
    )

    return [
        TimerAction(
            period=start_delay,
            actions=[adapter_node],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_type', default_value='l0_v3'),
            DeclareLaunchArgument('subpolicy', default_value='l0_v3_amp_06152'),
            DeclareLaunchArgument('start_delay', default_value='1.0'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument('wbc_enabled', default_value='false'),
            DeclareLaunchArgument('emergency_damping_kd', default_value='5.0'),
            DeclareLaunchArgument('arm_swing_enabled', default_value='true'),
            DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.05'),
            DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
            DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
            DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.35'),
            OpaqueFunction(function=_legged_mujoco_sim_setup),
        ]
    )
