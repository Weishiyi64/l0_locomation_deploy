#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    wbc_enabled = LaunchConfiguration('wbc_enabled')
    wbc_policies = LaunchConfiguration('wbc_policies')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_name = LaunchConfiguration('bag_name')
    rviz = LaunchConfiguration('rviz')
    head_script_enabled = LaunchConfiguration('head_script_enabled')
    head_script_config = LaunchConfiguration('head_script_config')
    joystick_imitation_enabled = LaunchConfiguration('joystick_imitation_enabled')
    joystick_imitation_config_path = LaunchConfiguration('joystick_imitation_config_path')
    joystick_sequence_enabled = LaunchConfiguration('joystick_sequence_enabled')
    joystick_sequence_config_path = LaunchConfiguration('joystick_sequence_config_path')
    joystick_back_emergency_sequence_enabled = LaunchConfiguration('joystick_back_emergency_sequence_enabled')
    joystick_back_emergency_sequence_path = LaunchConfiguration('joystick_back_emergency_sequence_path')
    emergency_damping_kd = LaunchConfiguration('emergency_damping_kd')
    arm_swing_enabled = LaunchConfiguration('arm_swing_enabled')
    arm_swing_stop_vx_threshold = LaunchConfiguration('arm_swing_stop_vx_threshold')
    arm_swing_high_vx_threshold = LaunchConfiguration('arm_swing_high_vx_threshold')
    arm_swing_vx_hysteresis = LaunchConfiguration('arm_swing_vx_hysteresis')
    arm_swing_blend_sec = LaunchConfiguration('arm_swing_blend_sec')

    args = [
        DeclareLaunchArgument('robot_type', default_value='l0_v3'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v3_amp_06152'),
        DeclareLaunchArgument('wbc_enabled', default_value='false'),
        DeclareLaunchArgument('wbc_policies', default_value='[]'),
        DeclareLaunchArgument('bag_enabled', default_value='false'),
        DeclareLaunchArgument(
            'bag_name',
            default_value=PythonExpression(["'", subpolicy, "_mujoco'"]),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('head_script_enabled', default_value='false'),
        DeclareLaunchArgument(
            'head_script_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'pcmd_yaml', 'l0_keepalone.yaml']
            ),
        ),
        DeclareLaunchArgument('joystick_imitation_enabled', default_value='false'),
        DeclareLaunchArgument(
            'joystick_imitation_config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'joystick_imitation_actions.yaml']
            ),
        ),
        DeclareLaunchArgument('joystick_sequence_enabled', default_value='true'),
        DeclareLaunchArgument(
            'joystick_sequence_config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'joystick_sequence_actions.yaml']
            ),
        ),
        DeclareLaunchArgument('joystick_back_emergency_sequence_enabled', default_value='false'),
        DeclareLaunchArgument(
            'joystick_back_emergency_sequence_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'action_sequences', 'walking_back_emergency.yaml']
            ),
        ),
        DeclareLaunchArgument('emergency_damping_kd', default_value='5.0'),
        DeclareLaunchArgument('arm_swing_enabled', default_value='false'),
        DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.1'),
        DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
        DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
        DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.5'),
    ]

    sequential_launch = Node(
        package='legged_controllers',
        executable='sequential_launch_sim.sh',
        name='sequential_launcher_sim',
        arguments=[
            robot_type,
            bag_enabled,
            subpolicy,
            bag_name,
            rviz,
            head_script_enabled,
            head_script_config,
            joystick_imitation_enabled,
            joystick_imitation_config_path,
            arm_swing_enabled,
            arm_swing_stop_vx_threshold,
            arm_swing_high_vx_threshold,
            arm_swing_vx_hysteresis,
            arm_swing_blend_sec,
            joystick_sequence_enabled,
            joystick_sequence_config_path,
            joystick_back_emergency_sequence_enabled,
            joystick_back_emergency_sequence_path,
            emergency_damping_kd,
            wbc_enabled,
            wbc_policies,
        ],
        output='screen',
        sigterm_timeout='15',
        sigkill_timeout='5',
    )

    return LaunchDescription(args + [sequential_launch])
