#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    wbc_frontend_enabled = LaunchConfiguration('wbc_enabled')
    txt2motion_enabled = LaunchConfiguration('txt2motion_enabled')
    wbc_enabled = PythonExpression([
        "str('", LaunchConfiguration('wbc_enabled'), "'.lower() == 'true' or '",
        txt2motion_enabled, "'.lower() == 'true').lower()"])
    wbc_policies = LaunchConfiguration('wbc_policies')
    wbc_lora_bundle = LaunchConfiguration('wbc_lora_bundle')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_name = LaunchConfiguration('bag_name')
    rviz = LaunchConfiguration('rviz')
    imitation_policies = LaunchConfiguration('imitation_policies')
    emergency_damping_kd = LaunchConfiguration('emergency_damping_kd')
    arm_swing_enabled = LaunchConfiguration('arm_swing_enabled')
    arm_swing_stop_vx_threshold = LaunchConfiguration('arm_swing_stop_vx_threshold')
    arm_swing_high_vx_threshold = LaunchConfiguration('arm_swing_high_vx_threshold')
    arm_swing_vx_hysteresis = LaunchConfiguration('arm_swing_vx_hysteresis')
    arm_swing_blend_sec = LaunchConfiguration('arm_swing_blend_sec')

    args = [
        DeclareLaunchArgument('robot_type', default_value='l0_v5'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v5_radar_velocity_tracking'),
        DeclareLaunchArgument('wbc_enabled', default_value='true'),
        DeclareLaunchArgument('txt2motion_enabled', default_value='true'),
        DeclareLaunchArgument('wbc_policies', default_value='[]'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora_fp16'),
        DeclareLaunchArgument('bag_enabled', default_value='false'),
        DeclareLaunchArgument(
            'bag_name',
            default_value=PythonExpression(["'", subpolicy, "_mujoco'"]),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('imitation_policies', default_value='[]'),
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
            imitation_policies,
            arm_swing_enabled,
            arm_swing_stop_vx_threshold,
            arm_swing_high_vx_threshold,
            arm_swing_vx_hysteresis,
            arm_swing_blend_sec,
            emergency_damping_kd,
            wbc_enabled,
            wbc_policies,
            wbc_lora_bundle,
            wbc_frontend_enabled,
            txt2motion_enabled,
        ],
        output='screen',
        sigterm_timeout='15',
        sigkill_timeout='5',
    )

    return LaunchDescription(args + [sequential_launch])
