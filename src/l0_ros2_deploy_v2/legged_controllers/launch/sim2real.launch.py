#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


import sys
from pathlib import Path

launch_dir = str(Path(__file__).parent)
if launch_dir not in sys.path:
    sys.path.insert(0, launch_dir)
from deployment_config import apply_settings, prepare_deployment
from launch.actions import OpaqueFunction


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    radar_control_enabled = LaunchConfiguration('radar_control_enabled')
    nav_planner_model_path = LaunchConfiguration('nav_planner_model_path')
    nav_check_sensor_timestamps = LaunchConfiguration('nav_check_sensor_timestamps')
    nav_reference_receive_timeout = LaunchConfiguration('nav_reference_receive_timeout')
    wbc_frontend_enabled = LaunchConfiguration('wbc_enabled')
    txt2motion_enabled = LaunchConfiguration('txt2motion_enabled')
    wbc_enabled = PythonExpression([
        "str('", LaunchConfiguration('wbc_enabled'), "'.lower() == 'true' or '",
        txt2motion_enabled, "'.lower() == 'true').lower()"])
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_name = LaunchConfiguration('bag_name')
    rviz = LaunchConfiguration('rviz')
    battery_enabled = LaunchConfiguration('battery_enabled')
    imu_enabled = LaunchConfiguration('imu_enabled')
    imitation_policies = LaunchConfiguration('imitation_policies')
    emergency_damping_kd = LaunchConfiguration('emergency_damping_kd')
    arm_swing_enabled = LaunchConfiguration('arm_swing_enabled')
    arm_swing_stop_vx_threshold = LaunchConfiguration('arm_swing_stop_vx_threshold')
    arm_swing_high_vx_threshold = LaunchConfiguration('arm_swing_high_vx_threshold')
    arm_swing_vx_hysteresis = LaunchConfiguration('arm_swing_vx_hysteresis')
    arm_swing_blend_sec = LaunchConfiguration('arm_swing_blend_sec')
    safety_orientation_roll_limit_deg = LaunchConfiguration('safety_orientation_roll_limit_deg')
    safety_orientation_pitch_limit_deg = LaunchConfiguration('safety_orientation_pitch_limit_deg')
    safety_orientation_consecutive_violation_count = LaunchConfiguration(
        'safety_orientation_consecutive_violation_count'
    )

    args = [
        DeclareLaunchArgument('robot_type', default_value='l0_v5'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v5_amp_1124'),
        DeclareLaunchArgument('radar_control_enabled', default_value='false'),
        DeclareLaunchArgument(
            'nav_planner_model_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('legged_deploy'),
                    'policy',
                    robot_type,
                    'local_planner',
                    'model_1600_dense.onnx',
                ]
            ),
        ),
        DeclareLaunchArgument('nav_check_sensor_timestamps', default_value='false'),
        DeclareLaunchArgument('nav_reference_receive_timeout', default_value='true'),
        DeclareLaunchArgument('cloud_enabled', default_value='false'),
        DeclareLaunchArgument('cloud_device_id', default_value='RUMI-TEST-007'),
        DeclareLaunchArgument('cloud_host', default_value='mqtt.luvbotics.com'),
        DeclareLaunchArgument('cloud_cert_dir', default_value=''),
        DeclareLaunchArgument('cloud_wake_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('cloud_health_overheat_c', default_value='85.0'),
        DeclareLaunchArgument('cloud_health_recover_c', default_value='80.0'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora_fp16'),
        DeclareLaunchArgument('wbc_mode', default_value='auto'),
        DeclareLaunchArgument('wbc_policies', default_value='[]'),
        DeclareLaunchArgument('wbc_enabled', default_value='true'),
        DeclareLaunchArgument('txt2motion_enabled', default_value='true'),
        DeclareLaunchArgument('bag_enabled', default_value='false'),
        DeclareLaunchArgument(
            'bag_name',
            default_value=PythonExpression(["'", subpolicy, "_real'"]),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('battery_enabled', default_value='false'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('imitation_policies', default_value='[]'),
        DeclareLaunchArgument('emergency_damping_kd', default_value='0.1'),
        DeclareLaunchArgument('arm_swing_enabled', default_value='false'),
        DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.1'),
        DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
        DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
        DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.5'),
        DeclareLaunchArgument('safety_orientation_roll_limit_deg', default_value='45.0'),
        DeclareLaunchArgument('safety_orientation_pitch_limit_deg', default_value='45.0'),
        DeclareLaunchArgument('safety_orientation_consecutive_violation_count', default_value='200'),
    ]

    # 调用 sequential_launch_real.sh：按顺序启动话题硬件桥和主控制器
    sequential_launch = Node(
        package='legged_controllers',
        executable='sequential_launch_real.sh',
        name='sequential_launcher_real',
        arguments=[
            robot_type,
            bag_enabled,
            subpolicy,
            bag_name,
            rviz,
            battery_enabled,
            imu_enabled,
            imitation_policies,
            arm_swing_enabled,
            arm_swing_stop_vx_threshold,
            arm_swing_high_vx_threshold,
            arm_swing_vx_hysteresis,
            arm_swing_blend_sec,
            safety_orientation_roll_limit_deg,
            safety_orientation_pitch_limit_deg,
            safety_orientation_consecutive_violation_count,
            emergency_damping_kd,
            radar_control_enabled,
            nav_planner_model_path,
            wbc_enabled,
            LaunchConfiguration('wbc_policies'),
            nav_check_sensor_timestamps,
            nav_reference_receive_timeout,
            LaunchConfiguration('wbc_lora_bundle'),
            wbc_frontend_enabled,
            txt2motion_enabled,
        ],
        output='screen',
        sigterm_timeout='15',
        sigkill_timeout='5',
    )

    cloud_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('luvbot_cloud_bridge'),
                    'launch',
                    'cloud_bridge.launch.py',
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration('cloud_enabled')),
        launch_arguments={
            'device_id': LaunchConfiguration('cloud_device_id'),
            'host': LaunchConfiguration('cloud_host'),
            'cert_dir': LaunchConfiguration('cloud_cert_dir'),
            'wake_timeout_sec': LaunchConfiguration('cloud_wake_timeout_sec'),
            'health_overheat_c': LaunchConfiguration('cloud_health_overheat_c'),
            'health_recover_c': LaunchConfiguration('cloud_health_recover_c'),
        }.items(),
    )

    return LaunchDescription(apply_settings(args) + [OpaqueFunction(function=prepare_deployment), sequential_launch, cloud_bridge])
