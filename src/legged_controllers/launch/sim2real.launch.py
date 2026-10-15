#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_name = LaunchConfiguration('bag_name')
    rviz = LaunchConfiguration('rviz')
    battery_enabled = LaunchConfiguration('battery_enabled')
    imu_enabled = LaunchConfiguration('imu_enabled')
    head_script_enabled = LaunchConfiguration('head_script_enabled')
    head_script_config = LaunchConfiguration('head_script_config')
    head_topic_config = LaunchConfiguration('head_topic_config')
    emotion_sequence_enabled = LaunchConfiguration('emotion_sequence_enabled')
    emotion_sequence_map_path = LaunchConfiguration('emotion_sequence_map_path')
    keyboard_action_enabled = LaunchConfiguration('keyboard_action_enabled')
    keyboard_action_config_path = LaunchConfiguration('keyboard_action_config_path')
    joystick_imitation_enabled = LaunchConfiguration('joystick_imitation_enabled')
    joystick_imitation_config_path = LaunchConfiguration('joystick_imitation_config_path')
    imitation_entry_blend_sec = LaunchConfiguration('imitation_entry_blend_sec')
    policy_root_path = LaunchConfiguration('policy_root_path')
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
    safety_orientation_roll_limit_deg = LaunchConfiguration('safety_orientation_roll_limit_deg')
    safety_orientation_pitch_limit_deg = LaunchConfiguration('safety_orientation_pitch_limit_deg')
    safety_orientation_consecutive_violation_count = LaunchConfiguration(
        'safety_orientation_consecutive_violation_count'
    )

    args = [
        DeclareLaunchArgument('robot_type', default_value='l0_v3'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v3_amp_07090'),
        DeclareLaunchArgument('bag_enabled', default_value='false'),
        DeclareLaunchArgument(
            'bag_name',
            default_value=PythonExpression(["'", subpolicy, "_real'"]),
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('battery_enabled', default_value='false'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('head_script_enabled', default_value='false'),
        DeclareLaunchArgument(
            'head_script_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'pcmd_yaml', 'l0_keepalone.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'head_topic_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'pcmd_yaml', 'head_topic.yaml']
            ),
        ),
        DeclareLaunchArgument('emotion_sequence_enabled', default_value='false'),
        DeclareLaunchArgument(
            'emotion_sequence_map_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'emotion_sequence_map.yaml']
            ),
        ),
        DeclareLaunchArgument('keyboard_action_enabled', default_value='false'),
        DeclareLaunchArgument(
            'keyboard_action_config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'keyboard_action_map.yaml']
            ),
        ),
        DeclareLaunchArgument('joystick_imitation_enabled', default_value='true'),
        DeclareLaunchArgument(
            'imitation_entry_blend_sec',
            default_value='0.8',
            description='Seconds to smoothly hand off AMP/RL commands to an imitation action; 0 disables it.',
        ),
        DeclareLaunchArgument(
            'joystick_imitation_config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'joystick_imitation_actions.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'policy_root_path',
            default_value=PathJoinSubstitution([FindPackageShare('legged_deploy'), 'policy']),
        ),
        DeclareLaunchArgument('joystick_sequence_enabled', default_value='false'),
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
        DeclareLaunchArgument('emergency_damping_kd', default_value='0.5'),
        DeclareLaunchArgument('arm_swing_enabled', default_value='false'),
        DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.1'),
        DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
        DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
        DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.5'),
        DeclareLaunchArgument('safety_orientation_roll_limit_deg', default_value='360.0'),
        DeclareLaunchArgument('safety_orientation_pitch_limit_deg', default_value='360.0'),
        DeclareLaunchArgument('safety_orientation_consecutive_violation_count', default_value='1000'),
    ]

    keyboard_trigger = Node(
        package='legged_controllers',
        executable='keyboard_action_trigger.py',
        name='keyboard_action_trigger',
        condition=IfCondition(keyboard_action_enabled),
        parameters=[
            {'config_path': keyboard_action_config_path},
            {'trigger_topic': '/keyboard_action_trigger'},
        ],
        output='screen',
        emulate_tty=True,
    )

    # 调用 sequential_launch_real.sh 脚本：按顺序进行 CAN 配置、硬件桥接、主控制器启动
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
            head_script_enabled,
            head_script_config,
            head_topic_config,
            emotion_sequence_enabled,
            emotion_sequence_map_path,
            keyboard_action_enabled,
            keyboard_action_config_path,
            joystick_imitation_enabled,
            joystick_imitation_config_path,
            arm_swing_enabled,
            arm_swing_stop_vx_threshold,
            arm_swing_high_vx_threshold,
            arm_swing_vx_hysteresis,
            arm_swing_blend_sec,
            safety_orientation_roll_limit_deg,
            safety_orientation_pitch_limit_deg,
            safety_orientation_consecutive_violation_count,
            joystick_sequence_enabled,
            joystick_sequence_config_path,
            joystick_back_emergency_sequence_enabled,
            joystick_back_emergency_sequence_path,
            emergency_damping_kd,
            policy_root_path,
            imitation_entry_blend_sec,
        ],
        output='screen',
        sigterm_timeout='15',
        sigkill_timeout='5',
    )

    return LaunchDescription(args + [keyboard_trigger, sequential_launch])
