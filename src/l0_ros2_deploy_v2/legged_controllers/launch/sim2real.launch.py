#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _configure_environment(context):
    mode = context.launch_configurations['hpm_mode']
    if mode not in ('real', 'isolated'):
        raise RuntimeError('hpm_mode must be real or isolated')
    if mode == 'isolated':
        domain = int(context.launch_configurations['isolated_ros_domain'])
        if not 1 <= domain <= 232:
            raise RuntimeError('isolated_ros_domain must be between 1 and 232')
        return [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
                SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
                LogInfo(msg=f'HPM isolated endpoints; ROS_DOMAIN_ID={domain}, ROS_LOCALHOST_ONLY=1')]
    return []


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    radar_control_enabled = LaunchConfiguration('radar_control_enabled')
    nav_planner_model_path = LaunchConfiguration('nav_planner_model_path')
    planner_depth_input_enabled = LaunchConfiguration('planner_depth_input_enabled')
    raw_depth_preview_enabled = LaunchConfiguration('raw_depth_preview_enabled')
    wbc_enabled = LaunchConfiguration('wbc_enabled')
    wbc_policies = LaunchConfiguration('wbc_policies')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_name = LaunchConfiguration('bag_name')
    rviz = LaunchConfiguration('rviz')
    battery_enabled = LaunchConfiguration('battery_enabled')
    imu_enabled = LaunchConfiguration('imu_enabled')
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
    safety_orientation_roll_limit_deg = LaunchConfiguration('safety_orientation_roll_limit_deg')
    safety_orientation_pitch_limit_deg = LaunchConfiguration('safety_orientation_pitch_limit_deg')
    safety_orientation_consecutive_violation_count = LaunchConfiguration(
        'safety_orientation_consecutive_violation_count'
    )

    args = [
        DeclareLaunchArgument('robot_type', default_value='l0_v5'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v5_amp_10281'),
        DeclareLaunchArgument('amp_to_wbc_transition_sec', default_value='1.0'),
        DeclareLaunchArgument('wbc_to_amp_transition_sec', default_value='0.35'),
        DeclareLaunchArgument('radar_control_enabled', default_value='false'),
        DeclareLaunchArgument(
            'nav_planner_model_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('legged_deploy'),
                    'policy',
                    robot_type,
                    'local_planner',
                    'planner_09090.onnx',
                ]
            ),
        ),
        DeclareLaunchArgument('planner_depth_input_enabled', default_value='false'),
        DeclareLaunchArgument('raw_depth_preview_enabled', default_value='true'),
        DeclareLaunchArgument('cloud_voltage_test_enabled', default_value='false'),
        DeclareLaunchArgument('cloud_device_id', default_value='LUV2026090001'),
        DeclareLaunchArgument('cloud_host', default_value='mqtt.luvbotics.com'),
        DeclareLaunchArgument('cloud_cert_dir', default_value=''),
        DeclareLaunchArgument('wbc_enabled', default_value='true'),
        DeclareLaunchArgument('joystick_wbc_enabled', default_value=LaunchConfiguration('wbc_enabled')),
        DeclareLaunchArgument('joystick_wbc_config_path', default_value=PathJoinSubstitution(
            [FindPackageShare('legged_controllers'), 'config', 'joystick_wbc_actions.yaml'])),
        DeclareLaunchArgument('wbc_json_base_action', default_value=''),
        DeclareLaunchArgument('wbc_mode', default_value='lora'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora_fp16'),
        DeclareLaunchArgument('hpm_mode', default_value='real'),
        DeclareLaunchArgument('isolated_ros_domain', default_value='97'),
        DeclareLaunchArgument('wbc_policies', default_value='["L0_Hugme", "L0_act_cute_1", "L0_act_cute_2", "L0_act_cute_3", "L0_act_cute_7", "L0_act_cute_new_1", "L0_act_cute_new_2", "L0_act_cute_new_3", "L0_act_cute_new_4", "L0_tired_hugme", "L0_NotLeave", "L0_SayNo_17", "L0_angry_10", "L0_angry_7", "L0_angry_8", "L0_angry_9", "L0_angry_baoshouqiaojiao", "L0_angry_chayaotaitou", "L0_angry_new_1", "L0_angry_new_2", "L0_peace_1", "L0_peace_2", "L0_peace_new_01_A", "L0_peace_new_02_A", "L0_peace_new_03_A", "L0_peace_new_04_A", "L0_peace_new_05_A", "L0_peace_new_06_A", "L0_peace_new_07_A", "L0_peace_new_08_A", "L0_16_strange", "L0_strange_1", "L0_strange_new_1_A", "L0_strange_new_2_A", "L0_strange_new_3_A", "L0_strange_new_4", "L0_strange_new_5", "L0_strange_new_5_1", "L0_strange_new_6_A", "L0_strange_new_8", "L0_Approve_13", "L0_excited_1", "L0_excited_7", "L0_excited_8", "L0_excited_new_1", "L0_excited_new_2", "L0_excited_new_3", "L0_excited_new_4", "L0_excited_new_5", "L0_excited_new_6", "L0_AfraidToLeave", "L0_SayNo_2", "L0_fear_10", "L0_fear_7_A", "L0_fear_8", "L0_fear_8_new_", "L0_fear_9", "L0_fear_defend", "L0_fear_new_1", "L0_fear_new_2", "L0_sad_1", "L0_sad_10", "L0_sad_7", "L0_sad_8", "L0_sad_9", "L0_sad_new_1", "L0_sad_new_2", "L0_sad_new_3", "L0_sad_new_4", "L0_sad_wanyao", "L0_14_satisfied", "L0_Approve_24", "L0_Approve_26", "L0_Approve_8", "L0_delicious", "L0_satisfied_10", "L0_satisfied_7", "L0_satisfied_8", "L0_satisfied_9", "L0_satisfied_new_1", "L0_crouch_off", "A-C04_mod", "A-C07_mod", "A-C08_mod", "A-S04_mod", "A-T08-02_mod"]'),
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
        DeclareLaunchArgument('joystick_imitation_enabled', default_value='false'),
        DeclareLaunchArgument(
            'joystick_imitation_config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_controllers'), 'config', 'joystick_imitation_actions.yaml']
            ),
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
        DeclareLaunchArgument('emergency_damping_kd', default_value='0.2'),
        DeclareLaunchArgument('arm_swing_enabled', default_value='false'),
        DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.1'),
        DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
        DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
        DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.5'),
        DeclareLaunchArgument('safety_orientation_roll_limit_deg', default_value='30.0'),
        DeclareLaunchArgument('safety_orientation_pitch_limit_deg', default_value='360.0'),
        DeclareLaunchArgument('safety_orientation_consecutive_violation_count', default_value='10'),
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
            head_script_enabled,
            head_script_config,
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
            radar_control_enabled,
            nav_planner_model_path,
            planner_depth_input_enabled,
            raw_depth_preview_enabled,
            wbc_enabled,
            wbc_policies,
            LaunchConfiguration('wbc_mode'),
            LaunchConfiguration('wbc_lora_bundle'),
            LaunchConfiguration('hpm_mode'),
            LaunchConfiguration('wbc_json_base_action'),
            LaunchConfiguration('joystick_wbc_enabled'),
            LaunchConfiguration('joystick_wbc_config_path'),
            LaunchConfiguration('amp_to_wbc_transition_sec'),
            LaunchConfiguration('wbc_to_amp_transition_sec'),
        ],
        output='screen',
        sigterm_timeout='15',
        sigkill_timeout='5',
    )

    cloud_voltage_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('luvbot_cloud_bridge'),
                    'launch',
                    'voltage_test.launch.py',
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration('cloud_voltage_test_enabled')),
        launch_arguments={
            'device_id': LaunchConfiguration('cloud_device_id'),
            'host': LaunchConfiguration('cloud_host'),
            'cert_dir': LaunchConfiguration('cloud_cert_dir'),
        }.items(),
    )

    return LaunchDescription(args + [OpaqueFunction(function=_configure_environment), sequential_launch, cloud_voltage_test])
