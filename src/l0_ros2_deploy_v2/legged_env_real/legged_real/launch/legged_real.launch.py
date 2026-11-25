import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # All launch args are supplied by sequential_launch_real.sh (from sim2real.launch.py); no in-file defaults.
    cfg = context.launch_configurations
    robot_type = cfg['robot_type']
    subpolicy = cfg['subpolicy']

    legged_controllers_share = get_package_share_directory('legged_controllers')
    command_arbiter_share = get_package_share_directory('system_command_arbiter')
    imu_enabled = cfg['imu_enabled'] == 'true'
    hpm_mode = cfg.get('hpm_mode', 'real')
    if hpm_mode not in ('real', 'isolated'):
        raise RuntimeError('hpm_mode must be real or isolated')
    if robot_type not in ('l0_v3', 'l0_v4', 'l0_v5'):
        raise RuntimeError(f'Unsupported HPM robot_type: {robot_type}')
    mapping_file = os.path.abspath(os.path.join(command_arbiter_share, 'config', f'{robot_type}.yaml'))
    head_script_enabled = cfg['head_script_enabled'] == 'true'
    head_script_config = cfg['head_script_config']
    joystick_imitation_enabled = cfg.get('joystick_imitation_enabled', 'false') == 'true'
    joystick_imitation_config_path = cfg.get('joystick_imitation_config_path', '')
    joystick_sequence_enabled = cfg.get('joystick_sequence_enabled', 'false') == 'true'
    joystick_sequence_config_path = cfg.get('joystick_sequence_config_path', '')
    radar_control_enabled = cfg.get('radar_control_enabled', 'false') == 'true'
    emergency_damping_kd = float(cfg.get('emergency_damping_kd', '5.0'))
    joystick_back_emergency_sequence_enabled = cfg.get('joystick_back_emergency_sequence_enabled', 'true') == 'true'
    joystick_back_emergency_sequence_path = cfg.get(
        'joystick_back_emergency_sequence_path',
        'action_sequences/walking_back_emergency.yaml',
    )
    arm_swing_enabled = cfg.get('arm_swing_enabled', 'true') == 'true'
    safety_orientation_roll_limit_deg = float(cfg['safety_orientation_roll_limit_deg'])
    safety_orientation_pitch_limit_deg = float(cfg['safety_orientation_pitch_limit_deg'])
    safety_orientation_consecutive_violation_count = int(cfg['safety_orientation_consecutive_violation_count'])

    legged_deploy_share = get_package_share_directory('legged_deploy')
    robot_cfg_path = os.path.abspath(
        os.path.join(legged_deploy_share, 'policy', robot_type, subpolicy, 'rl_config.yaml')
    )

    joy_cfg_path = os.path.abspath(
        os.path.join(legged_deploy_share, 'policy', robot_type, subpolicy, 'joy_config.yaml')
    )

    legged_controller_config_path = os.path.abspath(
        os.path.join(legged_controllers_share, 'config', 'legged_controller_config.yaml')
    )

    rviz = cfg['rviz'] == 'true'
    legged_real_params = {
        'mapping_file': mapping_file,
        'robot_type': robot_type,
        'robot_cfg_path': robot_cfg_path,
        'legged_controller_config_path': legged_controller_config_path,
        'joy_cfg_path': joy_cfg_path,
        'rviz': rviz,
        'isSim': False,
        '/RobotHw/imu_enabled': imu_enabled,
        'head_script_enabled': head_script_enabled,
        'head_script_config': head_script_config,
        'joystick_imitation_enabled': joystick_imitation_enabled,
        'joystick_imitation_config_path': joystick_imitation_config_path,
        'joystick_sequence_enabled': joystick_sequence_enabled,
        'joystick_sequence_config_path': joystick_sequence_config_path,
        'radar_control_enabled': radar_control_enabled,
        'face_follow_enabled': True,
        'wbc_enabled': cfg['wbc_enabled'] == 'true',
        'amp_to_wbc_transition_sec': float(cfg['amp_to_wbc_transition_sec']),
        'wbc_to_amp_transition_sec': float(cfg['wbc_to_amp_transition_sec']),
        'emergency.damping_kd': emergency_damping_kd,
        'joystick_back_emergency_sequence_enabled': joystick_back_emergency_sequence_enabled,
        'joystick_back_emergency_sequence_path': joystick_back_emergency_sequence_path,
        'arm_swing.enabled': arm_swing_enabled,
        'arm_swing.stop_vx_threshold': float(cfg.get('arm_swing_stop_vx_threshold', '0.05')),
        'arm_swing.high_vx_threshold': float(cfg.get('arm_swing_high_vx_threshold', '0.8')),
        'arm_swing.vx_hysteresis': float(cfg.get('arm_swing_vx_hysteresis', '0.03')),
        'arm_swing.blend_sec': float(cfg.get('arm_swing_blend_sec', '0.35')),
        'safety.orientation.roll_limit_deg': safety_orientation_roll_limit_deg,
        'safety.orientation.pitch_limit_deg': safety_orientation_pitch_limit_deg,
        'safety.orientation.consecutive_violation_count': safety_orientation_consecutive_violation_count,
    }
    legged_real_node = Node(
        package='legged_real',
        executable='topic_hardware_adapter',
        name='topic_hardware_adapter',
        output='screen',
        parameters=[legged_real_params],
        remappings=[
            ('emergency_stop', '/system/safety/emergency_stop'),
            ('hardware_fault', '/system/safety/hardware_fault'),
            ('motors_ready', '/hardware_status/motors_ready'),
        ] + ([
            ('actuator/motor/state', '/wbc_isolated/actuator/motor/state'),
            ('actuator/motor/command', '/wbc_isolated/actuator/motor/command'),
            ('actuator/motor/enable', '/wbc_isolated/actuator/motor/enable'),
            ('actuator/motor/set_zero', '/wbc_isolated/actuator/motor/set_zero'),
            ('imu/data', '/wbc_isolated/imu/data'),
            ('odom', '/wbc_isolated/odom'),
        ] if hpm_mode == 'isolated' else []),
    )

    return [legged_real_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_type'),
            DeclareLaunchArgument('subpolicy'),
            DeclareLaunchArgument('rviz'),
            DeclareLaunchArgument('imu_enabled'),
            DeclareLaunchArgument('head_script_enabled'),
            DeclareLaunchArgument('head_script_config'),
            DeclareLaunchArgument('joystick_imitation_enabled', default_value='false'),
            DeclareLaunchArgument('joystick_imitation_config_path', default_value=''),
            DeclareLaunchArgument('wbc_enabled', default_value='false'),
            DeclareLaunchArgument('amp_to_wbc_transition_sec', default_value='1.0'),
            DeclareLaunchArgument('wbc_to_amp_transition_sec', default_value='0.35'),
            DeclareLaunchArgument('hpm_mode', default_value='real'),
            DeclareLaunchArgument('joystick_sequence_enabled', default_value='false'),
            DeclareLaunchArgument('joystick_sequence_config_path', default_value=''),
            DeclareLaunchArgument('radar_control_enabled', default_value='false'),
            DeclareLaunchArgument('emergency_damping_kd', default_value='5.0'),
            DeclareLaunchArgument('joystick_back_emergency_sequence_enabled', default_value='true'),
            DeclareLaunchArgument(
                'joystick_back_emergency_sequence_path',
                default_value='action_sequences/walking_back_emergency.yaml',
            ),
            DeclareLaunchArgument('arm_swing_enabled', default_value='true'),
            DeclareLaunchArgument('arm_swing_stop_vx_threshold', default_value='0.05'),
            DeclareLaunchArgument('arm_swing_high_vx_threshold', default_value='0.8'),
            DeclareLaunchArgument('arm_swing_vx_hysteresis', default_value='0.03'),
            DeclareLaunchArgument('arm_swing_blend_sec', default_value='0.35'),
            DeclareLaunchArgument('safety_orientation_roll_limit_deg'),
            DeclareLaunchArgument('safety_orientation_pitch_limit_deg'),
            DeclareLaunchArgument('safety_orientation_consecutive_violation_count'),
            OpaqueFunction(function=launch_setup),
        ]
    )
