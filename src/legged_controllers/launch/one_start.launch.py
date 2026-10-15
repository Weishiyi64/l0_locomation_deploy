#!/usr/bin/env python3
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)

from action_sequence_policy import collect_action_sequence_manifest  # noqa: E402


_LOCAL_TEXT_MOTION_PYTHON = '/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python'
_TEXT_MOTION_PYTHON = os.environ.get(
    'TEXT_MOTION_PYTHON_EXECUTABLE',
    _LOCAL_TEXT_MOTION_PYTHON if os.path.isfile(_LOCAL_TEXT_MOTION_PYTHON) else sys.executable,
)
_TEXT_MOTION_WBC_ROOT_OVERRIDE = os.environ.get('TEXT_MOTION_WBC_DEPLOY_ROOT', '').strip()
_TEXT_MOTION_WBC_POLICY_OVERRIDE = os.environ.get('TEXT_MOTION_WBC_POLICY_ONNX', '').strip()
_TEXT_MOTION_WBC_CONFIG_OVERRIDE = os.environ.get('TEXT_MOTION_WBC_POLICY_CONFIG', '').strip()
_TEXT_MOTION_TRTEXEC = os.environ.get('TEXT_MOTION_TRTEXEC', '/usr/src/tensorrt/bin/trtexec')


def _launch_setup(context, *args, **kwargs):
    cfg = context.launch_configurations
    share_leg = get_package_share_directory('legged_controllers')
    legged_deploy_share = get_package_share_directory('legged_deploy')
    l0_v3_description_share = get_package_share_directory('l0_v3_description')
    bundled_wbc_root = os.path.join(legged_deploy_share, 'third_party', 'wbc_deploy')
    bundled_wbc_policy = os.path.join(
        legged_deploy_share, 'resources', 'text_motion', 'stg25_1000_onnx', 'policy.onnx'
    )
    bundled_wbc_config = os.path.join(
        legged_deploy_share, 'resources', 'text_motion', 'stg25_1000_onnx', 'config.yaml'
    )
    default_policy_root_path = os.path.join(legged_deploy_share, 'policy')
    policy_root_path = cfg.get('policy_root_path', '').strip() or default_policy_root_path
    try:
        action_manifest = collect_action_sequence_manifest(
            keyboard_action_enabled=cfg.get('keyboard_action_enabled', 'false').lower() == 'true',
            keyboard_action_config_path=cfg.get('keyboard_action_config_path', ''),
            emotion_sequence_enabled=cfg.get('emotion_sequence_enabled', 'false').lower() == 'true',
            emotion_sequence_map_path=cfg.get('emotion_sequence_map_path', ''),
            package_share_legged_controllers=share_leg,
            joystick_imitation_enabled=cfg.get('joystick_imitation_enabled', 'false').lower() == 'true',
            joystick_imitation_config_path=cfg.get('joystick_imitation_config_path', ''),
            joystick_sequence_enabled=cfg.get('joystick_sequence_enabled', 'false').lower() == 'true',
            joystick_sequence_config_path=cfg.get('joystick_sequence_config_path', ''),
            robot_type=cfg.get('robot_type', ''),
            policy_root_path=policy_root_path,
        )
    except (RuntimeError, ValueError) as e:
        raise RuntimeError(f'[one_start.launch.py] {e}') from e
    imitation_policies = list(action_manifest.imitation_policies)
    preload_imitation_policies = list(action_manifest.preload_imitation_policies)
    skipped_imitation_policies = action_manifest.skipped_imitation_policies
    text_motion_enabled = cfg.get('text_motion_enabled', 'true').lower() == 'true'
    text_motion_runtime = cfg.get('text_motion_runtime', 'dynamic_trt').strip().lower()
    if text_motion_runtime not in ('dynamic_trt', 'legacy_packager'):
        raise RuntimeError('[one_start.launch.py] text_motion_runtime must be dynamic_trt or legacy_packager')
    dynamic_text_motion = text_motion_runtime == 'dynamic_trt'
    text_motion_policy = (
        'text_motion_wbc_stg25_dynamic'
        if dynamic_text_motion
        else cfg.get('text_motion_policy', 'text_motion_wbc_stg25').strip()
    )
    text_motion_amp_stand_profile = cfg.get('text_motion_amp_stand_profile', '').strip() or os.path.join(
        legged_deploy_share,
        'config',
        'amp_stand_profiles',
        'l0_v3_amp_stand_inner_arms_profile.json',
    )
    if text_motion_enabled:
        if not text_motion_policy:
            raise RuntimeError('[one_start.launch.py] text_motion_policy must not be empty')
        if not os.path.isfile(text_motion_amp_stand_profile):
            raise RuntimeError(
                f'[one_start.launch.py] text motion AMP stand profile is missing: '
                f'{text_motion_amp_stand_profile}'
            )
        packaging_inputs = {
            'python': cfg.get(
                'text_motion_python_executable',
                _TEXT_MOTION_PYTHON,
            ),
            'wbc_deploy': os.path.join(
                cfg.get('text_motion_wbc_deploy_root', '').strip() or bundled_wbc_root,
                'scripts', 'deploy', 'prepare_monolithic_deploy.sh',
            ),
            'policy': cfg.get('text_motion_wbc_policy_onnx', '').strip() or bundled_wbc_policy,
            'policy config': cfg.get('text_motion_wbc_policy_config', '').strip() or bundled_wbc_config,
            'MuJoCo model': cfg.get('text_motion_mujoco_model', '').strip() or os.path.join(
                l0_v3_description_share, 'mjcf', 'l0_v3_description.xml'
            ),
        }
        if not dynamic_text_motion:
            missing = [f'{label}: {path}' for label, path in packaging_inputs.items() if not os.path.isfile(path)]
            if missing:
                raise RuntimeError(f'[one_start.launch.py] missing text WBC packaging inputs: {missing}')
            for slot in (f'{text_motion_policy}_a', f'{text_motion_policy}_b'):
                if slot not in imitation_policies:
                    imitation_policies.append(slot)
        elif text_motion_policy not in imitation_policies:
            imitation_policies.append(text_motion_policy)
        imitation_policies.sort()

    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    rviz = LaunchConfiguration('rviz')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_name = LaunchConfiguration('bag_name')
    bag_prefix = LaunchConfiguration('bag_prefix')
    bag_topics = LaunchConfiguration('bag_topics')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('legged_controllers'), 'launch', 'rviz_display.launch.py'])
        ),
        condition=IfCondition(rviz),
        launch_arguments={
            'robot_type': robot_type,
            'use_sim_time': 'false',
            'standalone': 'false',
        }.items(),
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('legged_controllers'), 'launch', 'joy_teleop.launch.py'])
        ),
    )

    rl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('legged_deploy'), 'launch', 'rl_controller.launch.py'])
        ),
        launch_arguments={
            'robot_type': robot_type,
            'subpolicy': subpolicy,
        }.items(),
    )

    dynamic_imitation_actions = []
    for skipped in skipped_imitation_policies:
        dynamic_imitation_actions.append(LogInfo(msg=f'[one_start] Skipping unavailable imitation policy: {skipped}'))

    if imitation_policies:
        dynamic_imitation_actions.append(
            LogInfo(
                msg=(
                    '[one_start] Starting dynamic imitation worker '
                    f'({len(imitation_policies)} policies, {len(preload_imitation_policies)} preloaded)'
                )
            )
        )
        dynamic_imitation_actions.append(
            Node(
                package='legged_deploy',
                executable='rl_controller',
                name='rl_controller_imitation',
                output='screen',
                parameters=[
                    {'robot_type': robot_type},
                    {
                        'robot_cfg_path': PathJoinSubstitution(
                            [FindPackageShare('legged_deploy'), 'policy', robot_type, subpolicy, 'rl_config.yaml']
                        )
                    },
                    {'subpolicy': ''},
                    {'policy_type': 'rl_imitation'},
                    {'dynamic_policy_switch_enabled': True},
                    {'dynamic_imitation_idle_frequency': 100},
                    {'policy_root_path': policy_root_path},
                    {
                        'rl_controller_config_path': PathJoinSubstitution(
                            [FindPackageShare('legged_deploy'), 'config', 'rl_controller_config.yaml']
                        )
                    },
                    {'available_imitation_policies': imitation_policies},
                    *(
                        [{'preload_imitation_policies': preload_imitation_policies}]
                        if preload_imitation_policies else []
                    ),
                    {'text_motion_enabled': text_motion_enabled},
                    {'text_motion_policy': text_motion_policy},
                    {'text_motion_external_packager': not dynamic_text_motion},
                    {'text_motion_json_topic': cfg.get('text_motion_json_topic', '/text_motion/json')},
                    {'text_motion_ready_topic': cfg.get('text_motion_ready_topic', '/text_motion/ready')},
                    {'text_motion_amp_stand_profile': text_motion_amp_stand_profile},
                    {'text_motion_hold_frames': int(cfg.get('text_motion_hold_frames', '10'))},
                    {'text_motion_transition_frames': int(cfg.get('text_motion_transition_frames', '40'))},
                    {'text_motion_return_frames': int(cfg.get('text_motion_return_frames', '12'))},
                    {'text_motion_return_settle_frames': int(cfg.get('text_motion_return_settle_frames', '2'))},
                    {'text_motion_lowpass_cutoff_hz': float(cfg.get('text_motion_lowpass_cutoff_hz', '4.0'))},
                ],
            )
        )
        if text_motion_enabled and not dynamic_text_motion:
            dynamic_imitation_actions.append(
                Node(
                    package='legged_deploy',
                    executable='text_motion_wbc_packager.py',
                    name='text_motion_wbc_packager',
                    output='screen',
                    parameters=[{
                        'json_topic': cfg.get('text_motion_json_topic', '/text_motion/json'),
                        'ready_topic': cfg.get('text_motion_ready_topic', '/text_motion/ready'),
                        'status_topic': '/text_motion/status',
                        'policy_root_path': policy_root_path,
                        'robot_type': cfg.get('robot_type', 'l0_v3'),
                        'policy_slot_prefix': text_motion_policy,
                        'python_executable': packaging_inputs['python'],
                        'wbc_deploy_root': cfg.get('text_motion_wbc_deploy_root', '').strip() or bundled_wbc_root,
                        'wbc_policy_onnx': packaging_inputs['policy'],
                        'wbc_policy_config': packaging_inputs['policy config'],
                        'amp_stand_profile': text_motion_amp_stand_profile,
                        'mujoco_model': packaging_inputs['MuJoCo model'],
                        'hold_frames': int(cfg.get('text_motion_hold_frames', '10')),
                        'transition_frames': int(cfg.get('text_motion_transition_frames', '40')),
                        'return_frames': int(cfg.get('text_motion_return_frames', '12')),
                        'return_settle_frames': int(cfg.get('text_motion_return_settle_frames', '2')),
                        'trtexec': cfg.get('text_motion_trtexec', _TEXT_MOTION_TRTEXEC),
                    }],
                )
            )
    else:
        dynamic_imitation_actions.append(
            LogInfo(
                msg=(
                    '[one_start] No enabled joystick/keyboard/emotion rl_imitation action '
                    f'({len(action_manifest.sequence_paths)} action sequence file(s) validated).'
                )
            )
        )

    bag_record = GroupAction(
        condition=IfCondition(bag_enabled),
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    PythonExpression(
                        [
                            "'mkdir -p ",
                            bag_dir,
                            " && cd ",
                            bag_dir,
                            " && ros2 bag record -o ",
                            bag_prefix,
                            " ",
                            bag_topics,
                            "'",
                        ]
                    ),
                ],
                output='screen',
            )
        ],
    )

    manifest_log = LogInfo(
        msg=(
            '[one_start] Validated '
            f'{len(action_manifest.sequence_paths)} action sequence file(s); '
            f'{len(action_manifest.rl_imitation_sequences)} use rl_imitation.'
        )
    )

    return [manifest_log, rviz_launch, joy_launch, rl_launch, *dynamic_imitation_actions, bag_record]


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    bag_name = LaunchConfiguration('bag_name')

    # Required args come from sequential_launch_*.sh (or sim2*.launch.py).
    declare = [
        DeclareLaunchArgument('robot_type'),
        DeclareLaunchArgument('subpolicy'),
        DeclareLaunchArgument('rviz'),
        DeclareLaunchArgument('bag_enabled'),
        DeclareLaunchArgument('bag_name'),
        DeclareLaunchArgument(
            'bag_dir',
            default_value=PathJoinSubstitution([FindPackageShare('legged_controllers'), '..', '..', '..', 'bag']),
        ),
        DeclareLaunchArgument('bag_prefix', default_value=PythonExpression(["'", robot_type, "_", bag_name, "'"])),
        DeclareLaunchArgument(
            'bag_topics',
            default_value=(
                '/tf /tf_static /clock /joint_states /imu/data /imu/data_raw /rlState /rlCmd_rl '
                '/rlCmd_imitation /rlImitationActive /eye_animation_cmd /keyboard_action_trigger '
                '/audio_play_cmd /battery/status /imitationref_joint_states'
            ),
        ),
        DeclareLaunchArgument('head_script_enabled', default_value='false'),
        DeclareLaunchArgument('head_script_config', default_value=''),
        DeclareLaunchArgument('head_topic_config', default_value=''),
        DeclareLaunchArgument('emotion_sequence_enabled', default_value='false'),
        DeclareLaunchArgument('emotion_sequence_map_path', default_value=''),
        DeclareLaunchArgument('keyboard_action_enabled', default_value='false'),
        DeclareLaunchArgument('keyboard_action_config_path', default_value=''),
        DeclareLaunchArgument('joystick_imitation_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_imitation_config_path', default_value=''),
        DeclareLaunchArgument(
            'policy_root_path',
            default_value=PathJoinSubstitution([FindPackageShare('legged_deploy'), 'policy']),
        ),
        DeclareLaunchArgument('joystick_sequence_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_sequence_config_path', default_value=''),
        DeclareLaunchArgument('text_motion_enabled', default_value='true'),
        DeclareLaunchArgument(
            'text_motion_runtime',
            default_value='dynamic_trt',
            description='dynamic_trt uses fixed policy + C++ JSON reference; legacy_packager restores per-motion packaging',
        ),
        DeclareLaunchArgument('text_motion_policy', default_value='text_motion_wbc_stg25'),
        DeclareLaunchArgument('text_motion_json_topic', default_value='/text_motion/json'),
        DeclareLaunchArgument('text_motion_ready_topic', default_value='/text_motion/ready'),
        DeclareLaunchArgument(
            'text_motion_amp_stand_profile',
            default_value=PathJoinSubstitution(
                [FindPackageShare('legged_deploy'), 'config', 'amp_stand_profiles',
                 'l0_v3_amp_stand_inner_arms_profile.json']
            ),
        ),
        DeclareLaunchArgument('text_motion_hold_frames', default_value='10'),
        DeclareLaunchArgument('text_motion_transition_frames', default_value='40'),
        DeclareLaunchArgument('text_motion_return_frames', default_value='12'),
        DeclareLaunchArgument('text_motion_return_settle_frames', default_value='2'),
        DeclareLaunchArgument('text_motion_lowpass_cutoff_hz', default_value='4.0'),
        DeclareLaunchArgument(
            'text_motion_python_executable',
            default_value=_TEXT_MOTION_PYTHON,
        ),
        DeclareLaunchArgument(
            'text_motion_wbc_deploy_root',
            default_value=(
                _TEXT_MOTION_WBC_ROOT_OVERRIDE
                or PathJoinSubstitution([FindPackageShare('legged_deploy'), 'third_party', 'wbc_deploy'])
            ),
        ),
        DeclareLaunchArgument(
            'text_motion_wbc_policy_onnx',
            default_value=(
                _TEXT_MOTION_WBC_POLICY_OVERRIDE
                or PathJoinSubstitution(
                    [FindPackageShare('legged_deploy'), 'resources', 'text_motion',
                     'stg25_1000_onnx', 'policy.onnx']
                )
            ),
        ),
        DeclareLaunchArgument(
            'text_motion_wbc_policy_config',
            default_value=(
                _TEXT_MOTION_WBC_CONFIG_OVERRIDE
                or PathJoinSubstitution(
                    [FindPackageShare('legged_deploy'), 'resources', 'text_motion',
                     'stg25_1000_onnx', 'config.yaml']
                )
            ),
        ),
        DeclareLaunchArgument(
            'text_motion_mujoco_model',
            default_value=PathJoinSubstitution(
                [FindPackageShare('l0_v3_description'), 'mjcf', 'l0_v3_description.xml']
            ),
        ),
        DeclareLaunchArgument('text_motion_trtexec', default_value=_TEXT_MOTION_TRTEXEC),
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch_setup)])
