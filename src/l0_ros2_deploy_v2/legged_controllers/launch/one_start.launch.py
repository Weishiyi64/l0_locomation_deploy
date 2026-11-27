#!/usr/bin/env python3
import os
import json
from pathlib import Path
import re
import sys

import yaml

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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)

from imitation_assets import load_imitation_assets  # noqa: E402
from motion_assets import load_motion_assets  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    cfg = context.launch_configurations
    legged_deploy_share = get_package_share_directory('legged_deploy')
    policy_root_path = os.path.join(legged_deploy_share, 'policy')
    wbc_enabled = cfg['wbc_enabled'].lower() == 'true'
    fixed_frontend_enabled = cfg['wbc_frontend_enabled'].lower() == 'true'
    text_frontend_enabled = cfg['txt2motion_enabled'].lower() == 'true'
    if (fixed_frontend_enabled or text_frontend_enabled) and not wbc_enabled:
        raise RuntimeError('Motion frontends require the shared backend')
    wbc_policies = []
    wbc_manifest = ''
    motion_assets = None
    if wbc_enabled:
        bundle_name = cfg['wbc_lora_bundle'] or 'wbc_lora_fp16'
        if not re.fullmatch(r'[A-Za-z0-9_-]+', bundle_name):
            raise RuntimeError('motion bundle must be a directory under the robot policy root')
        bundle = Path(policy_root_path) / cfg['robot_type'] / bundle_name
        wbc_manifest = str(bundle / ('deployment.json' if text_frontend_enabled else 'base_manifest.json'))
        motion_assets = load_motion_assets(wbc_manifest)
        if text_frontend_enabled and motion_assets['tracker'] is None:
            raise RuntimeError('txt2motion_enabled requires a deployment.json with text tracking parameters')
        wbc_robot_cfg = str((motion_assets['tracker'] or next(iter(motion_assets['actions'].values())))['config_path'])
        wbc_policies = yaml.safe_load(cfg['wbc_policies'])
        if not isinstance(wbc_policies, list) or any(not isinstance(p, str) or p not in motion_assets['actions'] for p in wbc_policies):
            raise RuntimeError('wbc_policies must be a list of IDs from the motion catalog (empty means all)')
        if len(wbc_policies) != len(set(wbc_policies)):
            raise RuntimeError('Duplicate WBC policy selection')
    try:
        imitation_assets = load_imitation_assets(
            imitation_policies=yaml.safe_load(cfg['imitation_policies']),
            robot_type=cfg.get('robot_type', ''),
            policy_root_path=policy_root_path,
        )
    except (RuntimeError, ValueError, yaml.YAMLError) as e:
        raise RuntimeError(f'[one_start.launch.py] {e}') from e
    imitation_policies = imitation_assets.imitation_policies
    preload_imitation_policies = imitation_assets.preload_imitation_policies
    skipped_imitation_policies = imitation_assets.skipped_imitation_policies

    robot_type_value = cfg.get('robot_type', '')
    radar_enabled = cfg.get('radar_control_enabled', 'false').lower() == 'true'
    planner_model_path = cfg.get('nav_planner_model_path', '')
    if radar_enabled and not os.path.isfile(planner_model_path):
        raise RuntimeError(
            '[one_start.launch.py] radar/planner is enabled, but no planner policy '
            f'was found for robot_type={robot_type_value}: {planner_model_path}'
        )

    robot_type = LaunchConfiguration('robot_type')
    subpolicy = LaunchConfiguration('subpolicy')
    rviz = LaunchConfiguration('rviz')
    bag_enabled = LaunchConfiguration('bag_enabled')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_prefix = LaunchConfiguration('bag_prefix')
    bag_topics = LaunchConfiguration('bag_topics')
    radar_control_enabled = LaunchConfiguration('radar_control_enabled')
    nav_planner_model_path = LaunchConfiguration('nav_planner_model_path')

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
        launch_arguments={'robot_type': robot_type}.items(),
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

    wbc_actions = []
    if wbc_enabled:
        wbc_actions.extend(
            [
                LogInfo(msg=f"[one_start] Starting one shared motion Base; fixed selection: {wbc_policies or 'all'}"),
                Node(
                    package='legged_deploy',
                    executable='rl_controller',
                    name='rl_controller_wbc',
                    output='screen',
                    parameters=[
                        {'robot_type': cfg['robot_type']},
                        {'robot_cfg_path': wbc_robot_cfg},
                        {'wbc_lora_manifest': wbc_manifest},
                        {'wbc_json_base_action': motion_assets['tracker']['action_id'] if motion_assets['tracker'] else ''},
                        {'standing_config': str(motion_assets['standing_path']) if motion_assets['standing_path'] else ''},
                        {'mapping_file': os.path.join(get_package_share_directory('system_command_arbiter'),
                                                     'config', cfg['robot_type'] + '.yaml')},
                        {'subpolicy': ''},
                        {'policy_root_path': policy_root_path},
                        {'available_wbc_policies': wbc_policies or list(motion_assets['actions'])},
                        {'policy_type': 'rl_wbc'},
                        {
                            'rl_controller_config_path': os.path.join(
                                legged_deploy_share, 'config', 'rl_controller_config.yaml'
                            )
                        },
                    ],
                ),
            ]
        )

    if wbc_enabled:
        wbc_actions.append(Node(
            package='legged_controllers', executable='motion_runtime', name='motion_backend',
            arguments=['--role', 'backend'], output='screen',
            parameters=[{'deployment': wbc_manifest},
                        {'fixed_actions_json': ParameterValue(json.dumps(wbc_policies), value_type=str)},
                        {'mapping': os.path.join(get_package_share_directory('system_command_arbiter'), 'config', cfg['robot_type'] + '.yaml')},
                        {'generation_timeout_sec': 3.0}],
        ))

    for enabled, role in ((fixed_frontend_enabled, 'wbc'), (text_frontend_enabled, 'txt2motion')):
        if enabled:
            frontend_parameters = [{'deployment': wbc_manifest}]
            if role == 'txt2motion':
                frontend_parameters.extend([
                    {'mapping': os.path.join(get_package_share_directory('system_command_arbiter'),
                                             'config', cfg['robot_type'] + '.yaml')},
                    {'model_root': os.path.join(policy_root_path, cfg['robot_type'], 'txt2motion')},
                ])
            wbc_actions.append(Node(
                package='legged_controllers', executable='motion_runtime', name=role + '_frontend',
                arguments=['--role', role], output='screen', parameters=frontend_parameters,
            ))

    radar_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('legged_controllers'), 'launch', 'local_planner.launch.py'])
        ),
        condition=IfCondition(radar_control_enabled),
        launch_arguments={
            'robot_type': robot_type,
            'planner_model_path': nav_planner_model_path,
            'nav_check_sensor_timestamps': LaunchConfiguration('nav_check_sensor_timestamps'),
            'nav_reference_receive_timeout': LaunchConfiguration('nav_reference_receive_timeout'),
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
                    {'preload_imitation_policies': preload_imitation_policies},
                ],
            )
        )
    else:
        dynamic_imitation_actions.append(LogInfo(msg='[one_start] No imitation policies selected.'))

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
                            " && ros2 bag record --include-hidden-topics -o ",
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

    return [
        rviz_launch,
        joy_launch,
        radar_planner_launch,
        rl_launch,
        *wbc_actions,
        *dynamic_imitation_actions,
        bag_record,
    ]


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
                '/tf /tf_static /clock /system/joint/jointstates /imu/data /actuator/motor/state '
                '/system/motion_controller/imu_rpy control/policy/observation '
                'control/policy/command/rl control/policy/command/imitation '
                'control/policy/imitation/active /control/policy/command/wbc /control/policy/wbc/active /control/policy/wbc/execution_active /control/policy/wbc/done /control/motion/status /algorithm/brain/wbc_action_cmd /algorithm/brain/txt2motion_cmd /system/state_machine/wbc_action_fdb '
                '/audio_play_cmd /battery/status /imitationref_joint_states '
                '/control/navigation/follow_path/_action/feedback /control/navigation/follow_path/_action/status '
                '/planner_input /local_planner/stop_reason control/cmd_vel'
            ),
        ),
        DeclareLaunchArgument('imitation_policies', default_value='[]',
                              description='Policies to preload; no joystick action trigger'),
        DeclareLaunchArgument('wbc_enabled', default_value='false'),
        DeclareLaunchArgument('wbc_frontend_enabled', default_value='false'),
        DeclareLaunchArgument('txt2motion_enabled', default_value='false'),
        DeclareLaunchArgument('wbc_policies', default_value='[]'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora_fp16'),
        DeclareLaunchArgument('radar_control_enabled', default_value='false'),
        DeclareLaunchArgument('nav_check_sensor_timestamps', default_value='true'),
        DeclareLaunchArgument('nav_reference_receive_timeout', default_value='false'),
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
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch_setup)])
