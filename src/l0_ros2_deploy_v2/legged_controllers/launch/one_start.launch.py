#!/usr/bin/env python3
import os
import hashlib
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
from launch_ros.substitutions import FindPackageShare

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)

from action_sequence_policy import collect_action_sequence_manifest  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    cfg = context.launch_configurations
    share_leg = get_package_share_directory('legged_controllers')
    legged_deploy_share = get_package_share_directory('legged_deploy')
    policy_root_path = os.path.join(legged_deploy_share, 'policy')
    wbc_enabled = cfg['wbc_enabled'].lower() == 'true'
    wbc_policies = []
    wbc_mode = cfg.get('wbc_mode', 'static')
    wbc_manifest = ''
    wbc_cpu = cfg.get('wbc_inference', 'rknn') == 'openvino'
    base_parameters = {}
    if wbc_mode not in ('static', 'lora') and not (wbc_cpu and wbc_mode == 'base'):
        raise RuntimeError('Unsupported WBC mode for this inference backend')
    if wbc_enabled:
        try:
            wbc_policies = yaml.safe_load(cfg['wbc_policies'])
        except yaml.YAMLError as error:
            raise RuntimeError(f'Invalid wbc_policies YAML list: {error}') from error
        if not isinstance(wbc_policies, list) or not wbc_policies:
            raise RuntimeError('WBC requires a non-empty wbc_policies YAML list from sim2real/sim2mujoco.')
        if any(not isinstance(policy, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', policy)
               for policy in wbc_policies):
            raise RuntimeError('wbc_policies must contain policy directory names, without path components.')
        if len(set(wbc_policies)) != len(wbc_policies):
            raise RuntimeError('wbc_policies contains duplicate policy directories.')
        if wbc_mode == 'static':
            for policy in wbc_policies:
                for filename in ('policy.rknn', 'reference_motion.rknn', 'rl_config.yaml'):
                    path = os.path.join(policy_root_path, cfg['robot_type'], policy, filename)
                    if not os.path.isfile(path):
                        raise RuntimeError(f'WBC deployment asset missing: {path}')
            wbc_robot_cfg = os.path.join(policy_root_path, cfg['robot_type'], wbc_policies[0], 'rl_config.yaml')
        else:
            bundle_name = cfg['wbc_lora_bundle']
            if not re.fullmatch(r'[A-Za-z0-9_-]+', bundle_name):
                raise RuntimeError('wbc_lora_bundle must be a directory name under the robot policy root')
            root = Path(policy_root_path) / cfg['robot_type'] / bundle_name
            root = root.resolve(strict=True)

            def asset(relative):
                if not isinstance(relative, str) or Path(relative).is_absolute():
                    raise RuntimeError('WBC manifest paths must be relative')
                path = (root / relative).resolve(strict=True)
                if not path.is_relative_to(root) or not path.is_file():
                    raise RuntimeError(f'Invalid WBC bundle asset: {relative}')
                return str(path)

            wbc_manifest = asset('base_manifest.json')
            with open(wbc_manifest, encoding='utf-8') as stream:
                graph = json.load(stream)
            expected_format = 'wbc_base_graph_v1' if wbc_mode == 'base' else 'wbc_lora_graph_v1'
            if graph.get('format') != expected_format or graph.get('kind') != 'policy':
                raise RuntimeError(f'Expected a {expected_format} policy manifest')
            def verified(relative, digest):
                path = asset(relative)
                if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                    raise RuntimeError(f'WBC asset hash mismatch: {relative}')
                return path

            if wbc_mode == 'base':
                base_parameters['wbc_base_model'] = verified(graph['model']['onnx'], graph['model']['sha256'])
                base_parameters['wbc_base_configs'] = []
                base_parameters['wbc_base_references'] = []
            else:
                asset(graph['model']['onnx' if wbc_cpu else 'rknn'])
            if not wbc_cpu:
                asset(graph['model']['rknn'] + '.json')
            catalog = {item['action_id']: item for item in graph['actions']}
            if len(catalog) != len(graph['actions']):
                raise RuntimeError('Duplicate WBC action IDs')
            for action in wbc_policies:
                if action not in catalog:
                    raise RuntimeError(f'WBC action is not in manifest: {action}')
                item = catalog[action]
                if wbc_mode == 'base':
                    base_parameters['wbc_base_configs'].append(verified(item['config'], item['config_sha256']))
                    base_parameters['wbc_base_references'].append(verified(item['reference_onnx'], item['reference_onnx_sha256']))
                    continue
                for key in (('config', 'adapter_manifest') if wbc_cpu and item.get('simulation_json_only') else
                            ('config', 'reference_onnx' if wbc_cpu else 'reference_rknn', 'adapter_manifest')):
                    asset(item[key])
                if not wbc_cpu:
                    asset(item['reference_rknn'] + '.json')
            wbc_robot_cfg = asset(catalog[wbc_policies[0]]['config'])
    try:
        action_manifest = collect_action_sequence_manifest(
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
    imitation_policies = action_manifest.imitation_policies
    preload_imitation_policies = action_manifest.preload_imitation_policies
    skipped_imitation_policies = action_manifest.skipped_imitation_policies

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
    bag_name = LaunchConfiguration('bag_name')
    bag_prefix = LaunchConfiguration('bag_prefix')
    bag_topics = LaunchConfiguration('bag_topics')
    radar_control_enabled = LaunchConfiguration('radar_control_enabled')
    nav_planner_model_path = LaunchConfiguration('nav_planner_model_path')
    planner_depth_input_enabled = LaunchConfiguration('planner_depth_input_enabled')
    raw_depth_preview_enabled = LaunchConfiguration('raw_depth_preview_enabled')
    depth_pipeline_enabled = (
        cfg.get('planner_depth_input_enabled', 'false').lower() == 'true'
        or cfg.get('raw_depth_preview_enabled', 'false').lower() == 'true'
    )
    odin_control_config = PathJoinSubstitution(
        [
            FindPackageShare('odin_ros_driver_rev1'),
            'config',
            'control_command.yaml' if depth_pipeline_enabled else 'control_command_odom_only.yaml',
        ]
    )

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
    joystick_wbc = cfg.get('joystick_wbc_enabled', 'false').lower() == 'true'
    if joystick_wbc:
        if not wbc_enabled:
            raise RuntimeError('joystick_wbc_enabled requires wbc_enabled')
        if any(cfg.get(key, 'false').lower() == 'true' for key in
               ('joystick_imitation_enabled', 'joystick_sequence_enabled')):
            raise RuntimeError('Disable legacy joystick imitation/sequence bindings before enabling WBC bindings')
        wbc_actions.append(Node(
            package='legged_controllers', executable='joystick_wbc_request',
            name='joystick_wbc_request', output='screen',
            parameters=[{'config_path': cfg['joystick_wbc_config_path']},
                        {'allowed_actions': wbc_policies}],
        ))
    if wbc_enabled:
        wbc_actions.extend(
            [
                LogInfo(msg=f"[one_start] WBC mode={wbc_mode}, actions={wbc_policies}, manifest={wbc_manifest}"),
                Node(
                    package='legged_deploy',
                    executable='rl_controller',
                    name='rl_controller_wbc',
                    output='screen',
                    parameters=[
                        {'robot_type': cfg['robot_type']},
                        {'robot_cfg_path': wbc_robot_cfg},
                        {'wbc_mode': wbc_mode},
                        {'wbc_lora_manifest': wbc_manifest},
                        base_parameters,
                        {'wbc_json_base_action': cfg.get('wbc_json_base_action', '')},
                        {'mapping_file': os.path.join(get_package_share_directory('system_command_arbiter'),
                                                     'config', cfg['robot_type'] + '.yaml')},
                        {'subpolicy': ''},
                        {'policy_root_path': policy_root_path},
                        {'available_wbc_policies': wbc_policies},
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

    # radar_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([FindPackageShare('odin_ros_driver_rev1'), 'launch', 'driver.launch.py'])
    #     ),
    #     condition=IfCondition(radar_control_enabled),
    #     launch_arguments={
    #         'start_rviz': 'false',
    #         'start_auxiliary_nodes': 'true' if depth_pipeline_enabled else 'false',
    #         'control_config_path': odin_control_config,
    #         'publish_tf_map': 'false',
    #         'raw_depth_preview_enabled': raw_depth_preview_enabled,
    #     }.items(),
    # )

    radar_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('legged_controllers'), 'launch', 'local_planner.launch.py'])
        ),
        condition=IfCondition(radar_control_enabled),
        launch_arguments={
            'planner_model_path': nav_planner_model_path,
            'planner_depth_input_enabled': planner_depth_input_enabled,
            'raw_depth_preview_enabled': raw_depth_preview_enabled,
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
        dynamic_imitation_actions.append(
            LogInfo(
                msg=(
                    '[one_start] No enabled joystick rl_imitation action '
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

    return [
        manifest_log,
        rviz_launch,
        joy_launch,
    #    radar_driver_launch,
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
                '/tf /tf_static /clock /joint_states /imu/data /actuator/motor/state '
                '/system/motion_controller/imu_rpy control/policy/observation '
                'control/policy/command/rl control/policy/command/imitation '
                'control/policy/imitation/active /control/policy/command/wbc /control/policy/wbc/active /control/policy/wbc/done /algorithm/brain/wbc_action_cmd /system/state_machine/wbc_action_fdb '
                '/audio_play_cmd /battery/status /imitationref_joint_states '
                '/navigation_goal /goal_pose /planner_input control/cmd_vel'
            ),
        ),
        DeclareLaunchArgument('head_script_enabled', default_value='false'),
        DeclareLaunchArgument('head_script_config', default_value=''),
        DeclareLaunchArgument('joystick_imitation_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_imitation_config_path', default_value=''),
        DeclareLaunchArgument('joystick_sequence_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_sequence_config_path', default_value=''),
        DeclareLaunchArgument('wbc_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_wbc_enabled', default_value='false'),
        DeclareLaunchArgument('joystick_wbc_config_path', default_value=PathJoinSubstitution(
            [FindPackageShare('legged_controllers'), 'config', 'joystick_wbc_actions.yaml'])),
        DeclareLaunchArgument('wbc_policies', default_value='[]'),
        DeclareLaunchArgument('wbc_inference', default_value='rknn', choices=['rknn', 'openvino']),
        DeclareLaunchArgument('wbc_mode', default_value='static'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora'),
        DeclareLaunchArgument('wbc_json_base_action', default_value=''),
        DeclareLaunchArgument('radar_control_enabled', default_value='false'),
        DeclareLaunchArgument('planner_depth_input_enabled', default_value='false'),
        DeclareLaunchArgument('raw_depth_preview_enabled', default_value='false'),
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
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch_setup)])
