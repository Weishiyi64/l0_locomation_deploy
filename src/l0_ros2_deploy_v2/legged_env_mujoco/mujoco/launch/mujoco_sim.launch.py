#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    description_pkg = LaunchConfiguration('description_pkg')
    body_name = LaunchConfiguration('body_name')
    mapping_file = LaunchConfiguration('mapping_file')
    command_watchdog_ms = LaunchConfiguration('command_watchdog_ms')

    declare = [
        DeclareLaunchArgument('robot_type', default_value='l0_v3'),
        DeclareLaunchArgument('description_pkg', default_value=''),
        DeclareLaunchArgument('body_name', default_value='base_link'),
        DeclareLaunchArgument('mapping_file', default_value=''),
        DeclareLaunchArgument('command_watchdog_ms', default_value='100.0'),
    ]

    def _launch(context, *args, **kwargs):
        rt = robot_type.perform(context)
        dp = description_pkg.perform(context).strip()
        if not dp:
            dp = f"{rt}_description"

        desc_share = get_package_share_directory(dp)
        scene_dir = os.path.join(desc_share, 'mjcf')
        scene_candidates = [
            os.path.join(scene_dir, f'{rt}.xml'),
            os.path.join(scene_dir, f'{dp}.xml'),
        ]
        scene = next((path for path in scene_candidates if os.path.isfile(path)), '')
        if not scene:
            raise RuntimeError(f"No MuJoCo scene found for robot_type={rt!r}; tried: " + ', '.join(scene_candidates))
        mapping = mapping_file.perform(context).strip()
        if not mapping:
            mapping = os.path.join(get_package_share_directory('system_command_arbiter'), 'config', f'{rt}.yaml')
        if not os.path.isfile(mapping):
            raise RuntimeError(f'Motor mapping file does not exist: {mapping}')

        mujoco_share = get_package_share_directory('mujoco')
        setup_script = os.path.join(mujoco_share, 'scripts', 'setup_mujoco_software_rendering.sh')

        # 等价于 ROS1 的 launch-prefix source setup_script
        # Node(prefix=...) 在 Foxy 上也能用，但用 bash -c 更稳定
        # cmd_prefix = f"bash -c 'source {setup_script}; exec $0 $@' -- gdbserver localhost:3000 "
        cmd_prefix = f"bash -c 'source {setup_script}; exec $0 $@' --"

        return [
            Node(
                package='mujoco',
                executable='mujoco_simulate',
                name='mujoco',
                output='screen',
                # prefix=['gdbserver',' localhost:3000'],
                arguments=[scene, body_name.perform(context), mapping],
                parameters=[{'command_watchdog_ms': ParameterValue(command_watchdog_ms, value_type=float)}],
                prefix=cmd_prefix,
            )
        ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch)])
