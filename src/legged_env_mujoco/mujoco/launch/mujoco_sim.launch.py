#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    description_pkg = LaunchConfiguration('description_pkg')
    body_name = LaunchConfiguration('body_name')

    declare = [
        DeclareLaunchArgument('robot_type', default_value='l0_v3'),
        DeclareLaunchArgument('description_pkg', default_value=''),
        DeclareLaunchArgument('body_name', default_value='base_link'),
    ]

    def _launch(context, *args, **kwargs):
        rt = robot_type.perform(context)
        dp = description_pkg.perform(context).strip()
        if not dp:
            dp = f"{rt}_description"

        desc_share = get_package_share_directory(dp)
        scene = os.path.join(desc_share, 'mjcf', f'{rt}.xml')

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
                arguments=[scene, body_name.perform(context)],
                parameters=[{'mujoco_scene': scene}],
                prefix=cmd_prefix,
            )
        ]

    return LaunchDescription(declare + [OpaqueFunction(function=_launch)])
