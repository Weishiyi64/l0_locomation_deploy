import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    cfg = context.launch_configurations
    robot = cfg['robot_type']
    if robot not in ('l0_v3', 'l0_v4', 'l0_v5'):
        raise RuntimeError('Unsupported isolated HPM robot_type')
    return [Node(
        package='legged_real', executable='isolated_hpm.py', name='isolated_hpm', output='screen',
        parameters=[{
            'mapping_file': os.path.join(get_package_share_directory('system_command_arbiter'), 'config', robot + '.yaml'),
            'robot_cfg_path': os.path.join(get_package_share_directory('legged_deploy'), 'policy', robot,
                                          cfg['subpolicy'], 'rl_config.yaml'),
        }])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_type'), DeclareLaunchArgument('subpolicy'),
                              OpaqueFunction(function=setup)])
