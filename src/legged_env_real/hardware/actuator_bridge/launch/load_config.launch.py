"""
ROS2 version of load_config.launch.

ROS1 equivalent:
  <arg name="robot_type" default="l0_v3"/>
  <rosparam file="$(find actuator_bridge)/config/$(arg robot_type).yaml" command="load"/>

In ROS2 there is no global parameter server. This launch:
  1. Declares robot_type argument (same as ROS1)
  2. Sets LaunchConfiguration 'actuator_config_file' with the resolved config path
  3. When included, parent launches can use LaunchConfiguration('actuator_config_file')
     to pass the config to nodes (e.g. legged_real's config_file parameter)

Note: legged_real.launch.py computes config_file directly and does not include this.
This launch is for compatibility when other launch files include it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import TextSubstitution


def launch_setup(context, *args, **kwargs):
    robot_type = context.launch_configurations.get('robot_type', 'l0_v3')
    pkg_share = get_package_share_directory('actuator_bridge')
    config_path = os.path.abspath(os.path.join(pkg_share, 'config', f'{robot_type}.yaml'))
    return [SetLaunchConfiguration('actuator_config_file', TextSubstitution(text=config_path))]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_type',
                default_value='l0_v3',
                description='Robot type for actuator_bridge config (l0_v3)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
