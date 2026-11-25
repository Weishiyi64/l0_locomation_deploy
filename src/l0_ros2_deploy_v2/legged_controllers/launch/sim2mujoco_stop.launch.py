"""MuJoCo stop tests using the single Sonic800 73983 WBC actor."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bundle = 'wbc_base_sonic800_73983_20261124_sim'
    policies = Path(get_package_share_directory('legged_deploy')) / 'policy' / 'l0_v5'
    graph = json.loads((policies / bundle / 'base_manifest.json').read_text())
    launch = Path(get_package_share_directory('legged_controllers')) / 'launch' / 'sim2mujoco.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments={
            'wbc_mode': 'base',
            'wbc_lora_bundle': bundle,
            'wbc_policies': json.dumps([action['action_id'] for action in graph['actions']]),
            'joystick_wbc_enabled': 'false',
            'wbc_emergency_after_actions': '[]',
            'stop_trial_profile': str(policies / bundle / 'stand_reference.json'),
        }.items()),
    ])
