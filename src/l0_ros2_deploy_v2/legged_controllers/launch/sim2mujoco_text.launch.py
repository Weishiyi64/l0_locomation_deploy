from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('legged_controllers'), 'launch', 'sim2mujoco.launch.py'])),
            launch_arguments={
                'joystick_wbc_enabled': 'false',
                'wbc_lora_bundle': 'text_wbc_sim',
                'wbc_policies': '[UpperBody_Enhanced]',
                'wbc_json_base_action': 'UpperBody_Enhanced',
            }.items()),
    ])
