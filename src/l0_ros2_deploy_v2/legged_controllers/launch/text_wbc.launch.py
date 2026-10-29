from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hpm_mode', default_value='real'),
        DeclareLaunchArgument('isolated_ros_domain', default_value='173'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v5_amp_10281'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='text_wbc_bundle'),
        DeclareLaunchArgument('wbc_json_base_action', default_value='A-C04_mod'),
        DeclareLaunchArgument('wbc_policies', default_value='[A-C04_mod, L0_Hugme, L0_act_cute_1]'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('legged_controllers'), 'launch', 'sim2real.launch.py'])), launch_arguments={
                'robot_type': 'l0_v5', 'subpolicy': LaunchConfiguration('subpolicy'),
                'wbc_enabled': 'true', 'wbc_mode': 'lora',
                'wbc_lora_bundle': LaunchConfiguration('wbc_lora_bundle'),
                'wbc_json_base_action': LaunchConfiguration('wbc_json_base_action'),
                'wbc_policies': LaunchConfiguration('wbc_policies'),
                'hpm_mode': LaunchConfiguration('hpm_mode'),
                'isolated_ros_domain': LaunchConfiguration('isolated_ros_domain'),
                'rviz': 'false', 'radar_control_enabled': 'false', 'battery_enabled': 'false',
            }.items()),
    ])
