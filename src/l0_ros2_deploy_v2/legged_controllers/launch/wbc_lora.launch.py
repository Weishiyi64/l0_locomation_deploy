from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joystick_wbc_enabled', default_value='true'),
        DeclareLaunchArgument('joystick_wbc_config_path', default_value=PathJoinSubstitution(
            [FindPackageShare('legged_controllers'), 'config', 'joystick_wbc_actions.yaml'])),
        DeclareLaunchArgument('amp_to_wbc_transition_sec', default_value='1.0'),
        DeclareLaunchArgument('wbc_to_amp_transition_sec', default_value='0.35'),
        DeclareLaunchArgument('hpm_mode', default_value='real'),
        DeclareLaunchArgument('isolated_ros_domain', default_value='97'),
        DeclareLaunchArgument('subpolicy', default_value='l0_v5_amp_10281'),
        DeclareLaunchArgument('wbc_lora_bundle', default_value='wbc_lora_fp16'),
        DeclareLaunchArgument('wbc_policies', default_value='["L0_Hugme", "L0_act_cute_1", "L0_act_cute_2", "L0_act_cute_3", "L0_act_cute_7", "L0_act_cute_new_1", "L0_act_cute_new_2", "L0_act_cute_new_3", "L0_act_cute_new_4", "L0_tired_hugme", "L0_NotLeave", "L0_SayNo_17", "L0_angry_10", "L0_angry_7", "L0_angry_8", "L0_angry_9", "L0_angry_baoshouqiaojiao", "L0_angry_chayaotaitou", "L0_angry_new_1", "L0_angry_new_2", "L0_peace_1", "L0_peace_2", "L0_peace_new_01_A", "L0_peace_new_02_A", "L0_peace_new_03_A", "L0_peace_new_04_A", "L0_peace_new_05_A", "L0_peace_new_06_A", "L0_peace_new_07_A", "L0_peace_new_08_A", "L0_16_strange", "L0_strange_1", "L0_strange_new_1_A", "L0_strange_new_2_A", "L0_strange_new_3_A", "L0_strange_new_4", "L0_strange_new_5", "L0_strange_new_5_1", "L0_strange_new_6_A", "L0_strange_new_8", "L0_Approve_13", "L0_excited_1", "L0_excited_7", "L0_excited_8", "L0_excited_new_1", "L0_excited_new_2", "L0_excited_new_3", "L0_excited_new_4", "L0_excited_new_5", "L0_excited_new_6", "L0_AfraidToLeave", "L0_SayNo_2", "L0_fear_10", "L0_fear_7_A", "L0_fear_8", "L0_fear_8_new_", "L0_fear_9", "L0_fear_defend", "L0_fear_new_1", "L0_fear_new_2", "L0_sad_1", "L0_sad_10", "L0_sad_7", "L0_sad_8", "L0_sad_9", "L0_sad_new_1", "L0_sad_new_2", "L0_sad_new_3", "L0_sad_new_4", "L0_sad_wanyao", "L0_14_satisfied", "L0_Approve_24", "L0_Approve_26", "L0_Approve_8", "L0_delicious", "L0_satisfied_10", "L0_satisfied_7", "L0_satisfied_8", "L0_satisfied_9", "L0_satisfied_new_1", "L0_crouch_off", "A-C04_mod", "A-C07_mod", "A-C08_mod", "A-S04_mod", "A-T08-02_mod"]'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('legged_controllers'), 'launch', 'sim2real.launch.py'])), launch_arguments={
                'robot_type': 'l0_v5', 'subpolicy': LaunchConfiguration('subpolicy'),
                'joystick_wbc_enabled': LaunchConfiguration('joystick_wbc_enabled'),
                'joystick_wbc_config_path': LaunchConfiguration('joystick_wbc_config_path'),
                'amp_to_wbc_transition_sec': LaunchConfiguration('amp_to_wbc_transition_sec'),
                'wbc_to_amp_transition_sec': LaunchConfiguration('wbc_to_amp_transition_sec'),
                'wbc_enabled': 'true', 'wbc_mode': 'lora',
                'wbc_lora_bundle': LaunchConfiguration('wbc_lora_bundle'),
                'wbc_policies': LaunchConfiguration('wbc_policies'),
                'hpm_mode': LaunchConfiguration('hpm_mode'),
                'isolated_ros_domain': LaunchConfiguration('isolated_ros_domain'),
            }.items()),
    ])
