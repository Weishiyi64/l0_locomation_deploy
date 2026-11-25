"""Start generation only; enable the existing robot controller separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_manifest', description='Manifest compiled from the current ONNX delivery'),
        DeclareLaunchArgument('mapping_file', default_value=PathJoinSubstitution([
            FindPackageShare('system_command_arbiter'), 'config', 'l0_v5.yaml'])),
        DeclareLaunchArgument('backend', default_value='lite2'),
        Node(package='legged_controllers', executable='text_motion_node', output='screen',
             parameters=[{
                 'model_manifest': LaunchConfiguration('model_manifest'),
                 'mapping_file': LaunchConfiguration('mapping_file'),
                 'backend': LaunchConfiguration('backend'),
             }]),
    ])
