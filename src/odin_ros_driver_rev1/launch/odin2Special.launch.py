import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('odin_ros_driver_rev1')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'odin2special_driver_view_ros2.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'driver.launch.py')),
            launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items(),
        ),
    ])
