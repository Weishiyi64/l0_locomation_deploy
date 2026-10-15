import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('yesense_imu')
    rviz_config = os.path.join(pkg_share, 'rviz', 'yesense.rviz')

    imu_node = Node(
        package='yesense_imu',
        executable='yesense_imu_node',
        name='yesense_imu_node',
        output='screen',
        parameters=[{'yesense_port': '/dev/yesense_imu', 'yesense_baudrate': 460800}],
    )

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz', arguments=['-d', rviz_config], output='screen')

    return LaunchDescription([imu_node, rviz_node])
