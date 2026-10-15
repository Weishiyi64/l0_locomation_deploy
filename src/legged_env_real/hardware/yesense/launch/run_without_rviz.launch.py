from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='yesense_imu',
        executable='yesense_imu_node',
        name='yesense_imu_node',
        output='screen',
        parameters=[{'yesense_port': '/dev/yesense_imu', 'yesense_baudrate': 460800}],
    )

    return LaunchDescription([imu_node])
