from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cloud_bridge = Node(
        package='luvbot_cloud_bridge', executable='cloud_bridge_node', output='screen',
        parameters=[{
            'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=str),
            'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
            'cert_dir': ParameterValue(LaunchConfiguration('cert_dir'), value_type=str),
            'ca_file': ParameterValue(LaunchConfiguration('ca_file'), value_type=str),
            'wake_timeout_sec': ParameterValue(LaunchConfiguration('wake_timeout_sec'), value_type=float),
            'health.overheat_c': ParameterValue(LaunchConfiguration('health_overheat_c'), value_type=float),
            'health.recover_c': ParameterValue(LaunchConfiguration('health_recover_c'), value_type=float),
        }],
    )
    network_gate = Node(
        package='luvbot_provisioning', executable='network_ready_gate', output='screen',
    )

    def start_cloud_bridge(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='Network gate failed; cloud bridge was not started. Restart this launch after fixing the error.')]
        return [LogInfo(msg='Network ready; starting cloud bridge.'), cloud_bridge]

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='RUMI-TEST-007'),
        DeclareLaunchArgument('device_sn', default_value='RUMI-TEST-007'),
        DeclareLaunchArgument('host', default_value='mqtt.luvbotics.com'),
        DeclareLaunchArgument('cert_dir', default_value=''),
        DeclareLaunchArgument('ca_file', default_value='/etc/ssl/certs/ca-certificates.crt'),
        DeclareLaunchArgument('wake_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('health_overheat_c', default_value='85.0'),
        DeclareLaunchArgument('health_recover_c', default_value='80.0'),
        # Register before starting the gate, including when it immediately finds an existing network.
        RegisterEventHandler(OnProcessExit(target_action=network_gate, on_exit=start_cloud_bridge)),
        network_gate,
        Node(
            package='luvbot_provisioning', executable='luvbot_bridge', output='screen',
            parameters=[{
                'wait_for_trigger': True,
                'device_sn': ParameterValue(LaunchConfiguration('device_sn'), value_type=str),
            }],
        ),
    ])
