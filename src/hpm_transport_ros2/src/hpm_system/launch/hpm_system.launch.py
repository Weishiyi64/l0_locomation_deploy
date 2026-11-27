from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


class _MiBToBytes(LaunchConfiguration):
    def perform(self, context):
        return str(int(super().perform(context)) * 1024 * 1024)


class _SchedulingPrefix(LaunchConfiguration):
    def perform(self, context):
        if super().perform(context) == "false":
            return ""
        policy = LaunchConfiguration("thread_scheduling_method").perform(context)
        priority = int(LaunchConfiguration("thread_priority").perform(context))
        if not 1 <= priority <= 99:
            raise ValueError("thread_priority must be 1..99 for SCHED_FIFO/SCHED_RR")
        flag = {"SCHED_FIFO": "--fifo", "SCHED_RR": "--rr"}[policy]
        return f"chrt {flag} {priority}"


def _start_system(context, parameters):
    enabled = LaunchConfiguration("launch_display").perform(context) == "true"
    parameters = {**parameters, "launch_display": enabled}
    if enabled:
        try:
            port = LaunchConfiguration("display_port").perform(context)
            display_port = int(port)
            if not 1 <= display_port <= 65535:
                raise ValueError(f"display_port must be 1..65535, got {port!r}")
            config_path = Path(get_package_share_directory("hpm_display")) / "config/display.yaml"
            with config_path.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream)["hpm_system"]["ros__parameters"]
            keys = ("ack_timeout_ms", "max_retries", "keepalive_ms", "hello_timeout_ms")
            if any(type(config[key]) is not int or not -(2**63) <= config[key] < 2**63 for key in keys):
                raise ValueError("display.yaml requires signed 64-bit integer timeout and retry parameters")
            parameters.update({key: config[key] for key in keys})
            parameters["display_port"] = display_port
        except (OSError, ValueError, LookupError, TypeError, yaml.YAMLError) as exc:
            parameters["display_config_error"] = str(exc)
    return [Node(
        package="hpm_system",
        executable="hpm_system_node",
        prefix=_SchedulingPrefix("set_realtime_priority"),
        name="hpm_system",
        output="screen",
        parameters=[parameters],
    )]


def generate_launch_description():
    with Path(__file__).with_name("config.yaml").open(encoding="utf-8") as stream:
        realtime = yaml.safe_load(stream)["hpm_system"]["ros__parameters"]
    board_ip = LaunchConfiguration("board_ip")
    port = LaunchConfiguration("port")
    reconnect_ms = LaunchConfiguration("reconnect_ms")
    enable_udp = LaunchConfiguration("enable_udp")
    enable_tactile_uart = LaunchConfiguration("enable_tactile_uart")
    tactile_uart_device = LaunchConfiguration("tactile_uart_device")
    tactile_uart_baud = LaunchConfiguration("tactile_uart_baud")
    tactile_uart_reconnect_ms = LaunchConfiguration("tactile_uart_reconnect_ms")

    return LaunchDescription([
        DeclareLaunchArgument("set_realtime_priority",
                              default_value=str(realtime["set_realtime_priority"]).lower(),
                              choices=["true", "false"],
                              description="Apply realtime scheduling before starting hpm_system_node; "
                                          "requires sufficient rtprio or CAP_SYS_NICE"),
        DeclareLaunchArgument("thread_scheduling_method",
                              default_value=realtime["thread_scheduling_method"],
                              choices=["SCHED_FIFO", "SCHED_RR"]),
        DeclareLaunchArgument("thread_priority", default_value=str(realtime["thread_priority"])),
        DeclareLaunchArgument("enable_udp", default_value="true"),
        DeclareLaunchArgument("board_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("port", default_value="5001"),
        DeclareLaunchArgument("reconnect_ms", default_value="1000"),
        DeclareLaunchArgument("launch_display", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("display_port", default_value="5002"),
        
        DeclareLaunchArgument("motor_log_enabled", default_value="false"),
        DeclareLaunchArgument("motor_log_dir", default_value="",
                              description="Default: motor_log under the active ROS workspace"),
        DeclareLaunchArgument("motor_log_shard_mib", default_value="64"),
        DeclareLaunchArgument("motor_log_total_mib", default_value="512"),
        DeclareLaunchArgument("launch_motor", default_value="true"),
        DeclareLaunchArgument(
            "ls_firmware_version", default_value="old", choices=["old", "new"],
            description="LS type 1 feedback firmware format",
        ),
        DeclareLaunchArgument("motor_id_calibration_mode", default_value="false"),
        DeclareLaunchArgument("motor_id_calibration_timeout_ms", default_value="1000"),
        
        DeclareLaunchArgument("launch_imu", default_value="true"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),

        DeclareLaunchArgument("launch_ultrasonic", default_value="true"),
        DeclareLaunchArgument("ultrasonic_frame_id", default_value="ultrasonic_link"),
        DeclareLaunchArgument("ultrasonic_field_of_view", default_value="0.6981317"),
        DeclareLaunchArgument("ultrasonic_timeout_ms", default_value="500"),

        DeclareLaunchArgument("launch_battery", default_value="true"),

        DeclareLaunchArgument("launch_tactile", default_value="true"),
        DeclareLaunchArgument("enable_tactile_uart", default_value="true"),
        DeclareLaunchArgument("tactile_uart_device", default_value="/dev/ttyS5"),
        DeclareLaunchArgument("tactile_uart_baud", default_value="921600"),
        DeclareLaunchArgument("tactile_uart_reconnect_ms", default_value="1000"),
        DeclareLaunchArgument(
            "motor_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hpm_motor"), "config", "motors.yaml",
            ]),
        ),
        OpaqueFunction(function=_start_system, kwargs={"parameters": {
                "set_realtime_priority": ParameterValue(LaunchConfiguration("set_realtime_priority"), value_type=bool),
                "thread_scheduling_method": ParameterValue(LaunchConfiguration("thread_scheduling_method"), value_type=str),
                "thread_priority": ParameterValue(LaunchConfiguration("thread_priority"), value_type=int),
                "board_ip": board_ip,
                "port": ParameterValue(port, value_type=int),
                "enable_udp": ParameterValue(enable_udp, value_type=bool),
                "reconnect_ms": ParameterValue(reconnect_ms, value_type=int),
                "enable_tactile_uart": ParameterValue(enable_tactile_uart, value_type=bool),
                "tactile_uart_device": tactile_uart_device,
                "tactile_uart_baud": ParameterValue(tactile_uart_baud, value_type=int),
                "tactile_uart_reconnect_ms": ParameterValue(
                    tactile_uart_reconnect_ms, value_type=int),
                "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                "launch_ultrasonic": ParameterValue(
                    LaunchConfiguration("launch_ultrasonic"), value_type=bool),
                "ultrasonic_frame_id": LaunchConfiguration("ultrasonic_frame_id"),
                "ultrasonic_field_of_view": ParameterValue(
                    LaunchConfiguration("ultrasonic_field_of_view"), value_type=float),
                "ultrasonic_timeout_ms": ParameterValue(
                    LaunchConfiguration("ultrasonic_timeout_ms"), value_type=int),
                "launch_motor": ParameterValue(
                    LaunchConfiguration("launch_motor"), value_type=bool),
                "motor_log_enabled": ParameterValue(LaunchConfiguration("motor_log_enabled"), value_type=bool),
                "motor_log_dir": ParameterValue(LaunchConfiguration("motor_log_dir"), value_type=str),
                "motor_log_shard_bytes": ParameterValue(LaunchConfiguration(
                    "motor_log_shard_bytes", default=_MiBToBytes("motor_log_shard_mib")), value_type=int),
                "motor_log_total_bytes": ParameterValue(LaunchConfiguration(
                    "motor_log_total_bytes", default=_MiBToBytes("motor_log_total_mib")), value_type=int),
                "motor_config_file": LaunchConfiguration("motor_config_file"),
                "ls_firmware_version": ParameterValue(
                    LaunchConfiguration("ls_firmware_version"), value_type=str),
                "motor_id_calibration_mode": ParameterValue(
                    LaunchConfiguration("motor_id_calibration_mode"), value_type=bool),
                "motor_id_calibration_timeout_ms": ParameterValue(
                    LaunchConfiguration("motor_id_calibration_timeout_ms"), value_type=int),
                "launch_tactile": ParameterValue(
                    LaunchConfiguration("launch_tactile"), value_type=bool),
                "launch_imu": ParameterValue(
                    LaunchConfiguration("launch_imu"), value_type=bool),
                "launch_battery": ParameterValue(
                    LaunchConfiguration("launch_battery"), value_type=bool),
        }}),
    ])
