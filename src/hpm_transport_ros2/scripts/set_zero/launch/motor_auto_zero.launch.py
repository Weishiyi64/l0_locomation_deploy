# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Start only the homing node; hpm_system must already be running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from set_zero.motor_auto_zero import OVERRIDABLE


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    if context.launch_configurations.get("module"):
        raise RuntimeError("整组标零请调用 /set_zero/calibrate Action")
    index = value("motor_index")
    if not index:
        raise RuntimeError("Specify motor_index:=... for single-motor calibration")
    arguments = ["--motor-index", index]
    for name, flag in (
        ("motor_config_file", "--config"), ("calibration_config", "--calibration-config"),
        ("urdf_file", "--urdf"), ("log_dir", "--log-dir"),
        ("service_timeout", "--service-timeout"), ("system_node", "--system-node"),
    ):
        arguments.extend([flag, value(name)])
    enabled_modes = []
    for name in ("dry_run", "check_only"):
        setting = value(name).lower()
        if setting not in {"true", "false"}:
            raise RuntimeError(f"{name} must be true or false")
        if setting == "true":
            enabled_modes.append(name)
            arguments.append("--" + name.replace("_", "-"))
    if len(enabled_modes) > 1:
        raise RuntimeError("dry_run and check_only are mutually exclusive")
    write_zero = value("write_zero").lower()
    if write_zero not in {"", "true", "false"}:
        raise RuntimeError("write_zero must be empty (use YAML), true or false")
    if write_zero:
        arguments.append("--write-zero" if write_zero == "true" else "--no-write-zero")
    for name in OVERRIDABLE:
        if value(name):
            arguments.extend(["--" + name.replace("_", "-"), value(name)])
    node = Node(package="set_zero", executable="motor_auto_zero", output="screen",
                arguments=arguments, respawn=False, sigterm_timeout="3", sigkill_timeout="2")

    def finished(event, context):
        if event.returncode not in (0, 130):
            raise RuntimeError(f"motor_auto_zero failed with exit code {event.returncode}; see its result above")
        return [EmitEvent(event=Shutdown(reason="motor_auto_zero exited"))]

    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=finished)), node]


def generate_launch_description():
    share = FindPackageShare("set_zero")
    defaults = {
        "motor_index": "", "dry_run": "false", "check_only": "false", "write_zero": "",
        "motor_config_file": PathJoinSubstitution([FindPackageShare("hpm_motor"), "config", "motors.yaml"]),
        "calibration_config": PathJoinSubstitution([share, "config", "motor_zero_calibration.yaml"]),
        "urdf_file": PathJoinSubstitution([share, "config", "l0_v5.urdf"]),
        "log_dir": "set_zero_logs", "service_timeout": "10.0", "system_node": "/hpm_system",
    }
    defaults.update({name: "" for name in OVERRIDABLE})
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=start),
    ])
