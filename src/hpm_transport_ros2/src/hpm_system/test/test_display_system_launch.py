#!/usr/bin/env python3
"""System launch isolation check. Both mock services bind only to loopback."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
from test_display_runtime import Gateway
from udp_test_peer import UdpPeer
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool


def imu_frame(message_id, payload):
    frame = struct.pack("!IBBHI16s", 0x0a4d5048, 2, message_id, 1, 0, payload)
    crc = 0xffff
    for byte in frame:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return frame + struct.pack("!H", crc)


def run(node, mode):
    gateway = Gateway()
    original = UdpPeer().__enter__()
    states, imu = [], []
    state_sub = node.create_subscription(String, "/display/connection/state",
        lambda msg: states.append(msg.data), QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    imu_sub = node.create_subscription(Imu, "/imu/data", imu.append, qos_profile_sensor_data)
    log = tempfile.TemporaryFile(mode="w+")
    port = original.port
    display_port = {"invalid": "abc", "conflict": port}.get(mode, gateway.port)
    env = os.environ.copy()
    yaml_prefix = None
    if mode == "yaml":
        yaml_prefix = tempfile.TemporaryDirectory()
        share = Path(yaml_prefix.name) / "share"
        resource = share / "ament_index/resource_index/packages/hpm_display"
        resource.parent.mkdir(parents=True)
        resource.touch()
        config = share / "hpm_display/config/display.yaml"
        config.parent.mkdir(parents=True)
        config.write_text("invalid_yaml: [", encoding="utf-8")
        env["AMENT_PREFIX_PATH"] = yaml_prefix.name + ":" + env.get("AMENT_PREFIX_PATH", "")
    fault = mode in ("publish", "worker")
    if fault:
        env.update(LD_PRELOAD=sys.argv[1], HPM_DISPLAY_TEST_FAULT=mode)
    gateway.respond_to_greeting = mode != "handshake"
    gateway.respond_to_hello = mode != "backoff"
    process = subprocess.Popen([
        "ros2", "launch", "hpm_system", "hpm_system.launch.py",
        f"launch_display:={'false' if mode == 'disabled' else 'true'}", "board_ip:=127.0.0.1",
        f"port:={port}", f"display_port:={display_port}",
        "set_realtime_priority:=false", "reconnect_ms:=1000", "enable_udp:=true", "enable_tactile_uart:=false",
        "launch_motor:=false", "launch_imu:=true", "launch_tactile:=false",
        "launch_ultrasonic:=false", "launch_battery:=false"], stdout=log,
        stderr=subprocess.STDOUT, env=env)

    def until(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, "system launch exited"
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(f"timed out in {mode}; display states={states}")

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def check_main_link():
        client = node.create_client(GetParameters, "/hpm_system/get_parameters")
        try:
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(GetParameters.Request(names=["board_ip"]))
            until(future.done)
            assert future.result().values[0].string_value == "127.0.0.1"
        finally:
            node.destroy_client(client)
        before = len(imu)
        for message_id in (0x30, 0x31, 0x32, 0x33):
            payload = struct.pack("!4i", 1_000_000, 0, 0, 0)
            original.send_frame(imu_frame(message_id, payload))
        until(lambda: len(imu) > before)
        assert imu[-1].orientation.w == 1.0

    try:
        original.wait_ready()
        until(lambda: node.count_publishers("/imu/data") == 1 and "hpm_system" in node.get_node_names())
        names = node.get_node_names()
        assert "hpm_display" not in names and names.count("hpm_system") == 1, names
        check_main_link()
        if mode in ("enabled", "handshake", "backoff", "maintenance") or fault:
            until(lambda: gateway.connections >= 1)
            assert gateway.connections == 1, "display did not use its independent port"
            endpoints = node.get_publishers_info_by_topic("/display/connection/state")
            assert [endpoint.node_name for endpoint in endpoints] == ["hpm_system"]
            service = node.create_client(SetBool, "/display/upgrade/maintenance")
            try:
                if mode == "publish":
                    until(lambda: "online" in states)
                    gateway.respond_to_hello = False
                if mode == "handshake":
                    until(lambda: states and states[-1] == "connecting")
                if mode == "backoff":
                    until(lambda: states and states[-1] == "offline")
                if fault or mode == "maintenance":
                    if fault:
                        until(lambda: states and states[-1] == "stopped")
                    else:
                        until(lambda: "online" in states)
                    future = service.call_async(SetBool.Request(data=True))
                    until(future.done)
                    assert future.result().success
                    until(lambda: gateway.peer is None)
                    check_main_link()
                    if fault:
                        future = service.call_async(SetBool.Request(data=False))
                        until(future.done)
                        assert not future.result().success
                        assert "injected display" in future.result().message
                        spin_for(1.1)
                        assert gateway.connections == 1, "failed display reconnected"
            finally:
                node.destroy_client(service)
        else:
            spin_for(0.3)
            assert gateway.connections == 0
            assert node.count_publishers("/display/connection/state") == 0, mode
            assert "/display/upgrade/maintenance" not in dict(node.get_service_names_and_types())
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        gateway.stop()
        original.__exit__(None, None, None)
        if yaml_prefix is not None:
            yaml_prefix.cleanup()
        node.destroy_subscription(state_sub)
        node.destroy_subscription(imu_sub)
        log.seek(0)
        text = log.read()
        log.close()
        if process.returncode != 0:
            raise AssertionError(f"launch shutdown failed: {process.returncode}\n{text}")
        if sys.exc_info()[0]:
            print(text, file=sys.stderr)
        assert gateway.peer is None, text
        if mode in ("invalid", "conflict", "yaml"):
            assert "Display disabled at startup:" in text, text
        if fault:
            assert text.count("Display stopped:") == 1 and "injected display" in text, text
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if "hpm_display" not in node.get_node_names() and "hpm_system" not in node.get_node_names():
            return
    raise AssertionError("launch child nodes survived shutdown")


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("display_system_launch_check")
    try:
        for mode in ("disabled", "enabled", "invalid", "conflict", "yaml", "handshake",
                     "backoff", "maintenance", "worker", "publish"):
            print(f"Checking display launch mode {mode}", flush=True)
            run(node, mode)
        print("PASS: shared board_ip, independent ports, launch disabled/enabled, outage isolation and shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()
