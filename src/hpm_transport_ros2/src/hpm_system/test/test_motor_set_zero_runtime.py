#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

"""Actual node/service/topic tests against a local UDP board simulator."""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import yaml

os.environ["ROS_DOMAIN_ID"] = "163"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorTarget
from luvbot_hardware_interfaces.srv import MotorSetZero
from udp_test_peer import UdpPeer


def reply(wire, can_id, data, *, port=None, ide=None, dlc=None, msg_id=None):
    packet = bytearray(wire)
    packet[15:19] = can_id.to_bytes(4, "big")
    packet[19] = len(data) if dlc is None else dlc
    packet[20:28] = bytes(data).ljust(8, b"\0")
    if port is not None:
        packet[13] = port
    if ide is not None:
        packet[14] = ide
    if msg_id is not None:
        packet[5] = msg_id
    crc = 0xFFFF
    for byte in packet[:28]:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    packet[28:30] = crc.to_bytes(2, "big")
    return packet


def check(executable):
    with tempfile.TemporaryDirectory(prefix="hpm-set-zero-check-") as tmp:
        cfg = Path(tmp) / "motors.yaml"
        cfg.write_text(
            "motors:\n"
            "  dm: {index: 4, motor_type: DM, bus_index: 0, motor_id: 1}\n"
            "  rs: {index: 0, motor_type: RS, bus_index: 1, motor_id: 2}\n"
            "  enc: {index: 3, motor_type: ENC, bus_index: 2, motor_id: 3}\n"
            "  mr: {index: 1, motor_type: MR, model: PA43-10-20.25, bus_index: 3, motor_id: 4}\n"
            "  ls: {index: 2, motor_type: LS, bus_index: 4, motor_id: 5}\n"
        )
        protocol_config = yaml.safe_load(
            (Path(__file__).resolve().parents[3] / "src/hpm_motor/config/motors.yaml").read_text())
        protocol_config["motors"] = yaml.safe_load(cfg.read_text())["motors"]
        cfg.write_text(yaml.safe_dump(protocol_config))
        with UdpPeer() as listener, open(Path(tmp) / "node.log", "w+") as log:
            proc = subprocess.Popen(
                [executable, "--ros-args", "-p", "board_ip:=127.0.0.1",
                 "-p", f"port:={listener.port}", "-p", "enable_udp:=true", "-p", "launch_motor:=true",
                 "-p", "launch_imu:=false", "-p", "launch_tactile:=false",
                 "-p", "motor_set_zero_timeout_ms:=700", "-p", f"motor_config_file:={cfg}"],
                stdout=log, stderr=log,
            )
            node = None
            cli = None
            try:
                conn = listener
                conn.wait_ready()
                with conn:
                    conn.settimeout(3)
                    rclpy.init()
                    node = rclpy.create_node("set_zero_runtime_test")
                    client = node.create_client(MotorSetZero, "/actuator/motor/set_zero")
                    assert client.wait_for_service(timeout_sec=5)
                    enable_pub = node.create_publisher(MotorEnableCommand, "/actuator/motor/enable", 10)
                    command_pub = node.create_publisher(MotorCommand, "/actuator/motor/command", 10)
                    deadline = time.monotonic() + 5
                    while enable_pub.get_subscription_count() == 0 or command_pub.get_subscription_count() == 0:
                        assert time.monotonic() < deadline
                        rclpy.spin_once(node, timeout_sec=0.05)

                    def receive_frame():
                        wire = b""
                        while len(wire) < 30:
                            chunk = conn.recv(30 - len(wire))
                            assert chunk, "TCP connection closed"
                            wire += chunk
                        return wire

                    def receive_batch(count):
                        wires = {}
                        for _ in range(count):
                            wire = receive_frame()
                            assert wire[13] not in wires
                            wires[wire[13]] = wire
                        return wires

                    def request(types):
                        return client.call_async(MotorSetZero.Request(motor_types=types))

                    def result(future):
                        deadline = time.monotonic() + 5
                        while not future.done():
                            assert time.monotonic() < deadline, "Service did not return"
                            rclpy.spin_once(node, timeout_sec=0.02)
                        return future.result()

                    def successful_feedback(wire):
                        port = wire[13]
                        if port == 0:  # DM: enabled status is healthy.
                            return reply(wire, 1, [0x11, 0x80, 0, 0x80, 0, 0, 40, 40])
                        if port == 1:  # RS feedback message type 2, ID in CAN bits 8..15.
                            return reply(wire, 0x02000200, [0x80, 0, 0x80, 0, 0x80, 0, 0, 0])
                        if port == 3:  # MR MIT status: enabled, no faults.
                            return reply(wire, 0x304, [0x80, 0, 0x80, 0, 0, 40, 0, 1])
                        # ENC/LS are identified by bus + full returned ID,
                        # independent of the logical msg_id low nibble.
                        return reply(wire, 0x7FF, [0, 3 if port == 2 else 5, 1, 3], msg_id=0x1F)

                    # The real CLI exercises all protocols and out-of-order feedback.
                    cli = subprocess.Popen(
                        ["ros2", "service", "call", "/actuator/motor/set_zero",
                         "luvbot_hardware_interfaces/srv/MotorSetZero", "{motor_types: [3, 255, 2, 1, 4, 0]}"],
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                    )
                    wires = receive_batch(5)
                    for port, wire in wires.items():
                        can_id = int.from_bytes(wire[15:19], "big")
                        if port in (0, 3):
                            assert can_id == (1 if port == 0 else 0x104)
                            assert wire[20:28] == b"\xff" * 7 + b"\xfe"
                        elif port in (2, 4):
                            assert can_id == 0x7FF and wire[19] == 4
                            assert wire[22:24] == bytes([0, 3])
                        else:
                            assert (can_id >> 24) & 0x1F == 6
                    # Command loopbacks, other IDs/buses, wrong opcodes and DLC
                    # must not complete a batch. Send valid replies afterwards.
                    for wire in wires.values():
                        conn.sendall(wire)
                    conn.sendall(reply(wires[2], 0x7FF, [0, 99, 1, 3]))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 3], port=7))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 1]))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 3], dlc=5))
                    conn.sendall(successful_feedback(wires[4]))
                    for port in (1, 0, 3, 2):
                        conn.sendall(successful_feedback(wires[port]))
                    output, error = cli.communicate(timeout=5)
                    assert cli.returncode == 0, error
                    assert "results=[0, 1, 0, 0, 0, 0]" in output, output
                    print("PASS: CLI, five SETZERO encodings, matching feedback and ordered batch results")

                    # Explicit device failures are distinct from timeouts.
                    future = request([2, 1, 4, 0])
                    wires = receive_batch(4)
                    conn.sendall(reply(wires[4], 0x7FF, [0, 5, 1, 0]))
                    conn.sendall(reply(wires[3], 0x304, [0x80, 0, 0x80, 0, 0, 40, 0, 2]))
                    conn.sendall(reply(wires[0], 1, [0x81, 0x80, 0, 0x80, 0, 0, 40, 40]))
                    conn.sendall(reply(wires[1], 0x02010200, [0x80, 0, 0x80, 0, 0x80, 0, 0, 0]))
                    response = result(future)
                    assert list(response.results) == [3, 3, 3, 3], response
                    print("PASS: ENC/LS status failure and DM/RS/MR MIT faults")

                    future = request([3, 2, 1])
                    wires = receive_batch(3)
                    conn.sendall(wires[2])
                    conn.sendall(reply(wires[2], 0x7FF, [0, 99, 1, 3]))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 3], port=7))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 1]))
                    conn.sendall(reply(wires[2], 0x7FF, [0, 3, 1, 3], dlc=5))
                    conn.sendall(successful_feedback(wires[4]))
                    conn.sendall(successful_feedback(wires[3]))
                    busy = request([4])
                    assert list(result(busy).results) == [1]
                    assert not future.done(), "Wrong ID/opcode/loopback confirmed ENC prematurely"
                    # Existing topics continue to work while the service waits.
                    enable = MotorEnableCommand(motor_types=[1], enabled=[True])
                    enable_pub.publish(enable)
                    assert receive_frame()[27] == 0xFC
                    command = MotorCommand(control_mode=MotorCommand.CONTROL_MODE_MIT)
                    target = MotorTarget()
                    target.route.motor_type = 1
                    target.kp = 1.0
                    command.commands = [target]
                    command_pub.publish(command)
                    assert int.from_bytes(receive_frame()[15:19], "big") == 0x204
                    enable.enabled = [False]
                    enable_pub.publish(enable)
                    assert receive_frame()[27] == 0xFD
                    assert list(result(future).results) == [4, 0, 0]
                    print("PASS: overlap rejected, partial timeout, topics work during wait")

                    # A late ENC reply cannot confirm a subsequent batch or clear quarantine.
                    future = request([3, 1])
                    wire = receive_frame()
                    assert wire[13] == 3
                    conn.sendall(successful_feedback(wires[2]))
                    conn.sendall(successful_feedback(wire))
                    assert list(result(future).results) == [1, 0]
                    conn.settimeout(0.2)
                    try:
                        unexpected = conn.recv(30)
                        raise AssertionError(f"Unexpected retry bytes: {unexpected.hex()}")
                    except socket.timeout:
                        pass
                    print("PASS: late feedback ignored and timed-out motor is not resent")
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if cli is not None and cli.poll() is None:
                    cli.kill()
                    cli.communicate()
                if node is not None:
                    node.destroy_node()
                    rclpy.shutdown()
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


if __name__ == "__main__":
    check(sys.argv[1])
