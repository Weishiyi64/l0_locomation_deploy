#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Run the installed launch twice against loopback in a disposable colcon workspace."""
import json
import math
import os
import re
import signal
import struct
import subprocess
import tempfile
import time
import unicodedata
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from luvbot_hardware_interfaces.msg import MotorEnableCommand, MotorCommand, MotorTarget, MotorStateArray
from udp_test_peer import DATA, HEADER, UdpPeer


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
    return struct.pack("!H", value)


def wire(port, can_id, data, ide=0, msg=0x15):
    body = struct.pack("!IBBHIBBBIB8s", 0x0a4d5048, 2, msg, 1, 42, 0, port,
                       ide, can_id, len(data), data)
    return body + crc(body)


class Board(UdpPeer):
    def packet(self, kind, count, payload):
        with self._lock:
            self._tx_sequence += 1
            packet = HEADER.pack(b"HPU1", kind, count, len(payload) + (2 if kind == 10 else 0),
                                 self._session, self._tx_sequence) + payload
            self.sock.sendto(packet + (crc(packet) if kind == 10 else b""), self._peer)

    def snapshot(self, slots):
        payload = bytearray(20 + 21 * 23)
        struct.pack_into("!4B4I", payload, 0, 1, 21, 0, 0, 1, 10, 20,
                         sum(1 << index for index in slots))
        for index, frame in slots.items():
            payload[20 + 23 * index:20 + 23 * (index + 1)] = frame[5:28]
        self.packet(10, 21, payload)


@contextmanager
def launching(tmp, env, *args):
    with Board() as peer, (tmp / "launch_output").open("w+") as output:
        proc = subprocess.Popen([
            "ros2", "launch", "hpm_system", "hpm_system.launch.py", "board_ip:=127.0.0.1",
            f"port:={peer.port}", "set_realtime_priority:=false", "launch_imu:=false", "launch_tactile:=false",
            "launch_battery:=false", "launch_ultrasonic:=false", "motor_log_enabled:=true",
            f"motor_config_file:={tmp / 'motors.yaml'}", *args
        ], cwd=tmp, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            peer.wait_ready(10)
            yield peer
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                raise
            output.seek(0)
            log = output.read()
            if proc.returncode != 0 or "recording disabled" in log or "dropped frames total" in log:
                raise AssertionError(log)


def until(check):
    deadline = time.monotonic() + 5
    while not check():
        assert time.monotonic() < deadline, "timed out waiting for recording or ROS discovery"
        time.sleep(0.025)


LINE = re.compile(
    r"(?P<stamp>\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{6}) (?P<direction>TX|RX)"
    r' bus=(?P<bus>\d+) +motor=(?P<name>"(?:[^"\\]|\\.)*") +motor_id=(?P<id>\d+|-)'
    r" +can_id=(?P<can_id>0x[0-9A-F]+) +format=(?P<format>STD|EXT) dlc=(?P<dlc>\d+)"
    r" data=\[(?P<data>[0-9A-F ]*)\] +elapsed_us=(?P<elapsed_us>\d+) +seq=(?P<seq>\d+) +topic=(?P<topic>\S+)(?: +(?P<content>.*))?"
)
FILENAME = re.compile(r"motor_can_(\d{8}_\d{6}_\d{6})_\d{6,}\.log")


def rows(path):
    records = []
    columns = None
    for line in path.read_text().splitlines():
        match = LINE.fullmatch(line)
        assert match, (path, line)
        offsets = [0]
        for char in line:
            width = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in ("W", "F") else 1)
            offsets.append(offsets[-1] + width)
        starts = [offsets[match.start(field)] for field in LINE.groupindex if field != "content"]
        if columns is None:
            columns = starts
        assert starts == columns, (path, line, columns, starts)
        records.append(match.groupdict())
    return records


def check_frames(actual, expected):
    assert len(actual) == len(expected), actual
    for row, (frame, direction, name, motor_id) in zip(actual, expected):
        assert row["direction"] == direction and json.loads(row["name"]) == name, row
        assert row["id"] == motor_id and int(row["bus"]) == frame[13], row
        assert int(row["can_id"], 16) == int.from_bytes(frame[15:19], "big"), row
        assert row["format"] == ("EXT" if frame[14] else "STD"), row
        assert int(row["dlc"]) == frame[19] and row["data"] == frame[20:20 + frame[19]].hex(" ").upper(), row
        assert abs(time.time() - datetime.strptime(row["stamp"], "%Y-%m-%d %H:%M:%S.%f").timestamp()) < 60
    assert [int(r["seq"]) for r in actual] == list(range(1, 1 + len(expected)))
    times = [int(r["elapsed_us"]) for r in actual]
    assert times == sorted(times) and times[0] >= 0


def check(tmp):
    prefix = tmp / "ros2_ws/install/hpm_system"
    prefix.parent.mkdir(parents=True)
    prefix.symlink_to(get_package_prefix("hpm_system"), target_is_directory=True)
    folder = tmp / "ros2_ws/motor_log"
    env = dict(os.environ, AMENT_PREFIX_PATH=str(prefix) + os.pathsep + os.environ["AMENT_PREFIX_PATH"])
    config = yaml.safe_load((Path(get_package_share_directory("hpm_motor")) / "config/motors.yaml").read_text())
    left_name = '左踝 "LS"\\\r\n\t'
    config["motors"] = {
        left_name: dict(index=0, motor_type="LS", bus_index=2, motor_id=5),
        "right": dict(index=1, motor_type="RS", bus_index=3, motor_id=19),
    }
    (tmp / "motors.yaml").write_text(yaml.safe_dump(config))
    node = rclpy.create_node("motor_log_test")
    pub = node.create_publisher(MotorEnableCommand, "/actuator/motor/enable", 10)
    commands = node.create_publisher(MotorCommand, "/actuator/motor/command", 10)
    received = []
    node.create_subscription(MotorStateArray, "/actuator/motor/state", received.append, qos_profile_sensor_data)
    left = wire(2, 0x7ff, bytes([0, 5, 1, 3]), msg=0x1f)
    right = wire(3, 0x02001300, bytes(range(8)), ide=1, msg=0x13)
    unknown = wire(2, 0x456, bytes([0xab, 0, 0xcd]))
    try:
        with launching(tmp, env) as peer:
            until(lambda: pub.get_subscription_count() > 0)
            pub.publish(MotorEnableCommand(sequence_id=91, motor_types=[0, 1], enabled=[True, False]))
            tx = [peer.recv(30), peer.recv(30)]
            assert [frame[13] for frame in tx] == [2, 3]
            peer.packet(DATA, 3, left + right + unknown)
            # An undecodable LS feedback must not hide the later present RS slot.
            bad_ls = wire(2, 5, bytes(8))
            peer.snapshot({0: bad_ls, 1: right})
            expected = [(tx[0], "TX", left_name, "5"), (tx[1], "TX", "right", "19"),
                        (left, "RX", left_name, "5"), (right, "RX", "right", "19"),
                        (unknown, "RX", "unknown", "-"), (bad_ls, "RX", left_name, "5"),
                        (right, "RX", "right", "19")]
            until(lambda: commands.get_subscription_count() and node.count_publishers("/actuator/motor/state"))
            # Repeated routes with an ignored entry between them must keep their own ROS values.
            msg = MotorCommand(sequence_id=123, control_mode=MotorCommand.CONTROL_MODE_MIT)
            msg.stamp.sec, msg.stamp.nanosec = 7, 42
            for idx, value in [(1, 0.123456789012345), (99, 0.0), (1, -0.375)]:
                target = MotorTarget(q=value, dq=1.25, tau=0.4, kp=4.5, kd=0.75)
                target.route.motor_type = idx
                msg.commands.append(target)
            commands.publish(msg)
            expected += [(peer.recv(30), "TX", "right", "19") for _ in range(2)]
            peer.snapshot({1: right})
            expected.append((right, "RX", "right", "19"))
            until(lambda: rclpy.spin_once(node, timeout_sec=0.05) or bool(received))
            until(lambda: any(len(rows(path)) == len(expected) for path in folder.glob("*.log")))
        first, = folder.iterdir()
        records = rows(first)
        check_frames(records, expected)
        def fields(row):
            return dict(pair.split("=", 1) for pair in row["content"].split())
        for i in (0, 1):
            assert records[i]["topic"] == "/actuator/motor/enable"
            assert int(fields(records[i])["motor_type"]) == i
            assert fields(records[i])["enabled"] == ("true" if i == 0 else "false")
            assert fields(records[i])["sequence_id"] == "91"
        assert all(row["topic"] == "-" for row in records[2:7])
        for row, target in zip(records[7:9], [msg.commands[0], msg.commands[2]]):
            values = fields(row)
            assert row["topic"] == "/actuator/motor/command"
            assert values["sequence_id"] == "123" and values["stamp"] == "7.000000042"
            assert int(values["control_mode"]) == msg.control_mode
            for key in ("motor_type", "bus_index", "motor_id"):
                assert int(values[key]) == getattr(target.route, key)
            for key in ("q", "dq", "tau", "kp", "kd"):
                assert float(values[key]) == getattr(target, key)
        published, = received
        state, = published.states
        row = records[-1]
        assert row["topic"] == "/actuator/motor/state"
        values = fields(row)
        assert int(values["sequence_id"]) == published.sequence_id
        assert int(values["feedback_mask"]) == published.feedback_mask == 2
        assert values["stamp"] == f"{published.stamp.sec}.{published.stamp.nanosec:09d}"
        for key in ("motor_type", "bus_index", "motor_id"):
            assert int(values[key]) == getattr(state.route, key)
        for key in ("error_code", "operating_mode", "enable_state"):
            assert int(values[key]) == getattr(state, key)
        assert values["valid"] == str(state.valid).lower()
        for key in ("q", "dq", "tau", "mos_temperature", "rotor_temperature"):
            actual, wanted = float(values[key]), getattr(state, key)
            assert actual == wanted or (math.isnan(actual) and math.isnan(wanted))
        original = first.read_bytes()
        with launching(tmp, env) as peer:
            peer.send_frame(right)
            until(lambda: any(rows(p) for p in folder.glob("*.log") if p != first))
        second, = set(folder.iterdir()) - {first}
        assert first.read_bytes() == original
        for path in [first, second]:
            match = FILENAME.fullmatch(path.name)
            assert match and abs(time.time() - datetime.strptime(
                match[1], "%Y%m%d_%H%M%S_%f").timestamp()) < 60, path
        check_frames(rows(second), [(right, "RX", "right", "19")])
        print("PASS: UDP TX, DATA, snapshots, exact CAN bytes, identity, timestamps and restart")

        # Over quota before startup: launch cannot evict history until data arrives.
        archive = tmp / "retention"
        archive.mkdir()
        old = [archive / f"motor_can_2020010{i}_000000_000000_000001.log" for i in (1, 2)]
        for i, path in enumerate(old):
            path.write_bytes(bytes([65 + i]) * 550)
            os.utime(path, (10 + i, 10 + i))
        keep = archive / "motor_can_notes.log"
        keep.write_text("manual notes")
        csv = archive / "motor_can_20200101_000000_000000+0800_000001.csv"
        csv.write_bytes(b"old csv" * 200)

        def managed():
            return sorted(path for path in archive.iterdir() if FILENAME.fullmatch(path.name))

        def last_frame_written():
            try:
                return any(r["seq"] == "21" for p in managed() if p not in old for r in rows(p))
            except FileNotFoundError:
                return False  # retention removed a shard between listing and reading it

        with launching(tmp, env, f"motor_log_dir:={archive}", "motor_log_shard_bytes:=380",
                       "motor_log_total_bytes:=1100") as peer:
            until(lambda: len(managed()) == 3)
            assert all(p.stat().st_size == 550 for p in old)
            empty, = set(managed()) - set(old)
            assert empty.stat().st_size == 0
            peer.send_frame(right)
            until(lambda: not old[0].exists())
            assert old[1].read_bytes() == b"B" * 550
            peer.packet(DATA, 20, right * 20)
            until(last_frame_written)
        shards = managed()
        assert len(shards) >= 2 and not any(p.exists() for p in old)
        assert max(p.stat().st_size for p in shards) <= 380
        assert sum(p.stat().st_size for p in shards) <= 1100
        seq = [int(r["seq"]) for p in shards for r in rows(p)]
        assert seq == list(range(seq[0], 22)) and seq[0] > 1, seq
        assert keep.read_text() == "manual notes"
        assert csv.read_bytes() == b"old csv" * 200
        print("PASS: cross-session quota, oldest-first eviction, shard size and unrelated files")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    os.environ["ROS_DOMAIN_ID"] = "167"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["TZ"] = "Asia/Kathmandu"  # distinguish local time from UTC and the developer's timezone
    time.tzset()
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory() as work:
            check(Path(work))
    finally:
        rclpy.shutdown()
