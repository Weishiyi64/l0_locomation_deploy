#!/usr/bin/env python3
"""Exercise the CLI and the real ROS service with a localhost-only fake HPM."""

import os
from pathlib import Path
import subprocess
import sys

from test_motor_id_calibration_runtime import can_id, no_send, receive, result, session
from test_motor_set_zero_runtime import reply

import rclpy
from rcl_interfaces.srv import GetParameters
from luvbot_hardware_interfaces.srv import MotorReadLsError


SCRIPT = Path(__file__).resolve().parents[3] / "scripts/motor/read_ls_error.py"
SERVICE = "/actuator/motor/read_ls_error"


def cli(index):
    return subprocess.Popen([sys.executable, str(SCRIPT), str(index)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def output(process, success, *expected):
    try:
        text, _ = process.communicate(timeout=8)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    assert (process.returncode == 0) == success, (process.returncode, text)
    assert all(item in text for item in expected), text
    return text


def query_frame(peer):
    wire = receive(peer)
    assert (wire[5], wire[13], wire[14], can_id(wire), wire[19]) == (0x11, 1, 0, 65, 2), wire.hex()
    assert wire[20:22] == bytes.fromhex("E0 24"), wire.hex()
    return wire


def run(executable):
    guard = rclpy.create_node("ls_error_offline_guard")
    try:
        assert not guard.create_client(MotorReadLsError, SERVICE).wait_for_service(timeout_sec=0.5), (
            "Test domain already has a motor service; refusing to issue any requests")
    finally:
        guard.destroy_node()
    output(cli(2), False, "未找到", SERVICE)
    with session(executable, mode=False) as (node, _, peer, _):
        owner = (peer._peer, peer._session)
        client = node.create_client(MotorReadLsError, SERVICE)
        assert client.wait_for_service(timeout_sec=3)
        params = node.create_client(GetParameters, "/hpm_system/get_parameters")
        assert params.wait_for_service(timeout_sec=3)

        def request(index):
            return client.call_async(MotorReadLsError.Request(motor_index=index))

        for index in (0, 1, 3, 4, 255):
            response = result(node, request(index))
            assert not response.success and response.message, response
        output(cli(1), False, "MR", "LS")
        output(cli(-1), False, "0..255")
        no_send(peer)

        with cli(2) as process:
            wire = query_frame(peer)
            busy = result(node, request(2))
            assert not busy.success and "进行中" in busy.message, busy
            assert result(node, params.call_async(GetParameters.Request(
                names=["launch_motor"]))).values[0].bool_value
            noise = bytes.fromhex("A7 24 04 10")
            for frame in (
                wire, reply(wire, 65, noise, port=0), reply(wire, 0x51, noise), reply(wire, 0x141, noise),
                reply(wire, 65, noise, ide=1), reply(wire, 65, noise, msg_id=0x20),
                reply(wire, 65, noise[:3]), reply(wire, 65, noise.ljust(8, b"\0")),
                reply(wire, 65, bytes.fromhex("27 24 04 10")),
                reply(wire, 65, bytes.fromhex("A7 23 04 10")),
            ):
                peer.send_frame(frame)
            peer.send_frame(reply(wire, 65, bytes.fromhex("B7 24 0E 60")))
            output(process, True, "motor_index=2", "errorcode=7", "0x0E60", "A.E60",
                   "CAN 总线连接异常")
        no_send(peer)

        with cli(2) as process:
            wire = query_frame(peer)
            peer.send_frame(reply(wire, 65, bytes.fromhex("A7 23 00 00")))
            text = output(process, False, "超时")
            assert "无报警" not in text and "报警码=0x" not in text, text
        no_send(peer)
        peer.send_frame(reply(wire, 65, bytes.fromhex("A7 24 04 00")))
        with cli(2) as process:
            wire = query_frame(peer)
            process.terminate()
            process.wait(timeout=4)
            peer.send_frame(reply(wire, 65, bytes.fromhex("A7 24 04 10")))
        for payload, expected in (("A7 24 FE DC", ("0xFEDC", "未知")),
                                  ("A7 24 09 10", ("A.910", "警告：过载")),
                                  ("A7 24 00 00", ("errorcode=7", "未返回具体报警码")),
                                  ("A0 24 00 00", ("errorcode=0", "0x0000", "无报警"))):
            with cli(2) as process:
                wire = query_frame(peer)
                peer.send_frame(reply(wire, 65, bytes.fromhex(payload)))
                output(process, True, *expected)
        no_send(peer)
        assert (peer._peer, peer._session) == owner, "HPM UDP owner/session changed"

    with session(executable, mode=False, ls_id=0x7FF) as (_, _, peer, _):
        output(cli(2), False, "CAN ID", "未发送")
        no_send(peer)

    with session(executable, mode=False, connected=False) as (_, _, peer, _):
        output(cli(2), False, "发送失败")
        no_send(peer)


if __name__ == "__main__":
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="167")
    rclpy.init(args=[])
    try:
        run(sys.argv[1])
    finally:
        rclpy.shutdown()
