#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Real ROS service/topic tests with a local UDP board simulator, never hardware.

source install/setup.bash
python3 src/hpm_system/test/test_motor_id_calibration_runtime.py \
  install/hpm_system/lib/hpm_system/hpm_system_node
"""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
import yaml

from test_motor_set_zero_runtime import reply
from udp_test_peer import UdpPeer

os.environ["ROS_DOMAIN_ID"] = "164"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorTarget
from luvbot_hardware_interfaces.srv import MotorCalibrateId as Id, MotorSetZero

UID = bytes.fromhex("0102030405060708")
SCRIPT = Path(__file__).resolve().parents[3] / "scripts/motor/motor_id_calibration.py"


@contextmanager
def session(executable, mode, connected=True, *, ls_id=65):
    with tempfile.TemporaryDirectory(prefix="hpm-id-check-") as tmp:
        cfg = Path(tmp) / "motors.yaml"
        cfg.write_text(
            "motors:\n"
            "  rs: {index: 0, motor_type: RS, bus_index: 0, motor_id: 2}\n"
            "  mr: {index: 1, motor_type: MR, model: PA43-10-40, bus_index: 3, motor_id: 2}\n"
            f"  ls: {{index: 2, motor_type: LS, bus_index: 1, motor_id: {ls_id}}}\n"
            "  dm: {index: 3, motor_type: DM, bus_index: 2, motor_id: 1}\n"
            "  enc: {index: 4, motor_type: ENC, bus_index: 5, motor_id: 1}\n"
            "  mr2: {index: 5, motor_type: MR, model: PA43-10-20.25, bus_index: 3, motor_id: 3}\n"
            "  mr35: {index: 6, motor_type: MR, model: PA35-10-8, bus_index: 3, motor_id: 4}\n"
        )
        protocol_config = yaml.safe_load(
            (Path(__file__).resolve().parents[3] / "src/hpm_motor/config/motors.yaml").read_text())
        protocol_config["motors"] = yaml.safe_load(cfg.read_text())["motors"]
        cfg.write_text(yaml.safe_dump(protocol_config))
        with UdpPeer() as listener, open(Path(tmp) / "node.log", "w+") as log:
            proc = subprocess.Popen([
                executable, "--ros-args", "-p", "board_ip:=127.0.0.1",
                "-p", f"port:={listener.port}",
                "-p", f"enable_udp:={'true' if connected else 'false'}",
                "-p", "launch_motor:=true",
                "-p", f"motor_log_dir:={tmp}/motor_log",
                "-p", "enable_tactile_uart:=false", "-p", "launch_tactile:=false",
                "-p", "launch_imu:=false", "-p", f"motor_config_file:={cfg}",
                "-p", f"motor_id_calibration_mode:={'true' if mode else 'false'}",
                "-p", "motor_id_calibration_timeout_ms:=250"], stdout=log, stderr=log)
            conn = node = None
            try:
                if connected:
                    conn = listener
                    conn.wait_ready()
                    conn.settimeout(3)
                else:
                    conn = listener
                node = rclpy.create_node("id_runtime_test")
                client = node.create_client(Id, "/actuator/motor/calibrate_id")
                assert client.wait_for_service(timeout_sec=5)
                # set_zero must read the mode before it can safely enable motors.
                parameters = node.create_client(GetParameters, "/hpm_system/get_parameters")
                assert parameters.wait_for_service(timeout_sec=5), "motor mode parameter service unavailable"
                values = result(node, parameters.call_async(GetParameters.Request(
                    names=["motor_id_calibration_mode", "launch_motor"]))).values
                assert [v.type for v in values] == [ParameterType.PARAMETER_BOOL] * 2
                assert [v.bool_value for v in values] == [mode, True]
                node.destroy_client(parameters)
                yield node, client, conn, cfg
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if conn:
                    conn.close()
                if node:
                    node.destroy_node()
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


def receive(conn):
    wire = b""
    while len(wire) < 30:
        chunk = conn.recv(30 - len(wire))
        assert chunk, "TCP closed"
        wire += chunk
    return wire


def no_send(conn, seconds=0.05):
    old = conn.gettimeout()
    conn.settimeout(seconds)
    try:
        assert not conn.recv(30), "Unexpected CAN command"
    except socket.timeout:
        pass
    finally:
        conn.settimeout(old)


def request(client, index, current, operation=Id.Request.QUERY):
    return client.call_async(Id.Request(operation=operation, motor_index=index, current_id=current))


def result(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=4)
    assert future.done(), "Service did not return"
    value = future.result()
    assert value is not None
    return value


def can_id(wire):
    return int.from_bytes(wire[15:19], "big")


def mr_reply(wire, motor_id, address=36, value=None, **changes):
    value = motor_id if value is None else value
    return reply(wire, 0x600 + motor_id,
                 bytes([motor_id, address]) + value.to_bytes(4, "little") + b"\x00\xff",
                 msg_id=0x1F, **changes)


def run(executable):
    rclpy.init()
    try:
        with session(executable, False, connected=False) as (node, client, _, __):
            assert result(node, request(client, 1, 1)).result == Id.Response.SEND_FAILED
            assert result(node, request(client, 1, 1, Id.Request.CONFIGURE)).result == Id.Response.REJECTED
            print("PASS: disconnected send failure and normal-mode write protection")

        with session(executable, False) as (node, client, conn, _):
            for index, current, operation, expected in (
                (255, 1, 0, Id.Response.REJECTED), (0, 1, 99, Id.Response.REJECTED),
                (3, 1, 0, Id.Response.UNSUPPORTED), (4, 1, 0, Id.Response.UNSUPPORTED),
                (1, 0, 0, Id.Response.REJECTED), (1, 256, 0, Id.Response.REJECTED),
                (2, 2048, 0, Id.Response.REJECTED), (0, 127, 1, Id.Response.REJECTED),
            ):
                assert result(node, request(client, index, current, operation)).result == expected
            no_send(conn)
            pending = request(client, 0, 2)
            wire = receive(conn)
            assert can_id(wire) == 2 and wire[14] == 1 and wire[20:28] == bytes(8)
            assert result(node, request(client, 1, 2)).result == Id.Response.REJECTED
            # Normal-mode queries do not block existing motor topics.
            pub = node.create_publisher(MotorEnableCommand, "/actuator/motor/enable", 10)
            until = time.monotonic() + 4
            while not pub.get_subscription_count():
                assert time.monotonic() < until
                rclpy.spin_once(node, timeout_sec=0.02)
            pub.publish(MotorEnableCommand(motor_types=[1], enabled=[True]))
            enabled = receive(conn)
            assert can_id(enabled) == 0x102 and enabled[27] == 0xFC
            conn.sendall(reply(wire, 0x2FE, UID))
            assert result(node, pending).result == Id.Response.CONFIRMED
            print("PASS: validation, overlap rejection, RS query and topics during query")

        with session(executable, True) as (node, client, conn, cfg):
            command_pub = node.create_publisher(MotorCommand, "/actuator/motor/command", 10)
            enable_pub = node.create_publisher(MotorEnableCommand, "/actuator/motor/enable", 10)
            zero = node.create_client(MotorSetZero, "/actuator/motor/set_zero")
            assert zero.wait_for_service(timeout_sec=4)
            until = time.monotonic() + 4
            while not command_pub.get_subscription_count() or not enable_pub.get_subscription_count():
                assert time.monotonic() < until
                rclpy.spin_once(node, timeout_sec=0.02)
            target = MotorTarget()
            target.route.motor_type = 1
            target.kp = 1.0
            command_pub.publish(MotorCommand(control_mode=MotorCommand.CONTROL_MODE_MIT, commands=[target]))
            for enabled in (True, False):
                enable_pub.publish(MotorEnableCommand(motor_types=[1], enabled=[enabled]))
            assert list(result(node, zero.call_async(MotorSetZero.Request(motor_types=[1]))).results) == [1]
            no_send(conn, 0.15)

            # Only mismatching feedback: none of these can confirm an MR ID query.
            pending = request(client, 1, 1)
            wire = receive(conn)
            assert can_id(wire) == 0x601 and wire[20:28].hex() == "6724000000000476"
            conn.sendall(wire)  # CAN echo
            conn.sendall(mr_reply(wire, 1, port=7))
            conn.sendall(mr_reply(wire, 1, ide=1))
            conn.sendall(mr_reply(wire, 1, dlc=7))
            conn.sendall(mr_reply(wire, 2))
            conn.sendall(mr_reply(wire, 1, address=0, value=0))
            conn.sendall(reply(wire, 0x301, bytes.fromhex("8000800000280001")))  # MIT
            conn.sendall(reply(wire, 0x601, bytes.fromhex("0124010000000076")))  # wrong trailer
            assert result(node, pending).result == Id.Response.TIMEOUT
            conn.sendall(mr_reply(wire, 1))  # late old-ID query response
            pending = request(client, 1, 2)
            new_query = receive(conn)
            conn.sendall(mr_reply(wire, 1))
            assert result(node, pending).result == Id.Response.TIMEOUT
            no_send(conn)
            print("PASS: mode blocks motion/enable/zero; wrong/echo/MIT/late frames do not confirm")

            # RS fixed vector, MCU identity continuity and new ID verification.
            pending = request(client, 0, 127, Id.Request.CONFIGURE)
            wire = receive(conn)
            assert can_id(wire) == 0x7F
            conn.sendall(reply(wire, 0x7FFE, UID))
            wire = receive(conn)
            assert can_id(wire) == 0x0702007F and wire[20:28] == bytes(8)
            conn.sendall(reply(wire, 0x02FE, UID))
            wire = receive(conn)
            assert can_id(wire) == 2
            conn.sendall(reply(wire, 0x02FE, UID))
            value = result(node, pending)
            assert value.result == 0 and value.power_cycle_required
            no_send(conn)  # no zero_sta or save-register writes for RS
            pending = request(client, 0, 2)
            wire = receive(conn)
            conn.sendall(reply(wire, 0x02FE, UID))
            conn.sendall(reply(wire, 0x02FE, bytes.fromhex("1122334455667788")))
            assert result(node, pending).result == Id.Response.DEVICE_FAILED

            # MR must wait for write reply before query/save. Save uses the NEW ID.
            pending = request(client, 1, 1, Id.Request.CONFIGURE)
            wire = receive(conn)
            conn.sendall(mr_reply(wire, 1))
            wire = receive(conn)
            assert can_id(wire) == 0x601 and wire[20:28].hex() == "6724020000001576"
            no_send(conn)
            conn.sendall(wire)
            conn.sendall(mr_reply(wire, 1))  # wrong ID, cannot acknowledge write
            no_send(conn)
            conn.sendall(mr_reply(wire, 2))
            wire = receive(conn)
            assert can_id(wire) == 0x602 and wire[20:28].hex() == "6724000000000476"
            conn.sendall(mr_reply(wire, 2))
            wire = receive(conn)
            assert can_id(wire) == 0x602 and wire[20:28].hex() == "6700000000001576"
            conn.sendall(mr_reply(wire, 2, address=0, value=0))
            value = result(node, pending)
            assert value.result == 0 and value.model == "PA43-10-40" and value.power_cycle_required

            # Second MR model uses the same protocol. Already-target ID is never rewritten.
            pending = request(client, 5, 3, Id.Request.CONFIGURE)
            wire = receive(conn)
            conn.sendall(mr_reply(wire, 3))
            value = result(node, pending)
            assert value.result == 0 and value.model == "PA43-10-20.25" and value.power_cycle_required
            no_send(conn)

            pending = request(client, 6, 4, Id.Request.CONFIGURE)
            wire = receive(conn)
            conn.sendall(mr_reply(wire, 4))
            value = result(node, pending)
            assert value.result == 0 and value.model == "PA35-10-8" and value.power_cycle_required
            no_send(conn)

            # LS broadcasts collect all IDs and refuse writes with multiple motors.
            pending = request(client, 2, 1, Id.Request.CONFIGURE)
            wire = receive(conn)
            assert can_id(wire) == 0x7FF and wire[19] == 4 and wire[20:24].hex() == "ffff0082"
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010001"), msg_id=0x1F))
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010042"), msg_id=0x10))
            assert result(node, pending).result == Id.Response.REJECTED
            no_send(conn)
            pending = request(client, 2, 1, Id.Request.CONFIGURE)
            wire = receive(conn)
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010001")))
            wire = receive(conn)
            assert wire[19] == 6 and wire[20:26].hex() == "000100040041"
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("00410104")))
            value = result(node, pending)
            assert value.result == 0 and value.stage == "await_power_cycle" and value.power_cycle_required
            assert "query_new" not in value.message
            no_send(conn)  # no premature new-ID query or CAN_Timeout writes
            # Before power cycling, the LS motor may still report the old active ID.
            pending = request(client, 2, 65)
            wire = receive(conn)
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010001")))
            assert list(result(node, pending).detected_ids) == [1]
            # After a simulated power cycle, a fresh broadcast discovers the new ID.
            pending = request(client, 2, 65)
            wire = receive(conn)
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010041")))
            assert list(result(node, pending).detected_ids) == [65]
            no_send(conn)
            print("PASS: RS/MR/LS vectors, UID, both MR models, separate MR save, LS inventory")

            # Write failure and timeout stop immediately, without retries or saves.
            pending = request(client, 2, 1, Id.Request.CONFIGURE)
            wire = receive(conn)
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("ffff010001")))
            wire = receive(conn)
            conn.sendall(reply(wire, 0x7FF, bytes.fromhex("00410100")))
            assert result(node, pending).result == Id.Response.DEVICE_FAILED
            no_send(conn)
            for fail_at in ("query_current", "write_id", "query_new", "save_id"):
                pending = request(client, 1, 1, Id.Request.CONFIGURE)
                wire = receive(conn)
                if fail_at != "query_current":
                    conn.sendall(mr_reply(wire, 1))
                    wire = receive(conn)
                    if fail_at != "write_id":
                        conn.sendall(mr_reply(wire, 2))
                        wire = receive(conn)
                        if fail_at != "query_new":
                            conn.sendall(mr_reply(wire, 2))
                            wire = receive(conn)
                value = result(node, pending)
                assert value.result == Id.Response.TIMEOUT and value.stage == fail_at, value
                no_send(conn)
            print("PASS: device failure, every stage timeout, no automatic retry/save")

            # Actual CLI service discovery/invocation (no hardware).
            cli = subprocess.Popen([
                "ros2", "service", "call", "/actuator/motor/calibrate_id",
                "luvbot_hardware_interfaces/srv/MotorCalibrateId", "{operation: 0, motor_index: 5, current_id: 3}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                wire = receive(conn)
                conn.sendall(mr_reply(wire, 3))
                output, error = cli.communicate(timeout=8)
                assert cli.returncode == 0 and "result=0" in output, (output, error)
            finally:
                if cli.poll() is None:
                    cli.kill()
                    cli.communicate()

            # Actual interactive script, including operator confirmation and three post-cycle queries.
            script = subprocess.Popen([
                sys.executable, str(SCRIPT), "--config", str(cfg), "--motor-index", "1"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                script.stdin.write("yes\nyes\n")
                script.stdin.flush()
                # QUERY preflight + CONFIGURE query-current, write, query-new, save + 3 final QUERYs.
                for stage in ("old", "old", "write", "new", "save", "new", "new", "new"):
                    wire = receive(conn)
                    if stage == "save":
                        assert can_id(wire) == 0x602
                        conn.sendall(mr_reply(wire, 2, address=0, value=0))
                    else:
                        conn.sendall(mr_reply(wire, 1 if stage == "old" else 2))
                output, error = script.communicate(timeout=8)
                assert script.returncode == 0 and "断电重启后 ID 复检" in output, (output, error)
                print("PASS: actual ros2 CLI and interactive script with three post-cycle queries")
            finally:
                if script.poll() is None:
                    script.kill()
                    script.communicate()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    run(sys.argv[1])
