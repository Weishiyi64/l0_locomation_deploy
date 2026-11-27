#!/usr/bin/env python3
"""Real ROS2 node against a loopback-only mock HPM/P4. No physical device access."""
import collections
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zlib

os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Illuminance
from std_msgs.msg import UInt8, Bool, String, Empty
from std_srvs.srv import SetBool
from luvbot_hardware_interfaces.msg import (
    DisplayBatteryState, DisplayWifiState, RobotProvisionState, DisplayFault,
    DisplayFaultState, RobotUpgradeState, DisplayIdentityState,
    DisplayVoiceState, DisplayRequest, DisplayResult, DisplayDeviceState,
)

HEADER = struct.Struct("<4sBBHIIHH")


def encode(kind, session, sequence, payload=b""):
    raw = HEADER.pack(b"P4UP", 1, kind, 0, session, sequence, len(payload), 0) + payload
    return raw + struct.pack("<I", zlib.crc32(raw))


class Gateway:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.running = True
        self.peer = None
        self.lock = threading.Lock()
        self.connections = 0
        self.records = []
        self.up_acks = []
        self.drop_volume_once = True
        self.respond_to_greeting = True
        self.respond_to_hello = True
        self.inject_on_hello = False
        self.held = set()
        self.frames = {}
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, data):
        with self.lock:
            assert self.peer is not None
            # Split one valid stream at an arbitrary point; TCP may also coalesce it.
            self.peer.sendall(data[:7])
            self.peer.sendall(data[7:])

    def uplinks(self):
        for action, value in [(1, 1), (2, 2), (3, 35), (4, 1), (5, 4), (6, 0)]:
            raw = encode(0x11, 153, 100 + action, struct.pack("<IHH", action, action, value))
            if action == 3:
                broken = bytearray(raw)
                broken[-1] ^= 1
                self.send(bytes(broken))
            self.send(raw + raw)  # duplicate must not publish two user requests
        for key, value in [(0x8000, b"\x02\x2a\x00"),
                           (0x8001, b"\x01\x00\x00\x32"),
                           (0x8002, b"\x02\x17\x00"),
                           (0x8003, b"\x01\x02\x34\x12\x32")]:  # retired indicator report from older firmware
            raw = encode(0x13, 153, key, struct.pack("<IIH", 153, 5, key) + value)
            self.send(raw + raw)

    def run(self):
        try:
            while self.running:
                try:
                    peer, _ = self.listener.accept()
                except socket.timeout:
                    continue
                self.peer = peer
                self.connections += 1
                peer.settimeout(0.1)
                data = b""
                buffer = bytearray()
                injected = False
                try:
                    while b"\n" not in data and self.running:
                        byte = peer.recv(1)
                        if not byte:
                            break
                        data += byte
                    if not data or not self.running:
                        continue
                    assert data == b"ESPOTA1\n"
                    if self.respond_to_greeting:
                        peer.sendall(b"ESP_READY\n")
                    while self.running:
                        try:
                            data = peer.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        buffer.extend(data)
                        while len(buffer) >= 24:
                            assert buffer[:4] == b"P4UP"
                            _, version, kind, flags, session, seq, size, reserved = HEADER.unpack_from(buffer)
                            assert version == 1 and not flags and not reserved and size <= 1100
                            if len(buffer) < size + 24:
                                break
                            raw = bytes(buffer[:size + 24])
                            del buffer[:size + 24]
                            assert zlib.crc32(raw[:-4]) == struct.unpack("<I", raw[-4:])[0]
                            body = raw[20:-4]
                            if kind == 1:
                                if not self.respond_to_hello:
                                    continue
                                self.send(encode(0x70, session, seq, bytes(100)))
                                if not injected and self.inject_on_hello:
                                    injected = True
                                    self.uplinks()
                            elif kind == 0x12 and session == 153:
                                self.up_acks.append((seq, struct.unpack("<IHHI", body)))
                            else:
                                self.records.append((self.connections, kind, seq, body))
                                revision = struct.unpack_from("<I", body, 4)[0] if kind == 0x10 else 0
                                key = struct.unpack_from("<H", body, 8)[0] if kind == 0x10 else 0
                                self.frames.setdefault((session, seq), raw)
                                assert self.frames[(session, seq)] == raw, "retry bytes changed"
                                if key in self.held:
                                    continue
                                if key == 0x20 and self.drop_volume_once:
                                    self.drop_volume_once = False
                                    continue
                                self.send(encode(0x12, session, seq, struct.pack("<IHHI", 0, 7, 0, revision)))
                except (ConnectionError, OSError):
                    pass
                finally:
                    peer.close()
                    self.peer = None
        except Exception as exc:
            if self.running:
                self.error = exc

    def stop(self):
        self.running = False
        peer = self.peer
        if peer:
            try:
                peer.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(2)
        self.listener.close()


def main():
    gateway = Gateway()
    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen([sys.argv[1], "--ros-args", "-p", "launch_display:=true",
                                "-p", "launch_imu:=false", "-p", "board_ip:=127.0.0.1",
                                "-p", f"display_port:={gateway.port}"],
                               stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("display_runtime_check")
    subscriptions = []
    received = collections.defaultdict(list)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name, message in [("connection", String), ("panel", DisplayDeviceState)]:
        subscriptions.append(node.create_subscription(message, f"/display/{name}/state",
                             lambda m, n=name: received[n].append(m), qos))
    for name in ["bluetooth", "mode", "volume", "mute", "fault", "provision"]:
        suffix = "acknowledge" if name == "fault" else "request"
        subscriptions.append(node.create_subscription(DisplayRequest, f"/display/{name}/{suffix}",
                             lambda m, n=name: received[n + "_request"].append(m), 16))
    def terminal_log():
        return os.pread(log.fileno(), os.fstat(log.fileno()).st_size, 0).decode(errors="replace")

    def until(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if gateway.error:
                raise gateway.error
            if process.poll() is not None:
                raise AssertionError("display node exited")
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("timed out waiting for display test condition")

    def pause(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        until(lambda: received["connection"] and received["connection"][-1].data == "online")
        connection_start = len(received["connection"])
        disconnected = time.monotonic()
        gateway.respond_to_hello = False
        until(lambda: received["connection"][-1].data == "offline", timeout=3.2)
        print(f"Idle link offline after {time.monotonic() - disconnected:.2f} s")
        # Restore P4 just as another reconnect wait starts, after several failures.
        until(lambda: sum(m.data == "offline" for m in received["connection"][connection_start:]) >= 4,
              timeout=15)
        restored = time.monotonic()
        gateway.respond_to_hello = True
        until(lambda: received["connection"][-1].data == "online", timeout=1.8)
        print(f"Link online {time.monotonic() - restored:.2f} s after P4 resumed replies")
        # Do not publish volatile user events before DDS discovers the readers.
        # Replaying the same event after discovery cannot recover it: dedup must suppress it.
        request_topics = [f"/display/{n}/{'acknowledge' if n == 'fault' else 'request'}"
                          for n in ["bluetooth", "mode", "volume", "mute", "fault", "provision"]]
        until(lambda: all(node.count_publishers(t) for t in request_topics)
              and node.count_publishers("/display/panel/state"))
        pause(0.3)
        gateway.uplinks()
        gateway.inject_on_hello = True  # Reconnect replays intentionally exercise dedup.
        until(lambda: all(len(received[n + "_request"]) == 1 for n in ["bluetooth", "mode", "volume", "mute", "fault", "provision"]))
        until(lambda: len(received["panel"]) >= 3)
        assert received["volume_request"][0].value == 35
        assert all((m.boot_id, m.revision) == (153, 5) for m in received["panel"])
        assert sorted((m.target, m.status, m.error, m.brightness) for m in received["panel"]) == [
            (0, 2, 42, 0), (1, 1, 0, 50), (2, 2, 23, 0)]
        until(lambda: all((key, (0, 7, 0, 5)) in gateway.up_acks for key in (0x8000, 0x8001, 0x8002)))
        until(lambda: (0x8003, (0, 4, 0, 5)) in gateway.up_acks)  # UNSUPPORTED
        assert node.count_publishers("/display/indicator/state") == 0
        cases = [
            ("battery/state", DisplayBatteryState(percent=75, charge=1, flags=0), 1),
            ("bluetooth/state", UInt8(data=3), 0x24),
            ("wifi/state", DisplayWifiState(state=2, signal=3), 0x25),
            ("volume/state", UInt8(data=30), 0x20),
            ("mute/state", Bool(data=True), 0x21),
            ("volume_lock/state", Bool(data=True), 0x23),
            ("mode/state", UInt8(data=2), 0x22),
            ("dock/state", UInt8(data=1), 4),
            ("provision/state", RobotProvisionState(id=7, phase=4, phase_left_s=60, window_left_s=240), 0x30),
            ("fault/state", DisplayFaultState(faults=[DisplayFault(id=4, severity=3, order=1)]), 6),
            ("upgrade/state", RobotUpgradeState(id=9, target=3, phase=2, progress=45), 0x31),
            ("identity/state", DisplayIdentityState(bound=True, name="小伴 RUMI"), 8),
            ("interaction/event", UInt8(data=2), 0x34),
            ("task/succeeded", Empty(), 0x35),
            ("voice/state", DisplayVoiceState(stage=3, bins=[32, 80, 160, 64]), 0x11),
            ("homing/state", Bool(data=True), 0x32),
            ("night/state", Bool(data=True), 0x33),
            ("ambient_light/illuminance", Illuminance(illuminance=12.5), 0x36),
            ("operation/state", UInt8(data=3), 0x37),
        ]
        publishers = {topic: node.create_publisher(type(value), "/robot/" + topic,
                      qos_profile_sensor_data if isinstance(value, Illuminance) else 10)
                      for topic, value, _ in cases}
        until(lambda: all(p.get_subscription_count() for p in publishers.values()))
        for topic, value, _ in cases:
            publishers[topic].publish(value)
        expected_keys = {key for _, _, key in cases}
        until(lambda: expected_keys <= {struct.unpack_from("<H", body, 8)[0]
                                        for _, kind, _, body in gateway.records if kind == 0x10})
        until(lambda: sum(kind == 0x10 and body[8:] == b"\x20\x00\x1e"
                          for _, kind, _, body in gateway.records) == 2)
        expected = {
            0x36: struct.pack("<d", 12.5), 0x37: bytes([3]),
            1: bytes([75, 1, 0, 0]), 0x24: bytes([3]), 0x25: bytes([2, 3]),
            0x20: bytes([30]), 0x21: bytes([1]), 0x23: bytes([1]), 0x22: bytes([2]), 4: bytes([1]),
            0x30: struct.pack("<IBBHH", 7, 4, 0, 60, 240),
            6: bytes([1]) + struct.pack("<HBBI", 4, 3, 0, 1),
            0x31: struct.pack("<IBBBB", 9, 3, 2, 45, 0),
            8: bytes([1, len("小伴 RUMI".encode())]) + "小伴 RUMI".encode(),
            0x32: b"\x01", 0x33: b"\x01", 0x34: b"\x02", 0x35: b"\x01", 0x11: bytes([3, 4, 32, 80, 160, 64]),
        }
        for _, kind, _, body in gateway.records:
            if kind == 0x10:
                key = struct.unpack_from("<H", body, 8)[0]
                if key in expected:
                    assert body[10:] == expected[key], f"incorrect semantic fields: {key:#x}"

        inputs = {name for name, _ in node.get_subscriber_names_and_types_by_node("hpm_system", "/")}
        assert {"/robot/"+topic for topic, _, _ in cases} <= inputs
        assert not {"/display/indicator/set", "/display/expression/set"} & inputs
        assert len([t for t in inputs if t.startswith("/robot/") or t.startswith("/display/")]) == len(cases) + 6
        outputs = node.get_publisher_names_and_types_by_node("hpm_system", "/")
        assert all(not name.endswith("/feedback") for name, _ in outputs)
        assert {name for name, types in outputs if "luvbot_hardware_interfaces/msg/DisplayDeviceState" in types} == {
            "/display/panel/state"}

        before = len(gateway.records)
        publishers["volume/state"].publish(UInt8(data=30))
        publishers["volume/state"].publish(UInt8(data=101))
        publishers["ambient_light/illuminance"].publish(Illuminance(illuminance=-1.0))
        publishers["operation/state"].publish(UInt8(data=5))
        pause(0.35)
        assert len(gateway.records) == before, "unchanged/invalid value reached UART"
        publishers["battery/state"].publish(DisplayBatteryState(percent=48, charge=3, flags=4))
        until(lambda: any(kind == 0x10 and body[8:] == struct.pack("<HBBH", 1, 48, 3, 4)
                          for _, kind, _, body in gateway.records))
        before = len(gateway.records)
        publishers["task/succeeded"].publish(Empty())
        until(lambda: len(gateway.records) > before)
        result_pub = node.create_publisher(DisplayResult, "/display/volume/result", 10)
        until(lambda: result_pub.get_subscription_count() > 0)
        result_pub.publish(DisplayResult(session=153, event_id=3, result=0, revision=9))
        until(lambda: any(kind == 0x12 for _, kind, _, _ in gateway.records))
        assert any(struct.unpack("<IHHI", body) == (3, 0, 0, 9) for _, kind, _, body in gateway.records if kind == 0x12)

        service = node.create_client(SetBool, "/display/upgrade/maintenance")
        assert service.wait_for_service(timeout_sec=3)
        # Enter maintenance while a command is pending; release must interrupt it.
        gateway.held.add(0x20)
        publishers["volume/state"].publish(UInt8(data=40))
        until(lambda: any(kind == 0x10 and body[10:] == b"\x28" and struct.unpack_from("<H", body, 8)[0] == 0x20 for _, kind, _, body in gateway.records))
        future = service.call_async(SetBool.Request(data=True))
        until(future.done)
        assert future.result().success
        until(lambda: gateway.peer is None)
        with socket.create_connection(("127.0.0.1", gateway.port), timeout=2) as ota:
            ota.sendall(b"ESPOTA1\n")
            assert ota.makefile("rb").readline() == b"ESP_READY\n"
        until(lambda: gateway.peer is None)
        connections = gateway.connections
        assert "P4 ACK missing:" not in terminal_log(), "maintenance interruption logged as a missed ACK"
        publishers["volume/state"].publish(UInt8(data=45))
        pause(0.5)
        assert gateway.connections == connections
        gateway.held.clear()
        future = service.call_async(SetBool.Request(data=False))
        until(future.done)
        assert future.result().success
        until(lambda: gateway.connections > connections)
        until(lambda: any(conn > connections and kind == 0x10 and struct.unpack_from("<H", body, 8)[0] == 0x20 and body[10:] == b"\x2d" for conn, kind, _, body in gateway.records))
        assert not any(conn > connections and kind == 0x10 and struct.unpack_from("<H", body, 8)[0] in (0x34, 0x35) for conn, kind, _, body in gateway.records), "one-shot event replayed"
        # The HPM handshake still completes while the P4 stops responding.
        connection_start = len(received["connection"])
        gateway.held.add(0x20)
        publishers["volume/state"].publish(UInt8(data=55))
        until(lambda: any(kind == 0x10 and body[8:] == b"\x20\x00\x37"
                          for _, kind, _, body in gateway.records))
        lost_connection = gateway.connections
        disconnected = time.monotonic()
        gateway.respond_to_hello = False
        until(lambda: received["connection"][-1].data == "offline", timeout=3.2)
        print(f"Link with a pending business ACK offline after {time.monotonic() - disconnected:.2f} s")
        until(lambda: any("P4 ACK missing: function=volume " in line and "P4 HELLO response timeout" in line
                          for line in terminal_log().splitlines()))
        until(lambda: sum(m.data == "offline" for m in received["connection"][connection_start:]) >= 3)
        gateway.held.clear()
        gateway.respond_to_hello = True
        until(lambda: any(conn > lost_connection and kind == 0x10 and body[8:] == b"\x20\x00\x37"
                          for conn, kind, _, body in gateway.records))
        # HELLO still responds, but the display command never receives its ACK.
        gateway.held.add(0x20)
        publishers["volume/state"].publish(UInt8(data=56))
        until(lambda: any("P4 ACK missing: function=volume " in line and "ACK retry limit reached" in line
                          for line in terminal_log().splitlines()), timeout=12)
        assert all(len(received[n + "_request"]) == 1 for n in ["bluetooth", "mode", "volume", "mute", "fault", "provision"]), "upstream request replayed across reconnect"
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        gateway.stop()
        node.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        text = log.read()
        log.close()
        if sys.exc_info()[0]:
            print(text, file=sys.stderr)
        else:
            assert process.returncode == 0, f"node shutdown failed: {process.returncode}\n{text}"
            outages = text.split("display link offline: P4 HELLO response timeout")
            assert len(outages) == 3, text
            assert all("display connection: online" in segment for segment in outages[1:]), text
            assert text.count("display link offline: ACK retry limit reached") == 1, text
            assert not any(f"display connection: {state}" in text
                           for state in ("offline", "connecting", "gateway_ready")), text
    print(f"PASS: {len(cases)} robot inputs, 6 request topics, 3 device topics, ACK logs, retries/dedup, results and maintenance/reconnect")


if __name__ == "__main__":
    main()
