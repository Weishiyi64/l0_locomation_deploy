#!/usr/bin/env python3
"""Offline socket/serial peers: retries, corrupt frames, commit loss and rollback."""
import argparse
import contextlib
import hashlib
import io
import queue
import socket
import struct
import sys
import threading
import time
import types
import zlib
from pathlib import Path
from unittest.mock import patch
import p4_ota as p4
import hpm_serial_ota as hpm

def image():
    body = bytearray(1400)
    struct.pack_into("<I", body, 0, 0xabcd5432)
    body[16:20] = b"test"
    raw = bytearray(24)
    raw[0], raw[1], raw[23] = 0xe9, 1, 1
    struct.pack_into("<H", raw, 12, 0x12)
    struct.pack_into("<HH", raw, 15, 100, 199)
    raw += struct.pack("<II", 0x40030020, len(body)) + body
    checksum = 0xef
    for byte in body:
        checksum ^= byte
    raw += bytes((len(raw) | 15) - len(raw)) + bytes([checksum])
    return bytes(raw) + hashlib.sha256(raw).digest()

class Device:
    def __init__(self, **faults):
        self.faults = faults
        self.state, self.offset, self.running, self.target = 0, 0, 0x20000, 0x1d0000
        self.written, self.last = bytearray(), None
        self.digest = bytes(32)
        self.commands, self.writes, self.duplicates = [], 0, 0
        self.begin_frames, self.begin_applied = [], 0
        self.stopped, self.errors = threading.Event(), queue.Queue()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0)); self.listener.listen()
        self.listener.settimeout(0.1)
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def info(self, status=0, query=False):
        prefix = struct.pack("<HHII", status, self.state, self.offset, 0)
        info = struct.pack("<IHHIIIB3x32s32s", p4.BOARD_ID, p4.CHIP_ID,
                           self.faults.get("revision", 103), 0x1a9000, self.running,
                           self.target, 2, b"test", self.digest if query else bytes(32))
        return prefix + info

    def serve(self):
        try:
            while not self.stopped.is_set():
                try:
                    peer, _ = self.listener.accept()
                except socket.timeout:
                    continue
                with peer:
                    peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    peer.settimeout(0.1)
                    magic = bytearray()
                    while len(magic) < 8:
                        part = peer.recv(8 - len(magic))
                        if not part:
                            break
                        magic.extend(part)
                    assert magic == b"ESPOTA1\n"
                    peer.sendall(b"ESP_RE"); peer.sendall(b"ADY\n")
                    channel = p4.Channel(peer, True)
                    while not self.stopped.is_set():
                        try:
                            kind, session, sequence, body = channel.frame(time.monotonic() + 0.1)
                        except TimeoutError:
                            continue
                        except (ConnectionError, ConnectionResetError):
                            break
                        self.commands.append(kind)
                        status = 0
                        if kind == p4.BEGIN:
                            self.begin_frames.append((session, sequence, body))
                            attempt = len(self.begin_frames)
                            length, _, board, abi, digest, image_type = struct.unpack("<IIII32sB3x", body)
                            assert board == p4.BOARD_ID and not abi and not image_type
                            if self.faults.get("silent_begin") or (self.faults.get("reboot_before_begin") and attempt == 1):
                                continue
                            if self.faults.get("reboot_before_begin") and attempt == 2:
                                status = 13  # P4 protocol is back, but boot_valid is still false.
                            elif self.state == 2:
                                assert self.begin_frames[-1] == self.begin_frames[0]
                            else:
                                self.begin_applied += 1
                                self.expected, self.expected_digest = length, digest
                                self.state, self.offset = 2, 0
                                if self.faults.get("drop_begin_ack"):
                                    break
                        elif kind == p4.DATA:
                            if self.faults.get("disconnect_during_data"):
                                break
                            offset = struct.unpack_from("<I", body)[0]
                            begin_session, begin_sequence, _ = self.begin_frames[-1]
                            assert session == begin_session and sequence == begin_sequence + 1 + offset // 1024
                            current = (sequence, offset, body[4:])
                            if current == self.last:
                                self.duplicates += 1
                            else:
                                assert offset == self.offset
                                self.written.extend(body[4:])
                                self.offset += len(body) - 4
                                self.last = current
                                self.writes += 1
                            if self.faults.get("drop_ack") and self.writes == 1 and not self.duplicates:
                                continue
                        elif kind == p4.END:
                            assert len(self.written) == self.expected
                            assert hashlib.sha256(self.written).digest() == self.expected_digest
                            if self.faults.get("bad_hash"):
                                status = 7
                            else:
                                self.state = 4
                        elif kind == p4.ABORT:
                            self.state = 0
                        elif kind == p4.COMMIT:
                            assert self.state == 4
                            self.state = 5
                            if not self.faults.get("rollback"):
                                self.running, self.digest = self.target, self.expected_digest
                            if self.faults.get("lost_commit"):
                                break
                        if (kind == p4.QUERY and self.state == 5 and
                                self.commands.count(p4.QUERY) <= self.faults.get("drop_boot_query", 0)):
                            continue
                        raw = p4.encode(p4.NACK if status else p4.ACK, session, sequence,
                                        self.info(status, kind == p4.QUERY))
                        if self.faults.get("corrupt_reply") and kind == p4.HELLO:
                            broken = bytearray(raw); broken[-1] ^= 1
                            peer.sendall(b"garbage" + broken)
                        # UART DMA sends alignment zeros outside frames.
                        peer.sendall(raw[:7]); peer.sendall(raw[7:] + bytes(4))
        except (BrokenPipeError, ConnectionResetError):
            if not self.stopped.is_set():
                self.errors.put(RuntimeError("unexpected disconnected test peer"))
        except BaseException as exc:
            self.errors.put(exc)

    def run(self, expected=None, boot_timeout=0.4):
        self.thread.start()
        args = argparse.Namespace(serial=None, host="127.0.0.1",
            port=self.listener.getsockname()[1], baud=1000000, timeout=0.04,
            retries=2, boot_timeout=boot_timeout)
        try:
            try:
                assert p4.upgrade(args, image()) == 0
                assert expected is None
            except (OSError, RuntimeError, ValueError) as exc:
                assert expected and expected in str(exc), str(exc)
        finally:
            self.stopped.set(); self.thread.join(2); self.listener.close()
        assert not self.thread.is_alive()
        if not self.errors.empty():
            raise self.errors.get()

def hpm_package(device=0x5151):
    body = bytes(range(256)) * 5
    header = bytearray(4096)
    struct.pack_into("<IIIII4BI", header, 0, 0xbeaf5aa5, 0xa5a55a5a,
                     device, len(body), 1, 0, 1, 1, 4, 0)
    header[28:60] = hashlib.sha256(body).digest()
    return bytes(header) + body

def check_serial_sender(drop_final_ack=False):
    package = hpm_package()
    class Port:
        def __init__(self, *args, **kwargs):
            self.pending, self.received = bytearray(), bytearray()
            self.last = None
            self.dropped = False
        def __enter__(self): return self
        def __exit__(self, *args): assert bytes(self.received) == package
        def reset_input_buffer(self): self.pending.clear()
        def flush(self): pass
        def read(self, size):
            result = bytes(self.pending[:size]); del self.pending[:size]
            return result
        def write(self, raw):
            if raw == hpm.MAGIC:
                self.pending += b"\x00old telemetryOTA_READY\n"
                return len(raw)
            magic, version, kind, size, sequence, offset, crc = struct.unpack_from("<4sBBHIII", raw)
            assert magic == b"OTAF" and version == kind == 1 and size == len(raw) - 20
            assert crc == zlib.crc32(raw[4:16] + raw[20:]) & 0xffffffff
            if raw != self.last:
                assert offset == len(self.received)
                self.received += raw[20:]; self.last = raw
            if not self.dropped:
                self.dropped = True
                return len(raw)
            if offset:
                self.pending += f"OTA_ACK:{offset}\n".encode()
            if not drop_final_ack or len(self.received) != len(package):
                self.pending += f"OTA_ACK:{offset + size}\n".encode()
            if len(self.received) == len(package):
                self.pending += b"OTA_OK\n"
            return len(raw)
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "update_sign.bin"; path.write_bytes(package)
        saved, old = sys.argv, sys.modules.get("serial")
        sys.modules["serial"] = types.SimpleNamespace(Serial=Port)
        sys.argv = ["ota_send.py", "/dev/mock", str(path), "--timeout", "0.01"]
        try:
            assert hpm.main() == 0
        finally:
            sys.argv = saved
            if old is None: del sys.modules["serial"]
            else: sys.modules["serial"] = old
    assert hpm.validate_package(hpm_package(0x6750), 0x6750, 0x100000)
    for bad in (hpm_package(0x6750), hpm_package()[:-1]):
        try: hpm.validate_package(bad)
        except ValueError: pass
        else: raise AssertionError("accepted invalid HPM package")

def check_query_timeout():
    class SilentChannel:
        def __init__(self):
            self.frames = []

        def write(self, frame):
            self.frames.append(frame)

        def frame(self, deadline):
            assert 0 < deadline - time.monotonic() <= 0.05
            raise TimeoutError("silent UART")

    channel = SilentChannel()
    args = argparse.Namespace(timeout=0.05, retries=1)
    saved = p4.connect
    p4.connect = lambda args: contextlib.nullcontext(channel)
    output = io.StringIO()
    try:
        with contextlib.redirect_stdout(output):
            try:
                p4.query(args)
            except TimeoutError as exc:
                assert "未收到匹配的 P4 应答" in str(exc)
            else:
                raise AssertionError("accepted missing P4 response")
    finally:
        p4.connect = saved
    assert len(channel.frames) == 2 and channel.frames[0] == channel.frames[1]
    assert channel.frames[0][5] == p4.QUERY
    assert "每次等待 0.05 秒" in output.getvalue()
    assert "第 1/2 次" in output.getvalue() and "第 2/2 次" in output.getvalue()


def check_boot_wait_and_progress():
    notice = "继续等待 P4 重启和串口初始化"
    device = Device(drop_ack=True, drop_boot_query=1)
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        device.run(boot_timeout=7)
    log = output.getvalue()
    assert notice not in log
    assert "命令=0x35" not in log
    assert "BOOT_CONFIRMED" in log
    assert device.commands.count(p4.QUERY) == 2
    assert log.count("\r发送 [") == 2
    size = len(image())
    assert f"发送 [{'=' * 30}] 100.00% {size}/{size} bytes" in log
    assert device.writes == 2 and device.duplicates >= 1

    device = Device(drop_boot_query=3)
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        device.run(boot_timeout=18)
    log = output.getvalue()
    assert log.count(notice) == 1 and "命令=0x35" not in log
    assert "BOOT_CONFIRMED" in log and device.commands.count(p4.QUERY) == 4
    print("P4 reboot wait and acknowledged progress checks passed")


def check_begin_recovery():
    for faults, attempts in (({"reboot_before_begin": True}, 3), ({"drop_begin_ack": True}, 2)):
        device = Device(**faults)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            device.run()
        assert len(device.begin_frames) == attempts and len(set(device.begin_frames)) == 1
        assert device.begin_applied == 1 and device.writes == 2
        assert device.commands.count(p4.COMMIT) == 1
        assert output.getvalue().count("暂未收到 P4 BEGIN 确认") == 1

    with patch.object(p4, "BEGIN_WAIT_SECONDS", 0.2):
        device = Device(silent_begin=True)
        device.run("P4 BEGIN 未获确认")
    assert p4.DATA not in device.commands and p4.COMMIT not in device.commands

    device = Device(disconnect_during_data=True)
    device.run("网关连接已关闭")
    assert device.commands.count(p4.BEGIN) == 1 and p4.COMMIT not in device.commands
    print("P4 BEGIN reboot, lost ACK, timeout and no DATA restart checks passed")


if __name__ == "__main__":
    check_begin_recovery()
    check_boot_wait_and_progress()
    check_query_timeout()
    assert p4.validate_image(image())["version"] == "test"
    broken = bytearray(image()); broken[400] ^= 1
    try: p4.validate_image(broken)
    except ValueError: pass
    else: raise AssertionError("accepted corrupt ESP image")
    device = Device(drop_ack=True, corrupt_reply=True, lost_commit=True)
    device.run()
    assert device.writes == 2 and device.duplicates >= 1
    device = Device(bad_hash=True)
    device.run("HASH")
    assert p4.ABORT in device.commands and p4.COMMIT not in device.commands
    device = Device(revision=301)
    device.run("不兼容")
    assert p4.BEGIN not in device.commands
    Device(rollback=True).run("未确认")
    check_serial_sender()
    check_serial_sender(drop_final_ack=True)
    print("P4 socket and HPM serial fault checks passed")
