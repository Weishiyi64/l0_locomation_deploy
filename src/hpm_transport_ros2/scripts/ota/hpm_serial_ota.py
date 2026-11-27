#!/usr/bin/env python3
"""Send an HPM5151 OTA package through the application UART with stop-and-wait ACKs."""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
import time
import zlib
from pathlib import Path


MAGIC = b"HPMOTA1\n"
FRAME_MAGIC = b"OTAF"
HEADER_SIZE = 0x1000
SLOT_SIZE = 0x40000
DEVICE_ID = 0x5151
FRAME_DATA = 1
DEFAULT_PACKAGE = Path(__file__).resolve().parents[5] / "hpm5100/build/ota/pack/update_sign.bin"


def read_line(port, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = bytearray()
    while time.monotonic() < deadline and len(data) < 128:
        byte = port.read(1)
        if byte:
            data += byte
            if byte == b"\n":
                return bytes(data)
    raise TimeoutError("等待设备响应超时")


def read_token(port, token: bytes, timeout: float) -> bytes:
    """Find a complete control token even when old binary telemetry precedes it."""
    deadline = time.monotonic() + timeout
    window = bytearray()
    while time.monotonic() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        window += byte
        if len(window) > len(token):
            del window[:-len(token)]
        if bytes(window) == token:
            return token
    raise TimeoutError("等待设备响应超时")


def validate_package(package: bytes, device_id: int = DEVICE_ID, slot_size: int = SLOT_SIZE) -> tuple[int, int]:
    if len(package) <= HEADER_SIZE or len(package) > slot_size:
        raise ValueError(f"升级包大小无效: {len(package)}")
    fields = struct.unpack_from("<IIIII4BI", package, 0)
    magic, touchid, device, body_len, version, image_type, hash_enable, pwr_hash, hash_type, _ = fields
    if magic != 0xBEAF5AA5 or touchid != 0xA5A55A5A:
        raise ValueError("升级包 Header magic/touchid 错误")
    if device != device_id:
        raise ValueError(f"升级包设备错误: 0x{device:04x}")
    if image_type != 0 or hash_enable != 1 or pwr_hash != 1 or hash_type != 4:
        raise ValueError("升级包类型或 SHA256 配置错误")
    if version == 0 or body_len != len(package) - HEADER_SIZE:
        raise ValueError("升级包版本或正文长度错误")
    if hashlib.sha256(package[HEADER_SIZE:]).digest() != package[28:60]:
        raise ValueError("升级包 SHA256 错误")
    return body_len, version


def make_frame(sequence: int, offset: int, payload: bytes) -> bytes:
    prefix = struct.pack("<4sBBHII", FRAME_MAGIC, 1, FRAME_DATA, len(payload), sequence, offset)
    crc = zlib.crc32(prefix[4:] + payload) & 0xFFFFFFFF
    return prefix + struct.pack("<I", crc) + payload


def self_test() -> int:
    if zlib.crc32(b"123456789") & 0xFFFFFFFF != 0xCBF43926:
        raise AssertionError("CRC32/IEEE self-test failed")
    frame = make_frame(0, 0, b"abc")
    if len(frame) != 23 or struct.unpack_from("<I", frame, 16)[0] != zlib.crc32(frame[4:16] + b"abc") & 0xFFFFFFFF:
        raise AssertionError("frame self-test failed")
    print("ota_send self-test: OK")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="通过应用串口发送 HPM5151 OTA 包")
    parser.add_argument("port", nargs="?", help="例如 /dev/ttyUSB0")
    parser.add_argument("package", nargs="?", type=Path, default=DEFAULT_PACKAGE)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--chunk", type=int, default=1024)
    parser.add_argument("--timeout", type=float, default=5.0, help="每帧等待 ACK 秒数")
    parser.add_argument("--retries", type=int, default=5)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    if not args.port:
        parser.error("缺少串口设备，例如 /dev/ttyUSB0")
    if not 1 <= args.chunk <= 1024 or args.timeout <= 0 or args.retries < 0:
        parser.error("--chunk 取值 1..1024，--timeout 必须大于0，--retries 不可为负")

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("串口模式需要 pyserial") from exc
    package = args.package.read_bytes()
    _, version = validate_package(package)
    started = time.monotonic()
    with serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1,
                       timeout=0.1, write_timeout=args.timeout, exclusive=True) as port:
        port.reset_input_buffer()
        port.write(MAGIC)
        port.flush()
        read_token(port, b"OTA_READY\n", args.timeout)

        completed = False
        offset = 0
        sequence = 0
        while offset < len(package):
            payload = package[offset:offset + args.chunk]
            frame = make_frame(sequence, offset, payload)
            for attempt in range(args.retries + 1):
                port.write(frame)
                port.flush()
                deadline = time.monotonic() + args.timeout
                try:
                    response = read_line(port, args.timeout)
                    # Late duplicate ACKs must not reject the following block.
                    while response.startswith(b"OTA_ACK:") and int(response[8:].strip()) <= offset:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError("等待当前块 ACK 超时")
                        response = read_line(port, remaining)
                except TimeoutError:
                    if attempt < args.retries:
                        continue
                    raise
                if response == b"OTA_OK\n" and offset + len(payload) == len(package):
                    completed = True
                    break
                if response == f"OTA_ACK:{offset + len(payload)}\n".encode():
                    break
                if response.startswith(b"OTA_NACK:") and attempt < args.retries:
                    continue
                raise RuntimeError(f"设备拒绝 offset={offset}: {response!r}")
            offset += len(payload)
            sequence += 1
            elapsed = max(time.monotonic() - started, 0.001)
            print(f"\r{offset}/{len(package)} ({offset * 100 / len(package):5.1f}%) "
                  f"{offset / elapsed / 1024:.1f} KiB/s", end="", flush=True)
        print()
        result = b"OTA_OK\n" if completed else read_line(port, 30.0)
        if result != b"OTA_OK\n":
            raise RuntimeError(f"设备返回失败: {result!r}")
    print(f"OTA_OK version={version}：设备已接受，启动未确认")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
