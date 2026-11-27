#!/usr/bin/env python3
"""Parse Yesense YIS320 UART/USB output frames and print JSON lines."""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import termios
import time
from typing import BinaryIO


HEADER = b"\x59\x53"
DEFAULT_UART_DEVICE = "/dev/ttyUSB2"
BAUDRATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: getattr(termios, "B460800", 460800),
    921600: getattr(termios, "B921600", 921600),
}


def checksum(data: bytes) -> tuple[int, int]:
    """Return Yesense CK1/CK2 for bytes from TID through MESSAGE."""
    ck1 = ck2 = 0
    for byte in data:
        ck1 = (ck1 + byte) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return ck1, ck2


def scaled_i32(data: bytes, scale: float) -> list[float]:
    return [value * scale for value in struct.unpack("<iii", data)]


def decode_item(data_id: int, data: bytes) -> dict:
    """Decode one YIS320 DATA ID item according to product manual V1.6."""
    expected = {
        0x01: 2,
        0x10: 12,
        0x20: 12,
        0x30: 12,
        0x31: 12,
        0x40: 12,
        0x41: 16,
    }.get(data_id)
    if expected is None:
        return {
            "data_id": f"0x{data_id:02X}",
            "length": len(data),
            "raw_hex": data.hex(),
        }
    if len(data) != expected:
        raise ValueError(
            f"DATA ID 0x{data_id:02X} length {len(data)}, expected {expected}"
        )

    if data_id == 0x01:
        return {"data_id": "0x01", "temperature_c": struct.unpack("<h", data)[0] * 0.01}
    if data_id == 0x10:
        x, y, z = scaled_i32(data, 0.000001)
        return {"data_id": "0x10", "acceleration_mps2": {"x": x, "y": y, "z": z}}
    if data_id == 0x20:
        x, y, z = scaled_i32(data, 0.000001)
        return {"data_id": "0x20", "angular_velocity_deg_s": {"x": x, "y": y, "z": z}}
    if data_id == 0x30:
        x, y, z = scaled_i32(data, 0.000001)
        return {"data_id": "0x30", "magnetic_normalized": {"x": x, "y": y, "z": z}}
    if data_id == 0x31:
        x, y, z = scaled_i32(data, 0.001)
        return {"data_id": "0x31", "magnetic_mgauss": {"x": x, "y": y, "z": z}}
    if data_id == 0x40:
        pitch, roll, yaw = scaled_i32(data, 0.000001)
        return {
            "data_id": "0x40",
            "euler_deg": {"pitch": pitch, "roll": roll, "yaw": yaw},
        }

    q0, q1, q2, q3 = struct.unpack("<iiii", data)
    return {
        "data_id": "0x41",
        "quaternion": {
            "q0": q0 * 0.000001,
            "q1": q1 * 0.000001,
            "q2": q2 * 0.000001,
            "q3": q3 * 0.000001,
        },
    }


def parse_packet(packet: bytes) -> dict:
    """Parse one complete packet, including header and CK1/CK2."""
    if len(packet) < 7 or packet[:2] != HEADER:
        raise ValueError("invalid header or packet length")
    tid = struct.unpack_from("<H", packet, 2)[0]
    message_length = packet[4]
    if len(packet) != 7 + message_length:
        raise ValueError("packet length does not match LEN")

    body = packet[2:-2]
    expected_ck1, expected_ck2 = checksum(body)
    if packet[-2:] != bytes((expected_ck1, expected_ck2)):
        raise ValueError(
            f"checksum mismatch: got {packet[-2:].hex()}, "
            f"expected {expected_ck1:02x}{expected_ck2:02x}"
        )

    items = []
    offset = 5
    end = 5 + message_length
    while offset < end:
        if offset + 2 > end:
            raise ValueError("truncated DATA ID/LEN")
        data_id = packet[offset]
        data_length = packet[offset + 1]
        offset += 2
        if offset + data_length > end:
            raise ValueError(f"DATA ID 0x{data_id:02X} exceeds MESSAGE")
        items.append(decode_item(data_id, packet[offset : offset + data_length]))
        offset += data_length

    return {"tid": tid, "message_length": message_length, "items": items}


class YIS320StreamParser:
    """Recover complete YIS320 packets from an arbitrary byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_packets = 0

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        packets = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                return packets
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                return packets
            total = 7 + self.buffer[4]
            if len(self.buffer) < total:
                return packets
            raw = bytes(self.buffer[:total])
            try:
                packets.append(parse_packet(raw))
                del self.buffer[:total]
            except ValueError:
                self.bad_packets += 1
                del self.buffer[:1]


class RateMeter:
    """Print the rate of valid YIS320 packets once per second."""

    def __init__(self) -> None:
        self.start = time.monotonic()
        self.last_time = self.start
        self.last_count = 0

    def report(self, count: int, force: bool = False) -> None:
        now = time.monotonic()
        elapsed = now - self.last_time
        if count == self.last_count or (not force and elapsed < 1.0):
            return
        rate = (count - self.last_count) / elapsed
        average = count / max(now - self.start, 1e-9)
        print(
            f"# imu_rate={rate:.2f} Hz  average={average:.2f} Hz  parsed={count}",
            file=sys.stderr,
            flush=True,
        )
        self.last_time = now
        self.last_count = count


def open_serial(device: str, baudrate: int) -> int:
    if baudrate not in BAUDRATES:
        raise ValueError(f"unsupported baudrate {baudrate}; choose from {sorted(BAUDRATES)}")
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = BAUDRATES[baudrate]
    attrs[5] = BAUDRATES[baudrate]
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def format_packet(packet: dict, number: int) -> str:
    """Format one packet for a compact, human-readable console display."""
    lines = [
        f"[{number:6d}] tid={packet['tid']:5d}  len={packet['message_length']:3d}"
    ]
    for item in packet["items"]:
        if "temperature_c" in item:
            lines.append(f"     TEMP[C]     {item['temperature_c']:10.2f}")
        elif "acceleration_mps2" in item:
            v = item["acceleration_mps2"]
            lines.append(
                f"     ACC[m/s^2]  x={v['x']:13.6f}  y={v['y']:13.6f}  "
                f"z={v['z']:13.6f}"
            )
        elif "angular_velocity_deg_s" in item:
            v = item["angular_velocity_deg_s"]
            lines.append(
                f"     GYRO[deg/s] x={v['x']:13.6f}  y={v['y']:13.6f}  "
                f"z={v['z']:13.6f}"
            )
        elif "magnetic_normalized" in item:
            v = item["magnetic_normalized"]
            lines.append(
                f"     MAG_NORM    x={v['x']:12.6f}  y={v['y']:12.6f}  "
                f"z={v['z']:12.6f}"
            )
        elif "magnetic_mgauss" in item:
            v = item["magnetic_mgauss"]
            lines.append(
                f"     MAG[mGauss] x={v['x']:12.6f}  y={v['y']:12.6f}  "
                f"z={v['z']:12.6f}"
            )
        elif "euler_deg" in item:
            v = item["euler_deg"]
            lines.append(
                f"     EULER[deg]  pitch={v['pitch']:12.6f}  "
                f"roll={v['roll']:12.6f}  yaw={v['yaw']:12.6f}"
            )
        elif "quaternion" in item:
            q = item["quaternion"]
            lines.append(
                f"     QUAT        q0={q['q0']:10.6f}  q1={q['q1']:10.6f}  "
                f"q2={q['q2']:10.6f}  q3={q['q3']:10.6f}"
            )
        else:
            lines.append(
                f"     DATA        id={item['data_id']} len={item['length']} "
                f"raw={item['raw_hex']}"
            )
    return "\n".join(lines)


def emit(
    parser: YIS320StreamParser,
    data: bytes,
    limit: int,
    count: int,
    json_output: bool,
    rate_meter: RateMeter,
) -> int:
    for packet in parser.feed(data):
        count += 1
        if json_output:
            print(json.dumps(packet, ensure_ascii=False, separators=(",", ":")), flush=True)
        else:
            print(format_packet(packet, count), flush=True)
        rate_meter.report(count)
        if limit and count >= limit:
            return count
    return count


def make_sample() -> bytes:
    message = bytes((0x01, 2)) + struct.pack("<h", 2534)
    message += bytes((0x40, 12)) + struct.pack("<iii", 1000000, -2000000, 3000000)
    body = struct.pack("<HB", 42, len(message)) + message
    return HEADER + body + bytes(checksum(body))


def self_test() -> None:
    parsed = parse_packet(make_sample())
    assert parsed["tid"] == 42
    assert parsed["items"][0]["temperature_c"] == 25.34
    assert parsed["items"][1]["euler_deg"] == {"pitch": 1.0, "roll": -2.0, "yaw": 3.0}
    stream = YIS320StreamParser()
    assert stream.feed(b"noise" + make_sample()[:4]) == []
    assert stream.feed(make_sample()[4:]) == [parsed]
    formatted = format_packet(parsed, 1)
    assert "EULER[deg]" in formatted and "pitch=    1.000000" in formatted


def main() -> int:
    ap = argparse.ArgumentParser(
        description="解析 Yesense YIS320 UART 输出协议并打印可读文本"
    )
    ap.add_argument(
        "--device",
        help=f"UART 串口，默认 {DEFAULT_UART_DEVICE}；USB CDC 可指定 /dev/ttyACM0",
    )
    ap.add_argument("--input", metavar="FILE", help="二进制数据文件；使用 - 表示标准输入")
    ap.add_argument("--baudrate", type=int, default=460800, choices=sorted(BAUDRATES))
    ap.add_argument("--limit", type=int, default=0, help="解析 N 个有效帧后退出，0 表示持续运行")
    ap.add_argument("--json", action="store_true", help="使用 JSON Lines 输出，便于程序处理")
    ap.add_argument("--self-test", action="store_true", help="运行内置协议自检")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        print("self-test: ok")
        return 0
    if args.device and args.input:
        ap.error("--device 和 --input 不能同时使用")
    if not args.device and not args.input:
        args.device = DEFAULT_UART_DEVICE

    parser = YIS320StreamParser()
    rate_meter = RateMeter()
    count = 0
    if args.device:
        fd = open_serial(args.device, args.baudrate)
        try:
            while not args.limit or count < args.limit:
                count = emit(
                    parser, os.read(fd, 4096), args.limit, count, args.json, rate_meter
                )
        except KeyboardInterrupt:
            pass
        finally:
            os.close(fd)
    else:
        stream: BinaryIO = sys.stdin.buffer if args.input == "-" else open(args.input, "rb")
        try:
            while not args.limit or count < args.limit:
                chunk = stream.read(4096)
                if not chunk:
                    break
                count = emit(parser, chunk, args.limit, count, args.json, rate_meter)
        finally:
            if stream is not sys.stdin.buffer:
                stream.close()
    rate_meter.report(count, force=True)
    print(f"# parsed={count} bad={parser.bad_packets}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
