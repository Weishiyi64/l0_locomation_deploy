#!/usr/bin/env python3
"""Listen to HPM TCP :5001 and dump 30B frames (header + payload + CRC + channels)."""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motor"))
from hpm_udp import UdpBusinessSocket

IDENT = 0x0A4D5048
WIRE = 30
VERSION = 2
VREF = 3.3
ADC_FULL = 65535.0
TOUCH_BASE = 0x20
TOUCH_END = TOUCH_BASE


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def msg_cat(msg_id: int) -> str:
    if 0x10 <= msg_id <= 0x1F:
        return f"MOTOR/{msg_id - 0x10}"
    if 0x20 <= msg_id <= 0x2F:
        return "TOUCH/dual_adc" if msg_id == TOUCH_BASE else f"TOUCH/0x{msg_id:02X}"
    if 0x30 <= msg_id <= 0x3F:
        return f"IMU/{msg_id - 0x30}"
    if 0x40 <= msg_id <= 0x4F:
        return f"BATTERY/{msg_id - 0x40}"
    return "UNKNOWN"


def ide_name(ide: int) -> str:
    if ide == 0:
        return "standard"
    if ide == 1:
        return "extended"
    if ide == 0xFF:
        return "unused"
    return f"0x{ide:02X}"


def parse_frame(wire: bytes) -> dict | None:
    if len(wire) != WIRE:
        return None
    ident, ver, msg_id, hb, ts, reserved, port, ide, can_id, dlc = struct.unpack(
        ">IBBHIBBBIB", wire[:20]
    )
    payload = wire[12:28]
    crc = struct.unpack(">H", wire[28:30])[0]
    if ident != IDENT or ver != VERSION or (0x10 <= msg_id <= 0x1F and dlc > 8):
        return None
    calc = crc16_modbus(wire[:28])
    if calc != crc:
        return None
    return {
        "identifier": ident,
        "version": ver,
        "msg_id": msg_id,
        "heartbeat": hb,
        "timestamp": ts,
        "reserved": reserved,
        "port": port,
        "ide": ide,
        "can_id": can_id,
        "dlc": dlc,
        "payload": payload,
        "data": wire[20:28],
        "crc": crc,
        "wire": wire,
    }


def adc_volt(raw: int) -> float:
    return raw / ADC_FULL * VREF


def fmt_channels(payload: bytes) -> str:
    """Print both four-channel ADC groups in one 0x20 frame."""
    groups = []
    for group in range(2):
        pts = struct.unpack(">HHHH", payload[group * 8 : group * 8 + 8])
        channels = "  ".join(
            f"ch{i}={raw:5d}({adc_volt(raw):.3f}V)" for i, raw in enumerate(pts)
        )
        groups.append(f"group{group}: {channels}")
    return "    ".join(groups)


def run_plot(sock: socket.socket, window: int = 500) -> int:
    """Live-plot both ADC groups from TOUCH 0x20. Socket thread → matplotlib main."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    ys = [deque(maxlen=window) for _ in range(8)]

    def reader() -> None:
        buf = b""
        try:
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= WIRE:
                    wire, buf = buf[:WIRE], buf[WIRE:]
                    f = parse_frame(wire)
                    if f is None or f["msg_id"] != TOUCH_BASE:
                        continue
                    for group in range(2):
                        for channel, raw in enumerate(
                            struct.unpack(">HHHH", f["payload"][group * 8 : group * 8 + 8])
                        ):
                            ys[group * 4 + channel].append(raw)
        except OSError:
            pass

    threading.Thread(target=reader, daemon=True).start()

    fig, ax = plt.subplots()
    lines = [
        ax.plot([], [], label=f"group{group}/ch{channel}")[0]
        for group in range(2)
        for channel in range(4)
    ]
    ax.set_xlim(0, window - 1)
    ax.set_ylim(0, 70000)
    ax.set_xlabel("sample")
    ax.set_ylabel("ADC")
    ax.set_title("TOUCH 0x20 (two ADC groups)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    def update(_):
        for i, line in enumerate(lines):
            y = list(ys[i])
            line.set_data(range(len(y)), y)
        return lines

    # ponytail: blit=False so ylim can track; blit if CPU becomes an issue
    _ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    return 0


def fmt_frame(f: dict) -> str:
    cat = msg_cat(f["msg_id"])
    hdr = (
        f"ident=0x{f['identifier']:08X} ver=0x{f['version']:02X} "
        f"msg_id=0x{f['msg_id']:02X}({cat}) "
        f"hb=0x{f['heartbeat']:04X}({f['heartbeat']}) "
        f"ts={f['timestamp']}"
    )
    payload = (
        f"port={f['port']} ide=0x{f['ide']:02X}({ide_name(f['ide'])}) "
        f"can_id=0x{f['can_id']:08X} dlc={f['dlc']} "
        f"payload={f['payload'].hex()}"
    )
    tail = f"crc=0x{f['crc']:04X} ok"
    if TOUCH_BASE <= f["msg_id"] <= TOUCH_END:
        ch = fmt_channels(f["payload"])
        return f"{hdr}\n         {payload}\n         {tail}\n         {ch}"
    return f"{hdr}\n         {payload}\n         {tail}"


def main() -> int:
    ap = argparse.ArgumentParser(description="解析板子上行 30B 帧（图中全部字段 + 分通道）")
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--all", action="store_true", help="也打印非触觉帧")
    ap.add_argument("--wire", action="store_true", help="附带 30B 线格式 hex")
    ap.add_argument("--limit", type=int, default=0, help="打 N 帧后退出，0=一直打")
    ap.add_argument("--plot", action="store_true", help="实时画 0x20 两组八通道 ADC 曲线")
    args = ap.parse_args()

    print(f"connect {args.ip}:{args.port}  30B frame  (Ctrl-C 退出)", flush=True)

    sock = UdpBusinessSocket((args.ip, args.port), timeout=5)
    sock.settimeout(None)
    if args.plot:
        try:
            return run_plot(sock)
        finally:
            sock.close()

    n = 0
    bad = 0
    try:
        buf = b""
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                print("TCP 断开", file=sys.stderr)
                return 1
            buf += chunk
            while len(buf) >= WIRE:
                wire, buf = buf[:WIRE], buf[WIRE:]
                f = parse_frame(wire)
                if f is None:
                    bad += 1
                    continue
                if not args.all and not (TOUCH_BASE <= f["msg_id"] <= TOUCH_END):
                    continue
                n += 1
                line = fmt_frame(f)
                if args.wire:
                    line += f"\n         wire={f['wire'].hex()}"
                print(f"[{n}] {line}", flush=True)
                if args.limit and n >= args.limit:
                    print(f"stop  ok={n} bad={bad}")
                    return 0
    except KeyboardInterrupt:
        print(f"\nstop  ok={n} bad={bad}")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    assert crc16_modbus(b"\x01\x02") == 0xE181
    assert struct.unpack(">HHHH", bytes.fromhex("1574000000000000")) == (5492, 0, 0, 0)
    raise SystemExit(main())
