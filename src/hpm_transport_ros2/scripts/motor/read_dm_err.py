#!/usr/bin/env python3
"""Listen to HPM CAN feedback and print Damiao MIT ERR from D[0] = ID | (ERR << 4)."""

from __future__ import annotations

import argparse
import socket
import struct
import sys

from hpm_udp import UdpBusinessSocket

IDENT = 0x0A4D5048
WIRE = 30
VERSION = 2
MOTOR_ID_DEFAULT = 0x0C

ERR_NAME = {
    0x0: "失能",
    0x1: "使能",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "驱动板MOS过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失",
    0xE: "过载",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def parse_frame(wire: bytes) -> dict | None:
    if len(wire) != WIRE:
        return None
    ident, ver, msg_id, hb, ts, reserved, port, ide, can_id, dlc = struct.unpack(
        ">IBBHIBBBIB", wire[:20]
    )
    payload = wire[20:28]
    crc = struct.unpack(">H", wire[28:30])[0]
    if ident != IDENT or ver != VERSION or dlc > 8:
        return None
    if crc16_modbus(wire[:28]) != crc:
        return None
    return {
        "msg_id": msg_id,
        "can_id": can_id,
        "dlc": dlc,
        "data": payload,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="读达妙 MIT 反馈 ERR（不发使能/控制）")
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--motor-id", type=lambda s: int(s, 0), default=MOTOR_ID_DEFAULT)
    ap.add_argument("--all", action="store_true", help="打印所有 CAN 回帧，不只过滤电机 ID")
    args = ap.parse_args()

    want = args.motor_id & 0x0F
    print(
        f"connect {args.ip}:{args.port}  filter motor_id=0x{want:X}  (Ctrl-C 退出)",
        flush=True,
    )

    sock = UdpBusinessSocket((args.ip, args.port), timeout=5)
    sock.settimeout(None)

    last = None
    n = 0
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
                if f is None or f["dlc"] < 1:
                    continue
                d0 = f["data"][0]
                mid = d0 & 0x0F
                err = (d0 >> 4) & 0x0F
                if not args.all and mid != want and (f["can_id"] & 0x7FF) != args.motor_id:
                    continue
                n += 1
                key = (f["can_id"], mid, err, f["data"][6], f["data"][7])
                if key == last:
                    continue
                last = key
                name = ERR_NAME.get(err, "未知")
                print(
                    f"[{n}] can_id=0x{f['can_id']:03X}  D0=0x{d0:02X}  "
                    f"ID=0x{mid:X}  ERR=0x{err:X}({name})  "
                    f"Tmos={f['data'][6]}C  Trotor={f['data'][7]}C  "
                    f"data={f['data'][:f['dlc']].hex()}",
                    flush=True,
                )
    except KeyboardInterrupt:
        print(f"\nstop  unique={n}")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    assert crc16_modbus(b"\x01\x02") == 0xE181
    raise SystemExit(main())
