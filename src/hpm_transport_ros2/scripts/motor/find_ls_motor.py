#!/usr/bin/env python3
"""查询 LS 电机 CAN ID：COB-ID 0x7FF 发 0xFFFF0082。"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

from hpm_udp import UdpBusinessSocket

IDENT = 0x0A4D5048
WIRE = 30
VERSION = 2
MSG_ID = 0x10
CAN_ID = 0x7FF
IDE_STD = 0
QUERY = bytes([0xFF, 0xFF, 0x00, 0x82])


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def serialize_frame(
    heartbeat: int, timestamp: int, port: int, can_id: int, data: bytes
) -> bytes:
    payload = data + bytes(8 - len(data))
    hdr = struct.pack(
        ">IBBHIBBBIB",
        IDENT, VERSION, MSG_ID, heartbeat, timestamp, 0, port, IDE_STD,
        can_id, len(data),
    )
    body = hdr + payload
    return body + struct.pack(">H", crc16_modbus(body))


def parse_frame(wire: bytes) -> dict | None:
    if len(wire) != WIRE:
        return None
    ident, ver, msg_id, _hb, _ts, reserved, port, ide, can_id, dlc = struct.unpack(
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
        "port": port,
        "ide": ide,
        "can_id": can_id,
        "dlc": dlc,
        "data": payload,
    }


def unpack_getid(data: bytes, dlc: int) -> tuple[int | None, bool, str]:
    d = data[:dlc]
    if len(d) >= 5 and d[0] == 0xFF and d[1] == 0xFF and d[2] == 0x01:
        motor_id = (d[3] << 8) | d[4]
        return motor_id, True, "ok"
    if len(d) >= 4 and d[0] == 0x80 and d[1] == 0x80 and d[2] == 0x01 and d[3] == 0x80:
        return None, False, "fail"
    return None, False, "unknown"


class RecvBuf:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buf = b""

    def drain(self, timeout_s: float) -> list[dict]:
        out: list[dict] = []
        deadline = time.monotonic() + timeout_s
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            old_timeout = self.sock.gettimeout()
            self.sock.settimeout(remain)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                break
            finally:
                self.sock.settimeout(old_timeout)
            self.buf += chunk
            while len(self.buf) >= WIRE:
                wire, self.buf = self.buf[:WIRE], self.buf[WIRE:]
                f = parse_frame(wire)
                if f is not None:
                    out.append(f)
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description="查询 LS 电机 CAN ID (0x7FF / 0xFFFF0082)")
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--can-port", type=int, default=4, help="MCAN 口，默认 4")
    ap.add_argument("--timeout-ms", type=float, default=200.0)
    ap.add_argument("--count", type=int, default=1, help="查询次数，默认 1")
    ap.add_argument("--interval-ms", type=float, default=100.0, help="多次查询间隔")
    args = ap.parse_args()

    wait_s = max(args.timeout_ms, 1.0) / 1000.0
    interval_s = max(args.interval_ms, 0.0) / 1000.0

    print(
        f"connect {args.ip}:{args.port}  MCAN{args.can_port}  "
        f"can_id=0x{CAN_ID:03X}  query={QUERY.hex()}  wait={args.timeout_ms:.0f}ms",
        flush=True,
    )

    sock = UdpBusinessSocket((args.ip, args.port), timeout=5)
    sock.settimeout(2.0)
    rx = RecvBuf(sock)

    hb = 0
    t0 = time.monotonic()
    found: dict[int, dict] = {}

    def send_query() -> None:
        nonlocal hb
        hb = 1 if hb == 0 or hb == 65535 else hb + 1
        ts = int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF
        sock.sendall(serialize_frame(hb, ts, args.can_port, CAN_ID, QUERY))

    def ingest(frames: list[dict]) -> None:
        for f in frames:
            if (f["can_id"] & 0x7FF) != CAN_ID:
                continue
            motor_id, ok, kind = unpack_getid(f["data"], f["dlc"])
            data = f["data"][: f["dlc"]]
            if kind == "unknown":
                continue
            if kind == "fail":
                print(
                    f"  FAIL  can=0x{f['can_id']:03X}  port={f['port']}  "
                    f"data={data.hex()}",
                    flush=True,
                )
                continue
            assert motor_id is not None
            rec = {"can_id": f["can_id"], "port": f["port"], "data": data}
            is_new = motor_id not in found
            found[motor_id] = rec
            if is_new:
                print(
                    f"  HIT motor=0x{motor_id:04X}  can=0x{f['can_id']:03X}  "
                    f"port={f['port']}  data={data.hex()}",
                    flush=True,
                )

    try:
        ingest(rx.drain(0.05))
        for i in range(max(1, args.count)):
            send_query()
            ingest(rx.drain(wait_s))
            if i + 1 < args.count and interval_s > 0:
                time.sleep(interval_s)
        ingest(rx.drain(0.05))
    except KeyboardInterrupt:
        print("\nstop")
    except ConnectionError as e:
        print(f"\nstop: {e}")
    except OSError as e:
        print(f"\nstop: {e}")
    finally:
        sock.close()

    print()
    if not found:
        print("没有成功应答。可换 --can-port，或确认总线上只有一台 LS 电机")
        return 1
    print(f"找到 {len(found)} 个 ID:")
    for motor_id in sorted(found):
        r = found[motor_id]
        print(f"  0x{motor_id:04X}  can=0x{r['can_id']:03X}  port={r['port']}")
    return 0


if __name__ == "__main__":
    assert crc16_modbus(b"\x01\x02") == 0xE181
    assert unpack_getid(bytes([0xFF, 0xFF, 0x01, 0x01, 0x02]), 5) == (0x0102, True, "ok")
    assert unpack_getid(bytes([0x80, 0x80, 0x01, 0x80]), 4) == (None, False, "fail")
    wire = serialize_frame(1, 0, 4, CAN_ID, QUERY)
    assert len(wire) == 30
    f = parse_frame(wire)
    assert f and f["can_id"] == CAN_ID and f["dlc"] == 4 and f["ide"] == IDE_STD
    raise SystemExit(main())
