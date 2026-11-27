#!/usr/bin/env python3
"""穷举 MR 电机 CAN ID：对各 ID 发使能，看谁回 MIT 状态帧，扫到后失能。"""

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
IDE_STD = 0
ENABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
MODE_NAME = {0: "复位", 1: "伺服", 2: "力位混合", 3: "速度", 4: "力矩", 5: "Torque4"}


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


def unpack_mit(data: bytes) -> dict | None:
    if len(data) < 8:
        return None
    mode = data[0]
    if mode > 5:
        return None
    return {"mode": mode, "err": data[6], "t_mos": data[7]}


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
    ap = argparse.ArgumentParser(description="穷举 MR 电机 ID：发使能，看 MIT 反馈")
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--can-port", type=int, default=4, help="MCAN 口，默认 4")
    ap.add_argument("--timeout-ms", type=float, default=50.0)
    ap.add_argument("--id-start", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--id-end", type=lambda s: int(s, 0), default=0xFF)
    args = ap.parse_args()

    id_lo = max(0, min(0x7FF, args.id_start))
    id_hi = max(0, min(0x7FF, args.id_end))
    if id_lo > id_hi:
        print("id-start > id-end", file=sys.stderr)
        return 2
    wait_s = max(args.timeout_ms, 1.0) / 1000.0

    print(
        f"connect {args.ip}:{args.port}  MCAN{args.can_port}  "
        f"id=0x{id_lo:02X}..0x{id_hi:02X}  wait={args.timeout_ms:.0f}ms",
        flush=True,
    )

    sock = UdpBusinessSocket((args.ip, args.port), timeout=5)
    sock.settimeout(2.0)
    rx = RecvBuf(sock)

    found: dict[int, dict] = {}
    pending_disable: set[int] = set()
    hb = 0
    t0 = time.monotonic()

    def send(can_id: int, data: bytes) -> None:
        nonlocal hb
        hb = 1 if hb == 0 or hb == 65535 else hb + 1
        ts = int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF
        sock.sendall(serialize_frame(hb, ts, args.can_port, can_id, data))

    def ingest(frames: list[dict], expect_id: int) -> None:
        for f in frames:
            if f["ide"] != IDE_STD or f["dlc"] != 8 or f["can_id"] != expect_id:
                continue
            mit = unpack_mit(f["data"])
            if mit is None:
                continue
            rec = {**mit, "port": f["port"], "data": f["data"][:8]}
            is_new = expect_id not in found
            found[expect_id] = rec
            if is_new:
                name = MODE_NAME.get(mit["mode"], "?")
                print(
                    f"  HIT id=0x{expect_id:02X}  port={f['port']}  "
                    f"mode={mit['mode']}({name})  err=0x{mit['err']:02X}  "
                    f"data={rec['data'].hex()}",
                    flush=True,
                )

    try:
        ingest(rx.drain(0.05), -1)
        for mid in range(id_lo, id_hi + 1):
            pending_disable.add(mid)
            try:
                send(mid, ENABLE)
                ingest(rx.drain(wait_s), mid)
            finally:
                # An enable command may have reached the motor even when its
                # feedback was late/malformed. Always undo every probe.
                send(mid, DISABLE)
                rx.drain(0.02)
                pending_disable.discard(mid)
            if (mid & 0x0F) == 0x0F:
                print(f"  ... probed 0x{id_lo:02X}..0x{mid:02X}", flush=True)
        ingest(rx.drain(0.05), -1)
    except KeyboardInterrupt:
        print("\nstop")
    except ConnectionError as e:
        print(f"\nstop: {e}")
    except OSError as e:
        print(f"\nstop: {e}")
    finally:
        # Also clean up probes when the scan is interrupted or a socket error
        # occurs before the per-ID finally block can complete.
        for mid in sorted(pending_disable):
            try:
                send(mid, DISABLE)
            except OSError:
                break
        sock.close()

    print()
    if not found:
        print("没有 MIT 反馈。可换 --can-port，或加大 --id-end / --timeout-ms")
        return 1
    print(f"找到 {len(found)} 台:")
    for mid in sorted(found):
        r = found[mid]
        name = MODE_NAME.get(r["mode"], "?")
        print(
            f"  0x{mid:02X}  port={r['port']}  mode={r['mode']}({name})  "
            f"err=0x{r['err']:02X}"
        )
    return 0


if __name__ == "__main__":
    assert crc16_modbus(b"\x01\x02") == 0xE181
    assert unpack_mit(bytes([0x02, 0x80, 0x00, 0x7F, 0xF7, 0xFF, 0x00, 25])) == {
        "mode": 2,
        "err": 0,
        "t_mos": 25,
    }
    assert unpack_mit(bytes([0xFF]) + bytes(7)) is None
    wire = serialize_frame(1, 0, 4, 0x01, ENABLE)
    assert len(wire) == 30
    f = parse_frame(wire)
    assert f and f["can_id"] == 1 and f["dlc"] == 8 and f["ide"] == IDE_STD
    assert f["data"][:8] == ENABLE
    raise SystemExit(main())
