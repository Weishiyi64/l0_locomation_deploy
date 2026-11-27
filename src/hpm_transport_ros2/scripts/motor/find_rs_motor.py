#!/usr/bin/env python3
"""穷举 RS 电机 CAN ID：发 GETID 查询帧，看 0xFE 应答。"""

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
IDE_EXT = 1
TYPE_GETID = 0x0
RSP_MARK = 0xFE


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
        IDENT, VERSION, MSG_ID, heartbeat, timestamp, 0, port, IDE_EXT,
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


def rs_getid_tx(master: int, target: int) -> int:
    return ((TYPE_GETID & 0x1F) << 24) | ((master & 0xFF) << 8) | (target & 0xFF)


def unpack_getid_rsp(can_id: int) -> int | None:
    if ((can_id >> 24) & 0x1F) != TYPE_GETID or (can_id & 0xFF) != RSP_MARK:
        return None
    return (can_id >> 8) & 0xFFFF


def decode_mcu_uid(data: bytes) -> int:
    return int.from_bytes(data[:8], "big")


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
    ap = argparse.ArgumentParser(description="穷举 RS 电机 ID：GETID 查询 / 上电广播")
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--can-port", type=int, default=4, help="MCAN 口，默认 4")
    ap.add_argument(
        "--master",
        type=lambda s: int(s, 0),
        default=0xFD,
        help="主机 CAN_ID，放 bits23-8，默认 0xFD",
    )
    ap.add_argument("--timeout-ms", type=float, default=30.0)
    ap.add_argument("--id-start", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--id-end", type=lambda s: int(s, 0), default=255)
    ap.add_argument(
        "--listen-only",
        action="store_true",
        help="只监听上电广播，不发查询",
    )
    args = ap.parse_args()

    master = args.master & 0xFF
    id_lo = max(0, min(255, args.id_start))
    id_hi = max(0, min(255, args.id_end))
    if id_lo > id_hi and not args.listen_only:
        print("id-start > id-end", file=sys.stderr)
        return 2
    wait_s = max(args.timeout_ms, 1.0) / 1000.0

    mode = "listen" if args.listen_only else "query"
    print(
        f"connect {args.ip}:{args.port}  MCAN{args.can_port}  mode={mode}  "
        f"master=0x{master:02X}  id=0x{id_lo:02X}..0x{id_hi:02X}  "
        f"wait={args.timeout_ms:.0f}ms",
        flush=True,
    )

    sock = UdpBusinessSocket((args.ip, args.port), timeout=5)
    sock.settimeout(2.0)
    rx = RecvBuf(sock)

    found: dict[int, dict] = {}
    hb = 0
    t0 = time.monotonic()

    def send_getid(target: int) -> None:
        nonlocal hb
        hb = 1 if hb == 0 or hb == 65535 else hb + 1
        ts = int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF
        can_id = rs_getid_tx(master, target)
        sock.sendall(serialize_frame(hb, ts, args.can_port, can_id, bytes(8)))

    def ingest(frames: list[dict]) -> None:
        for f in frames:
            motor = unpack_getid_rsp(f["can_id"])
            if motor is None:
                continue
            uid = decode_mcu_uid(f["data"])
            rec = {
                "can_id": f["can_id"],
                "port": f["port"],
                "uid": uid,
                "data": f["data"][: f["dlc"]],
            }
            is_new = motor not in found
            found[motor] = rec
            if is_new:
                print(
                    f"  HIT motor=0x{motor:02X}  can=0x{f['can_id']:08X}  "
                    f"port={f['port']}  mcu_uid=0x{uid:016X}  "
                    f"data={rec['data'].hex()}",
                    flush=True,
                )

    try:
        if args.listen_only:
            print("  等待上电广播 (Ctrl-C 退出)...", flush=True)
            while True:
                ingest(rx.drain(0.5))
        else:
            ingest(rx.drain(0.05))
            for target in range(id_lo, id_hi + 1):
                send_getid(target)
                ingest(rx.drain(wait_s))
                if (target & 0x1F) == 0x1F:
                    print(f"  ... probed 0x{id_lo:02X}..0x{target:02X}", flush=True)
            ingest(rx.drain(0.1))
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
        print("没有 GETID 应答。可试 --listen-only（上电广播）或换 --master / --can-port")
        return 1
    print(f"找到 {len(found)} 台:")
    for motor in sorted(found):
        r = found[motor]
        print(
            f"  0x{motor:02X}  can=0x{r['can_id']:08X}  port={r['port']}  "
            f"mcu_uid=0x{r['uid']:016X}"
        )
    return 0


if __name__ == "__main__":
    assert crc16_modbus(b"\x01\x02") == 0xE181
    assert rs_getid_tx(0xFD, 0x01) == 0x0000FD01
    assert rs_getid_tx(0, 0x01) == 0x00000001
    assert unpack_getid_rsp(0x000001FE) == 0x01
    assert unpack_getid_rsp(0x0300FD7F) is None
    raise SystemExit(main())
