#!/usr/bin/env python3
"""Passively capture unwrapped HPM UDP business bytes; send no CAN commands."""

import argparse
import secrets
import socket
import struct
import sys
import time
from datetime import datetime
from pathlib import Path


HEADER = struct.Struct("!4sBBHQI")
UDP_MAX = 1460
FRAME_SIZE = 30
DATA, HELLO, HELLO_ACK, PING, PONG, CLOSE, BUSY, RESET, OTA_BUSY = range(1, 10)


def control(packet_type: int, session: int) -> bytes:
    payload = struct.pack("!I", int(time.monotonic() * 1000) & 0xFFFFFFFF) \
        if packet_type in (HELLO, PING) else b""
    return HEADER.pack(b"HPU1", packet_type, 0, len(payload), session, 0) + payload


def new_session() -> int:
    return secrets.randbits(64) or 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Passively capture raw HPM UDP business payload bytes; sends no CAN commands."
    )
    parser.add_argument("--host", default="192.168.2.10", help="HPM address (default: 192.168.2.10)")
    parser.add_argument("--port", type=int, default=5001, help="HPM UDP port (default: 5001)")
    parser.add_argument("--duration", type=float, default=60.0, help="capture seconds (default: 60)")
    parser.add_argument("--output", type=Path, help="optional file for exact unwrapped business bytes")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.port <= 65535:
        parser.error("duration must be positive and port must be in 1..65535")

    print(f"Opening UDP session to {args.host}:{args.port}; capture={args.duration:g}s; CAN commands=none", flush=True)
    print("Use one business host; stop hpm_system first if it owns the UDP session.", flush=True)

    output = None
    try:
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            output = args.output.open("wb")
            print(f"Raw byte log: {args.output}", flush=True)

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("0.0.0.0", 0))
            sock.connect((args.host, args.port))
            sock.settimeout(0.1)
            session = new_session()
            ready = False
            last_rx = time.monotonic()
            last_hello = 0.0
            last_ping = 0.0
            last_rx_sequence = None
            deadline = time.monotonic() + args.duration
            total = 0
            datagrams = 0

            while time.monotonic() < deadline:
                now = time.monotonic()
                if ready and now - last_rx >= 6.0:
                    sock.send(control(CLOSE, session))
                    session = new_session()
                    ready = False
                    last_hello = 0.0
                    last_rx_sequence = None
                if not ready and now - last_hello >= 1.0:
                    sock.send(control(HELLO, session))
                    last_hello = now
                elif ready and now - last_ping >= 1.0:
                    sock.send(control(PING, session))
                    last_ping = now

                try:
                    packet = sock.recv(UDP_MAX + 1)
                except socket.timeout:
                    continue
                if len(packet) < HEADER.size or len(packet) > UDP_MAX:
                    continue
                magic, packet_type, count, payload_len, packet_session, sequence = HEADER.unpack_from(packet)
                if magic != b"HPU1" or packet_session != session or payload_len != len(packet) - HEADER.size:
                    continue
                payload = packet[HEADER.size:]
                if packet_type == HELLO_ACK:
                    ready = True
                    last_rx = time.monotonic()
                    last_ping = last_rx
                    print("READY: HPM UDP feedback session registered", flush=True)
                elif packet_type == PONG and ready:
                    last_rx = time.monotonic()
                elif packet_type in (BUSY, OTA_BUSY):
                    ready = False
                    print("BUSY: another host or OTA owns the board; retrying registration", flush=True)
                elif packet_type == RESET:
                    session = new_session()
                    ready = False
                    last_hello = 0.0
                    last_rx_sequence = None
                elif packet_type == DATA and ready and 1 <= count <= 48 and \
                        payload_len == count * FRAME_SIZE:
                    if last_rx_sequence is not None:
                        delta = (sequence - last_rx_sequence) & 0xFFFFFFFF
                        if delta == 0 or delta >= 0x80000000:
                            continue
                    last_rx_sequence = sequence
                    # Keep the capture byte-preserving after removing only HPU1.
                    stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
                    print(f"{stamp} RX datagram={datagrams + 1} len={len(payload)} hex={payload.hex(' ')}", flush=True)
                    if output:
                        output.write(payload)
                        output.flush()
                    total += len(payload)
                    datagrams += 1
                    last_rx = time.monotonic()

            if ready:
                sock.send(control(CLOSE, session))
            if total:
                print(f"CAPTURED: {total} raw business bytes in {datagrams} UDP datagrams", flush=True)
                return 0
            print(f"NO DATA: received 0 raw business bytes in {args.duration:g}s", flush=True)
            return 0
    except OSError as exc:
        print(f"UDP ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if output:
            output.close()


if __name__ == "__main__":
    raise SystemExit(main())
