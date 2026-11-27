#!/usr/bin/env python3
"""Verify UDP registration and byte-preserving business-payload capture."""

import socket
import struct
import subprocess
import sys
import tempfile
import threading
from pathlib import Path


HEADER = struct.Struct("!4sBBHQI")
DATA, HELLO, HELLO_ACK = 1, 2, 3


def main():
    payload = bytes(range(30))
    received = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as board, \
            tempfile.TemporaryDirectory() as directory:
        board.bind(("127.0.0.1", 0))
        board.settimeout(3)

        def respond():
            request, host = board.recvfrom(64)
            assert len(request) == HEADER.size + 4
            magic, packet_type, count, payload_len, session, sequence = HEADER.unpack_from(request)
            assert (magic, packet_type, count, payload_len, sequence) == (b"HPU1", HELLO, 0, 4, 0)
            received.append(request[HEADER.size:])
            board.sendto(HEADER.pack(b"HPU1", HELLO_ACK, 0, 0, session, 0), host)
            datagram = HEADER.pack(b"HPU1", DATA, 1, len(payload), session, 1) + payload
            board.sendto(datagram, host)

        worker = threading.Thread(target=respond)
        worker.start()
        output = Path(directory) / "raw.bin"
        run = subprocess.run([
            sys.executable, str(Path(__file__).with_name("rs_poweron_raw_capture.py")),
            "--host", "127.0.0.1", "--port", str(board.getsockname()[1]),
            "--duration", "0.5", "--output", str(output),
        ], capture_output=True, text=True, timeout=5)
        worker.join(timeout=3)
        assert not worker.is_alive()
        assert len(received) == 1 and len(received[0]) == 4
        assert run.returncode == 0, (run.stdout, run.stderr)
        assert output.read_bytes() == payload
    print("UDP registration and byte-preserving raw capture: PASS")


if __name__ == "__main__":
    main()
