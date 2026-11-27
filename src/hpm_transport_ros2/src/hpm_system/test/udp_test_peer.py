#!/usr/bin/env python3
"""Small UDP board simulator for hpm_system runtime tests."""

import queue
import socket
import struct
import threading


HEADER = struct.Struct("!4sBBHQI")
FRAME_SIZE = 30
DATA, HELLO, HELLO_ACK, PING, PONG, CLOSE = 1, 2, 3, 4, 5, 6


class UdpPeer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.ready = threading.Event()
        self.running = threading.Event()
        self.running.set()
        self.frames = queue.Queue()
        self._lock = threading.Lock()
        self._peer = None
        self._session = 0
        self._rx_sequence = None
        self._tx_sequence = 0
        self._timeout = 3.0
        self._pending = bytearray()
        self._context_depth = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def __enter__(self):
        self._context_depth += 1
        return self

    def __exit__(self, *_):
        self._context_depth -= 1
        if self._context_depth == 0:
            self.close()

    def _send_control(self, packet_type, session):
        packet = HEADER.pack(b"HPU1", packet_type, 0, 0, session, 0)
        self.sock.sendto(packet, self._peer)

    def _run(self):
        while self.running.is_set():
            try:
                data, peer = self.sock.recvfrom(1461)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) < HEADER.size:
                continue
            magic, packet_type, count, payload_len, session, sequence = HEADER.unpack_from(data)
            if magic != b"HPU1" or payload_len != len(data) - HEADER.size:
                continue
            with self._lock:
                if packet_type == HELLO and count == 0 and payload_len == 4:
                    self._peer = peer
                    self._session = session
                    self._rx_sequence = None
                    self._tx_sequence = 0
                    self.ready.set()
                    self._send_control(HELLO_ACK, session)
                    continue
                if session != self._session or peer != self._peer:
                    continue
                if packet_type == PING and count == 0 and payload_len == 4:
                    self._send_control(PONG, session)
                    continue
                if packet_type == CLOSE and count == 0 and payload_len == 0:
                    self.ready.clear()
                    continue
                if packet_type != DATA or count == 0 or payload_len != count * FRAME_SIZE:
                    continue
                if self._rx_sequence is not None:
                    delta = (sequence - self._rx_sequence) & 0xFFFFFFFF
                    if delta == 0 or delta >= 0x80000000:
                        continue
                self._rx_sequence = sequence
                for offset in range(HEADER.size, len(data), FRAME_SIZE):
                    self.frames.put(data[offset:offset + FRAME_SIZE])

    def wait_ready(self, timeout=5.0):
        assert self.ready.wait(timeout), "ROS UDP client did not register"

    def send_frame(self, wire):
        if len(wire) != FRAME_SIZE:
            raise ValueError("expected one 30-byte business frame")
        with self._lock:
            if not self.ready.is_set() or self._peer is None:
                raise RuntimeError("UDP client is not registered")
            self._tx_sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
            packet = HEADER.pack(b"HPU1", DATA, 1, FRAME_SIZE,
                                 self._session, self._tx_sequence) + bytes(wire)
            self.sock.sendto(packet, self._peer)

    def sendall(self, data):
        if len(data) % FRAME_SIZE:
            raise ValueError("business frames must be complete")
        for offset in range(0, len(data), FRAME_SIZE):
            self.send_frame(data[offset:offset + FRAME_SIZE])

    def recv(self, size):
        if not self._pending:
            try:
                self._pending.extend(self.frames.get(timeout=self._timeout))
            except queue.Empty as exc:
                raise socket.timeout from exc
        result = bytes(self._pending[:size])
        del self._pending[:size]
        return result

    def settimeout(self, timeout):
        self._timeout = timeout

    def gettimeout(self):
        return self._timeout

    def close(self):
        self.running.clear()
        self.sock.close()
        self._thread.join(timeout=1)
