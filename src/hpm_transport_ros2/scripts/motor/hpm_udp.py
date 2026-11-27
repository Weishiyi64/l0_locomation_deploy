#!/usr/bin/env python3
"""Frame-stream adapter for the HPU1 UDP business protocol."""

import errno
import queue
import secrets
import socket
import struct
import threading
import time


HEADER = struct.Struct("!4sBBHQI")
FRAME_SIZE = 30
MAX_FRAMES = 48
MAX_DATAGRAM = 1460
DATA, HELLO, HELLO_ACK, PING, PONG, CLOSE, BUSY, RESET, OTA_BUSY = range(1, 10)


class UdpBusinessSocket:
    """Socket-like byte stream over UDP frames; each send is immediate and lossy."""

    def __init__(self, address, timeout=5.0, reconnect_ms=1000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.connect(address)
        self.sock.settimeout(0.1)
        self._timeout = timeout
        self._reconnect_s = max(1, reconnect_ms) / 1000.0
        self._lock = threading.Lock()
        self._frames = queue.Queue()
        self._pending = bytearray()
        self._running = True
        self._ready = threading.Event()
        self._state = "registering"
        self._session = self._new_session()
        self._tx_sequence = 0
        self._rx_sequence = None
        self._last_rx = time.monotonic()
        self._last_hello = 0.0
        self._last_ping = 0.0
        self.stats = {
            "tx_datagrams": 0, "tx_frames": 0, "rx_datagrams": 0,
            "rx_frames": 0, "rx_malformed": 0, "rx_gaps": 0,
            "rx_duplicates": 0, "rx_reordered": 0, "tx_errors": 0,
        }
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            self.close()
            raise TimeoutError(f"UDP registration with {address[0]}:{address[1]} timed out")

    @staticmethod
    def _new_session():
        return secrets.randbits(64) or 1

    @staticmethod
    def _host_ms():
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def _control_packet(self, packet_type, session):
        payload = struct.pack("!I", self._host_ms()) if packet_type in (HELLO, PING) else b""
        return HEADER.pack(b"HPU1", packet_type, 0, len(payload), session, 0) + payload

    def _send_control(self, packet_type, session):
        try:
            sent = self.sock.send(self._control_packet(packet_type, session))
            self.stats["tx_datagrams"] += sent > 0
            return True
        except OSError:
            self.stats["tx_errors"] += 1
            return False

    def _register_new_session(self):
        with self._lock:
            old = self._session
            self._send_control(CLOSE, old)
            self._session = self._new_session()
            self._tx_sequence = 0
            self._rx_sequence = None
            self._ready.clear()
            self._state = "registering"
            self._last_hello = 0.0

    def _run(self):
        while self._running:
            now = time.monotonic()
            if self._ready.is_set() and now - self._last_rx >= 6.0:
                self._register_new_session()
            if not self._ready.is_set() and now - self._last_hello >= self._reconnect_s:
                self._send_control(HELLO, self._session)
                self._last_hello = now
            elif self._ready.is_set() and now - self._last_ping >= 1.0:
                self._send_control(PING, self._session)
                self._last_ping = now

            try:
                packet = self.sock.recv(MAX_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    self._state = "socket_error"
                    time.sleep(0.1)
                continue

            self.stats["rx_datagrams"] += 1
            if len(packet) < HEADER.size or len(packet) > MAX_DATAGRAM:
                self.stats["rx_malformed"] += 1
                continue
            magic, packet_type, count, payload_len, session, sequence = HEADER.unpack_from(packet)
            payload = packet[HEADER.size:]
            if magic != b"HPU1" or payload_len != len(payload) or session != self._session:
                self.stats["rx_malformed"] += 1
                continue

            if packet_type == HELLO_ACK and count == 0 and payload_len == 0 and sequence == 0:
                self._state = "ready"
                self._last_rx = time.monotonic()
                self._last_ping = self._last_rx
                self._ready.set()
                continue
            if packet_type == PONG and self._ready.is_set() and count == 0 and payload_len == 0 and sequence == 0:
                self._last_rx = time.monotonic()
                continue
            if packet_type == BUSY or packet_type == OTA_BUSY:
                if count != 0 or payload_len != 0 or sequence != 0:
                    self.stats["rx_malformed"] += 1
                    continue
                self._state = "busy" if packet_type == BUSY else "ota_busy"
                self._ready.clear()
                continue
            if packet_type == RESET:
                if count != 0 or payload_len != 0 or sequence != 0:
                    self.stats["rx_malformed"] += 1
                    continue
                self._register_new_session()
                continue
            if packet_type != DATA or not self._ready.is_set() or \
                    count < 1 or count > MAX_FRAMES or payload_len != count * FRAME_SIZE:
                self.stats["rx_malformed"] += 1
                continue

            if self._rx_sequence is None:
                if sequence != 1:
                    self.stats["rx_reordered"] += 1
                    continue
            else:
                delta = (sequence - self._rx_sequence) & 0xFFFFFFFF
                if delta == 0:
                    self.stats["rx_duplicates"] += 1
                    continue
                if delta >= 0x80000000:
                    self.stats["rx_reordered"] += 1
                    continue
                if delta > 1:
                    self.stats["rx_gaps"] += delta - 1
            self._rx_sequence = sequence
            self._last_rx = time.monotonic()
            for offset in range(0, len(payload), FRAME_SIZE):
                self._frames.put(payload[offset:offset + FRAME_SIZE])
                self.stats["rx_frames"] += 1

    def wait_ready(self, timeout=None):
        return self._ready.wait(self._timeout if timeout is None else timeout)

    def sendall(self, data):
        data = bytes(data)
        if not data or len(data) % FRAME_SIZE or len(data) > MAX_FRAMES * FRAME_SIZE:
            raise ValueError("UDP business sends must contain 1..48 complete 30-byte frames")
        with self._lock:
            if not self._running or not self._ready.is_set():
                raise OSError(errno.ENOTCONN, "UDP business session is not ready")
            sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
            packet = HEADER.pack(b"HPU1", DATA, len(data) // FRAME_SIZE,
                                 len(data), self._session, sequence) + data
            try:
                sent = self.sock.send(packet)
            except OSError:
                self.stats["tx_errors"] += 1
                raise
            if sent != len(packet):
                self.stats["tx_errors"] += 1
                raise OSError(errno.EIO, "short UDP datagram send")
            self._tx_sequence = sequence
            self.stats["tx_datagrams"] += 1
            self.stats["tx_frames"] += len(data) // FRAME_SIZE

    def recv(self, size):
        if size <= 0:
            return b""
        if not self._pending:
            try:
                self._pending.extend(self._frames.get(timeout=self._timeout))
            except queue.Empty as exc:
                raise socket.timeout from exc
        result = bytes(self._pending[:size])
        del self._pending[:size]
        return result

    def settimeout(self, timeout):
        self._timeout = timeout

    def gettimeout(self):
        return self._timeout

    def setblocking(self, blocking):
        self._timeout = None if blocking else 0.0

    def getsockopt(self, level, option):
        if level == socket.SOL_SOCKET and option == socket.SO_ERROR:
            return 0 if self._state == "ready" else errno.ENOTCONN
        raise OSError(errno.ENOPROTOOPT, "not supported by UDP frame adapter")

    def close(self):
        if not self._running:
            return
        self._running = False
        with self._lock:
            if self._ready.is_set():
                self._send_control(CLOSE, self._session)
        self.sock.close()
        self._thread.join(timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
