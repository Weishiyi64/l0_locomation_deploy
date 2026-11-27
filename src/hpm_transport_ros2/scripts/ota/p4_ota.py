"""P4UP v1: framed, stop-and-wait ESP32-P4 application OTA."""
from __future__ import annotations
import contextlib
import hashlib
import secrets
import socket
import struct
import time
import zlib
from pathlib import Path

BOARD_ID = 0x50340001
CHIP_ID = 0x12
MAX_PAYLOAD = 1100
PROGRESS_WIDTH = 30
BEGIN_WAIT_SECONDS = 30
HEADER = struct.Struct("<4sBBHIIHH")
HELLO, BEGIN, DATA, END, COMMIT, ABORT, QUERY = 1, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35
ACK, NACK = 0x70, 0x71
ERRORS = ("OK", "BAD_FRAME", "SESSION", "STATE", "LENGTH", "OFFSET", "SEQUENCE",
          "HASH", "FLASH", "TARGET", "BUSY", "TIMEOUT", "OVERFLOW", "NOT_READY")

class DeviceError(RuntimeError):
    def __init__(self, status, detail):
        self.status = status
        name = ERRORS[status] if status < len(ERRORS) else str(status)
        super().__init__(f"P4 NACK: {name}, detail=0x{detail:08x}")

class CommitUncertain(RuntimeError):
    pass

def encode(kind, session, sequence, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("P4 payload too large")
    data = HEADER.pack(b"P4UP", 1, kind, 0, session, sequence, len(payload), 0) + payload
    return data + struct.pack("<I", zlib.crc32(data) & 0xffffffff)

def validate_image(data):
    """Validate an unsigned native application, including checksum/appended SHA."""
    if not 288 <= len(data) <= 0x1a9000 or data[0] != 0xe9 or not 1 <= data[1] <= 16:
        raise ValueError("需要当前 OTA 槽可容纳的原生 App .bin，不能发送出厂合并镜像")
    if struct.unpack_from("<H", data, 12)[0] != CHIP_ID:
        raise ValueError("镜像目标不是 ESP32-P4")
    if struct.unpack_from("<I", data, 32)[0] != 0xabcd5432:
        raise ValueError("缺少 App 描述信息：不能发送 bootloader.bin")
    offset, checksum = 24, 0xef
    for _ in range(data[1]):
        if offset + 8 > len(data):
            raise ValueError("镜像段头截断")
        size = struct.unpack_from("<I", data, offset + 4)[0]
        offset += 8
        if size > len(data) - offset:
            raise ValueError("镜像段越界")
        for byte in data[offset:offset + size]:
            checksum ^= byte
        offset += size
    checksum_pos = offset | 15
    if checksum_pos >= len(data) or data[checksum_pos] != checksum:
        raise ValueError("ESP 镜像 checksum 错误")
    end = checksum_pos + 1
    if data[23] not in (0, 1):
        raise ValueError("无效的镜像 hash 标志")
    if data[23]:
        if data[end:end + 32] != hashlib.sha256(data[:end]).digest():
            raise ValueError("ESP 镜像内置 SHA256 错误")
        end += 32
    if end != len(data):
        raise ValueError("镜像包含额外数据；首版仅支持未签名的原生应用")
    return dict(min_rev=struct.unpack_from("<H", data, 15)[0],
                max_rev=struct.unpack_from("<H", data, 17)[0] or 0xffff,
                version=data[48:80].split(b"\0", 1)[0].decode("utf-8", "replace"))

class Channel:
    def __init__(self, stream, network):
        self.stream, self.network, self.buffer = stream, network, bytearray()

    def write(self, data):
        if self.network:
            self.stream.sendall(data)
        else:
            view = memoryview(data)
            while view:
                n = self.stream.write(view)
                if not n:
                    raise TimeoutError("串口写入超时")
                view = view[n:]
            self.stream.flush()

    def read(self, count, timeout):
        if timeout <= 0:
            raise TimeoutError("等待 P4 响应超时")
        if self.network:
            self.stream.settimeout(timeout)
            data = self.stream.recv(count)
            if not data:
                raise ConnectionError("网关连接已关闭")
            return data
        self.stream.timeout = min(timeout, 0.05)
        return self.stream.read(count)

    def frame(self, deadline):
        while True:
            index = self.buffer.find(b"P4UP")
            if index < 0:
                del self.buffer[:-3]
            elif index:
                del self.buffer[:index]
            if self.buffer[:4] == b"P4UP" and len(self.buffer) >= HEADER.size:
                _, version, kind, flags, session, sequence, size, reserved = HEADER.unpack_from(self.buffer)
                if version != 1 or flags or reserved or size > MAX_PAYLOAD:
                    del self.buffer[0]
                    continue
                length = HEADER.size + size + 4
                if len(self.buffer) >= length:
                    raw = bytes(self.buffer[:length])
                    if zlib.crc32(raw[:-4]) & 0xffffffff != struct.unpack_from("<I", raw, length - 4)[0]:
                        del self.buffer[0]
                        continue
                    del self.buffer[:length]
                    return kind, session, sequence, raw[HEADER.size:-4]
            self.buffer.extend(self.read(1024, deadline - time.monotonic()))

@contextlib.contextmanager
def connect(args):
    if args.serial:
        print(f"连接 P4 串口 {args.serial}，波特率 {args.baud}...", flush=True)
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("串口模式需要 pyserial") from exc
        with serial.Serial(args.serial, args.baud, timeout=0.05, write_timeout=2,
                           exclusive=True, rtscts=False, dsrdtr=False) as stream:
            yield Channel(stream, False)
    else:
        print(f"连接 HPM 网关 {args.host}:{args.port}...", flush=True)
        with socket.create_connection((args.host, args.port), timeout=5) as stream:
            stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            channel = Channel(stream, True)
            channel.write(b"ESPOTA1\n")
            line = bytearray()
            deadline = time.monotonic() + 5
            while len(line) < 128 and not line.endswith(b"\n"):
                line.extend(channel.read(1, deadline - time.monotonic()))
            if line == b"ESP_ERR:BUSY\n":
                raise ConnectionError("HPM 网关忙")
            if line != b"ESP_READY\n":
                raise RuntimeError(f"网关未准备就绪: {bytes(line)!r}")
            print("HPM 网关握手成功 ESP_READY，串口链路待 P4 应答确认", flush=True)
            yield channel

def decode_status(payload):
    if len(payload) != 100:
        raise RuntimeError(f"无效 P4 状态长度: {len(payload)}")
    status, state, offset, detail = struct.unpack_from("<HHII", payload)
    board, chip, revision, slot, running, target, boot = struct.unpack_from("<IHHIIIB", payload, 12)
    return dict(status=status, state=state, offset=offset, detail=detail, board=board,
                chip=chip, revision=revision, slot=slot, running=running, target=target,
                boot=boot, version=payload[36:68].split(b"\0", 1)[0].decode("utf-8", "replace"),
                sha256=payload[68:100])

class Client:
    def __init__(self, channel, timeout=2, retries=3):
        self.channel, self.timeout, self.retries = channel, timeout, retries
        self.session, self.sequence = secrets.randbelow(0xfffffffe) + 1, 0

    def request(self, kind, payload=b"", timeout=None, retries=None, *, boot_wait=False,
                repeat=False):
        if not repeat:
            self.sequence += 1
        frame = encode(kind, self.session, self.sequence, payload)
        attempts = self.retries if retries is None else retries
        for attempt in range(attempts + 1):
            self.channel.write(frame)
            deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
            try:
                while True:
                    response, session, sequence, body = self.channel.frame(deadline)
                    if session != self.session or sequence != self.sequence:
                        continue
                    if response not in (ACK, NACK):
                        raise RuntimeError(f"无效响应类型: {response}")
                    info = decode_status(body)
                    if response == NACK or info["status"]:
                        raise DeviceError(info["status"], info["detail"])
                    return info
            except TimeoutError as exc:
                if not boot_wait and kind != BEGIN:
                    print(f"P4 应答超时：命令=0x{kind:02x}，第 {attempt + 1}/{attempts + 1} 次", flush=True)
                if attempt == attempts:
                    message = ("暂未收到 P4 启动应答" if boot_wait else
                               "未收到匹配的 P4 应答；请检查两端串口配置、收发和接线")
                    raise TimeoutError(message) from exc
        raise AssertionError("unreachable")

@contextlib.contextmanager
def open_p4_session(args, opener, kind, payload=b"", timeout=None):
    """Reconnect a dropped gateway only during the initial HELLO or QUERY."""
    for attempt in range(args.retries + 1):
        with opener(args) as channel:
            client = Client(channel, args.timeout, args.retries)
            try:
                info = client.request(kind, payload, timeout=timeout)
            except ConnectionError as exc:
                if args.serial or attempt == args.retries:
                    raise
                print(f"P4 首次应答前连接断开（{exc}），"
                      f"准备第 {attempt + 1}/{args.retries} 次重连", flush=True)
            else:
                yield client, info
                return
        time.sleep(0.25)


def query(args, image=None):
    size = len(image) if image is not None else 0
    timeout = max(30, args.timeout) if image is not None else args.timeout
    print(f"发送 P4 QUERY，每次等待 {timeout:g} 秒，每条连接最多 {args.retries + 1} 次...", flush=True)
    with open_p4_session(args, connect, QUERY, struct.pack("<I", size), timeout) as (_, info):
        print(f"QUERY board=0x{info['board']:x} revision={info['revision']} "
              f"version={info['version']} running=0x{info['running']:x} "
              f"boot={info['boot']} ota={info['state']} offset={info['offset']}")
        if image is not None:
            matches = info["sha256"] == hashlib.sha256(image).digest()
            print(f"QUERY_IMAGE_MATCH={matches} sha256={info['sha256'].hex()}")
            return 0 if matches else 1
        return 0

@contextlib.contextmanager
def open_p4_upgrade(args, opener, image_size, metadata, digest):
    with contextlib.ExitStack() as links:
        client, hello = links.enter_context(open_p4_session(args, opener, HELLO))
        if hello["board"] != BOARD_ID or hello["chip"] != CHIP_ID:
            raise ValueError("P4 设备板型或芯片不匹配")
        if not metadata["min_rev"] <= hello["revision"] <= metadata["max_rev"]:
            raise ValueError(f"镜像不兼容芯片 revision={hello['revision']}")
        if image_size > hello["slot"]:
            raise ValueError("镜像超过设备备用槽容量")
        print(f"P4 version={hello['version']} revision={hello['revision']} size={image_size}")
        print("等待 P4 就绪并准备 OTA 分区（BEGIN）...", flush=True)
        begin = struct.pack("<IIII32sB3x", image_size, 0, BOARD_ID, 0, digest, 0)
        begin_sequence = client.sequence + 1
        deadline = time.monotonic() + BEGIN_WAIT_SECONDS
        failures = 0
        reconnect = warned = False
        last = "尚未收到 BEGIN 应答"
        while time.monotonic() < deadline and failures <= args.retries:
            try:
                if reconnect:
                    channel = links.enter_context(opener(args))
                    # A separate HELLO must not advance the OTA session's sequence.
                    wait = min(args.timeout, max(0.01, deadline - time.monotonic()))
                    current = Client(channel, wait, 0).request(HELLO)
                    if any(current[key] != hello[key] for key in ("board", "chip", "revision", "running", "slot")):
                        raise ValueError("重连后 P4 设备或运行分区发生变化")
                    client.channel = channel
                    reconnect = False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                wait = min(5, remaining) if failures < args.retries else remaining
                # Firmware treats the same BEGIN session, sequence and payload as a retry.
                ready = client.request(BEGIN, begin, timeout=wait, retries=0,
                                       repeat=client.sequence == begin_sequence)
            except DeviceError as exc:
                if exc.status != 13 or reconnect:
                    raise
                last = str(exc)
            except OSError as exc:
                if args.serial and not isinstance(exc, TimeoutError):
                    raise
                last = str(exc)
                failures += 1
                if reconnect or not isinstance(exc, TimeoutError):
                    links.close()
                    reconnect = True
            else:
                yield client, ready
                return
            if not warned and failures <= args.retries and time.monotonic() < deadline:
                print("暂未收到 P4 BEGIN 确认，继续等待设备就绪或分区准备完成...", flush=True)
                warned = True
            time.sleep(min(0.25, max(0, deadline - time.monotonic())))
        raise RuntimeError(f"P4 BEGIN 未获确认，升级已停止：{last}")


def upgrade(args, image, opener=connect):
    metadata = validate_image(image)
    digest = hashlib.sha256(image).digest()
    committed = False
    with open_p4_upgrade(args, opener, len(image), metadata, digest) as (client, ready):
        started = True
        try:
            if ready["state"] != 2 or ready["offset"] != 0:
                raise RuntimeError("设备未进入空的接收会话")
            target = ready["target"]
            for offset in range(0, len(image), 1024):
                block = image[offset:offset + 1024]
                info = client.request(DATA, struct.pack("<I", offset) + block)
                if info["offset"] != offset + len(block):
                    raise RuntimeError("P4 ACK 偏移不匹配")
                sent = info["offset"]
                ratio = sent / len(image)
                filled = int(PROGRESS_WIDTH * ratio)
                bar = "=" * filled + "-" * (PROGRESS_WIDTH - filled)
                print(f"\r发送 [{bar}] {ratio:6.2%} {sent}/{len(image)} bytes",
                      end="", flush=True)
            print()
            verified = client.request(END, timeout=30)
            if verified["state"] != 4 or verified["offset"] != len(image):
                raise RuntimeError("设备未确认完整镜像")
            print("IMAGE_VERIFIED")
            # COMMIT must never be automatically repeated after a lost response.
            try:
                info = client.request(COMMIT, retries=0)
                if info["state"] != 5:
                    raise RuntimeError("无效 COMMIT 状态")
                committed = True
            except (OSError, TimeoutError):
                print("COMMIT 响应丢失，查询实际启动结果")
            started = False
        finally:
            if started:
                try:
                    client.request(ABORT, retries=0)
                except (OSError, RuntimeError):
                    pass
    boot_wait_started = time.monotonic()
    deadline = boot_wait_started + args.boot_timeout
    notice_at = boot_wait_started + 10
    print(f"等待 P4 重启并确认新 App，最多 {args.boot_timeout:g} 秒...", flush=True)
    last = "尚未收到启动响应"
    while time.monotonic() < deadline:
        time.sleep(0.25)
        try:
            with opener(args) as channel:
                client = Client(channel, args.timeout, 0)
                info = client.request(QUERY, struct.pack("<I", len(image)),
                                      timeout=min(5, max(0.1, deadline - time.monotonic())),
                                      boot_wait=True)
            notice_at = None
            last = f"running=0x{info['running']:x}, boot={info['boot']}, version={info['version']}"
            if info["board"] != BOARD_ID or info["chip"] != CHIP_ID:
                raise RuntimeError("启动查询目标变化")
            if info["running"] == target and info["boot"] == 2 and info["sha256"] == digest:
                print(f"BOOT_CONFIRMED {last} sha256={digest.hex()}")
                return 0
        except (OSError, RuntimeError) as exc:
            last = str(exc)
            if notice_at is not None and notice_at <= time.monotonic() < deadline:
                print("暂未收到启动应答，继续等待 P4 重启和串口初始化...", flush=True)
                notice_at = None
    if committed:
        raise RuntimeError(f"新 App 未确认，可能已回滚：{last}")
    raise CommitUncertain(f"提交结果未确定，请查询设备后再决定重传：{last}")
