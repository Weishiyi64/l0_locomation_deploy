#!/usr/bin/env python3
"""Send one complete OTA package through TCP/5001."""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path


MAGIC = b"HPMOTA1\n"
DEFAULT_HOST = "192.168.1.10"
DEFAULT_PACKAGE = (
    Path(__file__).resolve().parents[2] / "ota/update_sign.bin"
    # Path(__file__).resolve().parents[2] / "ota/stress/update_sign.bin"

)
DEFAULT_P4_IMAGE = DEFAULT_PACKAGE.parent / "esp32p4/uart_dma_ota.bin"
PROGRESS_WIDTH = 30


def recv_line(sock: socket.socket) -> bytes:
    data = bytearray()
    while len(data) < 128:
        chunk = sock.recv(1)
        if not chunk:
            break
        data += chunk
        if chunk == b"\n":
            break
    return bytes(data)


def legacy_main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Send a full HPM OTA package")
    parser.add_argument("host", nargs="?", default=DEFAULT_HOST)
    parser.add_argument("package", nargs="?", type=Path, default=DEFAULT_PACKAGE)
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--chunk", type=int, default=4096)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(argv)

    if args.chunk <= 0:
        parser.error("--chunk 必须大于 0")
    from hpm_serial_ota import validate_package
    validate_package(args.package.read_bytes(), 0x6750, 0x100000)
    package_size = args.package.stat().st_size

    with socket.create_connection((args.host, args.port), timeout=args.timeout) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(args.timeout)
        sock.sendall(MAGIC)
        print("已连接，等待 OTA_ENTERED...", flush=True)
        entered = recv_line(sock)
        if entered != b"OTA_ENTERED\n":
            raise RuntimeError(f"设备未进入 OTA: {entered!r}")
        print("OTA_ENTERED，等待 OTA_READY...", flush=True)
        ready = recv_line(sock)
        if ready != b"OTA_READY\n":
            raise RuntimeError(f"设备未准备 OTA: {ready!r}")
        print("OTA_READY", flush=True)

        sent = 0
        with args.package.open("rb") as package:
            while chunk := package.read(args.chunk):
                sock.sendall(chunk)
                sent += len(chunk)
                ratio = sent / package_size if package_size else 1
                filled = int(PROGRESS_WIDTH * ratio)
                bar = "=" * filled + "-" * (PROGRESS_WIDTH - filled)
                print(
                    f"\r发送 [{bar}] {ratio:6.2%} {sent}/{package_size} bytes",
                    end="",
                    flush=True,
                )
        print()

        result = recv_line(sock)
        if result != b"OTA_OK\n":
            raise RuntimeError(f"设备返回失败: {result!r}")
        print("OTA_OK")
    return 0


def main() -> int:
    argv = sys.argv[1:]
    if not argv or argv[0] not in ("hpm6e80", "hpm5151", "esp32p4", "-h", "--help"):
        return legacy_main(argv)
    parser = argparse.ArgumentParser(description="HPM6E80 / HPM5151 / ESP32-P4 OTA")
    parser.add_argument("target", choices=("hpm6e80", "hpm5151", "esp32p4"))
    link = parser.add_mutually_exclusive_group()
    link.add_argument("--host")
    link.add_argument("--serial")
    parser.add_argument("--file", type=Path,
                        help="固件路径；P4 升级默认 ota/esp32p4/uart_dma_ota.bin")
    parser.add_argument("--port", type=int)
    parser.add_argument("--baud", type=int)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--retries", type=int)
    parser.add_argument("--boot-timeout", type=float, default=30)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--query", action="store_true", help="P4 只查询，不进入升级")
    args = parser.parse_args(argv)
    if args.timeout is None:
        args.timeout = {"hpm6e80": 60, "hpm5151": 5, "esp32p4": 2}[args.target]
    if args.retries is None:
        args.retries = 5 if args.target == "hpm5151" else 3
    if args.timeout <= 0 or args.boot_timeout <= 0 or args.retries < 0:
        parser.error("超时必须大于0，重试次数不可为负")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("端口范围为1..65535")
    if args.baud is not None and args.baud <= 0:
        parser.error("波特率必须大于0")
    if args.query and args.target != "esp32p4":
        parser.error("--query 仅支持 ESP32-P4")
    if args.target == "esp32p4" and not args.query and args.file is None:
        args.file = DEFAULT_P4_IMAGE
    if not args.file and not args.query:
        parser.error("升级需要 --file")
    image = args.file.read_bytes() if args.file else None
    if args.target == "esp32p4":
        import p4_ota
        metadata = p4_ota.validate_image(image) if image is not None else {}
        if args.dry_run:
            print(f"DRY_RUN ESP32-P4: {len(image) if image else 0} bytes query={args.query} {metadata}")
            return 0
        args.host = args.host or DEFAULT_HOST
        args.port = args.port or 5002
        args.baud = args.baud or 921600
        if args.query:
            return p4_ota.query(args, image)
        return p4_ota.upgrade(args, image)
    from hpm_serial_ota import validate_package
    device, capacity = (0x6750, 0x100000) if args.target == "hpm6e80" else (0x5151, 0x40000)
    validate_package(image, device, capacity)
    if args.dry_run:
        print(f"DRY_RUN {args.target}: {len(image)} bytes, device=0x{device:x}")
        return 0
    if args.target == "hpm6e80":
        if args.serial:
            parser.error("HPM6E80 使用 --host TCP 升级")
        return legacy_main([args.host or DEFAULT_HOST, str(args.file), "--port",
                            str(args.port or 5001), "--timeout", str(args.timeout)])
    if not args.serial:
        parser.error("HPM5151 需要 --serial")
    import hpm_serial_ota
    saved = sys.argv
    try:
        sys.argv = ["ota_send.py", args.serial, str(args.file), "--baud",
                    str(args.baud or 921600), "--timeout", str(args.timeout),
                    "--retries", str(args.retries)]
        return hpm_serial_ota.main()
    finally:
        sys.argv = saved


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}")
        raise SystemExit(3 if type(exc).__name__ == "CommitUncertain" else 1)
