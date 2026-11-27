#!/usr/bin/env python3
"""Monitor HPM board and Odin radar traffic via tcpdump."""

import argparse
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass


ENDPOINT_RE = re.compile(
    r"(?P<src>\d{1,3}(?:\.\d{1,3}){3})\.(?P<src_port>\d+)\s+>\s+"
    r"(?P<dst>\d{1,3}(?:\.\d{1,3}){3})\.(?P<dst_port>\d+):"
)
LENGTH_RE = re.compile(r"\blength\s+(\d+)\s*$")
FLAGS_RE = re.compile(r"Flags \[([^]]*)\]")


@dataclass
class Packet:
    direction: str
    payload: int
    flags: str


@dataclass
class Counters:
    board_to_host_bytes: int = 0
    host_to_board_bytes: int = 0
    board_to_host_packets: int = 0
    host_to_board_packets: int = 0
    radar_to_host_bytes: int = 0
    host_to_radar_bytes: int = 0
    radar_to_host_packets: int = 0
    host_to_radar_packets: int = 0
    packets: int = 0
    rst: int = 0
    syn: int = 0
    fin: int = 0

    def add(self, packet: Packet) -> None:
        self.packets += 1
        if packet.direction == "board->host":
            self.board_to_host_bytes += packet.payload
            self.board_to_host_packets += 1
        elif packet.direction == "host->board":
            self.host_to_board_bytes += packet.payload
            self.host_to_board_packets += 1
        elif packet.direction == "radar->host":
            self.radar_to_host_bytes += packet.payload
            self.radar_to_host_packets += 1
        elif packet.direction == "host->radar":
            self.host_to_radar_bytes += packet.payload
            self.host_to_radar_packets += 1
        if "R" in packet.flags:
            self.rst += 1
        if "S" in packet.flags:
            self.syn += 1
        if "F" in packet.flags:
            self.fin += 1


def parse_packet(line: str, board_ip: str, radar_ips: set[str]) -> Packet | None:
    match = ENDPOINT_RE.search(line)
    if match is None:
        return None

    src = match.group("src")
    dst = match.group("dst")
    if src == board_ip:
        direction = "board->host"
    elif dst == board_ip:
        direction = "host->board"
    elif src in radar_ips:
        direction = "radar->host"
    elif dst in radar_ips:
        direction = "host->radar"
    else:
        return None

    length_match = LENGTH_RE.search(line)
    flags_match = FLAGS_RE.search(line)
    return Packet(
        direction=direction,
        payload=int(length_match.group(1)) if length_match else 0,
        flags=flags_match.group(1) if flags_match else "",
    )


def mbps(byte_count: int, elapsed: float) -> float:
    return byte_count * 8.0 / max(elapsed, 0.001) / 1_000_000.0


def print_report(delta: Counters, total: Counters, elapsed: float) -> None:
    print(
        f"Δ {elapsed:5.1f}s  板→PC {mbps(delta.board_to_host_bytes, elapsed):7.3f} Mbps "
        f"({delta.board_to_host_packets:6d} pkt)  "
        f"PC→板 {mbps(delta.host_to_board_bytes, elapsed):7.3f} Mbps "
        f"({delta.host_to_board_packets:6d} pkt)  "
        f"RST={delta.rst} SYN={delta.syn} FIN={delta.fin}"
    )
    print(
        f"Δ 雷达→PC {mbps(delta.radar_to_host_bytes, elapsed):7.3f} Mbps "
        f"({delta.radar_to_host_packets:6d} pkt)  "
        f"PC→雷达 {mbps(delta.host_to_radar_bytes, elapsed):7.3f} Mbps "
        f"({delta.host_to_radar_packets:6d} pkt)"
    )
    print(
        f"Σ payload  板→PC {total.board_to_host_bytes / 1_000_000:8.2f} MB  "
        f"PC→板 {total.host_to_board_bytes / 1_000_000:8.2f} MB  "
        f"雷达→PC {total.radar_to_host_bytes / 1_000_000:8.2f} MB  "
        f"PC→雷达 {total.host_to_radar_bytes / 1_000_000:8.2f} MB  "
        f"packets={total.packets}  rst={total.rst}"
    )


def self_test() -> None:
    packet = parse_packet(
        "192.168.1.10.5001 > 192.168.1.5.53380: Flags [P.], length 360",
        "192.168.1.10",
        {"192.168.1.20"},
    )
    assert packet == Packet("board->host", 360, "P.")
    packet = parse_packet(
        "192.168.1.5.53380 > 192.168.1.10.5001: Flags [R.], length 0",
        "192.168.1.10",
        {"192.168.1.20"},
    )
    assert packet == Packet("host->board", 0, "R.")
    packet = parse_packet(
        "192.168.1.20.42000 > 192.168.1.5.50123: UDP, length 1472",
        "192.168.1.10",
        {"192.168.1.20"},
    )
    assert packet == Packet("radar->host", 1472, "")
    print("self-test passed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-ip", default="192.168.1.10")
    parser.add_argument(
        "--radar-ip", action="append", default=[],
        help="Odin radar IP; repeat for multiple radars",
    )
    parser.add_argument("--iface", default="any", help="capture interface, default: any")
    parser.add_argument("--filter", default=None, help="custom tcpdump filter")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if shutil.which("tcpdump") is None:
        print("error: tcpdump 未安装，请先安装 tcpdump", file=sys.stderr)
        return 2
    if args.interval <= 0:
        parser.error("--interval 必须大于 0")

    radar_ips = set(args.radar_ip)
    capture_hosts = [args.board_ip, *sorted(radar_ips)]
    capture_filter = args.filter or " or ".join(f"host {ip}" for ip in capture_hosts)
    command = [
        "tcpdump", "-i", args.iface, "-nn", "-tt", "-l", "-s", "0",
        capture_filter,
    ]
    print("capture:", " ".join(command))
    print("说明：速率按 tcpdump 的 payload length 统计；RST/SYN/FIN 为捕获到的 TCP 标志。")
    if radar_ips:
        print("雷达 IP:", ", ".join(sorted(radar_ips)))
    else:
        print("雷达统计未启用，请追加 --radar-ip <雷达IP>。")

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except OSError as exc:
        print(f"error: 无法启动 tcpdump: {exc}", file=sys.stderr)
        return 2

    lines: queue.Queue[str | None] = queue.Queue()

    def read_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            lines.put(line)
        lines.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    stop = False
    diagnostics = []

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    total = Counters()
    interval = Counters()
    report_at = time.monotonic() + args.interval
    started_at = time.monotonic()
    interval_started_at = started_at

    try:
        while not stop:
            try:
                line = lines.get(timeout=0.2)
            except queue.Empty:
                line = ""
            if line is None:
                break
            if line:
                packet = parse_packet(line, args.board_ip, radar_ips)
                if packet is not None:
                    total.add(packet)
                    interval.add(packet)
                elif len(diagnostics) < 20:
                    diagnostics.append(line.strip())
            now = time.monotonic()
            if now >= report_at:
                print_report(interval, total, now - interval_started_at)
                interval = Counters()
                interval_started_at = now
                report_at = now + args.interval
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.terminate()
        if not stop and process.returncode not in (None, 0):
            print(f"tcpdump 启动失败，退出码={process.returncode}", file=sys.stderr)
            for diagnostic in diagnostics:
                print(diagnostic, file=sys.stderr)
            if any("permission" in item.lower() or "operation not permitted" in item.lower()
                   for item in diagnostics):
                print("请使用 sudo 运行，或给 tcpdump 配置抓包权限。", file=sys.stderr)
        print("\nmonitor stopped")
        print_report(interval, total, time.monotonic() - interval_started_at)

    return process.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
