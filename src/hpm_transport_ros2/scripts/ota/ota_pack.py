#!/usr/bin/env python3
"""Pack the OTA user image into a v2 normal SHA256 update package.

This project intentionally supports only the first-stage format:
flash-XIP, normal firmware, no EXIP, no compression, no differential image.
The package is integrity-protected by the body SHA256 stored in its header;
no signing key is required.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import struct
import sys
import time
from pathlib import Path


HPM_APP_HEADER_SIZE = 0x1000
APP_SLOT_SIZE = 0x100000
HPM_APP_FILE_FLAG_MAGIC = 0xBEAF5AA5
HPM_APP_FILE_TOUCH_ID = 0xA5A55A5A
OTA_TYPE_NORMAL_FIRMWARE = 0
HASH_TYPE_SHA256 = 4
PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_INPUT = (
    PROJECT_ROOT
    / "build/lwip_with_tcp_udp_dual_core_printf_ota/core0/output/demo.bin"
)
DEFAULT_OUTPUT = PROJECT_ROOT / "build/ota/pack/update_sign.bin"


def make_header(body: bytes) -> bytes:
    if not body:
        raise ValueError("OTA 固件正文为空")
    if len(body) > APP_SLOT_SIZE - HPM_APP_HEADER_SIZE:
        raise ValueError(
            f"OTA 固件正文过大: {len(body)} > "
            f"{APP_SLOT_SIZE - HPM_APP_HEADER_SIZE}"
        )

    version = int(os.environ.get("HPM_OTA_VERSION", str(int(time.time()))))
    if version <= 0 or version > 0xFFFFFFFF:
        raise ValueError("HPM_OTA_VERSION 必须是 1..0xffffffff")

    header = bytearray(HPM_APP_HEADER_SIZE)
    struct.pack_into(
        "<IIIII4BI",
        header,
        0,
        HPM_APP_FILE_FLAG_MAGIC,
        HPM_APP_FILE_TOUCH_ID,
        0x6750,
        len(body),
        version,
        OTA_TYPE_NORMAL_FIRMWARE,
        1,
        1,
        HASH_TYPE_SHA256,
        0,
    )
    header[28:60] = hashlib.sha256(body).digest()
    return bytes(header)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="生成 HPM OTA SHA256 升级包")
    parser.add_argument("legacy", nargs="*",
                        help="兼容旧格式: 0 4 OUTPUT INPUT")
    args = parser.parse_args(argv[1:])

    if len(args.legacy) == 0:
        firmware_type, hash_type = "0", "4"
        output, source = DEFAULT_OUTPUT, DEFAULT_INPUT
    elif len(args.legacy) == 4:
        firmware_type, hash_type, output_name, input_name = args.legacy
        output, source = Path(output_name), Path(input_name)
    else:
        parser.error("位置参数必须为空或为: 0 4 OUTPUT INPUT")

    if firmware_type != "0" or hash_type != "4":
        print("错误: 当前只支持参数 0 4", file=sys.stderr)
        return 2

    image = source.read_bytes()
    if len(image) <= HPM_APP_HEADER_SIZE:
        raise ValueError(f"输入镜像不足一个 OTA Header: {source}")

    magic = struct.unpack_from("<I", image, 0)[0]
    if magic != HPM_APP_FILE_FLAG_MAGIC:
        raise ValueError(f"输入镜像 Header magic 错误: 0x{magic:08x}")

    body = image[HPM_APP_HEADER_SIZE:]
    header = make_header(body)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header + body)
    print(
        f"OTA package: {output} ({output.stat().st_size} bytes), "
        f"body={len(body)} bytes, SHA256={hashlib.sha256(body).hexdigest()}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except (OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
