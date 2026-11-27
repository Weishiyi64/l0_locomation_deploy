#!/usr/bin/env python3
"""Merge Bootuser and an OTA App1 package into one blank-chip image."""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_BOOTUSER = PROJECT_ROOT / "build/ota_bootuser/output/demo.bin"
DEFAULT_APP = PROJECT_ROOT / "build/ota/pack/update_sign.bin"
DEFAULT_OUTPUT = PROJECT_ROOT / "build/ota/factory/factory_full.bin"

NOR_CFG_OFFSET = 0x00000400
BOOTUSER_END = 0x00043000
APP1_OFFSET = 0x00044000
APP_SLOT_SIZE = 0x00100000
FLASH_LAYOUT_SIZE = 0x00400000
APP_HEADER_SIZE = 0x00001000

NOR_CFG_TAG = 0xFCF90000
APP_MAGIC = 0xBEAF5AA5
APP_TOUCH_ID = 0xA5A55A5A
OTA_TYPE_NORMAL_FIRMWARE = 0
HASH_TYPE_SHA256 = 4


def validate_bootuser(image: bytes) -> None:
    if len(image) < 8:
        raise ValueError("Bootuser BIN 太小")
    if NOR_CFG_OFFSET + len(image) > BOOTUSER_END:
        raise ValueError(
            f"Bootuser 超过保留区: 0x{NOR_CFG_OFFSET + len(image):08x} "
            f"> 0x{BOOTUSER_END:08x}"
        )

    option_tag, option = struct.unpack_from("<II", image)
    if option_tag & 0xFFFF0000 != NOR_CFG_TAG:
        raise ValueError(f"Bootuser NOR 配置标识错误: 0x{option_tag:08x}")
    if option != 0x00000007:
        raise ValueError(f"Bootuser NOR Flash option 不匹配当前工程: 0x{option:08x}")


def validate_app(package: bytes) -> None:
    if len(package) <= APP_HEADER_SIZE:
        raise ValueError("OTA App 包不足 4 KiB Header 加正文")
    if len(package) > APP_SLOT_SIZE:
        raise ValueError(f"OTA App 包超过 1 MiB 槽: {len(package)} bytes")

    (
        magic,
        touch_id,
        _device,
        body_len,
        _version,
        firmware_type,
        hash_enable,
        power_hash,
        hash_type,
        _reserved,
    ) = struct.unpack_from("<IIIII4BI", package)

    if magic != APP_MAGIC or touch_id != APP_TOUCH_ID:
        raise ValueError(
            f"OTA App Header 错误: magic=0x{magic:08x}, touch=0x{touch_id:08x}"
        )
    if firmware_type != OTA_TYPE_NORMAL_FIRMWARE:
        raise ValueError(f"OTA App 类型不支持: {firmware_type}")
    if hash_enable != 1 or power_hash != 1 or hash_type != HASH_TYPE_SHA256:
        raise ValueError("OTA App 必须启用 SHA256 完整性校验")

    body = package[APP_HEADER_SIZE:]
    if body_len != len(body):
        raise ValueError(f"OTA App 正文长度不匹配: Header={body_len}, 实际={len(body)}")
    expected_hash = package[28:60]
    actual_hash = hashlib.sha256(body).digest()
    if expected_hash != actual_hash:
        raise ValueError("OTA App 正文 SHA256 校验失败")


def build_factory_image(bootuser: bytes, app: bytes) -> bytes:
    validate_bootuser(bootuser)
    validate_app(app)

    image_size = APP1_OFFSET + len(app)
    if image_size > FLASH_LAYOUT_SIZE:
        raise ValueError(f"工厂镜像超过当前 4 MiB OTA 布局: {image_size} bytes")

    image = bytearray(b"\xff" * image_size)
    image[NOR_CFG_OFFSET : NOR_CFG_OFFSET + len(bootuser)] = bootuser
    image[APP1_OFFSET : APP1_OFFSET + len(app)] = app
    return bytes(image)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="生成可从 Flash 偏移 0 写入空片的 HPM6E80 工厂镜像"
    )
    parser.add_argument("--bootuser", type=Path, default=DEFAULT_BOOTUSER)
    parser.add_argument("--app", type=Path, default=DEFAULT_APP)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv[1:])

    bootuser_path = args.bootuser.resolve()
    app_path = args.app.resolve()
    output_path = args.output.resolve()
    if output_path in (bootuser_path, app_path):
        parser.error("--output 不能覆盖输入文件")

    bootuser = bootuser_path.read_bytes()
    app = app_path.read_bytes()
    factory = build_factory_image(bootuser, app)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(factory)
    if output_path.read_bytes() != factory:
        raise OSError(f"工厂镜像回读校验失败: {output_path}")

    digest = hashlib.sha256(factory).hexdigest()
    checksum_path = output_path.with_name(output_path.name + ".sha256")
    checksum_path.write_text(f"{digest}  {output_path.name}\n", encoding="ascii")
    print(
        f"Factory image: {output_path} ({len(factory)} bytes)\n"
        f"  Bootuser: offset=0x{NOR_CFG_OFFSET:08x}, size={len(bootuser)}\n"
        f"  App1:     offset=0x{APP1_OFFSET:08x}, size={len(app)}\n"
        f"  SHA256:   {digest}\n"
        f"  Checksum: {checksum_path}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except (OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
