#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""ROS ID commissioning: query -> configure -> manual power cycle -> verify.

Run hpm_system in motor_id_calibration_mode for writes. This tool never moves,
enables, disables or zeroes a motor, and never edits the configuration.
"""

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = WORKSPACE / "src/hpm_motor/config/motors.yaml"
SERVICE = "/actuator/motor/calibrate_id"
QUERY, CONFIGURE = 0, 1
CONFIRMED = 0
RESULT_NAMES = ("CONFIRMED", "REJECTED", "SEND_FAILED", "DEVICE_FAILED", "TIMEOUT", "UNSUPPORTED")
FACTORY_IDS = {"RS": 0x7F, "MR": 1, "LS": 1}
MR_MODELS = {"PA43-10-40", "PA43-10-20.25", "PA35-10-8"}


@dataclass(frozen=True)
class Motor:
    name: str
    index: int
    protocol: str
    model: str
    bus: int
    target_id: int


def integer(value):
    if isinstance(value, bool):
        raise ValueError("布尔值不能作为 ID/index")
    return value if isinstance(value, int) else int(str(value), 0)


def load_motors(path):
    with Path(path).open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    config = root.get("motors") if isinstance(root, dict) else None
    if not isinstance(config, dict) or not config:
        raise ValueError("配置必须包含非空 motors 映射")
    motors, indices, routes = [], set(), set()
    for name, entry in config.items():
        if not isinstance(name, str) or not name or not isinstance(entry, dict):
            raise ValueError("电机名称/配置无效")
        protocol = entry["motor_type"]
        if protocol not in {"RS", "MR", "LS", "DM", "ENC"}:
            raise ValueError(f"{name}: 不支持 motor_type={protocol}")
        model = str(entry.get("model", "")) if protocol == "MR" else ""
        if protocol == "MR" and model not in MR_MODELS:
            raise ValueError(f"{name}: MR 必须明确选择 {sorted(MR_MODELS)}")
        index, bus, target = (integer(entry[key]) for key in ("index", "bus_index", "motor_id"))
        limit = 0x7FF if protocol in {"LS", "ENC"} else 0xFF
        if not 0 <= index <= 255 or not 0 <= bus <= 7 or not 1 <= target <= limit:
            raise ValueError(f"{name}: index、bus_index 或 motor_id 越界")
        if index in indices or (bus, target) in routes:
            raise ValueError(f"{name}: index 或 bus_index/motor_id 重复")
        indices.add(index)
        routes.add((bus, target))
        motors.append(Motor(name, index, protocol, model, bus, target))
    return sorted(motors, key=lambda motor: motor.index)


def describe(motor):
    model = f"/{motor.model}" if motor.model else ""
    return f"[{motor.index:2}] {motor.name}  {motor.protocol}{model}  CAN{motor.bus}  ID=0x{motor.target_id:02X}"


def show_motors(motors, output=print):
    for bus in sorted({motor.bus for motor in motors}):
        output(f"CAN{bus}:")
        for motor in motors:
            if motor.bus == bus:
                output("  " + describe(motor))


def response_record(response):
    return {key: getattr(response, key) for key in (
        "result", "stage", "message", "motor_name", "motor_type", "model",
        "bus_index", "target_id", "power_cycle_required")} | {
        "detected_ids": list(response.detected_ids)}


class JsonlLogger:
    def __init__(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = directory / f"{stamp}.jsonl"

    def write(self, **record):
        record["time"] = datetime.now(timezone.utc).isoformat()
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")


def check_metadata(motor, response):
    expected = (motor.name, motor.protocol, motor.model, motor.bus, motor.target_id)
    actual = (response.motor_name, response.motor_type, response.model,
              response.bus_index, response.target_id)
    if expected != actual:
        raise RuntimeError(f"节点与脚本配置不一致，停止操作。脚本={expected}，节点={actual}")


class RosClient:
    def __init__(self, timeout, logger):
        # Deliberately lazy: --help/--dry-run require no ROS runtime or device.
        import rclpy
        from luvbot_hardware_interfaces.srv import MotorCalibrateId
        self.rclpy = rclpy
        self.service_type = MotorCalibrateId
        self.timeout = timeout
        self.logger = logger
        rclpy.init()
        self.node = rclpy.create_node("motor_id_calibration")
        self.client = self.node.create_client(MotorCalibrateId, SERVICE)
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.close()
            raise RuntimeError(f"未发现 {SERVICE}，请检查 hpm_system 是否启动并加载新版接口")

    def call(self, motor, operation, current_id):
        request = self.service_type.Request(
            operation=operation, motor_index=motor.index, current_id=current_id)
        self.logger.write(type="request", motor=asdict(motor),
                          operation=operation, current_id=current_id)
        future = self.client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            self.logger.write(type="client_timeout", motor=asdict(motor), operation=operation)
            raise RuntimeError("等待 service 超时，节点可能仍在执行，结果未知；不要重试写入")
        response = future.result()
        if response is None:
            raise RuntimeError("service 未返回响应，结果未知")
        self.logger.write(type="response", motor=asdict(motor), response=response_record(response))
        check_metadata(motor, response)
        return response

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def report(response, output=print):
    result = RESULT_NAMES[response.result] if 0 <= response.result < len(RESULT_NAMES) else "UNKNOWN"
    ids = ", ".join(f"0x{item:02X}" for item in response.detected_ids) or "无"
    output(f"{result} [{response.stage}] ID={ids}: {response.message}")


def query_three(client, motor, expected_ids, output=print):
    passed = True
    for attempt in range(1, 4):
        response = client.call(motor, QUERY, motor.target_id)
        report(response, output)
        detected = set(response.detected_ids)
        ok = response.result == CONFIRMED and detected == set(expected_ids)
        output(f"  查询 {attempt}/3: {'PASS' if ok else 'FAIL'}，"
               f"缺失={sorted(set(expected_ids) - detected)}，意外={sorted(detected - set(expected_ids))}")
        passed = passed and ok
    return passed


def check_all(client, motors, output=print):
    passed = set()
    inventory_ok = True
    checked_ls = set()
    for motor in motors:
        if motor.protocol not in FACTORY_IDS:
            output(f"UNSUPPORTED: {describe(motor)}")
            continue
        group = [motor]
        if motor.protocol == "LS":
            if motor.bus in checked_ls:
                continue
            checked_ls.add(motor.bus)
            group = [item for item in motors if item.protocol == "LS" and item.bus == motor.bus]
        output("自检: " + ", ".join(item.name for item in group))
        expected = {item.target_id for item in group}
        confirmed = set(expected)
        for attempt in range(1, 4):
            response = client.call(motor, QUERY, motor.target_id)
            report(response, output)
            detected = set(response.detected_ids)
            ok = response.result == CONFIRMED and detected == expected
            output(f"  查询 {attempt}/3: {'PASS' if ok else 'FAIL'}，"
                   f"缺失={sorted(expected - detected)}，意外={sorted(detected - expected)}")
            if motor.protocol == "LS":
                confirmed.intersection_update(detected if response.result == CONFIRMED else set())
            elif not ok:
                confirmed.clear()
            inventory_ok = inventory_ok and ok
        for item in group:
            ok = item.target_id in confirmed
            if ok:
                passed.add(item.index)
            output(f"{'PASS' if ok else 'FAIL'}: {describe(item)}")
    output(f"自检结果: {len(passed)}/{len(motors)} 个电机 PASS")
    for status, selected in (("PASS", True), ("FAIL", False)):
        items = [item for item in motors if (item.index in passed) == selected]
        output(f"{status} 电机 ({len(items)}):")
        for item in items:
            suffix = "（协议不支持）" if item.protocol not in FACTORY_IDS else ""
            output(f"  {describe(item)}{suffix}")
        if not items:
            output("  无")
    if len(passed) == len(motors) and not inventory_ok:
        output("所有配置电机通过，但检测到意外 ID，总线自检仍为 FAIL")
    return len(passed) == len(motors) and inventory_ok


def discover_ids(client, motor, output=print):
    output(f"自动查询: {describe(motor)}")
    output("查询的是所选总线/协议；确认关节身份时，请仅给待查电机上电。")
    candidates = [motor.target_id] if motor.protocol == "LS" else list(dict.fromkeys(
        [motor.target_id, FACTORY_IDS[motor.protocol]] + list(range(1, 256))))
    if motor.protocol != "LS":
        output("扫描 ID 1～255；默认每个无应答 ID 等待约 1 秒，全扫描约需 4 分钟，可 Ctrl+C 中断。")
    detected = set()
    for count, current in enumerate(candidates, 1):
        if len(candidates) > 1 and (count == 1 or count % 16 == 0):
            output(f"  探测 {count}/{len(candidates)}，ID=0x{current:02X}")
        reply = client.call(motor, QUERY, current)
        if reply.result == CONFIRMED:
            report(reply, output)
            detected.update(reply.detected_ids)
        elif reply.result != 4:  # TIMEOUT means no confirmed answer at this address.
            report(reply, output)
            output("查询未完成，停止扫描；不推断当前 ID。")
            return False
    output("检测到的当前 ID: " + (", ".join(f"0x{i:02X}" for i in sorted(detected)) or "无"))
    if len(detected) > 1:
        output("检测到多颗同协议电机，无法确定所选关节的当前 ID；请隔离后再查询。")
    elif not detected:
        output("未收到匹配 ID 应答；不能据此断定电机不存在。")
    else:
        output("仅检测到一个 ID；关节身份仍以单电机上电及接线确认为准。")
    return len(detected) == 1


def configure(client, motor, current_id, logger, prompt=input, output=print):
    output(describe(motor))
    output(f"当前 ID=0x{current_id:02X} → 目标 ID=0x{motor.target_id:02X}")
    # QUERY is non-mutating and validates node metadata before any CONFIGURE call.
    initial = client.call(motor, QUERY, current_id)
    check_metadata(motor, initial)
    report(initial, output)
    if initial.result != CONFIRMED or set(initial.detected_ids) != {current_id}:
        output("当前 ID 检测未通过，停止写入；请隔离待标定电机并检查当前 ID。")
        return False
    if prompt("确认该 CAN 总线只有这一颗电机上电，且已停止运动。输入 yes 写入，其他输入取消: ").strip().lower() != "yes":
        return False
    response = client.call(motor, CONFIGURE, current_id)
    report(response, output)
    if response.result != CONFIRMED or not response.power_cycle_required:
        output("未完成配置，禁止自动重试。先只读查询旧、新 ID，再决定如何处理。")
        return False
    output("当前应答尚不代表断电后保存成功。")
    if prompt("请给电机断电并重新上电；完成后输入 yes 进行复检，其他输入退出: ").strip().lower() != "yes":
        logger.write(type="power_cycle_not_verified", motor=asdict(motor))
        output("配置已应答，断电保存尚未验收。")
        return False
    logger.write(type="power_cycle_confirmed_by_operator", motor=asdict(motor))
    passed = query_three(client, motor, {motor.target_id}, output)
    logger.write(type="final", motor=asdict(motor), passed=passed)
    output(f"{'PASS' if passed else 'FAIL'}: {motor.name} 断电重启后 ID 复检")
    return passed


def select(motors):
    show_motors(motors)
    while True:
        answer = input("输入 index 配置单颗电机；a 全部 ID 自检；q 退出: ").strip().lower()
        if answer in {"a", "q"}:
            return answer
        if answer.isdigit():
            selected = next((motor for motor in motors if motor.index == int(answer)), None)
            if selected:
                return selected
        print("输入无效")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--motor-index", type=integer)
    mode.add_argument("--check-all", action="store_true")
    parser.add_argument("--current-id", type=integer, help="覆盖出厂 ID，接受 0x 十六进制")
    parser.add_argument("--query", action="store_true", help="只读查询：不填 --current-id 时自动发现当前 ID，填写时仅查询该 ID")
    parser.add_argument("--dry-run", action="store_true", help="只显示计划，不初始化 ROS、不调用服务")
    parser.add_argument("--service-timeout", type=float, default=10.0, help="服务发现/单次调用超时，秒")
    args = parser.parse_args(argv)
    if not math.isfinite(args.service_timeout) or args.service_timeout <= 0:
        parser.error("--service-timeout 必须为有限正数")
    if args.query and args.motor_index is None:
        parser.error("--query 必须同时提供 --motor-index")
    if args.current_id is not None and args.check_all:
        parser.error("--current-id 不能用于 --check-all")
    return args


def main(argv=None):
    client = None
    try:
        args = parse_args(argv)
        motors = load_motors(args.config)
        selected = next((motor for motor in motors if motor.index == args.motor_index), None)
        if args.motor_index is not None and selected is None:
            raise ValueError(f"未配置 index={args.motor_index}")
        if selected and selected.protocol not in FACTORY_IDS:
            raise ValueError(f"{selected.protocol} 暂不支持 ID 标定")
        current_id = args.current_id
        if selected:
            current_id = current_id if current_id is not None else (
                selected.target_id if args.query else FACTORY_IDS[selected.protocol])
            limit = 0x7FF if selected.protocol == "LS" else 0xFF
            if not 1 <= current_id <= limit:
                raise ValueError("当前 ID 超出协议范围")
        if args.dry_run:
            show_motors([selected] if selected else motors)
            print("DRY RUN: 不调用 ROS service、不发送电机命令。")
            if selected and args.query and args.current_id is None:
                print("QUERY: LS 广播查询；RS/MR 扫描 ID 1～255，不执行 ID 写入。")
            elif selected:
                print(f"{'QUERY' if args.query else 'CONFIGURE'}: 当前 0x{current_id:02X} → 目标 0x{selected.target_id:02X}")
            else:
                print("全部 ID 自检" if args.check_all else "交互选择单颗电机；查询、配置、人工断电、三次复检")
            return 0
        if not selected and not args.check_all:
            choice = select(motors)
            if choice == "q":
                return 0
            if choice == "a":
                args.check_all = True
            else:
                selected = choice
                if selected.protocol not in FACTORY_IDS:
                    raise ValueError(f"{selected.protocol} 暂不支持 ID 标定")
                current_id = args.current_id if args.current_id is not None else FACTORY_IDS[selected.protocol]
                if not 1 <= current_id <= (0x7FF if selected.protocol == "LS" else 0xFF):
                    raise ValueError("当前 ID 超出协议范围")
        logger = JsonlLogger(WORKSPACE / "log/motor_id_calibration")
        print(f"日志: {logger.path}")
        client = RosClient(args.service_timeout, logger)
        if args.check_all:
            return 0 if check_all(client, motors) else 1
        if args.query:
            if args.current_id is None:
                return 0 if discover_ids(client, selected) else 1
            response = client.call(selected, QUERY, current_id)
            report(response)
            return 0 if response.result == CONFIRMED else 1
        return 0 if configure(client, selected, current_id, logger) else 1
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, ImportError, yaml.YAMLError) as exc:
        print(f"停止: {exc}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("操作中断；已发出的写入可能仍在执行。查询旧、新 ID，勿自动重试。", file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()


if __name__ == "__main__":
    sys.exit(main())
