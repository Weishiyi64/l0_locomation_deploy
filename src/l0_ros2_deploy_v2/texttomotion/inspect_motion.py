#!/usr/bin/env python3
"""Compare text-motion status with commanded and measured MuJoCo joints."""

import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MotorCommand, MotorStateArray
from std_msgs.msg import String


JOINTS = {
    12: "left_shoulder_pitch",
    13: "left_shoulder_roll",
    14: "left_elbow_roll",
    15: "right_shoulder_pitch",
    16: "right_shoulder_roll",
    17: "right_elbow_roll",
    18: "neck_yaw",
    19: "neck_pitch",
    20: "neck_roll",
}


def main() -> None:
    parser = argparse.ArgumentParser(description="观察文本动作时关节目标和 MuJoCo 反馈")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--wbc-action", help="观察指定 WBC 动作 ID 的 started/completed 区间")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds 必须大于 0")

    rclpy.init()
    node = rclpy.create_node("text_motion_inspector")
    latest_command = {}
    latest_state = {}
    command_range = {index: [] for index in JOINTS}
    state_range = {index: [] for index in JOINTS}
    active_command_range = {index: [] for index in JOINTS}
    active_state_range = {index: [] for index in JOINTS}
    active = False

    def on_command(message: MotorCommand) -> None:
        latest_command.clear()
        latest_command.update({int(target.route.motor_type): target for target in message.commands})
        for index in JOINTS:
            if index in latest_command:
                command_range[index].append(latest_command[index].q)
                if active:
                    active_command_range[index].append(latest_command[index].q)

    def on_state(message: MotorStateArray) -> None:
        latest_state.clear()
        latest_state.update({int(state.route.motor_type): state for state in message.states})
        for index in JOINTS:
            if index in latest_state:
                state_range[index].append(latest_state[index].q)
                if active and latest_state[index].valid:
                    active_state_range[index].append(latest_state[index].q)

    def on_status(message: String) -> None:
        nonlocal active
        if args.wbc_action:
            try:
                status = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if not isinstance(status, dict):
                return
            if status.get("action_id") != args.wbc_action:
                return
            state = status.get("status", "")
        else:
            state = message.data
        print("status:", state, flush=True)
        if state == "started":
            active = True
        elif state in ("completed", "failed", "stopped", "rejected") or state.startswith(("stopped:", "rejected:")):
            active = False

    subscriptions = [
        node.create_subscription(
            MotorCommand, "/sim/actuator/motor/command", on_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        ),
        node.create_subscription(
            MotorStateArray, "/sim/actuator/motor/state", on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        ),
        node.create_subscription(
            String, "/system/state_machine/wbc_action_fdb" if args.wbc_action else "/text_motion/status", on_status,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        ),
    ]
    action_label = f"WBC 动作 {args.wbc_action}" if args.wbc_action else "文本动作"
    print(f"开始观察；现在在另一个终端发送{action_label}。q 为电机坐标，比较目标与反馈即可。", flush=True)
    deadline = time.monotonic() + args.seconds
    next_print = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() < next_print:
                continue
            next_print = time.monotonic() + 0.5
            for index, name in JOINTS.items():
                target = latest_command.get(index)
                state = latest_state.get(index)
                if target is not None and state is not None:
                    print(f"{name}: target={target.q:+.3f} actual={state.q:+.3f} "
                          f"kp={target.kp:.2f} valid={state.valid}", flush=True)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print("观测范围（目标 / 实际，单位 rad）：")
    for index, name in JOINTS.items():
        commanded = command_range[index]
        measured = state_range[index]
        if commanded and measured:
            print(f"{name}: [{min(commanded):+.3f}, {max(commanded):+.3f}] / "
                  f"[{min(measured):+.3f}, {max(measured):+.3f}]")
        else:
            print(f"{name}: 缺少命令或反馈；检查 MuJoCo 是否仍在运行")
    print(f"仅{action_label}播放期间的范围（started 到 completed，目标 / 有效反馈，单位 rad）：")
    for index, name in JOINTS.items():
        commanded = active_command_range[index]
        measured = active_state_range[index]
        if commanded and measured:
            print(f"{name}: [{min(commanded):+.3f}, {max(commanded):+.3f}] / "
                  f"[{min(measured):+.3f}, {max(measured):+.3f}]")
        else:
            print(f"{name}: 未收到播放期间的命令或有效反馈")


if __name__ == "__main__":
    main()
