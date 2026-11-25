#!/usr/bin/env python3
"""Capture the current L0 v5 upper-body joint pose for run_offline.sh."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from luvbot_control_interfaces.msg import PolicyObservation


JOINT_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint",
]
LIMITS = [
    (-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571),
    (-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571),
    (-1.571, 1.571), (-0.34906585, 0.17453293), (-0.297, 0.297),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="读取当前 L0 v5 上半身姿态，供 ONNX init_pose 使用")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/control/policy/observation")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout 必须大于 0")

    rclpy.init()
    node = rclpy.create_node("text_motion_init_pose_capture")
    captured = []

    def on_state(message: PolicyObservation) -> None:
        # Shared controller snapshot already contains calibrated joint angles.
        # Real HPM routes must never be decoded as simulated motor_type indices.
        joints = message.joints
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) * 1e-9
        if not message.inference_enabled or not 0.0 <= age <= 0.2 or                 len(joints.name) != len(joints.position) or len(set(joints.name)) != len(joints.name):
            return
        by_name = dict(zip(joints.name, joints.position))
        pose = []
        for name, (lower, upper) in zip(JOINT_NAMES, LIMITS):
            value = by_name.get(name)
            if value is None or not math.isfinite(value) or not lower - 0.01 <= value <= upper + 0.01:
                return
            pose.append(max(lower, min(upper, value)))
        captured.append(pose)

    subscription = node.create_subscription(
        PolicyObservation, args.topic, on_state,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    deadline = time.monotonic() + args.timeout
    try:
        while not captured and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not captured:
            raise RuntimeError(f"未收到有效的 9 关节反馈：{args.topic}；检查 MuJoCo 和 ROS_DOMAIN_ID")
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    result = {
        "robot": "l0_v5",
        "angle_unit": "rad",
        "joint_names": JOINT_NAMES,
        "joint_positions": captured[0],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(f"当前 AMP 初始姿态：{args.output.resolve()}")
    for name, value in zip(JOINT_NAMES, captured[0]):
        print(f"  {name}: {value:+.4f} rad")


if __name__ == "__main__":
    main()
