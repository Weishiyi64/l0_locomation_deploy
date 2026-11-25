#!/usr/bin/env python3
"""Capture a stationary AMP pose for packaging text JSON as WBC references."""

import argparse
import json
import math
import os
from pathlib import Path
from statistics import median, pstdev
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from luvbot_control_interfaces.msg import PolicyObservation


JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if not 1.0 <= args.seconds <= 10.0 or args.timeout <= args.seconds:
        parser.error("require 1 <= --seconds <= 10 and --timeout > --seconds")

    rclpy.init()
    node = rclpy.create_node("text_wbc_amp_stand_capture")
    samples = []
    last_stamp = None

    def on_observation(message):
        nonlocal last_stamp
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) * 1e-9
        if not message.inference_enabled or not 0.0 <= age <= 0.2 or stamp == last_stamp:
            return
        velocity = message.velocity_command
        if max(abs(velocity.linear.x), abs(velocity.linear.y), abs(velocity.angular.z)) > 0.05:
            return
        names, positions = message.joints.name, message.joints.position
        if len(names) != len(positions) or len(set(names)) != len(names):
            return
        by_name = dict(zip(names, positions))
        try:
            joints = [float(by_name[name]) for name in JOINTS]
        except (KeyError, TypeError, ValueError):
            return
        base = message.base_pose
        xyz = [float(base.position.x), float(base.position.y), float(base.position.z)]
        quat = [float(base.orientation.x), float(base.orientation.y),
                float(base.orientation.z), float(base.orientation.w)]
        if not all(map(math.isfinite, joints + xyz + quat)) or math.sqrt(sum(v*v for v in quat)) < 0.5:
            return
        if samples and sum(a*b for a, b in zip(quat, samples[0][2])) < 0:
            quat = [-v for v in quat]
        samples.append((joints, xyz, quat))
        last_stamp = stamp

    subscription = node.create_subscription(
        PolicyObservation, "/control/policy/observation", on_observation,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.timeout and (time.monotonic() - start < args.seconds or len(samples) < 50):
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if len(samples) < 50:
        raise RuntimeError("fewer than 50 fresh stationary AMP observations; check Walking/ROS domain")

    def column(group, count):
        return [[sample[group][index] for sample in samples] for index in range(count)]

    joints, base, quaternion = column(0, 21), column(1, 3), column(2, 4)
    joint_std = [pstdev(values) for values in joints]
    base_std = [pstdev(values) for values in base]
    if max(joint_std) > 0.05 or max(base_std) > 0.03:
        raise RuntimeError("AMP was not stationary enough for a WBC reference")
    q = [median(values) for values in quaternion]
    norm = math.sqrt(sum(value*value for value in q))
    q = [value / norm for value in q]
    result = {
        "schema": "l0_v5_amp_stand_profile_v1",
        "robot_type": "l0_v5",
        "source_policy": "l0_v5_amp_07090",
        "capture_environment": "mujoco",
        "capture_ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "sample_count": len(samples),
        "duration_seconds": time.monotonic() - start,
        "command_velocity_max_abs": 0.05,
        "quaternion_order": "xyzw",
        "joint_names": list(JOINTS),
        "base_position_median_m": [median(values) for values in base],
        "base_position_std_m": base_std,
        "base_quaternion_median_xyzw": q,
        "joint_position_median_rad": [median(values) for values in joints],
        "joint_position_std_rad": joint_std,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(f"AMP 站姿已保存：{args.output}；{len(samples)} 帧")
    print(f"neck_pitch_joint: {result['joint_position_median_rad'][19]:+.4f} rad")


if __name__ == "__main__":
    main()
