#!/usr/bin/env python3
"""Publish one complete, pre-generated text-motion JSON transaction to the controller."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> None:
    parser = argparse.ArgumentParser(description="向 L0 v5 控制器发送完整文本动作 JSON")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--topic", default="/text_motion/json")
    parser.add_argument("--subscriber-timeout", type=float, default=10.0)
    parser.add_argument("--status-topic", default="/text_motion/status")
    parser.add_argument("--status-timeout", type=float, help="等待播放完成的秒数；默认至少比动作时长多 10 秒")
    args = parser.parse_args()
    if args.subscriber_timeout <= 0.0 or (args.status_timeout is not None and args.status_timeout <= 0.0):
        parser.error("超时时间必须大于 0")

    root = json.loads(args.input.read_text(encoding="utf-8"))
    if root.get("format") != "gmr_upper_body_joint_trajectory_v1" or root.get("robot") != "l0_v5":
        parser.error("输入不是 L0 v5 文本动作 JSON")
    payload = json.dumps(root, ensure_ascii=False, separators=(",", ":"))
    status_timeout = args.status_timeout or max(30.0, root["frame_count"] / root["fps"] + 10.0)

    rclpy.init(args=None)
    node = rclpy.create_node("text_motion_json_publisher")
    statuses = []
    status_subscription = node.create_subscription(
        String,
        args.status_topic,
        lambda message: statuses.append(message.data),
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    publisher = node.create_publisher(
        String,
        args.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    try:
        deadline = time.monotonic() + args.subscriber_timeout
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"未发现控制器订阅者：{args.topic}")
        deadline = time.monotonic() + args.subscriber_timeout
        while node.count_publishers(args.status_topic) == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_publishers(args.status_topic) == 0:
            raise RuntimeError(f"未发现控制器状态发布者：{args.status_topic}")
        # Discard any status seen before this command. Run only one publisher at a time.
        statuses.clear()
        message = String()
        message.data = payload
        publisher.publish(message)
        print(f"已发送 {root['frame_count']} 帧；等待控制器确认和播放完成", flush=True)
        deadline = time.monotonic() + status_timeout
        received = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            while statuses:
                status = statuses.pop(0)
                print(f"/text_motion/status: {status}", flush=True)
                if status.startswith("received:"):
                    received = True
                elif status.startswith(("rejected:", "stopped:")):
                    raise RuntimeError(f"文本动作未完成：{status}")
                elif status == "completed":
                    print("文本动作播放完成")
                    return
        if not received:
            raise RuntimeError("控制器未确认收到完整 JSON；检查 ROS_DOMAIN_ID、DDS 和控制器日志")
        raise RuntimeError("控制器已收到 JSON，但播放未在超时时间内完成；检查 /text_motion/status")
    finally:
        node.destroy_subscription(status_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
