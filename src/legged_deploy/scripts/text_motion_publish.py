#!/usr/bin/env python3
"""Publish one already-complete motion JSON using the ROS system Python."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--topic', default='/text_motion/json')
    parser.add_argument('--subscriber-timeout', type=float, default=10.0)
    args = parser.parse_args()

    # Parse and re-encode before publication so truncated/partially-written
    # files can never reach the WBC receiver.
    root = json.loads(args.input.read_text(encoding='utf-8'))
    payload = json.dumps(root, ensure_ascii=False, separators=(',', ':'))

    rclpy.init(args=None)
    node = rclpy.create_node('text_motion_json_publisher')
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
            raise RuntimeError(f'{args.subscriber_timeout:g} 秒内未发现 WBC 订阅者：{args.topic}')
        message = String()
        message.data = payload
        publisher.publish(message)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
