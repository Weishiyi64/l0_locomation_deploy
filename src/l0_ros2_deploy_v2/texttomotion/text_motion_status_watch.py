#!/usr/bin/env python3
"""Forward received text JSON and playback events to the conda viewer."""

import json

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("text_motion_reference_watch")

    def on_status(message: String) -> None:
        if message.data == "started":
            print("STARTED", flush=True)
        elif message.data == "completed":
            print("COMPLETED", flush=True)

    def on_trajectory(message: String) -> None:
        # One escaped UTF-8 line preserves the exact complete ROS payload.
        # The viewer validates it before replacing its current reference.
        print("TRAJECTORY\t" + json.dumps(message.data, ensure_ascii=False), flush=True)

    subscription = node.create_subscription(
        String, "/text_motion/status", on_status,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    trajectory_subscription = node.create_subscription(
        String, "/text_motion/json", on_trajectory,
        QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
    )
    print("READY", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_subscription(trajectory_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
