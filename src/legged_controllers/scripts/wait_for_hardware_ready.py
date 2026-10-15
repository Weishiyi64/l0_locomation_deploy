#!/usr/bin/env python3
"""
Wait for a hardware ready topic (std_msgs/Bool, data=true).
Uses rclpy subscriber - more reliable than ros2 topic echo when check script
starts after nodes (e.g. ~3s delay in sequential launch).
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def main():
    if len(sys.argv) < 4:
        print("Usage: wait_for_hardware_ready.py <topic> <timeout_sec> <description>", file=sys.stderr)
        sys.exit(2)

    topic = sys.argv[1]
    timeout_sec = float(sys.argv[2])
    description = sys.argv[3]

    rclpy.init()
    node = Node('wait_for_hardware_ready')

    received = [False]  # use list to allow closure to modify

    def callback(msg):
        if msg.data:
            received[0] = True

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    sub = node.create_subscription(Bool, topic, callback, qos)
    start = node.get_clock().now()
    rate = node.create_rate(20)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if received[0]:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        if elapsed >= timeout_sec:
            break
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)
