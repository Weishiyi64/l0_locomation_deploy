#!/usr/bin/env python3
"""Publish one absolute odom goal continuously at 5 Hz."""

import argparse
import math
import time

import rclpy
from std_msgs.msg import Float32MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-x', '--start-x', dest='goal_x', type=float, default=1.0)
    parser.add_argument('--goal-y', '--start-y', dest='goal_y', type=float, default=0.5)
    parser.add_argument('--target-rad', '--start-rad', dest='target_rad', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--speed', type=float, default=0.0)
    parser.add_argument('--turn-speed', type=float, default=0.0)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in vars(args).values()) or args.duration <= 0.0:
        parser.error('all values must be finite and duration must be positive')
    if args.speed != 0.0 and not 0.3 <= args.speed <= 0.35:
        parser.error('speed must be zero (default) or between 0.3 and 0.35 m/s')
    if not 0.0 <= abs(args.turn_speed) <= 1.57:
        parser.error('turn speed magnitude must be between 0 and 1.57 rad/s')

    rclpy.init()
    node = rclpy.create_node('follow_goal_test')
    publisher = node.create_publisher(Float32MultiArray, '/navigation_goal', 1)
    result = 0
    try:
        deadline = time.monotonic() + 3.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            node.get_logger().error('No /navigation_goal subscriber; start goal_to_planner first')
            result = 1
        else:
            steps = max(1, round(args.duration * 5.0))
            started = time.monotonic()
            for step in range(steps + 1):
                message = Float32MultiArray()
                message.data = [
                    args.goal_x,
                    args.goal_y,
                    args.speed,
                    args.target_rad,
                    args.turn_speed,
                    1.0,
                ]
                publisher.publish(message)
                print(
                    f'\rgoal_odom=({message.data[0]:+.3f},{message.data[1]:+.3f}) '
                    f'target_rad={message.data[3]:+.3f} visible=1',
                    end='',
                    flush=True,
                )
                if step < steps:
                    next_tick = started + (step + 1) / 5.0
                    time.sleep(max(0.0, next_tick - time.monotonic()))
            print()
    except KeyboardInterrupt:
        print('\nInterrupted; sending Hold')
    finally:
        if rclpy.ok():
            stop = Float32MultiArray(data=[0.0] * 6)
            for _ in range(3):
                publisher.publish(stop)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.15)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
