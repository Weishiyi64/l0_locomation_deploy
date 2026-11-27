#!/usr/bin/env python3
"""Topic-only MIT motor demo; intentionally independent of hpm_motor."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorStateArray, MotorTarget


class MotorDemo(Node):
    def __init__(
        self, motor_type: int, rate_hz: float, duration: float, wait_feedback: bool,
    ) -> None:
        super().__init__("motor_demo")
        self.motor_type = motor_type
        self.period = 1.0 / rate_hz
        self.duration = duration
        self.wait_feedback = wait_feedback
        self.sequence_id = 0
        self.phase = "enable"
        self.phase_started = time.monotonic()
        self.initial_q = None
        self.command_pub = self.create_publisher(
            MotorCommand, "/actuator/motor/command", 10)
        self.enable_pub = self.create_publisher(
            MotorEnableCommand, "/actuator/motor/enable", 10)
        self.state_sub = self.create_subscription(
            MotorStateArray, "/actuator/motor/state", self.on_state,
            qos_profile_sensor_data)
        self.timer = self.create_timer(self.period, self.tick)
        self.publish_enable(True)

    def next_sequence(self) -> int:
        self.sequence_id = (self.sequence_id + 1) & 0xFFFFFFFF
        return self.sequence_id

    def publish_enable(self, enabled: bool) -> None:
        message = MotorEnableCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.sequence_id = self.next_sequence()
        message.motor_types = [self.motor_type]
        message.enabled = [enabled]
        self.enable_pub.publish(message)

    def publish_mit(self, q: float, dq: float, kp: float, kd: float) -> None:
        target = MotorTarget()
        target.route.motor_type = self.motor_type
        target.q = q
        target.dq = dq
        target.tau = 0.0
        target.kp = kp
        target.kd = kd

        message = MotorCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.sequence_id = self.next_sequence()
        message.control_mode = MotorCommand.CONTROL_MODE_MIT
        message.commands = [target]
        self.command_pub.publish(message)

    def on_state(self, message: MotorStateArray) -> None:
        for state in message.states:
            if state.valid and state.route.motor_type == self.motor_type:
                if self.initial_q is None:
                    self.initial_q = state.q
                    self.phase = "run"
                    self.phase_started = time.monotonic()
                    self.get_logger().info(
                        f"feedback received for motor_type={self.motor_type}, q0={state.q:.4f}")
                return

    def tick(self) -> None:
        elapsed = time.monotonic() - self.phase_started
        if self.phase == "enable":
            self.publish_mit(0.0, 0.0, 0.0, 0.0)
            if elapsed >= 0.1:
                self.publish_enable(True)
                if not self.wait_feedback:
                    self.initial_q = 0.0
                    self.phase = "run"
                    self.phase_started = time.monotonic()
                    self.get_logger().info(
                        f"no feedback, starting motor_type={self.motor_type} q0=0")
                    return
                self.phase_started = time.monotonic()
            return

        q0 = 0.0 if self.initial_q is None else self.initial_q
        if self.phase == "run":
            if elapsed < self.duration:
                omega = math.pi
                offset = 0.3 * (1.0 - math.cos(omega * elapsed))
                velocity = 0.3 * omega * math.sin(omega * elapsed)
                self.publish_mit(q0 + offset, velocity, 4.0, 0.5)
                return
            self.phase = "hold"
            self.phase_started = time.monotonic()

        if self.phase == "hold":
            if time.monotonic() - self.phase_started < 0.2:
                self.publish_mit(q0, 0.0, 4.0, 0.5)
                return
            self.publish_enable(False)
            self.get_logger().info(f"demo complete, disabled motor_type={self.motor_type}")
            self.timer.cancel()
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS topic-only HPM MIT motor demo")
    parser.add_argument("--motor-type", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=1000.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--no-feedback", action="store_true")
    args, ros_args = parser.parse_known_args()
    if args.rate_hz <= 0.0 or args.duration <= 0.0:
        parser.error("--rate-hz and --duration must be positive")
    if args.motor_type < 0 or args.motor_type > 255:
        parser.error("--motor-type must be 0..255")

    rclpy.init(args=ros_args)
    node = MotorDemo(
        args.motor_type, args.rate_hz, args.duration, wait_feedback=not args.no_feedback)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_enable(False)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
