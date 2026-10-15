#!/usr/bin/env python3

from typing import List

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Joy


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class SharedGamepadHeadController(Node):
    def __init__(self) -> None:
        super().__init__("shared_gamepad_head_controller")

        self.declare_parameter("input_joy_topic", "/joy_raw")
        self.declare_parameter("robot_joy_topic", "/joy")
        self.declare_parameter("head_target_topic", "/head_joystick")

        self.declare_parameter("toggle_lt_button", 8)
        self.declare_parameter("toggle_rt_button", 9)
        self.declare_parameter("axis_head_yaw", 0)
        self.declare_parameter("axis_head_pitch", 1)

        self.declare_parameter("robot_zero_axes", [0, 1, 2, 6, 7])
        self.declare_parameter("robot_trigger_axes", [4, 5])
        self.declare_parameter("robot_trigger_neutral", 1.0)

        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("head_target_span", 0.5)

        self.input_joy_topic = str(self.get_parameter("input_joy_topic").value)
        self.robot_joy_topic = str(self.get_parameter("robot_joy_topic").value)
        self.head_target_topic = str(self.get_parameter("head_target_topic").value)

        self.toggle_lt_button = int(self.get_parameter("toggle_lt_button").value)
        self.toggle_rt_button = int(self.get_parameter("toggle_rt_button").value)
        self.axis_head_yaw = int(self.get_parameter("axis_head_yaw").value)
        self.axis_head_pitch = int(self.get_parameter("axis_head_pitch").value)

        self.robot_zero_axes = [int(v) for v in self.get_parameter("robot_zero_axes").value]
        self.robot_trigger_axes = [int(v) for v in self.get_parameter("robot_trigger_axes").value]
        self.robot_trigger_neutral = float(self.get_parameter("robot_trigger_neutral").value)

        self.deadzone = float(self.get_parameter("deadzone").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.head_target_span = clamp(float(self.get_parameter("head_target_span").value), 0.0, 0.5)

        self.head_mode = False
        self.toggle_combo_was_pressed = False

        self.last_axes: List[float] = []
        self.last_buttons: List[int] = []
        self.last_joy = None

        self.joy_sub = self.create_subscription(Joy, self.input_joy_topic, self.on_joy, 10)
        self.robot_joy_pub = self.create_publisher(Joy, self.robot_joy_topic, 10)
        self.head_target_pub = self.create_publisher(Point, self.head_target_topic, 10)

        self.timer_period = 1.0 / max(1.0, self.update_rate_hz)
        self.timer = self.create_timer(self.timer_period, self.on_timer)

        self.get_logger().info(
            "shared_gamepad_head_controller started | "
            f"{self.input_joy_topic} -> {self.robot_joy_topic}, "
            f"head_target={self.head_target_topic}, "
            f"toggle=LT({self.toggle_lt_button})+RT({self.toggle_rt_button}), "
            f"yaw=axis[{self.axis_head_yaw}], pitch=axis[{self.axis_head_pitch}]"
        )

    def on_joy(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_axes = list(msg.axes)
        self.last_buttons = list(msg.buttons)

        toggle_combo_pressed = self.get_button(self.toggle_lt_button) and self.get_button(self.toggle_rt_button)
        toggled = toggle_combo_pressed and not self.toggle_combo_was_pressed
        self.toggle_combo_was_pressed = toggle_combo_pressed

        if toggled:
            self.head_mode = not self.head_mode
            mode_name = "head" if self.head_mode else "robot"
            self.get_logger().info(f"Shared gamepad mode switched to {mode_name}")

        if self.head_mode or toggled:
            self.publish_neutral_robot_joy(msg)
        else:
            self.robot_joy_pub.publish(msg)

    def on_timer(self) -> None:
        if self.head_mode and self.last_joy is not None:
            self.publish_head_target()
            self.publish_neutral_robot_joy(self.last_joy)

    def publish_neutral_robot_joy(self, source: Joy) -> None:
        msg = Joy()
        msg.header = source.header
        msg.axes = [0.0] * len(source.axes)
        msg.buttons = [0] * len(source.buttons)

        for idx in self.robot_zero_axes:
            if 0 <= idx < len(msg.axes):
                msg.axes[idx] = 0.0
        for idx in self.robot_trigger_axes:
            if 0 <= idx < len(msg.axes):
                msg.axes[idx] = self.robot_trigger_neutral

        self.robot_joy_pub.publish(msg)

    def get_axis(self, idx: int) -> float:
        if 0 <= idx < len(self.last_axes):
            value = clamp(float(self.last_axes[idx]), -1.0, 1.0)
            return 0.0 if abs(value) < self.deadzone else value
        return 0.0

    def get_button(self, idx: int) -> bool:
        return 0 <= idx < len(self.last_buttons) and bool(self.last_buttons[idx])

    def publish_head_target(self) -> None:
        yaw_input = self.get_axis(self.axis_head_yaw)
        pitch_input = self.get_axis(self.axis_head_pitch)

        msg = Point()
        # HeadTopicController treats offsets from 0.5 as yaw/pitch increments.
        # Publish continuously so each stick axis behaves as a direction/rate command.
        # Linux joystick axis 0 is positive left; positive yaw turns left.
        msg.x = clamp(0.5 + yaw_input * self.head_target_span, 0.0, 1.0)
        # Linux joystick axis 1 is positive up; head pitch is positive downward.
        msg.y = clamp(0.5 - pitch_input * self.head_target_span, 0.0, 1.0)
        msg.z = 0.0
        self.head_target_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SharedGamepadHeadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
