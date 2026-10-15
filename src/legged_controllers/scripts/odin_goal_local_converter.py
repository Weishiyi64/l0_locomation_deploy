#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Convert an RViz goal from the Odin odometry frame to robot-local XY."""

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray


def quaternion_to_yaw(orientation) -> float:
    """Return planar yaw from a geometry_msgs quaternion."""
    qx = float(orientation.x)
    qy = float(orientation.y)
    qz = float(orientation.z)
    qw = float(orientation.w)

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def world_delta_to_local(
    delta_world_x: float,
    delta_world_y: float,
    current_yaw: float,
) -> Tuple[float, float]:
    """Rotate a world-frame displacement into the robot's local frame."""
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)

    local_x = cos_yaw * delta_world_x + sin_yaw * delta_world_y
    local_y = -sin_yaw * delta_world_x + cos_yaw * delta_world_y
    return local_x, local_y


class OdinGoalLocalConverter(Node):
    """Publish an RViz goal as robot-local XY using Odin odometry."""

    def __init__(self) -> None:
        super().__init__("odin_goal_local_converter")

        self.declare_parameter(
            "odom_topic",
            "/manifold/ODIN2/device0/odometry",
        )
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("output_topic", "/rl_policy_command")
        self.declare_parameter("custom_float", 0.5)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("odom_timeout", 2.0)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.custom_float = float(self.get_parameter("custom_float").value)
        self.publish_rate = max(
            0.1,
            float(self.get_parameter("publish_rate").value),
        )
        self.goal_tolerance = max(
            0.0,
            float(self.get_parameter("goal_tolerance").value),
        )
        self.odom_timeout = max(
            0.0,
            float(self.get_parameter("odom_timeout").value),
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_frame = ""
        self.odom_message_time = None
        self.last_odom_receive_time = None
        self.have_odom = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_frame = ""
        self.have_goal = False
        self.goal_reached = False

        self.last_warning_time = 0.0

        self.command_publisher = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_command,
        )

        self.get_logger().info(
            "Odin goal converter ready: "
            f"odom={self.odom_topic}, "
            f"goal={self.goal_topic}, "
            f"output={self.output_topic}, "
            f"rate={self.publish_rate:.1f}Hz, "
            f"tolerance={self.goal_tolerance:.3f}m, "
            f"odom_timeout={self.odom_timeout:.3f}s"
        )

    def throttled_warning(self, text: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warning_time >= 1.0:
            self.get_logger().warning(text)
            self.last_warning_time = now

    def odom_callback(self, message: Odometry) -> None:
        self.current_x = float(message.pose.pose.position.x)
        self.current_y = float(message.pose.pose.position.y)
        self.current_yaw = quaternion_to_yaw(message.pose.pose.orientation)
        self.odom_frame = str(message.header.frame_id)
        self.odom_message_time = Time.from_msg(message.header.stamp)
        self.last_odom_receive_time = self.get_clock().now()
        self.have_odom = True

    def goal_callback(self, message: PoseStamped) -> None:
        self.goal_x = float(message.pose.position.x)
        self.goal_y = float(message.pose.position.y)
        self.goal_yaw = quaternion_to_yaw(message.pose.orientation)
        self.goal_frame = str(message.header.frame_id)
        self.have_goal = True
        self.goal_reached = False

        self.get_logger().info(
            "New RViz goal: "
            f"frame='{self.goal_frame}', "
            f"x={self.goal_x:.3f}, y={self.goal_y:.3f}, "
            f"yaw={self.goal_yaw:.3f}"
        )

    def publish_xy(self, local_x: float, local_y: float) -> None:
        command = Float32MultiArray()
        command.data = [
            float(local_x),
            float(local_y),
            float(self.custom_float),
        ]
        self.command_publisher.publish(command)

    def publish_stop(self) -> None:
        self.publish_xy(0.0, 0.0)

    def publish_command(self) -> None:
        # Before the first RViz goal, do not publish control commands.
        if not self.have_goal:
            return

        if not self.have_odom or self.last_odom_receive_time is None:
            self.publish_stop()
            self.throttled_warning("Command stopped: waiting for Odin odometry")
            return

        odom_age = (self.get_clock().now() - self.last_odom_receive_time).nanoseconds * 1e-9
        if odom_age > self.odom_timeout:
            self.publish_stop()
            self.throttled_warning(
                "Command stopped: Odin odometry is stale " f"({odom_age:.3f}s > {self.odom_timeout:.3f}s)"
            )
            return

        if not self.goal_frame or self.goal_frame != self.odom_frame:
            self.publish_stop()
            self.throttled_warning(
                "Command stopped: frame mismatch "
                f"goal='{self.goal_frame}' odom='{self.odom_frame}'. "
                "Set the RViz Fixed Frame to the Odin odometry frame."
            )
            return

        if self.goal_reached:
            self.publish_stop()
            return

        delta_world_x = self.goal_x - self.current_x
        delta_world_y = self.goal_y - self.current_y
        distance = math.hypot(delta_world_x, delta_world_y)

        if distance <= self.goal_tolerance:
            self.goal_reached = True
            self.publish_stop()
            self.get_logger().info(
                "RViz goal reached: " f"distance={distance:.3f}m, " f"tolerance={self.goal_tolerance:.3f}m"
            )
            return

        local_x, local_y = world_delta_to_local(
            delta_world_x,
            delta_world_y,
            self.current_yaw,
        )
        self.publish_xy(local_x, local_y)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdinGoalLocalConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
