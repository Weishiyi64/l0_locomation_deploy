#!/usr/bin/env python3
"""Single-flight ROS worker: fresh pose -> delivered pipeline -> whole trajectory."""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from luvbot_control_interfaces.msg import PolicyObservation

from motion_adapter import (JOINTS, TERMINAL, check_pose, generate, load_limits,
                            open_pipeline, valid_id, validate_request)


class TextMotionNode(Node):
    def __init__(self):
        super().__init__("text_motion_generator")
        self.declare_parameter("model_manifest", "")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("backend", "lite2")
        self.limits = load_limits(self.get_parameter("mapping_file").value)
        self.pipeline = open_pipeline(self.get_parameter("model_manifest").value,
                                      self.get_parameter("backend").value)
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.closing = False
        self.pose = None
        self.request = None
        self.cancelled = False
        self.sent = False
        self.deadline = 0.0
        self.timeout_reported = False
        self.recent = []
        self.cancel_requests = []
        self.status_pub = self.create_publisher(String, "/text_motion/status", 20)
        self.trajectory_pub = self.create_publisher(String, "/text_motion/trajectory", 1)
        self.cancel_pub = self.create_publisher(String, "/text_motion/cancel", 10)
        self.create_subscription(String, "/text_motion/request", self.on_request, 10)
        self.create_subscription(String, "/text_motion/playback_status", self.on_playback_status, 20)
        self.create_subscription(PolicyObservation, "/control/policy/observation", self.on_observation,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.1, self.check_timeout)
        self.worker = threading.Thread(target=self.work, name="text_motion_inference", daemon=False)
        self.worker.start()
        self.get_logger().info("Current ONNX delivery loaded; backend=" + self.pipeline.backend)

    def status(self, identifier, state, detail=""):
        if not rclpy.ok():
            return
        message = String()
        message.data = json.dumps({"request_id": identifier, "state": state, "detail": detail},
                                  ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)

    def cancel(self, identifier):
        if not rclpy.ok():
            return
        message = String()
        message.data = identifier
        self.cancel_pub.publish(message)

    def on_request(self, message):
        identifier = ""
        try:
            if len(message.data.encode("utf-8")) > 16384:
                raise ValueError("Request exceeds 16 KiB")
            request = json.loads(message.data)
            if not isinstance(request, dict) or not valid_id(request.get("request_id")):
                raise ValueError("Invalid request_id")
            identifier = request["request_id"]
            if request.get("command") == "cancel":
                with self.lock:
                    self.cancel_requests = (self.cancel_requests + [identifier])[-256:]
                    self.recent = (self.recent + [identifier])[-256:]
                    if self.request and self.request["request_id"] == identifier:
                        self.cancelled = True
                    self.cancel(identifier)
                return
            with self.lock:
                if identifier in self.recent:
                    self.status(identifier, "duplicate", "Request ID has already been handled")
                    return
                validate_request(request)
                if self.request is not None:
                    raise ValueError("Generator is busy; concurrent requests are refused")
                if self.trajectory_pub.get_subscription_count() < 1 or \
                        self.count_publishers("/text_motion/playback_status") != 1:
                    raise ValueError("Exactly one enabled text-motion controller is required")
                self.fresh_pose()
                self.recent = (self.recent + [identifier])[-256:]
                self.request = request
                self.cancelled = False
                self.sent = False
                self.timeout_reported = False
                self.status(identifier, "generating")
                self.wake.set()
        except Exception as exc:
            self.status(identifier, "rejected", str(exc))

    def on_observation(self, message):
        pose = None
        joints = message.joints
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if message.inference_enabled and 0 <= age <= 0.2 and len(joints.name) == len(joints.position) and \
                len(set(joints.name)) == len(joints.name):
            by_name = dict(zip(joints.name, joints.position))
            try:
                pose = check_pose([by_name[name] for name in JOINTS], self.limits)
            except (KeyError, ValueError):
                pass
        with self.lock:
            self.pose = None if pose is None else (pose, time.monotonic())
            if pose is None:
                self.cancel_generation_if_needed("Observation gate closed during generation")

    def cancel_generation_if_needed(self, detail):
        # Caller holds lock. Keep cancellation sticky across stop/resume and
        # retain the slot until the uninterruptible NPU call has returned.
        if self.request and not self.sent and not self.cancelled:
            self.cancelled = True
            identifier = self.request["request_id"]
            self.cancel(identifier)
            self.status(identifier, "cancelling", detail)

    def fresh_pose(self):
        # Caller holds lock. Receive time adds a steady-clock freshness check.
        if self.pose is None or time.monotonic() - self.pose[1] > 0.2:
            raise ValueError("No fresh calibrated Walking observation")
        return self.pose[0].copy()

    def on_playback_status(self, message):
        try:
            event = json.loads(message.data)
            identifier, state = event["request_id"], event["state"]
            with self.lock:
                matches = self.request and self.request["request_id"] == identifier
                if matches and not self.sent:
                    # A cancel-before-delivery acknowledgement must not free
                    # the generator slot while the old NPU call still runs.
                    return
                if not matches and identifier not in self.cancel_requests:
                    return
                if matches and state in TERMINAL:
                    self.request = None
                    self.sent = False
                if state in TERMINAL and identifier in self.cancel_requests:
                    self.cancel_requests.remove(identifier)
            self.status(identifier, state, event.get("detail", ""))
        except (ValueError, KeyError, TypeError):
            self.get_logger().error("Invalid controller text-motion feedback")

    def check_timeout(self):
        with self.lock:
            if self.pose is None or time.monotonic() - self.pose[1] > 0.2:
                self.cancel_generation_if_needed("Observation stream expired during generation")
            if self.request and self.sent and time.monotonic() >= self.deadline and not self.timeout_reported:
                self.timeout_reported = True
                self.cancelled = True
                identifier = self.request["request_id"]
                self.cancel(identifier)
                # Retain the slot until controller terminal feedback. Never start
                # another motion when cancellation/execution state is unknown.
                self.status(identifier, "cancelling", "Playback timeout; waiting for controller cancellation")

    def work(self):
        while True:
            self.wake.wait()
            self.wake.clear()
            with self.lock:
                if self.closing:
                    return
                request = self.request
            if request is None:
                continue
            identifier = request["request_id"]
            try:
                with self.lock:
                    if self.cancelled:
                        self.request = None
                        self.status(identifier, "cancelled", "Cancelled before inference")
                        continue
                    initial = self.fresh_pose()
                trajectory = generate(self.pipeline, request, initial, self.limits)
                with self.lock:
                    if self.closing or self.cancelled:
                        state, detail = "cancelled", "Generation cancelled; no trajectory sent"
                    elif trajectory is None:
                        state, detail = "no_motion", "All model body-part tokens are zero"
                    else:
                        latest = self.fresh_pose()
                        mask = np.asarray(trajectory["active_joints"], dtype=bool)
                        if np.max(np.abs(latest[mask] - initial[mask])) > 0.15:
                            raise ValueError("Robot pose moved during generation; generate again from its current pose")
                        if self.trajectory_pub.get_subscription_count() < 1 or \
                                self.count_publishers("/text_motion/playback_status") != 1:
                            raise ValueError("Controller connection changed during generation")
                        trajectory["expires_at_ms"] = int(time.time() * 1000) + 2000
                        message = String()
                        message.data = json.dumps(trajectory, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                        if len(message.data.encode("utf-8")) > 2 * 1024 * 1024:
                            raise ValueError("Complete trajectory exceeds 2 MiB transport limit")
                        self.sent = True
                        self.deadline = time.monotonic() + (trajectory["frame_count"] - 1) / 50 + 10
                        self.trajectory_pub.publish(message)
                        self.status(identifier, "submitted", "Complete trajectory sent for controller validation")
                        continue
                    self.request = None
                    self.status(identifier, state, detail)
            except Exception as exc:
                with self.lock:
                    if not self.request or self.request["request_id"] != identifier:
                        continue
                    if self.sent:
                        # A publishing exception does not prove DDS sent nothing.
                        # Cancel and retain the slot until execution is known.
                        self.cancelled = True
                        self.deadline = time.monotonic() + 2
                        self.cancel(identifier)
                        self.status(identifier, "cancelling", "Send state uncertain: " + str(exc))
                    else:
                        self.request = None
                        self.status(identifier, "failed", str(exc))

    def close(self):
        with self.lock:
            self.closing = True
            self.cancelled = True
            if self.request:
                self.cancel(self.request["request_id"])
        self.wake.set()
        self.worker.join()
        self.pipeline.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = TextMotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
