#!/usr/bin/env python3
"""Board NPU generation -> complete JSON -> existing controller/WBC lifecycle."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import uuid

import numpy as np
import yaml

from motion_adapter import SOURCE_SHA256, generate, load_limits, open_pipeline


def bundle_contract(path, base_action):
    path = Path(path).resolve()
    graph = json.loads(path.read_text(encoding="utf-8"))
    if graph.get("format") != "wbc_lora_graph_v1":
        raise ValueError("Expected the same shared WBC manifest used by LoRA")
    action = next(a for a in graph["actions"] if a["action_id"] == base_action)
    if action.get("parameter_source") not in ("base", "lora"):
        raise ValueError("Text WBC requires a complete Base or LoRA parameter pack")
    config = (path.parent / action["config"]).resolve()
    if not config.is_relative_to(path.parent) or hashlib.sha256(config.read_bytes()).hexdigest() != action["config_sha256"]:
        raise ValueError("Invalid Base config path/hash")
    names = yaml.safe_load(config.read_text())["RobotCfg"]["joint_order_mj"]
    if len(names) != 21 or len(set(names)) != 21:
        raise ValueError("Base config must use the 21-joint WBC ABI")
    return graph, names


def full_limits(mapping, names):
    motors = yaml.safe_load(Path(mapping).read_text())["motors"]
    result = []
    for name in names:
        matched = [m for m in motors if m["joint_name"] == name]
        if len(matched) != 1 or matched[0]["enabled"] is not True:
            raise ValueError("Missing/duplicate/disabled mapping joint: " + name)
        lo, hi = float(matched[0]["joint_lower"]), float(matched[0]["joint_upper"])
        if not np.isfinite([lo, hi]).all() or lo >= hi:
            raise ValueError("Invalid mapping limits")
        result.append((lo, hi))
    return np.asarray(result, dtype=np.float64)


def check_snapshot(snapshot, names, limits):
    if snapshot["joint_names"] != names:
        raise ValueError("Snapshot joint order differs from WBC Base")
    joints = np.asarray(snapshot["joint_positions"], dtype=np.float64)
    position = np.asarray(snapshot["base_position"], dtype=np.float64)
    orientation = np.asarray(snapshot["base_quaternion_wxyz"], dtype=np.float64)
    if joints.shape != (21,) or position.shape != (3,) or orientation.shape != (4,) or \
            not all(np.isfinite(x).all() for x in (joints, position, orientation)):
        raise ValueError("Invalid full-body snapshot dimensions/values")
    if np.any(joints < limits[:, 0] - 1e-6) or np.any(joints > limits[:, 1] + 1e-6):
        raise ValueError("Captured joint pose exceeds robot limits")
    # Ordinary WBC accepts a finite observation position, including z=0 when
    # odometry is absent. Its reference model supplies the target root pose.
    norm_squared = float(orientation @ orientation)
    if norm_squared <= 1e-12 or abs(norm_squared - 1) > 0.1:
        raise ValueError("Invalid IMU quaternion under the standard WBC observation contract")
    # Same roll/pitch-only reference convention as prepare_json_batch. The
    # existing WBC yaw anchor restores the robot's actual heading at prepare.
    w, x, y, z = orientation / np.linalg.norm(orientation)
    roll = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = np.arcsin(np.clip(2 * (w*y - z*x), -1, 1))
    cr, sr, cp, sp = np.cos(roll/2), np.sin(roll/2), np.cos(pitch/2), np.sin(pitch/2)
    result = dict(snapshot)
    result["joint_positions"] = np.clip(joints, limits[:, 0], limits[:, 1]).tolist()
    result["base_position"] = position.tolist()
    result["base_quaternion_wxyz"] = [float(cr*cp), float(sr*cp), float(cr*sp), float(-sr*sp)]
    return result


def make_document(trajectory, snapshot, graph):
    if trajectory is None:
        return None
    return {
        "request_id": trajectory["request_id"], "format": "gmr_upper_body_joint_trajectory_v1",
        "robot": "l0_v5", "angle_unit": "rad", "fps": 50,
        "coordinate_contract": "robot_anatomical_v1", "joint_names": trajectory["joint_names"],
        "frame_count": trajectory["frame_count"],
        "frames": [{"frame_index": i, "time": i / 50, "joint_positions": row}
                   for i, row in enumerate(trajectory["positions"])],
        "source_zip_sha256": SOURCE_SHA256,
        "wbc_base_checkpoint_sha256": graph["base_checkpoint_sha256"],
        "wbc_normalization_sha256": graph["normalization_sha256"],
        "wbc_start_state": snapshot,
        "root_reference_source": snapshot.get("root_reference_source", "snapshot"),
        "wbc_base_action": snapshot.get("wbc_base_action", ""),
        "model_diagnostics": dict(trajectory["model_diagnostics"], token_ids=trajectory["token_ids"],
                                  material_ids=trajectory["material_ids"], backend=trajectory["backend"],
                                  inference_ms=trajectory["inference_ms"]),
        "generation_request": trajectory["generation_request"],
        "safety_adjustments": trajectory["limit_adjustments"],
        "expires_at_ms": int(time.time() * 1000) + 5000,
    }


class RosTransaction:
    def __init__(self, names, root_height, base_action="UpperBody_Enhanced", simulation=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from luvbot_control_interfaces.msg import PolicyObservation
        self.rclpy, self.String = rclpy, String
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("text_wbc_transaction_" + uuid.uuid4().hex[:8])
        self.names, self.root_height, self.base_action = names, root_height, base_action
        self.simulation = simulation
        self.lock = threading.Lock()
        self.snapshot = None
        self.invalidated = False
        self.events = queue.Queue()
        self.json_pub = self.node.create_publisher(String, "/text_motion/json", 1)
        self.brain_pub = self.node.create_publisher(String, "/algorithm/brain/wbc_action_cmd", 10)
        self.cancel_pub = self.node.create_publisher(String, "/text_motion/cancel", 10)
        for topic, source in [("/text_motion/wbc_status", "worker"),
                              ("/system/state_machine/wbc_action_fdb", "controller")]:
            self.node.create_subscription(String, topic, lambda msg, src=source: self.on_status(src, msg), 20)
        self.node.create_subscription(PolicyObservation, "/control/policy/observation", self.on_observation,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.executor = SingleThreadedExecutor()
        self.node.create_timer(0.05, self.check_observation_timeout)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=False)
        self.thread.start()

    def on_status(self, source, message):
        try:
            self.events.put((source, json.loads(message.data)))
        except ValueError:
            pass

    def on_observation(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) * 1e-9
        valid = message.inference_enabled and 0 <= age <= 0.2 and \
            len(message.joints.name) == len(message.joints.position) and \
            len(set(message.joints.name)) == len(message.joints.name)
        snapshot = None
        if valid:
            try:
                by_name = dict(zip(message.joints.name, message.joints.position))
                position = message.base_pose.position
                height = position.z
                if height <= 0.05 and self.root_height is not None:
                    height = self.root_height
                q = message.imu.orientation
                snapshot = {"joint_names": self.names, "joint_positions": [by_name[n] for n in self.names],
                            "base_position": [position.x, position.y, height],
                            "base_quaternion_wxyz": [q.w, q.x, q.y, q.z],
                            "captured_stamp_ns": stamp, "root_height_override": self.root_height,
                            "root_reference_source": "wbc_base_action" if self.root_height is None and not self.simulation else "snapshot",
                            "wbc_base_action": self.base_action}
            except KeyError:
                pass
        with self.lock:
            if snapshot is None:
                self.invalidated = True
            self.snapshot = None if snapshot is None else (snapshot, time.monotonic())

    def current_snapshot(self):
        with self.lock:
            if self.snapshot is None or time.monotonic() - self.snapshot[1] > 0.2:
                raise RuntimeError("No fresh enabled policy observation")
            return self.snapshot[0]

    def check_observation_timeout(self):
        with self.lock:
            if self.snapshot is None or time.monotonic() - self.snapshot[1] > 0.2:
                self.invalidated = True

    def reset_observation_gate(self):
        with self.lock:
            self.invalidated = False

    def check_observation_gate(self):
        with self.lock:
            if self.invalidated:
                raise RuntimeError("Observation gate closed during generation; late result discarded")

    def wait_endpoints(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.json_pub.get_subscription_count() > 0 and self.brain_pub.get_subscription_count() > 0 and \
                    self.node.count_publishers("/text_motion/wbc_status") == 1 and \
                    self.node.count_publishers("/system/state_machine/wbc_action_fdb") == 1:
                try:
                    return self.current_snapshot()
                except RuntimeError:
                    pass
            time.sleep(0.05)
        raise RuntimeError("Text JSON worker, controller feedback and live observation must all be available")

    def publish(self, publisher, payload):
        message = self.String()
        message.data = payload if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False, allow_nan=False)
        publisher.publish(message)

    def execute(self, document, timeout):
        identifier = document["request_id"]
        self.publish(self.json_pub, document)
        ready_deadline = time.monotonic() + 7
        deadline = time.monotonic() + timeout
        commanded = False
        timestamp = 0
        while time.monotonic() < deadline:
            try:
                source, event = self.events.get(timeout=0.1)
            except queue.Empty:
                if not commanded and time.monotonic() > ready_deadline:
                    raise RuntimeError("Complete JSON admission timed out")
                continue
            if event.get("request_id", event.get("action_id")) != identifier:
                continue
            print(json.dumps(dict(event, source=source), ensure_ascii=False), flush=True)
            state = event.get("state", event.get("status"))
            if source == "worker" and state == "ready" and not commanded:
                self.check_observation_gate()
                self.current_snapshot()
                timestamp = int(time.time() * 1000)
                self.publish(self.brain_pub, {"timestamp_ms": timestamp, "action_id": identifier})
                commanded = True
            elif source == "controller" and event.get("timestamp_ms") == timestamp:
                if state in ("completed", "cancelled"):
                    return state
                if state in ("rejected", "failed"):
                    raise RuntimeError("Controller terminal state: " + state)
            elif source == "worker" and state == "duplicate" and commanded:
                continue
            elif source == "worker" and state == "cancelled" and commanded:
                continue  # Wait for the controller's actual WBC -> AMP terminal feedback.
            elif source == "worker" and state in ("rejected", "cancelled", "duplicate"):
                raise RuntimeError("JSON admission failed: " + state + ": " + event.get("detail", ""))
        raise RuntimeError("WBC completion timed out")

    def cancel(self, identifier):
        self.publish(self.cancel_pub, identifier)
        from rclpy.duration import Duration
        self.cancel_pub.wait_for_all_acked(Duration(seconds=2))

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()
        self.rclpy.shutdown()


def main(simulation=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--base-action", default="UpperBody_Enhanced",
                        help="Shared WBC parameter/reference action (default: UpperBody_Enhanced)")
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--text", default="")
    parser.add_argument("--tokens", nargs=3, type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--return-frames", type=int, choices=(-1, 30, 60, 90), default=-1)
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--limit-policy", choices=("reject", "clip"), default="reject")
    parser.add_argument("--backend", choices=("lite2", "onnx-reference"), default="onnx-reference" if simulation else "lite2")
    parser.add_argument("--snapshot", type=Path, help="Offline generation only: explicit full-body snapshot JSON")
    parser.add_argument("--root-height", type=float, help="Optional explicit reference height; default reuses the WBC Base reference")
    parser.add_argument("--execute", action="store_true", help="After writing JSON, request the standard WBC controller lifecycle")
    parser.add_argument("--timeout", type=float, default=130)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new JSON path")
    if simulation and (os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "77"):
        parser.error("MuJoCo text execution requires ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID=77")
    if simulation and args.root_height is not None:
        parser.error("MuJoCo uses measured live base height; root-height override is not supported")
    if args.execute and (args.snapshot is not None or args.backend != ("onnx-reference" if simulation else "lite2")):
        parser.error("execution requires fresh ROS observations and board Lite2 inference")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be finite and positive")
    if args.root_height is not None and (not math.isfinite(args.root_height) or not 0.05 < args.root_height < 2):
        parser.error("root-height must be finite and between 0.05 and 2 metres")
    graph, names = bundle_contract(args.bundle, args.base_action)
    limits = full_limits(args.mapping, names)
    identifier = "T_" + uuid.uuid4().hex[:28]
    transaction = None
    submitted = False
    try:
        if args.snapshot:
            initial = check_snapshot(json.loads(args.snapshot.read_text()), names, limits)
        else:
            transaction = RosTransaction(names, args.root_height, args.base_action, simulation=simulation)
            initial = check_snapshot(transaction.wait_endpoints(), names, limits)
            transaction.reset_observation_gate()
        request = {"request_id": identifier, "mode": "web" if args.tokens is not None else "text",
                   "text": args.text, "token_ids": args.tokens, "seed": args.seed, "speed": args.speed,
                   "return_home": not args.no_return, "return_frames": args.return_frames,
                   "limit_policy": args.limit_policy}
        print(json.dumps({"request_id": identifier, "state": "generating", "backend": args.backend,
                          "python_executable": sys.executable, "python_version": sys.version.split()[0],
                          "rknn_runtime_library": os.environ.get("RKNN_RUNTIME_LIB")}), flush=True)
        with open_pipeline(args.model, args.backend) as pipeline:
            trajectory = generate(pipeline, request, initial["joint_positions"][12:], load_limits(args.mapping))
        if transaction is not None:
            transaction.check_observation_gate()
            latest = check_snapshot(transaction.current_snapshot(), names, limits)
            if np.max(np.abs(np.asarray(latest["joint_positions"]) - initial["joint_positions"])) > 0.15:
                raise RuntimeError("Robot moved during generation; generate again from current state")
        document = make_document(trajectory, initial, graph)
        if document is None:
            print(json.dumps({"request_id": identifier, "state": "no_motion"}), flush=True)
            return
        encoded = json.dumps(document, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        if len(encoded.encode("utf-8")) > 4 * 1024 * 1024:
            raise ValueError("Complete JSON exceeds the WBC transport limit")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=args.output.parent,
                                         prefix=".text_wbc_", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded + "\n")
        try:
            os.link(temporary, args.output)  # Atomic exclusive publication; never overwrite another writer.
        finally:
            temporary.unlink(missing_ok=True)
        print(json.dumps({"request_id": identifier, "state": "generated", "file": str(args.output),
                          "frames": document["frame_count"], "base": graph["base_checkpoint_sha256"]}), flush=True)
        if args.execute:
            submitted = True
            result = transaction.execute(document, args.timeout)
            submitted = False
            print(json.dumps({"request_id": identifier, "state": result}), flush=True)
    finally:
        if transaction is not None:
            try:
                if submitted:
                    transaction.cancel(identifier)
            finally:
                transaction.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(json.dumps({"state": "failed", "error_type": type(error).__name__, "detail": str(error),
                          "python_executable": sys.executable}, ensure_ascii=False), file=sys.stderr, flush=True)
        raise
