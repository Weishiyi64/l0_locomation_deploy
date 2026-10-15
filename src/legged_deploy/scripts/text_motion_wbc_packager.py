#!/usr/bin/env python3
"""Non-blocking ROS 2 text-motion packager for the stg25 WBC policy."""

from __future__ import annotations

import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class TextMotionWbcPackager(Node):
    def __init__(self) -> None:
        super().__init__("text_motion_wbc_packager")
        defaults = {
            "json_topic": "/text_motion/json",
            "ready_topic": "/text_motion/ready",
            "status_topic": "/text_motion/status",
            "policy_root_path": "",
            "robot_type": "l0_v3",
            "policy_slot_prefix": "text_motion_wbc_stg25",
            "python_executable": "",
            "wbc_deploy_root": "",
            "wbc_policy_onnx": "",
            "wbc_policy_config": "",
            "amp_stand_profile": "",
            "mujoco_model": "",
            "max_payload_bytes": 4 * 1024 * 1024,
            "hold_frames": 10,
            "transition_frames": 40,
            "return_frames": 12,
            "return_settle_frames": 2,
            "upper_lowpass_cutoff_hz": 4.0,
            # Keep the generated motion at the infer model's original FPS and
            # frame count.  A zero limit disables duration-changing time
            # stretching while retaining the same-duration low-pass filter.
            "upper_max_velocity_rad_s": 0.0,
            "upper_max_acceleration_rad_s2": 0.0,
            "trtexec": "/usr/src/tensorrt/bin/trtexec",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name: str):
            return self.get_parameter(name).value

        self._policy_root = Path(value("policy_root_path")).expanduser().resolve()
        self._robot_type = str(value("robot_type"))
        self._slot_prefix = str(value("policy_slot_prefix"))
        self._python = Path(value("python_executable")).expanduser().resolve()
        self._wbc_root = Path(value("wbc_deploy_root")).expanduser().resolve()
        self._policy_onnx = Path(value("wbc_policy_onnx")).expanduser().resolve()
        self._policy_config = Path(value("wbc_policy_config")).expanduser().resolve()
        self._profile = Path(value("amp_stand_profile")).expanduser().resolve()
        self._model = Path(value("mujoco_model")).expanduser().resolve()
        self._max_payload = max(1024, int(value("max_payload_bytes")))
        self._hold = max(0, int(value("hold_frames")))
        self._transition = max(2, int(value("transition_frames")))
        self._return = max(2, int(value("return_frames")))
        self._settle = max(1, int(value("return_settle_frames")))
        self._upper_lowpass_cutoff = max(0.0, float(value("upper_lowpass_cutoff_hz")))
        self._upper_max_velocity = max(0.0, float(value("upper_max_velocity_rad_s")))
        self._upper_max_acceleration = max(0.0, float(value("upper_max_acceleration_rad_s2")))
        self._trtexec = Path(value("trtexec")).expanduser()

        script_dir = Path(__file__).resolve().parent
        self._json_converter = script_dir / "json_to_motion_onnx.py"
        self._motion_builder = script_dir / "build_wbc_motion_from_amp_stand.py"
        self._bundle_builder = self._wbc_root / "scripts/deploy/prepare_monolithic_deploy.sh"
        required = (
            self._python, self._policy_onnx, self._policy_config, self._profile,
            self._model, self._json_converter, self._motion_builder, self._bundle_builder,
        )
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise RuntimeError(f"missing WBC packaging inputs: {missing}")
        self._robot_policy_root = self._policy_root / self._robot_type
        self._robot_policy_root.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        active_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, str(value("status_topic")), qos)
        self._ready_pub = self.create_publisher(String, str(value("ready_topic")), qos)
        self._json_sub = self.create_subscription(
            String, str(value("json_topic")), self._on_json, qos)
        self._active_sub = self.create_subscription(
            Bool, "/rlImitationActive", self._on_active, active_qos)
        self._condition = threading.Condition()
        self._pending: str | None = None
        self._stop = False
        self._active = False
        self._busy = False
        self._next_slot = 0
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="text-motion-wbc-packager",
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info("stg25 WBC text-motion packager ready (A/B atomic slots)")

    def destroy_node(self) -> bool:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        self._worker.join(timeout=5.0)
        return super().destroy_node()

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_pub.publish(message)

    def _on_active(self, message: Bool) -> None:
        with self._condition:
            self._active = bool(message.data)

    def _on_json(self, message: String) -> None:
        if not message.data or len(message.data.encode("utf-8")) > self._max_payload:
            self._publish_status("rejected: JSON payload is empty or too large")
            return
        with self._condition:
            if self._active or self._busy:
                reason = (
                    "another WBC action is active"
                    if self._active else "WBC packaging is busy"
                )
                self._publish_status(f"rejected: {reason}")
                return
            self._pending = message.data
            self._condition.notify()
        self._publish_status("received: complete JSON queued for background stg25 WBC packaging")

    @staticmethod
    def _run(args: list[str], env: dict[str, str] | None = None) -> None:
        result = subprocess.run(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, check=False)
        if result.returncode:
            tail = "\n".join(result.stdout.splitlines()[-12:])
            raise RuntimeError(f"command failed ({result.returncode}): {args[0]}\n{tail}")

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stop or self._pending is not None)
                if self._stop:
                    return
                payload, self._pending = self._pending, None
                self._busy = True
            try:
                assert payload is not None
                self._package(payload)
            except Exception as error:  # Report all packaging failures over ROS.
                self.get_logger().error(f"WBC packaging rejected: {error}")
                self._publish_status(f"rejected: WBC packaging failed: {error}")
            finally:
                with self._condition:
                    self._busy = False

    def _package(self, payload: str) -> None:
        document = json.loads(payload)
        text = str(document.get("text", ""))
        frame_count = int(document.get("frame_count", 0))
        slot_name = f"{self._slot_prefix}_{'a' if self._next_slot == 0 else 'b'}"
        self._publish_status(
            f"packaging: conditioning upper body and building full AMP reference for {frame_count} frames"
        )
        build_root = Path(tempfile.mkdtemp(
            prefix=".text_motion_wbc_", dir=self._robot_policy_root))
        try:
            json_path = build_root / "source.json"
            upper_path = build_root / "source_upper_motion.onnx"
            reference_path = build_root / "reference_full.onnx"
            policy_input_dir = build_root / "stg25_input"
            bundle_dir = build_root / "bundle"
            policy_input_dir.mkdir()
            json_path.write_text(payload, encoding="utf-8")
            policy_build_input = policy_input_dir / "policy.onnx"
            try:
                os.link(self._policy_onnx, policy_build_input)
            except OSError:
                shutil.copy2(self._policy_onnx, policy_build_input)
            shutil.copy2(self._policy_config, policy_input_dir / "policy.yaml")
            self._run([str(self._python), str(self._json_converter), "--input", str(json_path),
                       "--output", str(upper_path)])
            self._run([
                str(self._python), str(self._motion_builder), "--upper-motion", str(upper_path),
                "--amp-profile", str(self._profile), "--output", str(reference_path),
                "--model", str(self._model),
                "--policy-config", str(policy_input_dir / "policy.yaml"),
                "--hold-frames", str(self._hold), "--transition-frames", str(self._transition),
                "--return-frames", str(self._return), "--return-settle-frames", str(self._settle),
                "--joint-limit-margin", "0.01", "--direct-upper-body", "--mirror-arms",
                "--upper-lowpass-cutoff-hz", str(self._upper_lowpass_cutoff),
                "--upper-max-velocity-rad-s", str(self._upper_max_velocity),
                "--upper-max-acceleration-rad-s2", str(self._upper_max_acceleration),
            ])
            self._publish_status("packaging: embedding motion into stg25 WBC policy")
            env = os.environ.copy()
            env["WBC_DEPLOY_PYTHON"] = str(self._python)
            env["MIMIC_LITE_POLICY_FREQUENCY"] = "50"
            self._run([str(self._bundle_builder), str(policy_input_dir / "policy.onnx"),
                       str(reference_path), str(bundle_dir)], env=env)
            shutil.copy2(reference_path, bundle_dir / "reference_full.onnx")
            shutil.copy2(json_path, bundle_dir / "source.json")
            if platform.machine().lower() in ("aarch64", "arm64"):
                if not self._trtexec.is_file():
                    raise RuntimeError(f"TensorRT target requires trtexec: {self._trtexec}")
                self._publish_status("packaging: building TensorRT engines")
                self._run([str(self._trtexec), f"--onnx={bundle_dir / 'policy.onnx'}",
                           f"--saveEngine={bundle_dir / 'policy.trt'}", "--fp16"])
                self._run([str(self._trtexec), f"--onnx={bundle_dir / 'motion.onnx'}",
                           f"--saveEngine={bundle_dir / 'motion.trt'}", "--fp16"])
            slot_path = self._robot_policy_root / slot_name
            if slot_path.exists():
                shutil.rmtree(slot_path)
            os.replace(bundle_dir, slot_path)
            self._next_slot = 1 - self._next_slot
            ready = String()
            ready.data = slot_name
            self._ready_pub.publish(ready)
            detail = f", text={text}" if text else ""
            self._publish_status(f"ready: {slot_name}, {frame_count} source frames{detail}")
            self.get_logger().info(f"WBC bundle ready: {slot_path}")
        finally:
            shutil.rmtree(build_root, ignore_errors=True)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = TextMotionWbcPackager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
