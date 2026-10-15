#!/usr/bin/env python3
"""Build a one-input MimicLite policy for the unchanged ROS2 imitation controller."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
import yaml
from onnx import compose
from torch import nn


HISTORY_STEPS = (0, 1, 2, 3, 4, 8, 16)
FUTURE_STEPS = (-8, -4, -2, 0, 1, 2, 3, 4)
SINGLE_OBS_DIM = 80


def quat_to_matrix(q: torch.Tensor) -> torch.Tensor:
    q = q / torch.linalg.vector_norm(q, dim=-1, keepdim=True).clamp_min(1e-8)
    w, x, y, z = q.unbind(-1)
    return torch.stack(
        (
            1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
            2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
        ),
        dim=-1,
    ).reshape(q.shape[:-1] + (3, 3))


def yaw_inverse_rotate(v: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = torch.cos(yaw), torch.sin(yaw)
    vx, vy, vz = v.unbind(-1)
    return torch.stack((c * vx + s * vy, -s * vx + c * vy, vz), dim=-1)


class MonolithicInputAdapter(nn.Module):
    """Convert seven stacked legacy observations to native 399/240 inputs."""

    def __init__(
        self,
        motion_joint_pos: np.ndarray,
        motion_joint_vel: np.ndarray,
        motion_root_pos: np.ndarray,
        motion_root_quat: np.ndarray,
        motion_root_ang_vel_w: np.ndarray,
    ) -> None:
        super().__init__()
        self.register_buffer("motion_joint_pos", torch.from_numpy(motion_joint_pos.copy()))
        self.register_buffer("motion_root_pos", torch.from_numpy(motion_root_pos.copy()))
        self.register_buffer("motion_root_quat", torch.from_numpy(motion_root_quat.copy()))
        initial_quat = torch.from_numpy(motion_root_quat[0].copy())
        initial_matrix = quat_to_matrix(initial_quat)
        initial_ang_vel_w = torch.from_numpy(motion_root_ang_vel_w[0].copy())
        self.register_buffer(
            "initial_root_ang_vel_b",
            initial_matrix.transpose(-1, -2) @ initial_ang_vel_w,
        )
        self.register_buffer(
            "initial_projected_gravity",
            initial_matrix.transpose(-1, -2)
            @ torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
        )
        self.register_buffer(
            "initial_joint_pos", torch.from_numpy(motion_joint_pos[0].copy())
        )
        self.register_buffer(
            "initial_joint_vel", torch.from_numpy(motion_joint_vel[0].copy())
        )
        self.register_buffer("history_steps", torch.tensor(HISTORY_STEPS, dtype=torch.int64))
        self.register_buffer("future_steps", torch.tensor(FUTURE_STEPS, dtype=torch.int64))
        self.num_frames = int(motion_joint_pos.shape[0])

    def forward(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        # frame_stack order in rl_config is [0,-1,-2,-3,-4,-8,-16].
        frames = obs.reshape(1, len(HISTORY_STEPS), SINGLE_OBS_DIM)
        root_pos = frames[..., 0:3]
        root_quat = frames[..., 3:7]
        time_step = torch.round(frames[:, 0, 7]).to(torch.int64).clamp(0, self.num_frames - 1)
        anchor6 = frames[..., 8:14]
        ang_vel = frames[..., 14:17]
        joint_pos = frames[..., 17:38]
        joint_vel = frames[..., 38:59]
        actions = frames[..., 59:80]

        # Training reset repeats the initial state across the full history.
        # Replace unavailable controller history with that same initial state.
        valid = (time_step[:, None] >= self.history_steps[None, :]).unsqueeze(-1)
        ang_vel = torch.where(
            valid, ang_vel, self.initial_root_ang_vel_b.reshape(1, 1, 3)
        )
        joint_pos = torch.where(
            valid, joint_pos, self.initial_joint_pos.reshape(1, 1, -1)
        )
        joint_vel = torch.where(
            valid, joint_vel, self.initial_joint_vel.reshape(1, 1, -1)
        )
        anchor6 = torch.where(valid, anchor6, anchor6[:, :1])
        root_quat = torch.where(valid, root_quat, root_quat[:, :1])

        # Existing controller exports the first two rotation-matrix columns.
        col0 = torch.stack((anchor6[..., 0], anchor6[..., 2], anchor6[..., 4]), dim=-1)
        col1 = torch.stack((anchor6[..., 1], anchor6[..., 3], anchor6[..., 5]), dim=-1)
        col0 = col0 / torch.linalg.vector_norm(col0, dim=-1, keepdim=True).clamp_min(1e-8)
        col1 = col1 - (col0 * col1).sum(-1, keepdim=True) * col0
        col1 = col1 / torch.linalg.vector_norm(col1, dim=-1, keepdim=True).clamp_min(1e-8)
        col2 = torch.stack(
            (
                col0[..., 1] * col1[..., 2] - col0[..., 2] * col1[..., 1],
                col0[..., 2] * col1[..., 0] - col0[..., 0] * col1[..., 2],
                col0[..., 0] * col1[..., 1] - col0[..., 1] * col1[..., 0],
            ),
            dim=-1,
        )
        anchor_matrix = torch.stack((col0, col1, col2), dim=-1)

        # Each stacked C++ frame already carries the matching reference root
        # quaternion, avoiding a second history lookup inside TensorRT.
        motion_hist_matrix = quat_to_matrix(root_quat)
        # R^T @ [0,0,-1] is the negative third row of R. Keep the
        # matrix-vector product elementwise because this exports identically
        # across ONNX Runtime and TensorRT batch-broadcast implementations.
        motion_gravity = -motion_hist_matrix[..., 2, :]
        projected_gravity = torch.stack(
            (
                (anchor_matrix[..., 0, :] * motion_gravity).sum(-1),
                (anchor_matrix[..., 1, :] * motion_gravity).sum(-1),
                (anchor_matrix[..., 2, :] * motion_gravity).sum(-1),
            ),
            dim=-1,
        )
        projected_gravity = torch.where(
            valid,
            projected_gravity,
            self.initial_projected_gravity.reshape(1, 1, 3),
        )

        policy = torch.cat(
            (
                ang_vel.reshape(1, -1),
                projected_gravity.reshape(1, -1),
                joint_pos.reshape(1, -1),
                joint_vel.reshape(1, -1),
                actions[:, :3].reshape(1, -1),
            ),
            dim=-1,
        )

        ref_indices = (time_step[:, None] + self.future_steps[None, :]).clamp(0, self.num_frames - 1)
        ref_pos = self.motion_root_pos[ref_indices]
        ref_quat = self.motion_root_quat[ref_indices]
        ref_joint_pos = self.motion_joint_pos[ref_indices]
        current_index = (time_step + 0).clamp(0, self.num_frames - 1)
        current_pos = self.motion_root_pos[current_index].clone()
        current_pos[..., 2] = 0.0
        current_quat = self.motion_root_quat[current_index]
        local_pos = yaw_inverse_rotate(ref_pos - current_pos[:, None, :], current_quat[:, None, :])

        # anchor = R_robot_aligned^T R_motion_current_aligned
        current_anchor = anchor_matrix[:, 0]
        current_motion_matrix = quat_to_matrix(current_quat)
        future_motion_matrix = quat_to_matrix(ref_quat)
        relative_matrix = current_anchor[:, None] @ current_motion_matrix[:, None].transpose(-1, -2) @ future_motion_matrix
        relative_6d = relative_matrix[..., :2, :].reshape(1, -1)
        command = torch.cat(
            (local_pos.reshape(1, -1), relative_6d, ref_joint_pos.reshape(1, -1)),
            dim=-1,
        )
        return policy, command


class ClockMotion(nn.Module):
    """Motion lookup exposing the fields understood by the existing C++ parser."""

    def __init__(self, root_pos: np.ndarray, root_quat: np.ndarray):
        super().__init__()
        self.register_buffer("root_pos", torch.from_numpy(root_pos.copy()))
        self.register_buffer("root_quat", torch.from_numpy(root_quat.copy()))
        self.num_frames = int(root_pos.shape[0])

    def forward(self, time_step: torch.Tensor):
        index = time_step.to(torch.int64).reshape(-1).clamp(0, self.num_frames - 1)
        # contact_mask is intentionally a one-value timestep carrier.
        clock = index.to(dtype=time_step.dtype).reshape(-1, 1)
        return self.root_pos[index], self.root_quat[index], clock, self.root_quat[index]


def load_motion(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    model = onnx.load(str(path))
    meta = {item.key: item.value for item in model.metadata_props}
    frames = int(meta["num_frames"])
    body_names = meta["body_names"].split(",")
    root_index = body_names.index("base_link")
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    joint_pos, joint_vel, root_pos, root_quat, root_ang_vel_w = [], [], [], [], []
    for index in range(frames):
        values = dict(zip(
            [item.name for item in session.get_outputs()],
            session.run(None, {"time_step": np.array([[index]], np.float32)}),
            strict=True,
        ))
        joint_pos.append(values["joint_pos"][0])
        joint_vel.append(values["joint_vel"][0])
        root_pos.append(values["body_pos_w"][0, root_index])
        root_quat.append(values["body_quat_w"][0, root_index])
        root_ang_vel_w.append(values["body_ang_vel_w"][0, root_index])
    return tuple(
        np.asarray(value, dtype=np.float32)
        for value in (joint_pos, joint_vel, root_pos, root_quat, root_ang_vel_w)
    )


def resolve_joint_values(
    mapping: dict[str, float], joint_names: list[str], label: str
) -> list[float]:
    values = []
    for joint_name in joint_names:
        matches = [
            float(value)
            for pattern, value in mapping.items()
            if re.fullmatch(pattern, joint_name)
        ]
        if len(matches) != 1:
            raise ValueError(
                f"Expected exactly one {label} entry for {joint_name}, got {len(matches)}"
            )
        values.append(matches[0])
    return values


def load_robot_config(
    path: Path, lowlevel_frequency: int, policy_frequency_override: int | None = None
) -> dict:
    with path.open(encoding="utf-8") as stream:
        policy_cfg = yaml.safe_load(stream)

    required = (
        "policy_joint_names",
        "joint_kp",
        "joint_kd",
        "default_joint_pos",
        "action_scale",
    )
    missing = [name for name in required if name not in policy_cfg]
    if missing:
        raise ValueError(
            f"Policy config {path} is missing deploy metadata: {missing}. "
            "Re-export the checkpoint with projects/mimic-lite/scripts/play.py."
        )

    joint_names = list(policy_cfg["policy_joint_names"])
    action_scale = [float(value) for value in policy_cfg["action_scale"]]
    if len(action_scale) != len(joint_names):
        raise ValueError("action_scale dimension does not match policy_joint_names")

    history_steps = policy_cfg["observation"]["policy"][
        "root_ang_vel_history"
    ]["history_steps"]
    history_groups = (
        "projected_gravity_history",
        "joint_pos_history",
        "joint_vel_history",
    )
    if tuple(history_steps) != HISTORY_STEPS or any(
        policy_cfg["observation"]["policy"][name]["history_steps"]
        != history_steps
        for name in history_groups
    ):
        raise ValueError(
            f"Deploy adapter expects history steps {HISTORY_STEPS}, got {history_steps}"
        )
    previous_action_steps = int(
        policy_cfg["observation"]["policy"]["prev_actions"]["steps"]
    )
    if previous_action_steps != 3:
        raise ValueError(
            f"Deploy adapter expects 3 previous actions, got {previous_action_steps}"
        )
    future_steps = policy_cfg["motion"]["future_steps"]
    if tuple(future_steps) != FUTURE_STEPS:
        raise ValueError(
            f"Deploy adapter expects future steps {FUTURE_STEPS}, got {future_steps}"
        )

    yaml_policy_frequency = policy_cfg.get("policy_frequency")
    if policy_frequency_override is None and yaml_policy_frequency is None:
        raise ValueError(
            f"Policy config {path} has no policy_frequency; "
            "pass --policy-frequency with the frequency used during training"
        )
    if (
        policy_frequency_override is not None
        and yaml_policy_frequency is not None
        and int(yaml_policy_frequency) != policy_frequency_override
    ):
        raise ValueError(
            f"Policy frequency override {policy_frequency_override} conflicts with "
            f"policy config value {yaml_policy_frequency}"
        )
    policy_frequency = int(
        policy_frequency_override if policy_frequency_override is not None
        else yaml_policy_frequency
    )
    if policy_frequency <= 0 or lowlevel_frequency <= 0:
        raise ValueError("policy and low-level frequencies must be positive")
    if lowlevel_frequency % policy_frequency:
        raise ValueError("low-level frequency must be divisible by policy frequency")
    training_physics_frequency = policy_cfg.get("training_physics_frequency")
    if training_physics_frequency is not None:
        training_physics_frequency = int(training_physics_frequency)
        if training_physics_frequency <= 0:
            raise ValueError("training physics frequency must be positive")
        if training_physics_frequency % policy_frequency:
            raise ValueError(
                "training physics frequency must be divisible by policy frequency"
            )

    return {
        "policy_type": "rl_imitation",
        "policy_name": "policy",
        "motion_name": "motion",
        "pub_imitationref_joint_states": False,
        "joint_order_isaac": joint_names,
        "joint_order_mj": joint_names,
        "kps": resolve_joint_values(policy_cfg["joint_kp"], joint_names, "joint_kp"),
        "kds": resolve_joint_values(policy_cfg["joint_kd"], joint_names, "joint_kd"),
        "default_joint_angles": resolve_joint_values(
            policy_cfg["default_joint_pos"], joint_names, "default_joint_pos"
        ),
        "action_scale": action_scale,
        "policy_frequency": policy_frequency,
        "lowlevel_frequency": lowlevel_frequency,
        "num_actions": len(joint_names),
        "num_single_obs": SINGLE_OBS_DIM,
        "frame_stack": [0, -1, -2, -3, -4, -8, -16],
        "clip_obs": 2500000.0,
        "clip_actions": 1000000.0,
        "command_config": {
            "items": [
                {"type": "root_pos", "dim": 3},
                {"type": "root_quat", "dim": 4},
                {"type": "contact_mask", "dim": 1},
            ]
        },
        "observation_config": {
            "items": [
                {"type": "command", "dim": 8, "scale": 1.0},
                {"type": "anchor_ori", "dim": 6, "scale": 1.0},
                {"type": "base_ang_vel", "dim": 3, "scale": 1.0},
                {"type": "joint_pos", "dim": 21, "scale": 1.0},
                {"type": "joint_vel", "dim": 21, "scale": 1.0},
                {"type": "actions", "dim": 21, "scale": 1.0},
            ]
        },
    }


def export(args: argparse.Namespace) -> None:
    robot_config = load_robot_config(
        args.policy_config, args.lowlevel_frequency, args.policy_frequency
    )
    joint_pos, joint_vel, root_pos, root_quat, root_ang_vel_w = load_motion(args.motion)
    if joint_pos.shape[1] != len(robot_config["joint_order_isaac"]):
        raise ValueError("Motion joint dimension does not match the policy config")
    adapter = MonolithicInputAdapter(
        joint_pos, joint_vel, root_pos, root_quat, root_ang_vel_w
    ).eval()
    clock = ClockMotion(root_pos, root_quat).eval()
    adapter_path = args.policy_output.with_name(".adapter-preprocess.onnx")
    torch.onnx.export(
        adapter,
        (torch.zeros((1, len(HISTORY_STEPS) * SINGLE_OBS_DIM), dtype=torch.float32),),
        str(adapter_path),
        opset_version=18,
        input_names=["obs"],
        output_names=["adapter_policy", "adapter_command"],
        dynamo=False,
    )
    torch.onnx.export(
        clock,
        (torch.zeros((1, 1), dtype=torch.float32),),
        str(args.motion_output),
        opset_version=18,
        input_names=["time_step"],
        output_names=["root_pos", "root_quat", "contact_mask", "body_quat_w"],
        dynamo=False,
    )
    preprocess = onnx.load(str(adapter_path))
    native_source = onnx.load(str(args.policy))
    preprocess.ir_version = native_source.ir_version
    native = compose.add_prefix(native_source, "native_")
    merged = compose.merge_models(
        preprocess,
        native,
        [("adapter_policy", "native_policy"), ("adapter_command", "native_command")],
        outputs=["native_action"],
        name="MimicLiteL0V2MonolithicDeploy",
    )
    for node in merged.graph.node:
        for index, name in enumerate(node.output):
            if name == "native_action":
                node.output[index] = "actions"
    merged.graph.output[0].name = "actions"
    unique_opsets = {}
    for item in merged.opset_import:
        unique_opsets[item.domain] = max(unique_opsets.get(item.domain, 0), item.version)
    del merged.opset_import[:]
    for domain, version in unique_opsets.items():
        merged.opset_import.append(onnx.OperatorSetIdProto(domain=domain, version=version))
    del merged.metadata_props[:]
    for key, value in {
        "artifact_type": "mimic_lite_policy_monolithic_ros2",
        "deploy_abi": "obs:[1,560]->action:[1,21]",
        "frame_stack_indices": "0,-1,-2,-3,-4,-8,-16",
        "single_obs_layout": "root_pos:3,root_quat:4,time_step:1,anchor_ori:6,base_ang_vel:3,joint_pos:21,joint_vel:21,actions:21",
        "embedded_motion": args.motion.name,
    }.items():
        merged.metadata_props.append(onnx.StringStringEntryProto(key=key, value=value))
    onnx.checker.check_model(merged)
    onnx.save(merged, str(args.policy_output))

    args.config_output.write_text(
        yaml.safe_dump({"RobotCfg": robot_config}, sort_keys=False),
        encoding="utf-8",
    )
    adapter_path.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--policy-config", type=Path, required=True)
    parser.add_argument("--lowlevel-frequency", type=int, default=500)
    parser.add_argument("--policy-frequency", type=int)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--policy-output", type=Path, required=True)
    parser.add_argument("--motion-output", type=Path, required=True)
    parser.add_argument("--config-output", type=Path, required=True)
    args = parser.parse_args()
    for name in (
        "policy",
        "policy_config",
        "motion",
        "policy_output",
        "motion_output",
        "config_output",
    ):
        setattr(args, name, getattr(args, name).expanduser().resolve())
    args.policy_output.parent.mkdir(parents=True, exist_ok=True)
    args.motion_output.parent.mkdir(parents=True, exist_ok=True)
    export(args)
    print(args.policy_output)
    print(args.motion_output)
    print(args.config_output)


if __name__ == "__main__":
    main()
