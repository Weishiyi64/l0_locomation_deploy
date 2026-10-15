#!/usr/bin/env python3
"""Export one MimicLite NPZ or trusted GMR PKL as a motion lookup ONNX."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import mujoco
import numpy as np
import onnx
import torch
from torch import nn


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = (
    SCRIPT_DIR.parent.parent
    / "assets"
    / "l0_v2"
    / "xmls"
    / "l0_v2.xml"
)
DEFAULT_OUTPUT_DIR = SCRIPT_DIR.parent.parent / "motion_onnx_cfg"
OUTPUT_NAMES = (
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)


def resolve_output_path(motion_path: Path, output_path: Path | None) -> Path:
    if output_path is None:
        return DEFAULT_OUTPUT_DIR / f"{motion_path.stem}.onnx"
    return output_path.expanduser().resolve()


class MotionLookup(nn.Module):
    """Return a reference frame selected by a clamped integer time step."""

    def __init__(self, arrays: dict[str, np.ndarray]):
        super().__init__()
        lengths = {int(np.asarray(arrays[name]).shape[0]) for name in OUTPUT_NAMES}
        if len(lengths) != 1 or next(iter(lengths)) <= 0:
            raise ValueError("All motion fields must have the same positive frame count")
        self.num_frames = next(iter(lengths))
        for name in OUTPUT_NAMES:
            value = np.asarray(arrays[name], dtype=np.float32)
            self.register_buffer(name, torch.from_numpy(value.copy()))

    def forward(self, time_step: torch.Tensor) -> tuple[torch.Tensor, ...]:
        index = torch.clamp(
            time_step.to(dtype=torch.int64).reshape(-1), 0, self.num_frames - 1
        )
        return tuple(getattr(self, name)[index] for name in OUTPUT_NAMES)


def _model_names(model: mujoco.MjModel) -> tuple[list[str], list[str]]:
    joint_names = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name is None:
            raise ValueError(f"Unnamed non-free joint at index {joint_id}")
        joint_names.append(name)
    body_names = []
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name is None:
            raise ValueError(f"Unnamed body at index {body_id}")
        body_names.append(name)
    return joint_names, body_names


def _assemble_qpos(
    model: mujoco.MjModel,
    joint_names: list[str],
    root_pos: np.ndarray,
    root_quat_w: np.ndarray,
    joint_pos: np.ndarray,
) -> np.ndarray:
    frames = joint_pos.shape[0]
    if joint_pos.shape != (frames, len(joint_names)):
        raise ValueError(
            f"joint_pos has shape {joint_pos.shape}, expected "
            f"({frames}, {len(joint_names)})"
        )
    if root_pos.shape != (frames, 3) or root_quat_w.shape != (frames, 4):
        raise ValueError("root_pos/root_quat_w shapes must be [T,3] and [T,4]")
    quat_norm = np.linalg.norm(root_quat_w, axis=1, keepdims=True)
    if np.any(quat_norm < 1e-6):
        raise ValueError("root_quat_w contains a zero quaternion")
    root_quat_w = root_quat_w / quat_norm

    qpos = np.zeros((frames, model.nq), dtype=np.float32)
    scalar_qpos = []
    free_joints = []
    for joint_id in range(model.njnt):
        joint_type = model.jnt_type[joint_id]
        address = int(model.jnt_qposadr[joint_id])
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            free_joints.append(address)
        elif joint_type in (
            mujoco.mjtJoint.mjJNT_HINGE,
            mujoco.mjtJoint.mjJNT_SLIDE,
        ):
            scalar_qpos.append(address)
    if len(free_joints) != 1:
        raise ValueError(f"Expected exactly one free root joint, got {len(free_joints)}")
    root_address = free_joints[0]
    qpos[:, root_address : root_address + 3] = root_pos
    qpos[:, root_address + 3 : root_address + 7] = root_quat_w
    qpos[:, scalar_qpos] = joint_pos
    return qpos


def _load_gmr_pickle(
    motion_path: Path, model: mujoco.MjModel, joint_names: list[str]
) -> tuple[np.ndarray, float]:
    # Pickle is code-executable by design. This path is intentionally limited to
    # explicit, trusted local GMR artifacts supplied by the caller.
    with motion_path.open("rb") as stream:
        payload = pickle.load(stream)
    if not isinstance(payload, dict):
        raise TypeError(f"Expected a dict in GMR pickle, got {type(payload).__name__}")
    required = {"fps", "root_pos", "root_rot", "dof_pos"}
    missing = sorted(required.difference(payload))
    if missing:
        raise KeyError(f"GMR pickle is missing fields: {', '.join(missing)}")

    fps = float(np.asarray(payload["fps"]).reshape(-1)[0])
    root_pos = np.asarray(payload["root_pos"], dtype=np.float32)
    root_quat_xyzw = np.asarray(payload["root_rot"], dtype=np.float32)
    joint_pos = np.asarray(payload["dof_pos"], dtype=np.float32)
    if root_quat_xyzw.ndim != 2 or root_quat_xyzw.shape[1] != 4:
        raise ValueError(
            f"GMR root_rot has shape {root_quat_xyzw.shape}, expected [T,4]"
        )

    actuator_joint_names = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name is None:
            raise ValueError(f"Actuator {actuator_id} targets an unnamed joint")
        actuator_joint_names.append(name)
    if set(actuator_joint_names) != set(joint_names):
        raise ValueError("Actuated joints do not match the model's scalar joints")
    if joint_pos.ndim != 2 or joint_pos.shape[1] != len(actuator_joint_names):
        raise ValueError(
            f"GMR dof_pos has shape {joint_pos.shape}, expected "
            f"[T,{len(actuator_joint_names)}]"
        )
    joint_pos = joint_pos[
        :, [actuator_joint_names.index(name) for name in joint_names]
    ]
    root_quat_w = root_quat_xyzw[:, [3, 0, 1, 2]]
    return _assemble_qpos(
        model, joint_names, root_pos, root_quat_w, joint_pos
    ), fps


def _load_motion_arrays(
    motion_path: Path, model_path: Path, target_fps: float
) -> tuple[dict[str, np.ndarray], float, list[str], list[str]]:
    from any4hdmi.dataset.interpolation import interpolate_qpos_qvel_packed_torch
    from any4hdmi.fk.runner import FKRunner
    from any4hdmi.utils.dataset import compute_motion_qvel

    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_names, body_names = _model_names(model)
    if motion_path.suffix.lower() in {".pkl", ".pickle"}:
        qpos, fps = _load_gmr_pickle(motion_path, model, joint_names)
    else:
        with np.load(motion_path, allow_pickle=False) as payload:
            if "qpos" in payload.files:
                qpos = np.asarray(payload["qpos"], dtype=np.float32)
                if qpos.ndim != 2 or qpos.shape[1] != model.nq:
                    raise ValueError(
                        f"qpos has shape {qpos.shape}, expected [T,{model.nq}]"
                    )
                manifest_path = motion_path.parent.parent / "manifest.json"
                if manifest_path.is_file():
                    manifest = json.loads(manifest_path.read_text())
                    timestep = float(manifest.get("timestep", 0.0))
                    if timestep <= 0 or not np.isfinite(timestep):
                        raise ValueError(
                            f"Invalid timestep in motion manifest {manifest_path}: {timestep}"
                        )
                    fps = 1.0 / timestep
                else:
                    fps = float(target_fps)
            else:
                required = {"fps", "joint_pos"}
                missing = sorted(required.difference(payload.files))
                if missing:
                    raise KeyError(f"Motion NPZ is missing fields: {', '.join(missing)}")
                fps = float(np.asarray(payload["fps"]).reshape(-1)[0])
                joint_pos = np.asarray(payload["joint_pos"], dtype=np.float32)
                if "joint_names" in payload.files:
                    source_joint_names = np.asarray(payload["joint_names"]).astype(str).tolist()
                    missing_joints = [name for name in joint_names if name not in source_joint_names]
                    if missing_joints:
                        raise ValueError(f"Motion is missing model joints: {missing_joints}")
                    joint_pos = joint_pos[
                        :, [source_joint_names.index(name) for name in joint_names]
                    ]

                if {"root_pos", "root_quat_w"}.issubset(payload.files):
                    root_pos = np.asarray(payload["root_pos"], dtype=np.float32)
                    root_quat = np.asarray(payload["root_quat_w"], dtype=np.float32)
                elif {"body_pos_w", "body_quat_w", "body_names"}.issubset(payload.files):
                    source_body_names = np.asarray(payload["body_names"]).astype(str).tolist()
                    root_body_name = body_names[0]
                    if root_body_name not in source_body_names:
                        raise ValueError(
                            f"Motion body_names has no model root body {root_body_name!r}"
                        )
                    root_index = source_body_names.index(root_body_name)
                    root_pos = np.asarray(payload["body_pos_w"], dtype=np.float32)[
                        :, root_index
                    ]
                    root_quat = np.asarray(payload["body_quat_w"], dtype=np.float32)[
                        :, root_index
                    ]
                else:
                    raise KeyError(
                        "Motion NPZ needs root_pos/root_quat_w or "
                        "body_pos_w/body_quat_w/body_names"
                    )

                qpos = _assemble_qpos(
                    model, joint_names, root_pos, root_quat, joint_pos
                )
    if fps <= 0 or not np.isfinite(fps):
        raise ValueError(f"Motion FPS must be positive and finite, got {fps}")
    qvel = compute_motion_qvel(model, qpos, fps)
    qpos_t, qvel_t, lengths = interpolate_qpos_qvel_packed_torch(
        [torch.from_numpy(qpos)],
        [torch.from_numpy(qvel)],
        source_fps=fps,
        target_fps=target_fps,
    )
    if len(lengths) != 1:
        raise RuntimeError("Expected one resampled motion clip")

    fk = FKRunner(model_path, batch_size=max(lengths[0], 1), device="cpu")
    outputs = fk.forward_kinematics(qpos_t, qvel_t)
    arrays = {
        name: outputs[name].detach().cpu().numpy().astype(np.float32)[:, 1:]
        if name.startswith("body_") and outputs[name].shape[1] == model.nbody
        else outputs[name].detach().cpu().numpy().astype(np.float32)
        for name in OUTPUT_NAMES
    }
    return arrays, float(target_fps), joint_names, body_names


def _set_metadata(
    output_path: Path,
    *,
    motion_path: Path,
    fps: float,
    num_frames: int,
    joint_names: list[str],
    body_names: list[str],
) -> None:
    model = onnx.load(str(output_path))
    values = {
        "artifact_type": "mimic_lite_motion_reference",
        "source_motion": str(motion_path),
        "fps": format(fps, ".17g"),
        "num_frames": str(num_frames),
        "duration_s": format(max(num_frames - 1, 0) / fps, ".17g"),
        "joint_names": ",".join(joint_names),
        "body_names": ",".join(body_names),
        "quaternion_order": "wxyz",
        "time_step_semantics": "zero_based_frame_index_clamped",
    }
    existing = {item.key: item.value for item in model.metadata_props}
    existing.update(values)
    del model.metadata_props[:]
    for key, value in existing.items():
        model.metadata_props.append(onnx.StringStringEntryProto(key=key, value=value))
    onnx.save(model, str(output_path))


def export_motion_to_onnx(
    motion_path: Path,
    output_path: Path | None = None,
    model_path: Path = DEFAULT_MODEL,
    target_fps: float = 50.0,
) -> Path:
    motion_path = motion_path.expanduser().resolve()
    output_path = resolve_output_path(motion_path, output_path)
    model_path = model_path.expanduser().resolve()
    if not motion_path.is_file():
        raise FileNotFoundError(f"Motion file not found: {motion_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"Training XML not found: {model_path}")
    if target_fps <= 0 or not np.isfinite(target_fps):
        raise ValueError(f"target_fps must be positive and finite, got {target_fps}")
    arrays, fps, joint_names, body_names = _load_motion_arrays(
        motion_path, model_path, target_fps
    )
    lookup = MotionLookup(arrays).eval()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        lookup,
        (torch.zeros((1, 1), dtype=torch.float32),),
        str(output_path),
        export_params=True,
        opset_version=18,
        input_names=["time_step"],
        output_names=list(OUTPUT_NAMES),
        dynamo=False,
    )
    _set_metadata(
        output_path,
        motion_path=motion_path,
        fps=fps,
        num_frames=lookup.num_frames,
        joint_names=joint_names,
        body_names=body_names,
    )

    import onnxruntime as ort

    session = ort.InferenceSession(str(output_path), providers=["CPUExecutionProvider"])
    checks = sorted({0, lookup.num_frames // 2, lookup.num_frames - 1})
    for index in checks:
        actual = session.run(None, {"time_step": np.array([[index]], np.float32)})
        for name, value in zip(OUTPUT_NAMES, actual, strict=True):
            np.testing.assert_allclose(value[0], arrays[name][index], rtol=1e-5, atol=1e-6)
    return output_path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-file", type=Path, required=True)
    parser.add_argument(
        "--output-path",
        type=Path,
        help=f"Output ONNX path (default: {DEFAULT_OUTPUT_DIR}/<motion-name>.onnx).",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--target-fps",
        type=float,
        default=50.0,
        help="FPS used during training and by policy motion-step offsets.",
    )
    return parser


def main() -> None:
    args = _parser().parse_args()
    output_path = export_motion_to_onnx(
        args.motion_file, args.output_path, args.model, target_fps=args.target_fps
    )
    print(f"Exported motion ONNX: {output_path}")


if __name__ == "__main__":
    main()
