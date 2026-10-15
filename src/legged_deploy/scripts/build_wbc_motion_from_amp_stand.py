#!/usr/bin/env python3
"""Build a full L0 WBC motion with a common AMP standing start and end pose.

The floating base and all joints start from a captured AMP standing profile.
A short hold and smooth transition are prepended before the source upper-body
trajectory. Trailing static frames in the source are removed, then a short
smooth return to the same standing profile is appended.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import onnx
import onnxruntime as ort
import torch

from wbc_motion_onnx_utils import MotionLookup, _model_names, _set_metadata


OUTPUT_NAMES = (
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)

DEFAULT_AMP_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "amp_stand_profiles"
    / "l0_v3_amp_stand_inner_arms_profile.json"
)


def _trajectory_peaks(values: np.ndarray, fps: float) -> tuple[float, float]:
    """Return peak per-joint velocity and acceleration magnitudes."""
    peak_velocity = float(np.max(np.abs(np.diff(values, axis=0))) * fps)
    peak_acceleration = 0.0
    if len(values) > 2:
        peak_acceleration = float(
            np.max(np.abs(np.diff(values, n=2, axis=0))) * fps * fps
        )
    return peak_velocity, peak_acceleration


def _mirror_arm_trajectory(values: np.ndarray, joint_names: list[str]) -> np.ndarray:
    """Convert camera-view left/right arm semantics to robot anatomical sides."""
    pairs = (
        ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint", 1.0),
        ("left_shoulder_roll_joint", "right_shoulder_roll_joint", -1.0),
        ("left_elbow_roll_joint", "right_elbow_roll_joint", -1.0),
    )
    required = {name for left, right, _ in pairs for name in (left, right)}
    missing = sorted(required - set(joint_names))
    if missing:
        raise ValueError(f"cannot mirror arms; source joints are missing: {missing}")

    mirrored = values.copy()
    for left, right, sign in pairs:
        left_column = joint_names.index(left)
        right_column = joint_names.index(right)
        mirrored[:, left_column] = sign * values[:, right_column]
        mirrored[:, right_column] = sign * values[:, left_column]
    return mirrored


def _condition_upper_body_trajectory(
    values: np.ndarray,
    fps: float,
    lowpass_cutoff_hz: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float,
) -> tuple[np.ndarray, dict[str, float]]:
    """Low-pass and time-stretch an upper-body trajectory without adding poses."""
    conditioned = values.astype(np.float64, copy=True)
    input_velocity, input_acceleration = _trajectory_peaks(conditioned, fps)

    if lowpass_cutoff_hz > 0.0:
        if lowpass_cutoff_hz >= fps * 0.5:
            raise ValueError("upper low-pass cutoff must be below the Nyquist frequency")
        sigma_frames = fps / (2.0 * np.pi * lowpass_cutoff_hz)
        radius = max(1, int(np.ceil(3.0 * sigma_frames)))
        offsets = np.arange(-radius, radius + 1, dtype=np.float64)
        kernel = np.exp(-0.5 * np.square(offsets / sigma_frames))
        kernel /= np.sum(kernel)
        filtered = np.stack(
            [
                np.convolve(
                    np.pad(conditioned[:, column], (radius, radius), mode="edge"),
                    kernel,
                    mode="valid",
                )
                for column in range(conditioned.shape[1])
            ],
            axis=1,
        )
        # Preserve the generated first and last poses exactly.  Smoothly fade
        # the filter in/out instead of creating a new entry or exit posture.
        distance_to_edge = np.minimum(
            np.arange(len(conditioned)), np.arange(len(conditioned))[::-1]
        )
        blend = np.clip(distance_to_edge / radius, 0.0, 1.0)
        blend = blend * blend * (3.0 - 2.0 * blend)
        conditioned += blend[:, None] * (filtered - conditioned)

    # Limit speed and acceleration by increasing duration at the same 50 Hz
    # policy rate.  This preserves the path and endpoints instead of clipping
    # amplitudes or inserting an AMP standing pose.
    original_positions = np.arange(len(conditioned), dtype=np.float64)
    time_scale = 1.0
    for _ in range(12):
        output_count = int(np.ceil((len(conditioned) - 1) * time_scale)) + 1
        if output_count > 10000:
            raise ValueError("upper-body conditioning would exceed 10000 frames")
        sample_positions = np.linspace(
            0.0, len(conditioned) - 1, output_count, dtype=np.float64
        )
        output = np.stack(
            [
                np.interp(sample_positions, original_positions, conditioned[:, column])
                for column in range(conditioned.shape[1])
            ],
            axis=1,
        )
        peak_velocity, peak_acceleration = _trajectory_peaks(output, fps)
        ratios = [1.0]
        if max_velocity_rad_s > 0.0:
            ratios.append(peak_velocity / max_velocity_rad_s)
        if max_acceleration_rad_s2 > 0.0:
            ratios.append(np.sqrt(peak_acceleration / max_acceleration_rad_s2))
        ratio = max(ratios)
        if ratio <= 1.001:
            break
        time_scale *= ratio * 1.01
    else:
        raise ValueError("upper-body velocity/acceleration conditioning did not converge")

    return output.astype(np.float32), {
        "input_peak_velocity_rad_s": input_velocity,
        "input_peak_acceleration_rad_s2": input_acceleration,
        "output_peak_velocity_rad_s": peak_velocity,
        "output_peak_acceleration_rad_s2": peak_acceleration,
        "time_scale": (len(output) - 1) / (len(values) - 1),
    }


def _yaw_free_wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    """Remove world yaw while preserving captured roll and pitch."""
    x, y, z, w = np.asarray(quat_xyzw, dtype=np.float64)
    norm = np.linalg.norm([w, x, y, z])
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("captured root quaternion is invalid")
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    roll = np.arctan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)
    )
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    result = np.asarray(
        [cr * cp, sr * cp, cr * sp, -sr * sp], dtype=np.float32
    )
    return result / np.linalg.norm(result)


def _angular_velocity(quat_wxyz: np.ndarray, fps: float) -> np.ndarray:
    """World-frame angular velocity from adjacent body orientations."""
    count, bodies, _ = quat_wxyz.shape
    result = np.zeros((count, bodies, 3), dtype=np.float32)
    if count == 1:
        return result
    previous = quat_wxyz[:-1]
    current = quat_wxyz[1:]
    # Relative world rotation: q_current * conjugate(q_previous).
    scalar = current[..., 0] * previous[..., 0] + np.sum(current[..., 1:] * previous[..., 1:], axis=-1)
    vector = (
        -current[..., 0:1] * previous[..., 1:]
        + previous[..., 0:1] * current[..., 1:]
        - np.cross(current[..., 1:], previous[..., 1:])
    )
    shortest = np.where(scalar < 0.0, -1.0, 1.0)
    scalar *= shortest
    vector *= shortest[..., None]
    length = np.linalg.norm(vector, axis=-1)
    angle = 2.0 * np.arctan2(length, scalar)
    scale = np.divide(angle, length, out=np.full_like(length, 2.0), where=length > 1e-8)
    intervals = vector * scale[..., None] * fps
    result[0] = intervals[0]
    result[-1] = intervals[-1]
    if count > 2:
        result[1:-1] = (intervals[:-1] + intervals[1:]) * 0.5
    return result


def complete(
    source: Path,
    output: Path,
    model_path: Path,
    profile_path: Path,
    policy_config: Path | None,
    require_arm_motion: bool,
    hold_frames: int,
    transition_frames: int,
    return_frames: int,
    return_settle_frames: int,
    tail_motion_threshold: float,
    joint_limit_margin: float,
    direct_upper_body: bool,
    upper_lowpass_cutoff_hz: float,
    upper_max_velocity_rad_s: float,
    upper_max_acceleration_rad_s2: float,
    mirror_arms: bool,
) -> None:
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    model_path = model_path.expanduser().resolve()
    profile_path = profile_path.expanduser().resolve()
    if source == output:
        raise ValueError("output must differ from source motion")
    if not direct_upper_body and (hold_frames < 0 or transition_frames < 2):
        raise ValueError(
            "hold_frames must be non-negative and transition_frames at least 2"
        )
    if not direct_upper_body and (return_frames < 2 or return_settle_frames < 1):
        raise ValueError(
            "return_frames must be at least 2 and return_settle_frames at least 1"
        )
    if tail_motion_threshold < 0.0:
        raise ValueError("tail_motion_threshold must be non-negative")
    if joint_limit_margin < 0.0:
        raise ValueError("joint_limit_margin must be non-negative")
    if upper_lowpass_cutoff_hz < 0.0:
        raise ValueError("upper_lowpass_cutoff_hz must be non-negative")
    if upper_max_velocity_rad_s < 0.0 or upper_max_acceleration_rad_s2 < 0.0:
        raise ValueError("upper-body velocity and acceleration limits must be non-negative")
    model_proto = onnx.load(str(source))
    onnx.checker.check_model(model_proto)
    metadata = {item.key: item.value for item in model_proto.metadata_props}
    fps = float(metadata["fps"])
    source_frame_count = int(metadata["frame_count"])
    upper_names = json.loads(metadata["joint_names"])
    if not np.isfinite(fps) or fps <= 0 or source_frame_count < 2:
        raise ValueError("motion metadata must contain positive fps and at least two frames")
    if not isinstance(upper_names, list) or len(set(upper_names)) != len(upper_names):
        raise ValueError("motion metadata has invalid joint_names")
    session = ort.InferenceSession(str(source), providers=["CPUExecutionProvider"])
    if len(session.get_inputs()) != 1 or session.get_inputs()[0].name != "time_seconds":
        raise ValueError("expected one time_seconds input")
    if [item.name for item in session.get_outputs()] != ["joint_positions"]:
        raise ValueError("expected one joint_positions output")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_names, body_names = _model_names(model)
    if len(joint_names) != 21 or not body_names or body_names[0] != "base_link":
        raise ValueError("expected an L0 model with 21 joints and base_link first")
    if not set(upper_names).issubset(joint_names):
        raise ValueError(f"unknown source joints: {sorted(set(upper_names) - set(joint_names))}")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if profile["joint_names"] != joint_names:
        raise ValueError("AMP profile joint order differs from the MuJoCo model")
    stable_joints = np.asarray(
        profile["joint_position_median_rad"], dtype=np.float32
    )
    if stable_joints.shape != (len(joint_names),) or not np.all(
        np.isfinite(stable_joints)
    ):
        raise ValueError("AMP profile must contain 21 finite joint positions")
    root_height = float(profile["base_position_median_m"][2])
    root_quat = _yaw_free_wxyz(
        np.asarray(profile["base_quaternion_median_xyzw"], dtype=np.float32)
    )
    if policy_config is not None:
        import yaml
        with policy_config.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if config["policy_joint_names"] != joint_names:
            raise ValueError("policy YAML joint order differs from the MuJoCo model")
        if config["motion"]["root_body_name"] != body_names[0]:
            raise ValueError("policy YAML root body differs from the MuJoCo model")

    values = np.stack([
        np.asarray(session.run(None, {"time_seconds": np.asarray(i / fps, dtype=np.float32)})[0], dtype=np.float32)
        for i in range(source_frame_count)
    ])
    if values.shape != (source_frame_count, len(upper_names)) or not np.all(np.isfinite(values)):
        raise ValueError(f"invalid trajectory shape or values: {values.shape}")
    if mirror_arms:
        values = _mirror_arm_trajectory(values, upper_names)
        print("MIRROR_ARMS camera-view left/right -> robot anatomical left/right")
    # Apply physical joint bounds before filtering/rate limiting.  Clipping
    # afterwards could create a fresh velocity or acceleration discontinuity.
    for column, name in enumerate(upper_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            safe_low, safe_high = (
                low + joint_limit_margin,
                high - joint_limit_margin,
            )
            if safe_low > safe_high:
                raise ValueError(f"joint limit margin is too large for {name}")
            clipped = np.clip(values[:, column], safe_low, safe_high)
            changed = int(
                np.count_nonzero(np.abs(clipped - values[:, column]) > 1e-7)
            )
            if changed:
                print(f"JOINT_LIMIT_CLIP {name}: {changed} source frames")
            values[:, column] = clipped
    arm_columns = [
        index for index, name in enumerate(upper_names)
        if "shoulder" in name or "elbow" in name
    ]
    arm_span = float(np.max(np.ptp(values[:, arm_columns], axis=0))) if arm_columns else 0.0
    if arm_span < 0.05:
        message = f"source has no visible arm motion (max arm joint span {arm_span:.4f} rad)"
        if require_arm_motion:
            raise ValueError(message)
        print(f"ARM_MOTION_WARNING {message}")

    # Source videos commonly contain a static standing tail after the arm has
    # returned. Preserve the final frame reached by meaningful source motion,
    # discard only the trailing no-motion portion, and append our own bounded
    # return to the standard standing profile below.  The test deliberately
    # considers every exported source joint so head-only WBC actions are also
    # handled correctly.
    source_steps = np.max(np.abs(np.diff(values, axis=0)), axis=1)
    moving_steps = np.flatnonzero(source_steps > tail_motion_threshold)
    kept_source_frame_count = source_frame_count
    if moving_steps.size:
        kept_source_frame_count = min(
            source_frame_count, int(moving_steps[-1]) + 2
        )
    trimmed_source_frames = source_frame_count - kept_source_frame_count
    if trimmed_source_frames:
        values = values[:kept_source_frame_count].copy()
    print(
        "TRIM_TRAILING_STATIC "
        f"original={source_frame_count} kept={kept_source_frame_count} "
        f"removed={trimmed_source_frames} threshold={tail_motion_threshold:g}"
    )

    conditioning_stats = {
        "input_peak_velocity_rad_s": _trajectory_peaks(values, fps)[0],
        "input_peak_acceleration_rad_s2": _trajectory_peaks(values, fps)[1],
        "output_peak_velocity_rad_s": _trajectory_peaks(values, fps)[0],
        "output_peak_acceleration_rad_s2": _trajectory_peaks(values, fps)[1],
        "time_scale": 1.0,
    }
    if direct_upper_body and (
        upper_lowpass_cutoff_hz > 0.0
        or upper_max_velocity_rad_s > 0.0
        or upper_max_acceleration_rad_s2 > 0.0
    ):
        input_frame_count = len(values)
        values, conditioning_stats = _condition_upper_body_trajectory(
            values,
            fps,
            upper_lowpass_cutoff_hz,
            upper_max_velocity_rad_s,
            upper_max_acceleration_rad_s2,
        )
        print(
            "CONDITION_UPPER_BODY "
            f"frames={input_frame_count}->{len(values)} "
            f"time_scale={conditioning_stats['time_scale']:.4f} "
            f"peak_velocity={conditioning_stats['input_peak_velocity_rad_s']:.4f}"
            f"->{conditioning_stats['output_peak_velocity_rad_s']:.4f}rad/s "
            f"peak_acceleration={conditioning_stats['input_peak_acceleration_rad_s2']:.4f}"
            f"->{conditioning_stats['output_peak_acceleration_rad_s2']:.4f}rad/s^2"
        )

    if direct_upper_body:
        # Text WBC references must begin at the generated motion itself.  Do
        # not prepend/append the AMP upper-body pose: only use AMP values for
        # joints absent from the source (the 12 lower-body joints).
        effective_hold_frames = 0
        effective_transition_frames = 0
        effective_return_frames = 0
        effective_return_settle_frames = 0
    else:
        effective_hold_frames = hold_frames
        effective_transition_frames = transition_frames
        effective_return_frames = return_frames
        effective_return_settle_frames = return_settle_frames

    prefix_frames = effective_hold_frames + effective_transition_frames
    processed_source_frame_count = len(values)
    source_end_frame = prefix_frames + processed_source_frame_count
    return_end_frame = source_end_frame + effective_return_frames
    frame_count = return_end_frame + effective_return_settle_frames
    joint_pos = np.repeat(stable_joints[None, :], frame_count, axis=0)
    if not direct_upper_body:
        alpha = np.linspace(0.0, 1.0, effective_transition_frames, dtype=np.float32)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)

    for column, name in enumerate(upper_names):
        target_column = joint_names.index(name)
        joint_pos[prefix_frames:source_end_frame, target_column] = values[:, column]
        if not direct_upper_body:
            joint_pos[effective_hold_frames:prefix_frames, target_column] = (
                stable_joints[target_column]
                + alpha * (values[0, column] - stable_joints[target_column])
            )
            return_alpha = np.linspace(
                1.0 / effective_return_frames, 1.0, effective_return_frames, dtype=np.float32
            )
            return_alpha = return_alpha * return_alpha * (3.0 - 2.0 * return_alpha)
            joint_pos[source_end_frame:return_end_frame, target_column] = (
                values[-1, column]
                + return_alpha * (stable_joints[target_column] - values[-1, column])
            )

    data = mujoco.MjData(model)
    qpos = np.repeat(model.qpos0[None, :], frame_count, axis=0)
    free_addresses = [
        int(model.jnt_qposadr[joint_id])
        for joint_id in range(model.njnt)
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
    ]
    if len(free_addresses) != 1:
        raise ValueError(f"expected one free root joint, got {len(free_addresses)}")
    root_address = free_addresses[0]
    qpos[:, root_address : root_address + 3] = [0.0, 0.0, root_height]
    qpos[:, root_address + 3 : root_address + 7] = root_quat
    for column, name in enumerate(joint_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos[:, model.jnt_qposadr[joint_id]] = joint_pos[:, column]
    body_pos = np.empty((frame_count, len(body_names), 3), dtype=np.float32)
    body_quat = np.empty((frame_count, len(body_names), 4), dtype=np.float32)
    for frame in range(frame_count):
        data.qpos[:] = qpos[frame]
        mujoco.mj_forward(model, data)
        body_pos[frame] = data.xpos[1:]
        body_quat[frame] = data.xquat[1:]

    dt = 1.0 / fps
    arrays = {
        "joint_pos": joint_pos,
        "joint_vel": np.gradient(joint_pos, dt, axis=0).astype(np.float32),
        "body_pos_w": body_pos,
        "body_quat_w": body_quat,
        "body_lin_vel_w": np.gradient(body_pos, dt, axis=0).astype(np.float32),
        "body_ang_vel_w": _angular_velocity(body_quat, fps),
    }
    lookup = MotionLookup(arrays).eval()
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        lookup,
        (torch.zeros((1, 1), dtype=torch.float32),),
        str(output),
        export_params=True,
        opset_version=18,
        input_names=["time_step"],
        output_names=list(OUTPUT_NAMES),
        dynamo=False,
    )
    _set_metadata(
        output,
        motion_path=source,
        fps=fps,
        num_frames=frame_count,
        joint_names=joint_names,
        body_names=body_names,
    )
    result = onnx.load(str(output))
    result.metadata_props.append(
        onnx.StringStringEntryProto(
            key="completion",
            value=(
                "AMP stable root/lower-body with conditioned direct source upper-body frames"
                if direct_upper_body else
                "AMP stable root/joints with a smooth upper-body pre-roll, "
                "trailing-static trim and return-to-stand suffix"
            ),
        )
    )
    result.metadata_props.append(onnx.StringStringEntryProto(key="source_joint_names", value=",".join(upper_names)))
    for key, value in {
        "amp_stand_profile": profile_path.name,
        "direct_upper_body": str(direct_upper_body).lower(),
        "mirror_arms": str(mirror_arms).lower(),
        "hold_frames": str(effective_hold_frames),
        "transition_frames": str(effective_transition_frames),
        "source_frames_original": str(source_frame_count),
        "source_frames_kept": str(kept_source_frame_count),
        "source_frames_processed": str(processed_source_frame_count),
        "source_frames_trimmed": str(trimmed_source_frames),
        "tail_motion_threshold": str(tail_motion_threshold),
        "upper_lowpass_cutoff_hz": str(upper_lowpass_cutoff_hz),
        "upper_max_velocity_rad_s": str(upper_max_velocity_rad_s),
        "upper_max_acceleration_rad_s2": str(upper_max_acceleration_rad_s2),
        "upper_input_peak_velocity_rad_s": str(conditioning_stats["input_peak_velocity_rad_s"]),
        "upper_input_peak_acceleration_rad_s2": str(conditioning_stats["input_peak_acceleration_rad_s2"]),
        "upper_output_peak_velocity_rad_s": str(conditioning_stats["output_peak_velocity_rad_s"]),
        "upper_output_peak_acceleration_rad_s2": str(conditioning_stats["output_peak_acceleration_rad_s2"]),
        "upper_time_scale": str(conditioning_stats["time_scale"]),
        "return_frames": str(effective_return_frames),
        "return_settle_frames": str(effective_return_settle_frames),
        "amp_root_quaternion_wxyz": json.dumps(root_quat.tolist()),
    }.items():
        result.metadata_props.append(
            onnx.StringStringEntryProto(key=key, value=value)
        )
    onnx.checker.check_model(result)
    onnx.save(result, str(output))
    verify = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    frames_to_check = sorted(
        frame
        for frame in {
            0,
            effective_hold_frames,
            prefix_frames - 1,
            prefix_frames,
            source_end_frame - 1,
            source_end_frame,
            return_end_frame - 1,
            frame_count - 1,
        }
        if 0 <= frame < frame_count
    )
    for frame in frames_to_check:
        actual = verify.run(None, {"time_step": np.asarray([[frame]], dtype=np.float32)})
        for name, value in zip(OUTPUT_NAMES, actual, strict=True):
            np.testing.assert_allclose(value[0], arrays[name][frame], rtol=1e-5, atol=1e-5)
    print(f"AMP_ROOT_WXYZ {root_quat.tolist()}")
    print(
        f"Exported {frame_count} frames at {fps:g} Hz "
        f"({frame_count / fps:.3f} s), with {effective_return_frames} return and "
        f"{effective_return_settle_frames} settle frames: {output}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upper-motion", type=Path, required=True)
    parser.add_argument(
        "--amp-profile",
        type=Path,
        default=DEFAULT_AMP_PROFILE,
        help=(
            "Standard L0 AMP standing profile used for both the start and end "
            f"pose (default: {DEFAULT_AMP_PROFILE})"
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--policy-config", type=Path)
    parser.add_argument(
        "--require-arm-motion", action="store_true",
        help="Reject references whose shoulder and elbow joints move less than 0.05 rad",
    )
    parser.add_argument("--hold-frames", type=int, default=10)
    parser.add_argument("--transition-frames", type=int, default=40)
    parser.add_argument(
        "--return-frames", type=int, default=12,
        help="Smooth return-to-standard-stand duration (default: 12 frames)",
    )
    parser.add_argument(
        "--return-settle-frames", type=int, default=2,
        help="Exact standard-stand frames after the return (default: 2)",
    )
    parser.add_argument(
        "--tail-motion-threshold", type=float, default=0.002,
        help="Per-frame joint delta below which a trailing source tail is static",
    )
    parser.add_argument("--joint-limit-margin", type=float, default=0.01)
    parser.add_argument(
        "--direct-upper-body",
        action="store_true",
        help=(
            "Export source upper-body frames directly with AMP lower-body values; "
            "disable the AMP upper-body hold, entry transition, return and settle"
        ),
    )
    parser.add_argument("--upper-lowpass-cutoff-hz", type=float, default=0.0)
    parser.add_argument("--upper-max-velocity-rad-s", type=float, default=0.0)
    parser.add_argument("--upper-max-acceleration-rad-s2", type=float, default=0.0)
    parser.add_argument(
        "--mirror-arms",
        action="store_true",
        help=(
            "Mirror camera-view arm semantics to robot anatomical sides; shoulder "
            "pitch is swapped and shoulder/elbow roll are swapped with sign inversion"
        ),
    )
    args = parser.parse_args()
    complete(
        args.upper_motion,
        args.output,
        args.model,
        args.amp_profile,
        args.policy_config,
        args.require_arm_motion,
        args.hold_frames,
        args.transition_frames,
        args.return_frames,
        args.return_settle_frames,
        args.tail_motion_threshold,
        args.joint_limit_margin,
        args.direct_upper_body,
        args.upper_lowpass_cutoff_hz,
        args.upper_max_velocity_rad_s,
        args.upper_max_acceleration_rad_s2,
        args.mirror_arms,
    )


if __name__ == "__main__":
    main()
