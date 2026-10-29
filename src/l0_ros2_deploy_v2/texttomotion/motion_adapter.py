"""Robot boundary for the unmodified text_to_motion_onnx.zip delivery."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
import time

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
BASELINE = HERE / "baseline"
SOURCE_SHA256 = "48485ea7f14fd7e52920f7f59e4da20515b9f043e0aa388ee2a3e43130a84119"
JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint",
]
TERMINAL = {"completed", "cancelled", "rejected", "failed", "stopped", "no_motion"}


def valid_id(value):
    return isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value) is not None


def verify_baseline():
    record = json.loads((BASELINE / "BASELINE.json").read_text(encoding="utf-8"))
    if record["source_sha256"] != SOURCE_SHA256:
        raise ValueError("Wrong text-motion source delivery")
    for relative, expected in record["files"].items():
        path = (BASELINE / relative).resolve()
        if not path.is_relative_to(BASELINE.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError("Changed baseline file: " + relative)


def open_pipeline(manifest, backend):
    verify_baseline()
    source_manifest = BASELINE / "onnx/manifest.json"
    expected = hashlib.sha256(source_manifest.read_bytes()).hexdigest()
    manifest = Path(manifest).resolve()
    if not manifest.is_file():
        raise FileNotFoundError("Provide model_manifest pointing to the current delivery's manifest.json")
    info = json.loads(manifest.read_text(encoding="utf-8"))
    if backend == "lite2":
        if info.get("preparation_manifest_sha256") != expected:
            raise ValueError("RKNN must be compiled from the current ONNX delivery; old compiled packages are refused")
    elif backend == "onnx-reference":
        if hashlib.sha256(manifest.read_bytes()).hexdigest() != expected:
            raise ValueError("CPU reference must use the current delivery's ONNX manifest")
    else:
        raise ValueError("backend must be lite2 or onnx-reference")
    if info.get("trajectory_columns") != ["right_hand[0:3]", "left_hand[3:6]", "head[6:9]"]:
        raise ValueError("Unknown model coordinate order")
    if info["motion_metadata"]["joint_names"] != JOINTS or info.get("fps") != 50:
        raise ValueError("Unknown model joint metadata/FPS")
    # Always run the host implementation shipped in the current source zip.
    if backend == "lite2":
        from board_runtime import rknn_lite_class
        rknn_lite_class()
    sys.path.insert(0, str(BASELINE / "onnx"))
    from deploy_rknn.pipeline import RknnMotionPipeline
    return RknnMotionPipeline(manifest, backend)


def load_limits(mapping_file):
    mapping = yaml.safe_load(Path(mapping_file).read_text(encoding="utf-8"))
    limits = []
    for name in JOINTS:
        matches = [motor for motor in mapping["motors"] if motor["joint_name"] == name]
        if len(matches) != 1 or matches[0].get("enabled") is not True:
            raise ValueError("Missing/duplicate/disabled joint in mapping: " + name)
        motor = matches[0]
        lo, hi = float(motor["joint_lower"]), float(motor["joint_upper"])
        if not np.isfinite([lo, hi]).all() or lo >= hi:
            raise ValueError("Invalid joint limits: " + name)
        limits.append((lo, hi))
    return np.asarray(limits, dtype=np.float64)


def check_pose(pose, limits):
    if np.asarray(pose).dtype.kind not in "fiu":
        raise ValueError("Pose values must be numeric, excluding booleans")
    result = np.asarray(pose, dtype=np.float32)
    if result.shape != (9,) or not np.isfinite(result).all():
        raise ValueError("Pose must contain nine finite radians in robot joint order")
    if np.any(result < limits[:, 0] - 1e-6) or np.any(result > limits[:, 1] + 1e-6):
        raise ValueError("Pose exceeds the current robot mapping limits")
    return np.clip(result, limits[:, 0], limits[:, 1]).astype(np.float32)


def mirror_arms(value):
    """Involution between delivered viewer-side columns and anatomical joints.

    Swap arms and invert shoulder/elbow roll; pitch and head axes are unchanged.
    Apply once to BOTH model inputs and outputs, never to already adapted JSON.
    """
    value = np.asarray(value, dtype=np.float32)
    result = value.copy()
    sign = np.asarray([1, -1, -1], dtype=np.float32)
    result[..., :3] = value[..., 3:6] * sign
    result[..., 3:6] = value[..., :3] * sign
    return result


def configure_generation_limits(pipeline, limits):
    """Use mapping limits in every original composer operation, per pipeline.

    The frozen composer keeps its limits in module globals. Give this pipeline
    a private module instance so another pipeline's mapping cannot change them.
    Source files, model tensors and the original module remain untouched.
    """
    limits = np.asarray(limits, dtype=np.float64)
    if limits.shape != (9, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError("Generation needs nine finite ordered robot joint limits")
    if np.array_equal(getattr(pipeline, "_robot_generation_limits", None), limits):
        return
    lower, upper = mirror_arms(limits[:, 0]), mirror_arms(limits[:, 1])
    model_limits = np.stack([np.minimum(lower, upper), np.maximum(lower, upper)], axis=1)
    spec = importlib.util.spec_from_file_location("_l0_robot_motion", BASELINE / "onnx/deploy_rknn/motion.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The source composer uses head/left/right parts; trajectory columns use
    # right/left/head. mirror_arms already handles anatomical sides and signs.
    module.LIMITS = np.stack([model_limits[6:9], model_limits[3:6], model_limits[:3]])
    module.FULL_LIMITS = model_limits
    module.LIMITS.setflags(write=False)
    module.FULL_LIMITS.setflags(write=False)
    previous = pipeline.composer
    pipeline.composer = module.MotionComposer(previous.metadata, previous.atoms, previous.lengths, previous.a_pose)
    pipeline._robot_generation_limits = limits.copy()


def validate_request(request):
    allowed = {"command", "request_id", "mode", "text", "token_ids", "speed", "seed",
               "return_home", "return_frames", "end_pose", "limit_policy"}
    if not isinstance(request, dict) or set(request) - allowed:
        raise ValueError("Unknown request fields")
    if not valid_id(request.get("request_id")) or request.get("command", "generate") != "generate":
        raise ValueError("Invalid generate request/id")
    mode = request.get("mode", "text")
    if mode not in ("text", "web"):
        raise ValueError("mode must be text or web")
    if mode == "text":
        text = request.get("text", "")
        if not isinstance(text, str) or not text.strip() or len(text) > 128:
            raise ValueError("text must contain 1..128 characters; whitespace is preserved")
    else:
        tokens = request.get("token_ids")
        if not isinstance(tokens, list) or len(tokens) != 3 or any(type(t) is not int for t in tokens):
            raise ValueError("web mode requires three integer token IDs: head/left/right")
    speed = request.get("speed", 1.2)
    if type(speed) not in (int, float) or not np.isfinite(speed) or not 0.1 <= speed <= 4.0:
        raise ValueError("Robot playback accepts speed in [0.1, 4.0]")
    if type(request.get("return_home", True)) is not bool:
        raise ValueError("return_home must be boolean")
    frames = request.get("return_frames", -1)
    if type(frames) is not int or frames not in (-1, 30, 60, 90):
        raise ValueError("return_frames must be -1, 30, 60 or 90")
    seed = request.get("seed")
    if seed is not None and (type(seed) is not int or not 0 <= seed < 2**32):
        raise ValueError("seed must be an integer in [0, 2**32)")
    if request.get("limit_policy", "reject") not in ("reject", "clip"):
        raise ValueError("limit_policy must be reject or clip")


def generate(pipeline, request, initial, limits):
    validate_request(request)
    configure_generation_limits(pipeline, limits)
    initial = check_pose(initial, limits)
    end = check_pose(request.get("end_pose", initial), limits)
    output = pipeline.generate(
        mode=request.get("mode", "text"), text=request.get("text", ""), token_ids=request.get("token_ids"),
        init_pose=mirror_arms(initial), end_pose=mirror_arms(end),
        return_home=request.get("return_home", True), return_frames=request.get("return_frames", -1),
        speed=request.get("speed", 1.2), seed=request.get("seed"))
    tokens = np.asarray(output["token_ids"])
    if tokens.shape != (3,) or tokens.dtype.kind not in "iu":
        raise ValueError("Invalid model token IDs")
    # Token order is head/left/right. Own all three axes of each selected part,
    # including constant axes; variation thresholds do not express ownership.
    mask = np.repeat(tokens[[1, 2, 0]] != 0, 3)
    if not mask.any():
        return None
    motion = mirror_arms(output["motion"])
    count = np.asarray(output["frame_count"])
    if motion.ndim != 2 or motion.shape[1] != 9 or not 2 <= len(motion) <= 5000 or \
            count.shape != (1,) or count.dtype.kind not in "iu" or int(count[0]) != len(motion) or \
            not np.isfinite(motion).all():
        raise ValueError("Invalid complete model trajectory")
    if np.max(np.abs(motion[0, mask] - initial[mask])) > 0.005:
        raise ValueError("Generated first frame differs from the captured start pose")
    if request.get("return_home", True) and np.max(np.abs(motion[-1, mask] - end[mask])) > 0.005:
        raise ValueError("Generated active parts do not reach the requested end pose")
    clipped = np.clip(motion, limits[:, 0], limits[:, 1])
    adjustments = np.abs(clipped - motion)
    if np.any(adjustments > 1e-6) and request.get("limit_policy", "reject") == "reject":
        names = [name for j, name in enumerate(JOINTS) if np.any(adjustments[:, j] > 1e-6)]
        raise ValueError("Model exceeds robot limits: " + ", ".join(names) + "; explicit limit_policy=clip is required")
    return {
        "format": "l0_text_motion_v2", "robot": "l0_v5", "angle_unit": "rad", "fps": 50,
        "coordinate_contract": "robot_anatomical_v1", "request_id": request["request_id"],
        "source_zip_sha256": SOURCE_SHA256, "joint_names": JOINTS, "active_joints": mask.tolist(),
        "positions": clipped.tolist(), "frame_count": len(motion),
        "token_ids": tokens.tolist(), "material_ids": np.asarray(output["material_ids"]).tolist(),
        "backend": pipeline.backend, "inference_ms": output["inference_ms"],
        "generation_request": request, "start_pose": initial.tolist(), "end_pose": end.tolist(),
        "model_diagnostics": {
            "generation_joint_limits_rad": {name: list(map(float, bound)) for name, bound in zip(JOINTS, limits)},
            **{key: np.asarray(output[key]).tolist()
            for key in ("part_transition_frames", "action_frame_count_before_speed", "frame_count_before_speed",
                        "return_home_start_frame", "selector")}},
        "limit_policy": request.get("limit_policy", "reject"),
        "limit_adjustments": {name: {"count": int(np.count_nonzero(adjustments[:, j] > 1e-6)),
                                      "max_rad": float(adjustments[:, j].max())}
                              for j, name in enumerate(JOINTS)},
        # Renewed immediately before sending; this default also bounds offline replay.
        "expires_at_ms": int(time.time() * 1000) + 2000,
    }
