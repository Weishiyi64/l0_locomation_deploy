#!/usr/bin/env python3
"""Package a static MimicLite export or a shared WBC LoRA training delivery."""

import argparse
import hashlib
import json
import re
import shutil
import tempfile
from pathlib import Path

import numpy as np
import yaml

from convert_onnx_to_rknn_rk3588_wbc_policy import check_model
from convert_onnx_to_rknn_rk3588_wbc_reference import check_reference_model

HISTORY_STEPS = [0, 1, 2, 3, 4, 8, 16]
FUTURE_STEPS = [-8, -4, -2, 0, 1, 2, 3, 4]


def joint_values(mapping, names, label):
    values = []
    for name in names:
        matches = [float(value) for pattern, value in mapping.items() if re.fullmatch(pattern, name)]
        if len(matches) != 1 or not np.isfinite(matches[0]):
            raise ValueError(f"{label}: expected one finite value for {name}")
        values.append(matches[0])
    return values


def prepare(source, output):
    name = source.name
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", name):
        raise ValueError("source directory name must be a motion id of 1-32 letters, digits, '_' or '-'")
    manifest_path = source / f"{name}.export.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["robot"] != "l0_v5_wbc_pd" or manifest["motion"].split("/")[-1] != name:
        raise ValueError("expected a matching l0_v5_wbc_pd motion export")
    files = {
        "checkpoint": source / f"{name}_bestpoint.pt",
        "onnx": source / f"{name}.onnx",
        "yaml": source / f"{name}.yaml",
        "reference_motion": source / f"{name}_reference_motion.onnx",
    }
    for key, path in files.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest[f"{key}_sha256"]:
            raise ValueError(f"source checksum mismatch: {path}")
    inputs, _ = check_model(files["onnx"])
    command_name = next(key for key, _ in inputs if key != "policy")
    config = yaml.safe_load(files["yaml"].read_text(encoding="utf-8"))
    names = config["policy_joint_names"]
    if len(names) != 21 or len(set(names)) != 21 or names != config["joint_names_simulation"]:
        raise ValueError("expected 21 unique joints in the simulation/policy order")
    policy_obs = config["observation"]["policy"]
    if list(policy_obs) != ["root_ang_vel_history", "projected_gravity_history", "joint_pos_history",
                            "joint_vel_history", "prev_actions"]:
        raise ValueError("unexpected policy observation order")
    if any(policy_obs[key]["history_steps"] != HISTORY_STEPS for key in list(policy_obs)[:4]) or \
            policy_obs["prev_actions"]["steps"] != 3:
        raise ValueError("expected history offsets 0/1/2/3/4/8/16 and 3 previous actions")
    command = config["observation"]["command_short"]
    if list(command) != ["ref_root_pos_future_local", "ref_root_ori_future_b", "ref_joint_pos_future"] or \
            any(value["future_steps"] != FUTURE_STEPS for value in command.values()) or \
            config["motion"]["future_steps"] != FUTURE_STEPS or \
            command["ref_root_pos_future_local"]["body_names"] != "base_link":
        raise ValueError("unexpected command signals, order, root body or future offsets")

    _, _, meta = check_reference_model(files["reference_motion"])
    if meta["joint_names"].split(",") != names:
        raise ValueError("reference joint order does not match policy")
    fps, count = float(meta["fps"]), int(meta["num_frames"])
    if float(config.get("policy_frequency", fps)) != fps:
        raise ValueError("reference and policy frequency mismatch")
    root_index = meta["body_names"].split(",").index("base_link")

    robot = {
        "policy_type": "rl_wbc", "policy_name": "policy", "policy_frequency": 50,
        "lowlevel_frequency": 500, "num_actions": 21,
        "joint_order_mj": names, "joint_order_isaac": names.copy(),
        "default_joint_angles": joint_values(config["default_joint_pos"], names, "default_joint_pos"),
        "kps": joint_values(config["joint_kp"], names, "joint_kp"),
        "kds": joint_values(config["joint_kd"], names, "joint_kd"),
        "action_scale": [float(value) for value in config["action_scale"]],
        # Same default as the handoff's build_monolithic_robot_policy.py.
        "clip_actions": float(config.get("clip_actions", 1000000.0)),
    }
    if len(robot["action_scale"]) != 21 or not np.isfinite(robot["action_scale"]).all() or \
            min(robot["kps"] + robot["kds"] + robot["action_scale"]) < 0 or \
            not np.isfinite(robot["clip_actions"]) or robot["clip_actions"] <= 0:
        raise ValueError("invalid gains, action scales or action clipping")
    deploy = {"RobotCfg": robot, "WbcCfg": {
        "motion_id": name,
        "reference_num_frames": count, "reference_root_body_index": root_index,
        "control_dt": 0.02, "start_hold_sec": 1.0, "history_steps": HISTORY_STEPS,
        "future_steps": FUTURE_STEPS, "previous_action_steps": 3,
        "policy_input_name": "policy", "command_input_name": command_name, "output_name": "action",
    }}
    if output.exists() and any(output.iterdir()):
        raise ValueError("output directory must be empty; do not mix an old RKNN with a new motion")
    output.mkdir(parents=True, exist_ok=True)
    (output / "rl_config.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False), encoding="utf-8")
    for path, filename in [(files["onnx"], "policy.onnx"), (files["yaml"], "source_policy.yaml"),
                           (files["reference_motion"], "reference_motion.onnx")]:
        shutil.copyfile(path, output / filename)
    print(f"Packaged {name}: {count} frames at {fps:g} Hz, {count / fps:.2f}s + 1s start hold")
    print(f"Convert {output / 'policy.onnx'} and {output / 'reference_motion.onnx'} to RKNN before deployment")


def install_lora(source, output):
    """Stage a complete converted bundle for colcon; runtime acceptance is separate."""
    from wbc_lora_assets import ID_PATTERN, asset_path, check_graph, load_adapter, read_json, sha256

    if output.exists():
        raise ValueError("install-lora requires a new output directory; use a versioned bundle name for updates")
    if output.is_relative_to(source):
        raise ValueError("installation output must be outside the source delivery")
    manifest_path = source / "base_manifest.json"
    manifest = read_json(manifest_path)
    graph = check_graph(asset_path(source, manifest["model"]["onnx"]), manifest_path)
    if graph["kind"] != "policy":
        raise ValueError("install-lora requires the full shared policy, not a probe")
    files = {manifest_path: sha256(manifest_path)}

    def include(relative, expected=None):
        path = asset_path(source, relative)
        if not path.is_file():
            raise ValueError(f"Missing deployment asset: {path}")
        actual = sha256(path)
        if expected is not None and actual != expected:
            raise ValueError(f"Deployment asset checksum mismatch: {path}")
        files[path] = actual
        return path

    def model(onnx, rknn, record_name, expected, graph_record=False):
        include(onnx, expected)
        if record_name != rknn + ".json":
            raise ValueError("RKNN conversion record must be beside its model")
        record = read_json(include(record_name))
        if record.get("format") != "wbc_rknn_conversion_v1" or record.get("source_sha256") != expected or \
                record.get("precision") != "float16" or record.get("do_quantization") is not False:
            raise ValueError(f"Invalid conversion record: {record_name}")
        if graph_record and record.get("graph_manifest_sha256") != files[manifest_path]:
            raise ValueError("Shared RKNN was converted with another graph manifest")
        include(rknn, record["rknn_sha256"])

    shared = graph["model"]
    model(shared["onnx"], shared["rknn"], shared["rknn_record"], shared["sha256"], True)
    seen = set()
    for action in graph["actions"]:
        action_id = action["action_id"]
        if not ID_PATTERN.fullmatch(action_id) or action_id in seen:
            raise ValueError("Invalid or duplicate action ID")
        seen.add(action_id)
        adapter = include(action["adapter_manifest"])
        load_adapter(adapter, graph, action_id)
        info = read_json(adapter)
        weights = asset_path(adapter.parent, info["weights"]["path"])
        include(str(weights.relative_to(source)), info["weights"]["sha256"])
        config = include(action["config"], action["config_sha256"])
        if yaml.safe_load(config.read_text(encoding="utf-8"))["WbcCfg"]["motion_id"] != action_id:
            raise ValueError(f"Action/config identity mismatch: {action_id}")
        check_reference_model(asset_path(source, action["reference_onnx"]))
        model(action["reference_onnx"], action["reference_rknn"], action["reference_rknn_record"],
              action["reference_onnx_sha256"])
    if not seen:
        raise ValueError("No actions in deployment bundle")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wbc_lora_stage_", dir=output.parent) as temporary:
        staged = Path(temporary) / "bundle"
        staged.mkdir()
        for path, digest in files.items():
            destination = staged / path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
            if sha256(destination) != digest:
                raise ValueError(f"Source changed during installation: {path}")
        (staged / "INSTALLATION.json").write_text(json.dumps({
            "format": "wbc_lora_installation_v1", "validation_status": "pending_formal_runtime",
            "files": {str(path.relative_to(source)): digest for path, digest in files.items()},
        }, indent=2) + "\n", encoding="utf-8")
        if output.exists():
            raise ValueError("Installation destination appeared during staging")
        staged.rename(output)
    print(f"Installed {len(seen)} actions / one shared WBC Base at {output}")
    print("Asset checks passed; colcon build and formal C++ runtime validation remain required")


def prepare_json_batch(source, output, checkpoint_dir, profile_path, actions, checkpoint_stem="checkpoint_10000"):
    """Complete upper-body JSON with a measured L0 v5 AMP stand, sharing one policy."""
    import mujoco
    import onnx
    import onnxruntime as ort
    from onnx import TensorProto, helper, numpy_helper

    if output.exists():
        raise ValueError("JSON batch output must be a new directory")
    if not actions or len(set(actions)) != len(actions) or any(
            not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", name) for name in actions):
        raise ValueError("expected unique action stems without paths")
    if not re.fullmatch(r"checkpoint_[0-9]+", checkpoint_stem):
        raise ValueError("checkpoint stem must be checkpoint_<step>")
    policy = checkpoint_dir / (checkpoint_stem + ".onnx")
    policy_yaml = checkpoint_dir / (checkpoint_stem + ".yaml")
    specs, _ = check_model(policy)
    command_name = next(name for name, _ in specs if name != "policy")
    cfg = yaml.safe_load(policy_yaml.read_text(encoding="utf-8"))
    names = cfg["policy_joint_names"]
    if len(names) != 21 or len(set(names)) != 21 or names != cfg["joint_names_simulation"]:
        raise ValueError("checkpoint must use the L0 21-joint policy order")
    obs = cfg["observation"]["policy"]
    if list(obs) != ["root_ang_vel_history", "projected_gravity_history", "joint_pos_history",
                     "joint_vel_history", "prev_actions"] or any(
            obs[key]["history_steps"] != HISTORY_STEPS for key in list(obs)[:4]) or obs["prev_actions"]["steps"] != 3:
        raise ValueError("checkpoint observation history does not match the WBC runtime")
    command = cfg["observation"]["command_short"]
    if list(command) != ["ref_root_pos_future_local", "ref_root_ori_future_b", "ref_joint_pos_future"] or any(
            value["future_steps"] != FUTURE_STEPS for value in command.values()):
        raise ValueError("checkpoint future reference inputs do not match the WBC runtime")
    model_path = Path(__file__).resolve().parent / "legged_description/l0_v5_description/mjcf/l0_v5.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
    if min(joint_ids) < 0:
        raise ValueError("checkpoint joints are absent from the L0 v5 model")
    bodies = list(cfg["body_names_simulation"])
    # Training lists 22 bodies. Deployment additionally models the rigid lidar.
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name not in bodies:
            bodies.append(name)
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in bodies]
    if len(bodies) != 23 or len(set(bodies)) != 23 or min(body_ids) < 1 or bodies[0] != "base_link":
        raise ValueError("expected 23 named reference bodies with base_link first")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if profile["robot_type"] != "l0_v5" or profile["joint_names"] != names:
        raise ValueError("capture a current L0 v5 AMP profile in policy joint order")
    stand = np.asarray(profile["joint_position_median_rad"], dtype=np.float32)
    bounds = np.asarray([model.jnt_range[index] for index in joint_ids], dtype=np.float32)
    if stand.shape != (21,) or not np.isfinite(stand).all() or np.max(
            np.abs(np.clip(stand, bounds[:, 0], bounds[:, 1]) - stand)) > 0.01:
        raise ValueError("captured AMP joints are invalid or outside mechanical bounds")
    stand = np.clip(stand, bounds[:, 0], bounds[:, 1])
    x, y, z, w = np.asarray(profile["base_quaternion_median_xyzw"], dtype=np.float64)
    norm = np.linalg.norm([w, x, y, z])
    if not np.isfinite(norm) or norm < 0.5:
        raise ValueError("invalid AMP root quaternion")
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    roll = np.arctan2(2 * (w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    cr, sr, cp, sp = np.cos(roll/2), np.sin(roll/2), np.cos(pitch/2), np.sin(pitch/2)
    root_quat = np.asarray([cr*cp, sr*cp, cr*sp, -sr*sp])
    height = float(profile["base_position_median_m"][2])
    if not np.isfinite(height) or height <= 0:
        raise ValueError("invalid AMP root height")
    robot = {"policy_type": "rl_wbc", "policy_name": "policy", "policy_frequency": 50,
             "lowlevel_frequency": 500, "num_actions": 21,
             "joint_order_mj": names, "joint_order_isaac": names.copy(),
             "default_joint_angles": joint_values(cfg["default_joint_pos"], names, "default_joint_pos"),
             "kps": joint_values(cfg["joint_kp"], names, "joint_kp"),
             "kds": joint_values(cfg["joint_kd"], names, "joint_kd"),
             "action_scale": [float(x) for x in cfg["action_scale"]], "clip_actions": 1000000.0}
    if len(robot["action_scale"]) != 21 or not np.isfinite(robot["action_scale"]).all():
        raise ValueError("invalid checkpoint action scale")
    output.parent.mkdir(parents=True, exist_ok=True)
    records = []
    with tempfile.TemporaryDirectory(prefix=".wbc_json_", dir=output.parent) as temporary:
        staged = Path(temporary) / "wbc"
        staged.mkdir()
        shutil.copyfile(policy, staged / policy.name)
        shutil.copyfile(policy_yaml, staged / policy_yaml.name)
        shutil.copyfile(profile_path, staged / "amp_stand_profile.json")
        for name in actions:
            source_json = source / (name + ".json")
            document = json.loads(source_json.read_text(encoding="utf-8"))
            if document.get("format") != "gmr_upper_body_joint_trajectory_v1" or document.get("robot") not in ("l0_v3", "l0_v5") or \
                    document.get("angle_unit") not in ("rad", "radian") or document["joint_names"] != names[12:] or float(document["fps"]) != 50:
                raise ValueError(f"{name}: invalid upper-body schema, joint order or FPS")
            frames = document["frames"]
            upper = np.asarray([frame["joint_positions"] for frame in frames], dtype=np.float32)
            if upper.shape != (len(frames), 9) or not 2 <= len(frames) <= 10000 or not np.isfinite(upper).all() or any(
                    f.get("frame_index", i) != i or abs(float(f.get("time", i/50))-i/50) > 1e-5 for i, f in enumerate(frames)):
                raise ValueError(f"{name}: invalid frames")
            # Legacy JSON uses camera-view arm sides. New texttomotion JSON
            # already records the robot-side mirror in coordinate_adjustments.
            if document.get("coordinate_adjustments", {}).get("arm_side_mirror") is not True:
                original = upper.copy()
                upper[:, :3] = original[:, 3:6] * [1, -1, -1]
                upper[:, 3:6] = original[:, :3] * [1, -1, -1]
            bounded = np.clip(upper, bounds[12:, 0], bounds[12:, 1])
            clipped_count = int(np.count_nonzero(bounded != upper))
            upper = bounded
            hold, transition, returning, settle = 10, 40, 40, 10
            alpha = np.linspace(0, 1, transition, dtype=np.float32)[:, None]
            alpha = alpha * alpha * (3-2*alpha)
            entry = stand[12:] + alpha * (upper[0] - stand[12:])
            exit_pose = upper[-1] + alpha * (stand[12:] - upper[-1])
            upper = np.concatenate([np.repeat(stand[None, 12:], hold, axis=0), entry, upper,
                                    exit_pose, np.repeat(stand[None, 12:], settle, axis=0)])
            count = len(upper)
            joints = np.repeat(stand[None, :], count, axis=0)
            joints[:, 12:] = upper
            data = mujoco.MjData(model)
            data.qpos[:3] = [0, 0, height]
            data.qpos[3:7] = root_quat
            body_pos = np.empty((count, 23, 3), dtype=np.float32)
            body_quat = np.empty((count, 23, 4), dtype=np.float32)
            addresses = model.jnt_qposadr[joint_ids]
            for i, row in enumerate(joints):
                data.qpos[addresses] = row
                mujoco.mj_forward(model, data)
                body_pos[i] = data.xpos[body_ids]
                body_quat[i] = data.xquat[body_ids]
            previous, current = body_quat[:-1], body_quat[1:]
            scalar = current[..., 0]*previous[..., 0] + np.sum(current[..., 1:]*previous[..., 1:], axis=-1)
            vector = (-current[..., :1]*previous[..., 1:] + previous[..., :1]*current[..., 1:] -
                      np.cross(current[..., 1:], previous[..., 1:]))
            sign = np.where(scalar < 0, -1, 1)
            scalar *= sign
            vector *= sign[..., None]
            length = np.linalg.norm(vector, axis=-1)
            scale = np.divide(2*np.arctan2(length, scalar), length, out=np.full_like(length, 2), where=length > 1e-8)
            intervals = vector * scale[..., None] * 50
            angular = np.concatenate([intervals[:1], (intervals[:-1]+intervals[1:])*0.5, intervals[-1:]])
            arrays = {"joint_pos": joints, "joint_vel": np.gradient(joints, 0.02, axis=0),
                      "body_pos_w": body_pos, "body_quat_w": body_quat,
                      "body_lin_vel_w": np.gradient(body_pos, 0.02, axis=0), "body_ang_vel_w": angular}
            if any(not np.isfinite(value).all() for value in arrays.values()) or np.ptp(joints[:, :12], axis=0).max() != 0:
                raise ValueError(f"{name}: invalid reference or moving lower-body reference")
            directory = staged / name
            directory.mkdir()
            initializers = [numpy_helper.from_array(np.asarray(v, dtype=np.float32), k+"_table") for k, v in arrays.items()]
            initializers += [numpy_helper.from_array(np.asarray([v] if k == "shape" else v, dtype=np.int64), k)
                             for k, v in (("zero", 0), ("last", count-1), ("shape", 1))]
            nodes = [helper.make_node("Cast", ["time_step"], ["integer"], to=TensorProto.INT64),
                     helper.make_node("Clip", ["integer", "zero", "last"], ["bounded"]),
                     helper.make_node("Reshape", ["bounded", "shape"], ["index"])]
            nodes += [helper.make_node("Gather", [k+"_table", "index"], [k], axis=0) for k in arrays]
            graph = helper.make_graph(nodes, name, [helper.make_tensor_value_info("time_step", TensorProto.FLOAT, [1, 1])],
                                      [helper.make_tensor_value_info(k, TensorProto.FLOAT, [1]+list(v.shape[1:])) for k, v in arrays.items()], initializers)
            reference = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 18)], ir_version=8)
            metadata = {"fps": "50", "num_frames": str(count), "joint_names": ",".join(names),
                        "body_names": ",".join(bodies), "quaternion_order": "wxyz",
                        "time_step_semantics": "zero_based_frame_index_clamped",
                        "lower_body": "constant_measured_l0_v5_amp_stand", "source_json": source_json.name}
            helper.set_model_props(reference, metadata)
            onnx.checker.check_model(reference)
            reference_path = directory / "reference_motion.onnx"
            onnx.save(reference, reference_path)
            check_reference_model(reference_path)
            session = ort.InferenceSession(str(reference_path), providers=["CPUExecutionProvider"])
            for i in (0, hold+transition, count-1):
                actual = session.run(list(arrays), {"time_step": np.asarray([[i]], dtype=np.float32)})
                for key, value in zip(arrays, actual):
                    np.testing.assert_allclose(value[0], arrays[key][i], atol=1e-6)
            deploy = {"RobotCfg": robot, "WbcCfg": {"motion_id": name, "reference_num_frames": count,
                      "reference_root_body_index": 0, "control_dt": 0.02, "start_hold_sec": 1.0,
                      "history_steps": HISTORY_STEPS, "future_steps": FUTURE_STEPS, "previous_action_steps": 3,
                      "policy_input_name": "policy", "command_input_name": command_name, "output_name": "action"}}
            (directory / "rl_config.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False), encoding="utf-8")
            (directory / "policy.onnx").symlink_to("../" + checkpoint_stem + ".onnx")
            shutil.copyfile(source_json, directory / "source.json")
            records.append({"motion_id": name, "source_sha256": hashlib.sha256(source_json.read_bytes()).hexdigest(),
                            "reference_sha256": hashlib.sha256(reference_path.read_bytes()).hexdigest(),
                            "source_frames": len(frames), "reference_frames": count, "clipped_values": clipped_count,
                            "hold_frames": hold, "transition_frames": transition, "return_frames": returning, "settle_frames": settle})
            print(f"Packaged {name}: {len(frames)} source / {count} reference frames, clipped={clipped_count}")
        manifest = {"format": "l0_v5_wbc_json_batch_v1", "checkpoint_stem": checkpoint_stem,
                    "checkpoint_sha256": hashlib.sha256(policy.read_bytes()).hexdigest(),
                    "checkpoint_yaml_sha256": hashlib.sha256(policy_yaml.read_bytes()).hexdigest(),
                    "amp_profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(), "actions": records,
                    "rknn_status": "not_converted"}
        (staged / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
        staged.rename(output)
    print(f"WBC JSON bundle: {output}; ONNX/reference validation passed; RKNN is required only for RK3588 deployment")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="complete exported motion directory")
    parser.add_argument("output", type=Path, help="empty deployment policy directory")
    parser.add_argument("--mode", choices=("static", "lora", "install-lora", "json-batch"), default="static")
    parser.add_argument("--checkpoint-dir", type=Path, help="WBC checkpoint_<step>.onnx/yaml directory")
    parser.add_argument("--checkpoint-stem", default="checkpoint_10000", help="json-batch model stem")
    parser.add_argument("--amp-profile", type=Path, help="measured L0 v5 AMP standing profile")
    parser.add_argument("--actions", nargs="+", help="selected LoRA action IDs")
    parser.add_argument("--base-actions", nargs="+", default=[], help="subset of --actions using complete Base parameters")
    parser.add_argument("--samples", type=int, default=33, help="LoRA CPU verification samples")
    args = parser.parse_args()
    if args.mode == "json-batch":
        if not args.checkpoint_dir or not args.amp_profile or not args.actions:
            parser.error("json-batch requires --checkpoint-dir, --amp-profile and --actions")
        prepare_json_batch(args.source.resolve(), args.output.resolve(), args.checkpoint_dir.resolve(),
                           args.amp_profile.resolve(), args.actions, args.checkpoint_stem)
    elif args.mode == "lora":
        if not args.actions:
            parser.error("--mode lora requires --actions")
        from export_wbc_lora import export_delivery
        if len(set(args.base_actions)) != len(args.base_actions) or not set(args.base_actions) <= set(args.actions):
            parser.error("--base-actions must be a unique subset of --actions")
        export_delivery(args.source, args.output, args.actions, args.samples, args.base_actions)
    else:
        if args.actions or args.base_actions:
            parser.error("--actions/--base-actions require --mode lora")
        operation = install_lora if args.mode == "install-lora" else prepare
        operation(args.source.resolve(), args.output.resolve())
