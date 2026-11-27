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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="complete exported motion directory")
    parser.add_argument("output", type=Path, help="empty deployment policy directory")
    parser.add_argument("--mode", choices=("static", "lora", "install-lora"), default="static")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--actions", nargs="+", help="selected LoRA action IDs")
    selection.add_argument("--all-actions", action="store_true", help="export every action in emotion_index.json")
    parser.add_argument("--base-actions", nargs="+", default=[], help="subset of --actions using complete Base parameters")
    parser.add_argument("--samples", type=int, default=33, help="LoRA CPU verification samples")
    parser.add_argument("--fp16", action="store_true", help="export and verify FP16 Base/reference storage in LoRA mode")
    args = parser.parse_args()
    if args.mode == "lora":
        if not args.actions and not args.all_actions:
            parser.error("--mode lora requires --actions or --all-actions")
        from export_wbc_lora import export_delivery
        if len(set(args.base_actions)) != len(args.base_actions) or (args.actions and not set(args.base_actions) <= set(args.actions)):
            parser.error("--base-actions must be a unique subset of --actions")
        export_delivery(args.source, args.output, args.actions, args.samples, args.base_actions, args.fp16)
    else:
        if args.actions or args.all_actions or args.base_actions or args.fp16:
            parser.error("--actions/--all-actions/--base-actions/--fp16 require --mode lora")
        operation = install_lora if args.mode == "install-lora" else prepare
        operation(args.source.resolve(), args.output.resolve())
