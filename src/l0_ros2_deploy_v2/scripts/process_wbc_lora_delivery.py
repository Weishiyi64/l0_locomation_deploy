#!/usr/bin/env python3
"""Process one training delivery locally into a verified, checkpoint-free board bundle.

Run with the export Python environment (torch/numpy/PyYAML/onnx/onnxruntime).
The source is read-only. No network, RKNN build, ROS or hardware is invoked.
"""

import argparse
import datetime
import json
import os
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parents[2]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def training_files(source, actions):
    """Freeze the selected inputs, including any supplied full checkpoints."""
    from wbc_lora_assets import ID_PATTERN, asset_path, sha256

    index = json.loads((source / "emotion_index.json").read_text(encoding="utf-8"))
    if not isinstance(index, list) or not index:
        raise ValueError("emotion_index.json must be a nonempty array")
    ids = [entry["motion"] for entry in index]
    if len(ids) != len(set(ids)) or any(not ID_PATTERN.fullmatch(x) for x in ids):
        raise ValueError("Invalid/duplicate action IDs in emotion_index.json")
    actions = ids if actions is None else actions
    if not actions or len(actions) != len(set(actions)) or not set(actions) <= set(ids):
        raise ValueError("--actions must select unique IDs from emotion_index.json")
    paths = {name: None for name in (
        "emotion_index.json", "base/base_ckpt.pt", "base/policy.onnx", "base/policy.yaml")}
    for entry in index:
        if entry["motion"] not in actions:
            continue
        paths[entry["adapter"]] = entry["adapter_sha256"]
        paths[entry["motion_onnx"]] = entry.get("motion_onnx_sha256")
        if entry.get("checkpoint") is not None:
            checkpoint = asset_path(source, entry["checkpoint"])
            if checkpoint.exists():
                if not entry.get("checkpoint_sha256"):
                    raise ValueError(f"Existing checkpoint requires a checksum: {checkpoint}")
                paths[entry["checkpoint"]] = entry["checkpoint_sha256"]
    hashes = {}
    for name, expected in paths.items():
        digest = sha256(asset_path(source, name))
        if expected is not None and digest != expected:
            raise ValueError(f"Training checksum mismatch: {name}")
        hashes[name] = digest
    return actions, hashes


def verify_assets(root, expected=None):
    """Check the exact file set and every SHA256; never ship links or checkpoints."""
    from wbc_lora_assets import asset_path, sha256

    paths = list(root.rglob("*"))
    if any(p.is_symlink() for p in paths):
        raise ValueError(f"Symlinks are forbidden in generated bundles: {root}")
    actual = {p.relative_to(root).as_posix() for p in paths if p.is_file()}
    if any(Path(name).suffix.lower() in (".pt", ".pth", ".ckpt") for name in actual):
        raise ValueError(f"Training checkpoints are forbidden in generated bundles: {root}")
    if expected is not None and actual != expected:
        raise ValueError(f"Unexpected board bundle files: extra={sorted(actual - expected)}, "
                         f"missing={sorted(expected - actual)}")
    hashes = {}
    for line in (root / "ASSETS.sha256").read_text(encoding="utf-8").splitlines():
        digest, name = line.split("  ", 1)
        if not re.fullmatch(r"[0-9a-f]{64}", digest) or name in hashes:
            raise ValueError("Invalid/duplicate ASSETS.sha256 entry")
        if sha256(asset_path(root, name)) != digest:
            raise ValueError(f"Generated asset checksum mismatch: {name}")
        hashes[name] = digest
    if set(hashes) != actual - {"ASSETS.sha256"}:
        raise ValueError("ASSETS.sha256 must cover every generated file exactly once")
    return {"file_count": len(actual), "size_bytes": sum((root / n).stat().st_size for n in actual),
            "checkpoint_count": 0, "sha256": "passed"}


def board_files(actions):
    # Exported paths are fixed by export_wbc_lora, not selected by a glob.
    files = {"base_manifest.json", "base_lora.onnx", "PREPROCESSING.json", "ASSETS.sha256"}
    for action in actions:
        files.update({f"adapters/{action}/manifest.json", f"adapters/{action}/weights.bin",
                      f"actions/{action}/rl_config.yaml", f"actions/{action}/reference_motion.onnx"})
    return files


def process(args):
    from wbc_lora_assets import asset_path, sha256

    source, workspace = args.source.resolve(), args.workspace.resolve()
    name = args.name or datetime.datetime.now().strftime("wbc_lora_%Y%m%d_%H%M%S_%f")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", name):
        raise ValueError("--name must contain 1-96 letters, digits, underscores or hyphens")
    if args.samples < 20:
        raise ValueError("At least 20 verification samples are required")
    archive = workspace / "model_inbox" / name
    board = workspace / "model_sources" / name
    logs = workspace / "smoke_logs" / name
    for path in (archive, board, logs):
        if path.exists() or path.is_symlink():
            raise ValueError(f"Output already exists; choose a new --name: {path}")
        resolved = path.resolve()
        if resolved.is_relative_to(source) or source.is_relative_to(resolved):
            raise ValueError("Training input and output directories must not overlap")
    actions, input_hashes = training_files(source, args.actions)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="1",
               MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    # Check the actual subprocess environment before creating any output.
    subprocess.run([sys.executable, "-B", "-c", "import torch, numpy, yaml, onnx, onnxruntime"],
                   check=True, env=env)
    logs.mkdir(parents=True, exist_ok=False)
    report_path = logs / "PROCESSING.json"
    report = {"format": "wbc_lora_processing_v1", "status": "running", "source": str(source),
              "local_archive": str(archive), "board_bundle": str(board), "actions": actions,
              "samples_per_action": args.samples, "source_files": input_hashes, "steps": [],
              "python": sys.executable, "board_operations": "not_run", "rknn_compilation": "not_run"}
    write_json(report_path, report)

    def run_step(label, script, *arguments):
        command = [sys.executable, "-B", str(SCRIPT_DIR / script), *map(str, arguments)]
        log = logs / (label + ".log")
        print(f"[{label}] {log}", flush=True)
        with log.open("x", encoding="utf-8") as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, env=env, check=False)
        report["steps"].append({"step": label, "command": command, "log": str(log), "exit_code": result.returncode})
        write_json(report_path, report)
        if result.returncode:
            tail = "\n".join(log.read_text(encoding="utf-8", errors="replace").splitlines()[-20:])
            raise RuntimeError(f"{label} failed (exit {result.returncode}):\n{tail}")

    try:
        run_step("01_export_fp16", "prepare_wbc_deploy.py", source, archive,
                 "--mode", "lora", "--fp16", "--samples", args.samples, "--actions", *actions)
        report["archive_audit"] = verify_assets(archive)
        run_step("02_preprocess", "preprocess_wbc_lora_fp16.py", archive, board)
        report["board_audit"] = verify_assets(board, board_files(actions))
        # Exercise relocated converter discovery and manifest planning without RKNN.
        run_step("03_conversion_plan", "batch_convert_onnx_to_rknn_rk3588.py", board,
                 "--build-only", "--dry-run")
        for relative, digest in input_hashes.items():
            if sha256(asset_path(source, relative)) != digest:
                raise ValueError(f"Training input changed during processing: {relative}")
        validation = json.loads((archive / "validation_report.json").read_text(encoding="utf-8"))
        report.update(status="passed", source_unchanged=True,
                      validation_report=str(archive / "validation_report.json"),
                      validation_report_sha256=sha256(archive / "validation_report.json"),
                      full_checkpoint_verification={a["action_id"]: a["full_checkpoint_verification"]
                                                    for a in validation["actions"]})
    except BaseException as error:
        report.update(status="failed", error=str(error))
        write_json(report_path, report)
        raise
    write_json(report_path, report)
    audit = report["board_audit"]
    print(f"Completed {len(actions)} LoRA actions; {audit['file_count']} board files, "
          f"{audit['size_bytes'] / 1024**2:.2f} MiB, 0 checkpoints", flush=True)
    print(f"Board bundle (transfer only this directory): {board}")
    print(f"Local archive: {archive}\nProcessing report: {report_path}")
    print("RKNN compilation and robot runtime validation remain on the board; no board operations performed.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="training root containing emotion_index.json, base/ and lora/")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE,
                        help="output workspace (default: workspace containing this repository)")
    parser.add_argument("--name", help="new bundle name (default: timestamp); existing outputs are never replaced")
    parser.add_argument("--actions", nargs="+", help="selected actions (default: every action in emotion_index.json)")
    parser.add_argument("--samples", type=int, default=33, help="CPU policy verification samples, minimum 20")
    process(parser.parse_args())


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
