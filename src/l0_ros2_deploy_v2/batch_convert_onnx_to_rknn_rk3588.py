#!/usr/bin/env python3
"""Batch-build RK3588 FP16 WBC: one shared LoRA graph plus each action's reference.

Detects base_manifest.json at the bundle root, or accepts --manifest explicitly.
Without a manifest, retains the static policy/reference subdirectory traversal.
Conversions are sequential; adapters are validated, never compiled as policies.
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

from wbc_lora_assets import asset_path, check_graph, load_adapter, load_graph_manifest, read_json, sha256


CONVERTERS = (
    ("policy.onnx", "convert_onnx_to_rknn_rk3588_wbc_policy.py"),
    ("reference_motion.onnx", "convert_onnx_to_rknn_rk3588_wbc_reference.py"),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="absolute path whose subfolders contain ONNX models")
    parser.add_argument("--skip-existing", action="store_true", help="manifest mode: skip only matching provenance; legacy static mode: skip existing files")
    parser.add_argument("--dry-run", action="store_true", help="show commands without converting")
    parser.add_argument("--manifest", type=Path, help="explicit LoRA graph/deployment manifest")
    parser.add_argument("--device-id", help="connected device ID; requires --target rk3588")
    parser.add_argument("--target", help="runtime target platform, e.g. rk3588; otherwise toolkit simulator")
    parser.add_argument("--build-only", action="store_true", help="export models; leave device validation to the formal C++ worker")
    parser.add_argument("--policy-only", action="store_true", help="manifest mode: build only the shared policy/probe")
    parser.add_argument("--include-probes", action="store_true", help="manifest mode: also build the listed diagnostic probes")
    parser.add_argument("--log-dir", type=Path, help="manifest mode: new directory for per-model logs, exit codes and summary JSON")
    args = parser.parse_args()
    if args.build_only and (args.target or args.device_id):
        parser.error("--build-only cannot be combined with --target/--device-id")
    if not args.root.is_absolute():
        parser.error("root must be an absolute path")
    if not args.root.is_dir():
        parser.error(f"root directory does not exist: {args.root}")
    if args.policy_only and args.include_probes:
        parser.error("--policy-only and --include-probes are mutually exclusive")
    if args.manifest is None and (args.root / "base_manifest.json").is_file():
        args.manifest = args.root / "base_manifest.json"
        print(f"[AUTO manifest] {args.manifest}", flush=True)
    if not args.manifest and (args.policy_only or args.include_probes or args.log_dir):
        parser.error("--policy-only/--include-probes/--log-dir require a LoRA manifest")
    return args


def convert_manifest(args):
    import importlib.metadata
    manifest_path = args.manifest.resolve()
    root = manifest_path.parent
    if root != args.root.resolve():
        raise ValueError("--manifest must be inside the selected root directory")
    graph = load_graph_manifest(manifest_path)
    check_graph(asset_path(root, graph["model"]["onnx"]), manifest_path)
    commands = [(asset_path(root, graph["model"]["onnx"]), "convert_onnx_to_rknn_rk3588_wbc_policy.py",
                 asset_path(root, graph["model"]["rknn"]), manifest_path)]
    if graph["kind"] == "policy":
        if not graph.get("actions") or len({a["action_id"] for a in graph["actions"]}) != len(graph["actions"]):
            raise ValueError("LoRA manifest requires unique non-empty actions")
        for action in graph["actions"]:
            load_adapter(asset_path(root, action["adapter_manifest"]), graph, action["action_id"])
            for key, digest in (("config", "config_sha256"), ("reference_onnx", "reference_onnx_sha256")):
                if sha256(asset_path(root, action[key])) != action[digest]:
                    raise ValueError(f"Action source checksum mismatch: {action['action_id']}")
            if not args.policy_only:
                commands.append((asset_path(root, action["reference_onnx"]),
                                 "convert_onnx_to_rknn_rk3588_wbc_reference.py",
                                 asset_path(root, action["reference_rknn"]), None))
    if args.include_probes:
        for relative in graph.get("probes", []):
            probe_path = asset_path(root, relative)
            probe = load_graph_manifest(probe_path)
            if probe["kind"] != "probe" or any(probe[k] != graph[k] for k in
                    ("base_checkpoint_sha256", "normalization_sha256", "parameter_schema_sha256", "actions")):
                raise ValueError(f"Probe is not from this policy bundle: {probe_path}")
            check_graph(asset_path(root, probe["model"]["onnx"]), probe_path)
            commands.append((asset_path(root, probe["model"]["onnx"]),
                             "convert_onnx_to_rknn_rk3588_wbc_policy.py",
                             asset_path(root, probe["model"]["rknn"]), probe_path))
    if len({str(item[2]) for item in commands}) != len(commands):
        raise ValueError("Duplicate RKNN output destinations in manifest")
    for source, converter, _, _ in commands:
        if not source.is_file() or not Path(__file__).with_name(converter).is_file():
            raise ValueError(f"Missing ONNX or converter: {source} / {converter}")
    log_dir = args.log_dir.resolve() if args.log_dir else None
    if log_dir and not args.dry_run:
        log_dir.mkdir(parents=True, exist_ok=False)
    summary = {"manifest": str(manifest_path), "graph_manifest_sha256": sha256(manifest_path),
               "target_platform": "rk3588", "build_only": args.build_only,
               "formal_runtime": "not_run", "planned": len(commands), "models": []}
    failures = 0
    for index, (source, converter, output, graph_path) in enumerate(commands, 1):
        skip = False
        provenance = Path(str(output) + ".json")
        if args.skip_existing and output.is_file() and provenance.is_file():
            try:
                record = read_json(provenance)
            except (ValueError, OSError):
                record = {}
            try:
                toolkit = importlib.metadata.version("rknn-toolkit2")
            except importlib.metadata.PackageNotFoundError:
                toolkit = "unknown"
            skip = (record.get("format") == "wbc_rknn_conversion_v1" and record.get("do_quantization") is False and
                    record.get("source_sha256") == sha256(source) and
                    record.get("rknn_sha256") == sha256(output) and record.get("precision") == "float16" and
                    record.get("toolkit_version") == toolkit and toolkit != "unknown" and
                    record.get("target_platform") == "rk3588" and
                    record.get("converter_sha256") == sha256(Path(__file__).with_name(converter)) and
                    record.get("asset_helper_sha256") == sha256(Path(__file__).with_name("wbc_lora_assets.py")) and
                    record.get("target") == args.target and record.get("device_id") == args.device_id)
            if not args.build_only:
                skip = skip and record.get("verification_backend") in ("target", "toolkit_simulator")
            if graph_path:
                skip = skip and record.get("graph_manifest_sha256") == sha256(graph_path)
                expected = [(a["action_id"], sha256(asset_path(root, a["adapter_manifest"]))) for a in graph["actions"]]
                verified = [(a["action_id"], a["adapter_manifest_sha256"]) for a in (record.get("verification") or {}).get("actions", [])]
                if not args.build_only:
                    skip = skip and expected == verified
        if skip:
            print(f"[SKIP matching conversion; validation={record.get('verification_backend')}] {output}", flush=True)
            summary["models"].append({"output": str(output), "status": "skipped_matching_record"})
            continue
        command = [sys.executable, "-B", str(Path(__file__).resolve().parent / converter), str(source), "-o", str(output)]
        if args.build_only:
            command += ["--build-only"]
        if graph_path:
            command += ["--manifest", str(graph_path)]
        if args.target:
            command += ["--target", args.target]
        if args.device_id:
            command += ["--device-id", args.device_id]
        print(f"[{'DRY-RUN' if args.dry_run else 'RUN'}] {shlex.join(command)}", flush=True)
        if args.dry_run:
            continue
        entry = {"source": str(source), "output": str(output), "command": command}
        try:
            if log_dir:
                log_path = log_dir / f"{index:02d}_{output.stem}.log"
                print(f"[LOG] {log_path}", flush=True)
                with log_path.open("x") as stream:
                    result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=False)
                entry["log"] = str(log_path)
                log_path.with_suffix(".exitcode").write_text(str(result.returncode) + "\n")
            else:
                result = subprocess.run(command, check=False)
            entry.update(exit_code=result.returncode, status="converted" if result.returncode == 0 else "failed")
        except OSError as error:
            entry.update(exit_code=1, status="failed", error=str(error))
        summary["models"].append(entry)
        if entry["exit_code"]:
            failures += 1
            print(f"[FAIL] {source}; remaining models not attempted", file=sys.stderr, flush=True)
            break
    summary["failed"] = failures
    summary["not_attempted"] = 0 if args.dry_run else len(commands) - len(summary["models"])
    if log_dir and not args.dry_run:
        (log_dir / "conversion_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"LoRA manifest conversions={len(commands)}, failed={failures}, dry_run={args.dry_run}")
    return 1 if failures else 0


def main():
    args = parse_args()
    if args.device_id and not args.target:
        raise ValueError("--device-id requires --target rk3588")
    if args.manifest:
        return convert_manifest(args)
    root = args.root.resolve()
    script_dir = Path(__file__).resolve().parent
    scripts = {name: script_dir / filename for name, filename in CONVERTERS}
    for name, script in scripts.items():
        if not script.is_file():
            print(f"ERROR: converter for {name} not found: {script}", file=sys.stderr)
            return 2

    found = succeeded = failed = skipped = 0
    for directory, subdirs, filenames in os.walk(root):
        subdirs.sort()
        current_dir = Path(directory)
        if current_dir == root:  # Only process subfolders (including nested ones).
            continue
        present = set(filenames)
        for onnx_name, _ in CONVERTERS:
            if onnx_name not in present:
                continue
            onnx_path = current_dir / onnx_name
            if not onnx_path.is_file():
                continue
            found += 1
            output_path = onnx_path.with_suffix(".rknn")
            if args.skip_existing and output_path.is_file():
                skipped += 1
                print(f"[SKIP] {output_path} already exists", flush=True)
                continue

            command = [sys.executable, str(scripts[onnx_name]), str(onnx_path),
                       "-o", str(output_path)]
            if args.build_only:
                command.append("--build-only")
            if args.target:
                command.extend(["--target", args.target])
            if args.device_id:
                command.extend(["--device-id", args.device_id])
            print(f"\n[{'DRY-RUN' if args.dry_run else 'RUN'}] {shlex.join(command)}", flush=True)
            if args.dry_run:
                continue

            try:
                result = subprocess.run(command, check=False)
            except OSError as exc:
                print(f"[FAIL] {onnx_path}: {exc}", file=sys.stderr, flush=True)
                failed += 1
                continue
            if result.returncode == 0:
                succeeded += 1
                print(f"[OK] {output_path}", flush=True)
            else:
                failed += 1
                print(f"[FAIL] {onnx_path} (exit code {result.returncode})",
                      file=sys.stderr, flush=True)

    print(f"\nSummary: found={found}, succeeded={succeeded}, "
          f"failed={failed}, skipped={skipped}, dry_run={args.dry_run}")
    if not found:
        print("No policy.onnx or reference_motion.onnx found in subfolders.", file=sys.stderr)
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
