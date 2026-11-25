#!/usr/bin/env python3
"""Package generated text JSON as new motions beside the existing L0 v5 WBC actions."""

import argparse
import hashlib
import json
import os
import re
import shutil
import tempfile
from pathlib import Path

from prepare_wbc_deploy import prepare_json_batch


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def install_file(source, destination):
    if os.path.lexists(destination):
        if digest(source) != digest(destination):
            raise ValueError("existing WBC asset differs: " + str(destination))
        return
    os.replace(source, destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="directory containing ACTION.json files")
    parser.add_argument("--checkpoint-dir", type=Path, required=True, help="static checkpoint_2000.onnx/yaml export")
    parser.add_argument("--amp-profile", type=Path, required=True, help="fresh stationary AMP profile JSON")
    parser.add_argument("--destination", type=Path, default=Path(__file__).resolve().parent /
                        "legged_deploy/policy/l0_v5/wbc")
    parser.add_argument("--actions", nargs="+", required=True, help="new WBC motion IDs, e.g. TEXT01 TEXT02")
    args = parser.parse_args()
    if not args.destination.is_dir() or not args.source.is_dir() or not args.checkpoint_dir.is_dir():
        parser.error("source/checkpoint/destination directories must exist")
    if len(set(args.actions)) != len(args.actions) or any(
            not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", name) for name in args.actions):
        parser.error("actions must have unique safe WBC motion IDs")
    missing = [args.source / (action + ".json") for action in args.actions
               if not (args.source / (action + ".json")).is_file()]
    if missing:
        parser.error("missing generated JSON: " + ", ".join(map(str, missing)))
    for action in args.actions:
        if os.path.lexists(args.destination / action):
            raise ValueError("WBC motion already exists; choose a new ID: " + action)
    name = "text_checkpoint_2000_" + hashlib.sha256(
        ",".join(args.actions).encode("ascii")).hexdigest()[:12] + ".json"
    destination = args.destination / name
    if destination.exists():
        raise ValueError("WBC batch manifest already exists: " + str(destination))
    for source, target in (
        (args.checkpoint_dir / "checkpoint_2000.onnx", args.destination / "checkpoint_2000.onnx"),
        (args.checkpoint_dir / "checkpoint_2000.yaml", args.destination / "checkpoint_2000.yaml"),
        (args.amp_profile, args.destination / "checkpoint_2000_amp_stand_profile.json"),
    ):
        if not source.is_file():
            raise ValueError("missing WBC source asset: " + str(source))
        if os.path.lexists(target) and digest(source) != digest(target):
            raise ValueError("existing WBC asset differs: " + str(target))

    with tempfile.TemporaryDirectory(prefix=".text_wbc_", dir=args.destination.parent) as temporary:
        bundle = Path(temporary) / "bundle"
        prepare_json_batch(args.source.resolve(), bundle, args.checkpoint_dir.resolve(),
                           args.amp_profile.resolve(), args.actions, "checkpoint_2000")
        manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
        if [item["motion_id"] for item in manifest["actions"]] != args.actions:
            raise ValueError("WBC package action order changed")
        install_file(bundle / "checkpoint_2000.onnx", args.destination / "checkpoint_2000.onnx")
        install_file(bundle / "checkpoint_2000.yaml", args.destination / "checkpoint_2000.yaml")
        install_file(bundle / "amp_stand_profile.json", args.destination / "checkpoint_2000_amp_stand_profile.json")
        if (args.checkpoint_dir / "export_manifest.json").is_file():
            export_manifest = args.destination / "checkpoint_2000_export_manifest.json"
            if os.path.lexists(export_manifest):
                if digest(args.checkpoint_dir / "export_manifest.json") != digest(export_manifest):
                    raise ValueError("existing WBC export manifest differs: " + str(export_manifest))
            else:
                shutil.copyfile(args.checkpoint_dir / "export_manifest.json", export_manifest)
        for action in args.actions:
            os.replace(bundle / action, args.destination / action)
        manifest["source_model"] = "checkpoint_2000.pt"
        manifest["installed_actions"] = args.actions
        destination.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Installed {len(args.actions)} text WBC motions in {args.destination}")
    print("Rebuild legged_deploy and restart MuJoCo to preload these motions")


if __name__ == "__main__":
    main()
