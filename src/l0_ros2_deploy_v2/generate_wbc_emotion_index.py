#!/usr/bin/env python3
"""Generate emotion_index.json from lora/<emotion>/<motion> without loading .pt.

Requires Base checkpoint/ONNX/YAML and each action's lorackpt_N.pt and
motion/reference_motion.onnx. Matching checkpoint_N.pt is optional.
Writes a new index only.
"""

import argparse
import hashlib
import json
import os
import re
import sys
import tempfile
from pathlib import Path


ACTION_ID = re.compile(r"[A-Za-z0-9_-]{1,32}")


def checked_file(root, path):
    if not path.is_file() or not path.resolve().is_relative_to(root):
        raise ValueError(f"Missing file or path outside training root: {path}")
    return path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def saved_steps(directory, prefix):
    result = {}
    pattern = re.compile(re.escape(prefix) + r"_([0-9]+)\.pt")
    for path in sorted(directory.glob(prefix + "_*.pt")):
        match = pattern.fullmatch(path.name)
        if not match:
            continue
        step = int(match.group(1))
        if step in result:
            raise ValueError(f"Ambiguous filenames for {prefix} step {step}: {directory}")
        result[step] = path
    return result


def generate(root, actions=None, step=None):
    root = root.resolve()
    if not root.is_dir():
        raise ValueError(f"Training root does not exist: {root}")
    for name in ("base/base_ckpt.pt", "base/policy.onnx", "base/policy.yaml"):
        checked_file(root, root / name)
    if step is not None and step < 0:
        raise ValueError("--step must be nonnegative")
    if actions is not None and (not actions or len(actions) != len(set(actions)) or
                                any(not ACTION_ID.fullmatch(x) for x in actions)):
        raise ValueError("--actions must contain unique valid action IDs")
    selected = set(actions) if actions is not None else None
    entries, seen = [], set()
    for directory in sorted((root / "lora").glob("*/*")):
        if not directory.is_dir() or (selected is not None and directory.name not in selected):
            continue
        action = directory.name
        if not directory.resolve().is_relative_to(root):
            raise ValueError(f"Action directory escapes training root: {directory}")
        if not ACTION_ID.fullmatch(action) or action in seen:
            raise ValueError(f"Invalid or duplicate action ID: {action}")
        seen.add(action)
        adapters, checkpoints = saved_steps(directory, "lorackpt"), saved_steps(directory, "checkpoint")
        if not adapters:
            raise ValueError(f"{action}: missing lorackpt_N.pt")
        chosen = max(adapters) if step is None else step
        if chosen not in adapters:
            raise ValueError(f"{action}: lorackpt file is required for step {chosen}")
        adapter = checked_file(root, adapters[chosen])
        reference = checked_file(root, directory / "motion/reference_motion.onnx")
        entry = {
            "emotion": directory.parent.name, "motion": action,
            "key": str(directory.relative_to(root / "lora")), "checkpoint_step": chosen,
            "adapter": str(adapter.relative_to(root)),
            "motion_onnx": str(reference.relative_to(root)),
            "adapter_sha256": sha256(adapter),
            "motion_onnx_sha256": sha256(reference),
        }
        if chosen in checkpoints:
            checkpoint = checked_file(root, checkpoints[chosen])
            entry.update(checkpoint=str(checkpoint.relative_to(root)), checkpoint_sha256=sha256(checkpoint))
        motion_npz = directory / "motion" / (action + ".npz")
        if motion_npz.exists() or motion_npz.is_symlink():
            checked_file(root, motion_npz)
            entry["motion_npz"] = str(motion_npz.relative_to(root))
        entries.append(entry)
    if selected is not None and selected - seen:
        raise ValueError("Actions not found: " + ", ".join(sorted(selected - seen)))
    if not entries:
        raise ValueError("No action directories found under lora/<emotion>/<motion>")
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="training root containing base/ and lora/")
    parser.add_argument("--output", type=Path, default=Path("emotion_index.json"),
                        help="new JSON file; relative paths are relative to source (default: emotion_index.json)")
    parser.add_argument("--actions", nargs="+", help="only scan these action IDs (default: all actions)")
    parser.add_argument("--step", type=int, help="use this checkpoint filename step for every selected action")
    parser.add_argument("--dry-run", action="store_true", help="print the generated JSON without writing or replacing any file")
    args = parser.parse_args()
    root = args.source.resolve()
    output = args.output if args.output.is_absolute() else root / args.output
    if not args.dry_run and (output.exists() or output.is_symlink()):
        parser.error(f"output already exists; choose a new --output path: {output}")
    entries = generate(root, args.actions, args.step)
    content = json.dumps(entries, indent=2, ensure_ascii=False) + "\n"
    if args.dry_run:
        sys.stdout.write(content)
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".emotion_index_", dir=output.parent) as temporary:
            candidate = Path(temporary) / "index.json"
            candidate.write_text(content, encoding="utf-8")
            # Atomic publication, including refusal to overwrite a file/symlink
            # that appeared after the initial existence check.
            os.link(candidate, output)
        print(f"Wrote {len(entries)} actions: {output}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
