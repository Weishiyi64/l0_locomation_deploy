#!/usr/bin/env python3
"""Export one motion-assets NPZ to materials/motion_assets.bin and assets.json.

The output directory defaults to materials/ beside the NPZ and must not exist.
Only NumPy is needed; no ONNX/RKNN model, manifest or inference runtime is loaded.
"""
import argparse
import json
import struct
import sys
import tempfile
from pathlib import Path


def read_materials(source):
    """Validate NPZ and form exactly the little-endian C++ material ABI in memory."""
    import numpy as np

    with np.load(source, allow_pickle=False) as archive:
        atoms = archive["atom_motion"]
        lengths = archive["atom_lengths"]
        pose = archive["a_pose"].reshape(-1)
        if atoms.dtype != np.dtype("float32") or atoms.ndim != 3 or atoms.shape[2] != 3 or \
                not np.isfinite(atoms).all() or not 1 <= atoms.shape[0] <= 65536 or not 1 <= atoms.shape[1] <= 5000:
            raise ValueError("Invalid finite FP32 atom_motion table")
        if lengths.dtype.kind not in "iu" or lengths.shape != (len(atoms),) or \
                np.any(lengths < 1) or np.any(lengths > atoms.shape[1]):
            raise ValueError("Invalid atom_lengths table")
        if pose.dtype != np.dtype("float32") or pose.shape != (9,) or not np.isfinite(pose).all():
            raise ValueError("Invalid finite FP32 A-pose")
        if 16 + atoms.size * 4 + lengths.size * 4 > 64 * 1024 * 1024:
            raise ValueError("Native material table exceeds the C++ loader's 64 MiB limit")
        payload = (b"T2MAT001" + struct.pack("<II", atoms.shape[0], atoms.shape[1]) +
                   atoms.astype("<f4", copy=False).tobytes(order="C") + lengths.astype("<u4", copy=False).tobytes(order="C"))
    record = {"format": "txt2motion_cpp_assets_v1",
              "data": {"file": "motion_assets.bin",
                       "materials": atoms.shape[0], "padded_frames": atoms.shape[1]}, "a_pose": pose.tolist()}
    return payload, record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path, help="input motion-assets NPZ")
    parser.add_argument("-o", "--output", type=Path, help="new output directory (default: materials/ beside NPZ)")
    parser.add_argument("--dry-run", action="store_true", help="check and prepare data in memory without writing")
    args = parser.parse_args(argv)

    source = args.npz.resolve(strict=True)
    if not source.is_file() or source.suffix.lower() != ".npz":
        raise ValueError("Input must be an NPZ file")
    output = (args.output or source.parent / "materials").absolute()
    if not args.dry_run and (output.exists() or output.is_symlink()):
        raise FileExistsError(f"Output directory already exists; choose a new directory: {output}")
    payload, record = read_materials(source)
    print(json.dumps({"status": "static_preflight_passed", "source": str(source), "output": str(output),
                      "binary_bytes": len(payload), "assets": record}, ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".txt2motion_materials_", dir=output.parent) as work:
        candidate = Path(work) / "materials"
        candidate.mkdir()
        (candidate / "motion_assets.bin").write_bytes(payload)
        (candidate / "assets.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if output.exists() or output.is_symlink():
            raise FileExistsError(f"Output directory appeared during export: {output}")
        candidate.rename(output)
    print(f"Exported: {output / 'motion_assets.bin'} and {output / 'assets.json'}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
