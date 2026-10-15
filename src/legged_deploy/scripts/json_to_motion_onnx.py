#!/usr/bin/env python3
"""Export a uniformly sampled upper-body trajectory JSON as an ONNX lookup."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, checker, helper, numpy_helper


def convert(source: Path, destination: Path) -> None:
    motion = json.loads(source.read_text(encoding="utf-8"))
    if motion.get("format") != "gmr_upper_body_joint_trajectory_v1":
        raise ValueError("unsupported trajectory format")
    if motion.get("robot") != "l0_v3" or motion.get("angle_unit") != "radian":
        raise ValueError("trajectory must use l0_v3 joint angles in radians")
    names, frames, fps = motion["joint_names"], motion["frames"], float(motion["fps"])
    if not names or len(set(names)) != len(names) or len(frames) < 2:
        raise ValueError("joint names and at least two frames are required")
    if int(motion.get("frame_count", -1)) != len(frames):
        raise ValueError("frame_count does not match frames")
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive and finite")

    positions = np.asarray([frame["joint_positions"] for frame in frames], dtype=np.float32)
    times = np.asarray([frame.get("time", index / fps) for index, frame in enumerate(frames)])
    if positions.shape != (len(frames), len(names)) or not np.isfinite(positions).all():
        raise ValueError("invalid joint positions")
    expected_times = np.arange(len(frames)) / fps
    if not np.isfinite(times).all() or not np.allclose(
            times, expected_times, rtol=0, atol=1e-6):
        raise ValueError("timestamps must start at zero and use a uniform fps grid")
    if any(frame.get("frame_index", index) != index for index, frame in enumerate(frames)):
        raise ValueError("frame indices must be consecutive from zero")

    last = len(frames) - 1
    initializers = [numpy_helper.from_array(positions, name="poses")]
    for name, value, dtype in (
        ("fps", fps, np.float32), ("zero_float", 0, np.float32),
        ("last_float", last, np.float32), ("one_int", 1, np.int64),
        ("last_int", last, np.int64),
    ):
        initializers.append(numpy_helper.from_array(np.array(value, dtype=dtype), name=name))
    nodes = [
        helper.make_node("Mul", ["time_seconds", "fps"], ["frame_float"]),
        helper.make_node("Clip", ["frame_float", "zero_float", "last_float"], ["clamped"]),
        helper.make_node("Floor", ["clamped"], ["frame_floor"]),
        helper.make_node("Cast", ["frame_floor"], ["index0"], to=TensorProto.INT64),
        helper.make_node("Add", ["index0", "one_int"], ["index1_raw"]),
        helper.make_node("Min", ["index1_raw", "last_int"], ["index1"]),
        helper.make_node("Gather", ["poses", "index0"], ["pose0"], axis=0),
        helper.make_node("Gather", ["poses", "index1"], ["pose1"], axis=0),
        helper.make_node("Sub", ["clamped", "frame_floor"], ["fraction"]),
        helper.make_node("Sub", ["pose1", "pose0"], ["delta"]),
        helper.make_node("Mul", ["delta", "fraction"], ["weighted_delta"]),
        helper.make_node("Add", ["pose0", "weighted_delta"], ["joint_positions"]),
    ]
    graph = helper.make_graph(
        nodes, "text_joint_trajectory",
        [helper.make_tensor_value_info("time_seconds", TensorProto.FLOAT, [])],
        [helper.make_tensor_value_info("joint_positions", TensorProto.FLOAT, [len(names)])],
        initializer=initializers,
    )
    model = helper.make_model(graph, producer_name="legged_deploy/json_to_motion_onnx.py",
                              opset_imports=[helper.make_opsetid("", 13)], ir_version=8)
    metadata = {
        "source_format": motion["format"], "angle_unit": "radian", "fps": str(fps),
        "frame_count": str(len(frames)), "duration_seconds": str(len(frames) / fps),
        "joint_names": json.dumps(names, ensure_ascii=False), "text": str(motion.get("text", "")),
    }
    for key, value in metadata.items():
        item = model.metadata_props.add()
        item.key, item.value = key, value
    checker.check_model(model)
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    convert(args.input.expanduser().resolve(), args.output.expanduser().resolve())


if __name__ == "__main__":
    main()
