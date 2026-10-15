#!/usr/bin/env python3
"""Prepare fixed-batch MimicLite ONNX artifacts for the ROS2 deploy adapter."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnx import helper, numpy_helper, version_converter


POLICY_INPUT_DIMS = {"policy": 399, "command": 240}
POLICY_COMMAND_INPUT_NAMES = ("command", "command_short")
POLICY_OUTPUT_DIMS = {"action": 21}
MOTION_INPUT_DIMS = {"time_step": (1, 1)}
MOTION_OUTPUT_TAILS = {
    "joint_pos": (21,),
    "joint_vel": (21,),
    "body_pos_w": (3,),
    "body_quat_w": (4,),
    "body_lin_vel_w": (3,),
    "body_ang_vel_w": (3,),
}


def _set_shape(value_info: onnx.ValueInfoProto, shape: tuple[int, ...]) -> None:
    tensor_shape = value_info.type.tensor_type.shape
    del tensor_shape.dim[:]
    for size in shape:
        tensor_shape.dim.add().dim_value = size


def _metadata(model: onnx.ModelProto) -> dict[str, str]:
    return {entry.key: entry.value for entry in model.metadata_props}


def _set_metadata(model: onnx.ModelProto, values: dict[str, str]) -> None:
    merged = _metadata(model)
    merged.update(values)
    del model.metadata_props[:]
    for key, value in sorted(merged.items()):
        model.metadata_props.append(
            onnx.StringStringEntryProto(key=key, value=value)
        )


def _value_infos(values: list[onnx.ValueInfoProto]) -> dict[str, onnx.ValueInfoProto]:
    return {value.name: value for value in values}


def _wrap_fixed_batch(model: onnx.ModelProto) -> None:
    """Wrap the original rank-1 graph without changing its internal math."""
    graph = model.graph
    prefix_nodes = []
    axes_name = "deploy_batch_axis"
    graph.initializer.append(
        numpy_helper.from_array(np.asarray([0], dtype=np.int64), axes_name)
    )
    for graph_input in graph.input:
        name = graph_input.name
        if name not in POLICY_INPUT_DIMS:
            continue
        internal_name = f"{name}_unbatched"
        for node in graph.node:
            for index, node_input in enumerate(node.input):
                if node_input == name:
                    node.input[index] = internal_name
        prefix_nodes.append(
            helper.make_node(
                "Squeeze",
                inputs=[name, axes_name],
                outputs=[internal_name],
                name=f"deploy_squeeze_{name}",
            )
        )
        _set_shape(graph_input, (1, POLICY_INPUT_DIMS[name]))

    output = graph.output[0]
    original_output_name = output.name
    internal_output_name = f"{original_output_name}_unbatched"
    for node in graph.node:
        for index, node_output in enumerate(node.output):
            if node_output == original_output_name:
                node.output[index] = internal_output_name
    suffix_node = helper.make_node(
        "Unsqueeze",
        inputs=[internal_output_name, axes_name],
        outputs=[original_output_name],
        name="deploy_unsqueeze_action",
    )
    _set_shape(output, (1, POLICY_OUTPUT_DIMS[original_output_name]))
    original_nodes = list(graph.node)
    del graph.node[:]
    graph.node.extend([*prefix_nodes, *original_nodes, suffix_node])
    # Exported rank-1 intermediate annotations are no longer valid after the
    # wrapper changes the public graph rank. They are optional in ONNX.
    del graph.value_info[:]


def _session(path: Path) -> ort.InferenceSession:
    return ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])


def prepare_policy(source: Path, output: Path, opset: int) -> None:
    source_model = onnx.load(str(source))
    source_inputs = _value_infos(list(source_model.graph.input))
    source_outputs = _value_infos(list(source_model.graph.output))
    command_inputs = [
        name for name in POLICY_COMMAND_INPUT_NAMES if name in source_inputs
    ]
    if set(source_inputs) - set(POLICY_COMMAND_INPUT_NAMES) != {"policy"} or len(command_inputs) != 1:
        raise ValueError(
            "Policy inputs must be policy plus exactly one of "
            f"{POLICY_COMMAND_INPUT_NAMES}, got {sorted(source_inputs)}"
        )
    if set(source_outputs) != set(POLICY_OUTPUT_DIMS):
        raise ValueError(
            f"Policy outputs must be {sorted(POLICY_OUTPUT_DIMS)}, got {sorted(source_outputs)}"
        )

    source_command_name = command_inputs[0]
    command_info = source_inputs[source_command_name]
    command_shape = command_info.type.tensor_type.shape
    command_dim = command_shape.dim[-1].dim_value
    if command_dim != POLICY_INPUT_DIMS["command"]:
        raise ValueError(
            f"Policy command input must have dimension 240, got {command_dim}"
        )

    model = source_model
    if source_command_name != "command":
        command_info.name = "command"
        for node in model.graph.node:
            for index, node_input in enumerate(node.input):
                if node_input == source_command_name:
                    node.input[index] = "command"
    current_opset = next(
        item.version for item in model.opset_import if item.domain in ("", "ai.onnx")
    )
    if current_opset != opset:
        model = version_converter.convert_version(model, opset)

    _wrap_fixed_batch(model)
    _set_metadata(
        model,
        {
            "artifact_type": "mimic_lite_policy_ros2_adapter",
            "deploy_abi": "policy:[1,399];command:[1,240]->action:[1,21]",
            "deploy_adapter_required": "true",
            "history_offsets_policy": "0,1,2,3,4,8,16",
            "history_offsets_action": "0,1,2",
            "motion_offsets": "-8,-4,-2,0,1,2,3,4",
            "source_onnx": source.name,
        },
    )
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output))

    rng = np.random.default_rng(0)
    unbatched = {
        name: rng.standard_normal(dim, dtype=np.float32)
        for name, dim in POLICY_INPUT_DIMS.items()
    }
    source_feed = {
        "policy": unbatched["policy"],
        source_command_name: unbatched["command"],
    }
    source_result = _session(source).run(None, source_feed)[0]
    deploy_result = _session(output).run(
        None, {name: value[None] for name, value in unbatched.items()}
    )[0]
    np.testing.assert_allclose(
        deploy_result[0], source_result, rtol=1e-5, atol=2e-6
    )


def prepare_motion(source: Path, output: Path) -> None:
    model = onnx.load(str(source))
    inputs = _value_infos(list(model.graph.input))
    outputs = _value_infos(list(model.graph.output))
    if set(inputs) != set(MOTION_INPUT_DIMS):
        raise ValueError(
            f"Motion inputs must be {sorted(MOTION_INPUT_DIMS)}, got {sorted(inputs)}"
        )
    if set(outputs) != set(MOTION_OUTPUT_TAILS):
        raise ValueError(
            f"Motion outputs must be {sorted(MOTION_OUTPUT_TAILS)}, got {sorted(outputs)}"
        )
    _set_metadata(
        model,
        {
            "deploy_abi": (
                "time_step:[1,1]->joint_pos:[1,21],joint_vel:[1,21],"
                "body_pos_w:[1,B,3],body_quat_w:[1,B,4],"
                "body_lin_vel_w:[1,B,3],body_ang_vel_w:[1,B,3]"
            ),
            "deploy_adapter_required": "true",
            "source_onnx": source.name,
        },
    )
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output))

    source_session = _session(source)
    deploy_session = _session(output)
    for frame in (0.0, 8.0, 100000.0):
        value = np.array([[frame]], dtype=np.float32)
        expected = source_session.run(None, {"time_step": value})
        actual = deploy_session.run(None, {"time_step": value})
        for expected_item, actual_item in zip(expected, actual, strict=True):
            np.testing.assert_array_equal(actual_item, expected_item)


def _default_output(source: Path) -> Path:
    return source.with_name(f"{source.stem}-deploy{source.suffix}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--motion", type=Path)
    parser.add_argument("--policy-output", type=Path)
    parser.add_argument("--motion-output", type=Path)
    parser.add_argument("--opset", type=int, default=18)
    args = parser.parse_args()

    policy = args.policy.expanduser().resolve()
    motion = args.motion.expanduser().resolve() if args.motion else None
    policy_output = (args.policy_output or _default_output(policy)).expanduser().resolve()
    motion_output = ((args.motion_output or _default_output(motion)).expanduser().resolve()
                     if motion else None)
    pairs = [(policy, policy_output)]
    if motion is not None:
        pairs.append((motion, motion_output))
    elif args.motion_output is not None:
        raise ValueError("--motion-output requires --motion")
    for source, output in pairs:
        if not source.is_file():
            raise FileNotFoundError(source)
        if source == output:
            raise ValueError(f"Refusing to overwrite source ONNX: {source}")

    prepare_policy(policy, policy_output, args.opset)
    print(f"Prepared policy ONNX: {policy_output}")
    if motion is not None:
        prepare_motion(motion, motion_output)
        print(f"Prepared motion ONNX: {motion_output}")


if __name__ == "__main__":
    main()
