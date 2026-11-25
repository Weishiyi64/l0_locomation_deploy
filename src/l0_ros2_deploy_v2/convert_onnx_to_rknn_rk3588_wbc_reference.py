#!/usr/bin/env python3
"""Convert a MimicLite WBC reference-motion ONNX to non-quantized RK3588 RKNN."""

import argparse
import math
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

from wbc_lora_assets import conversion_record, runtime_options

REFERENCE_OUTPUTS = {
    "joint_pos": (1, 21), "joint_vel": (1, 21),
    "body_pos_w": (1, 23, 3), "body_quat_w": (1, 23, 4),
    "body_lin_vel_w": (1, 23, 3), "body_ang_vel_w": (1, 23, 3),
}
RKNN_MAX_ONNX_OPSET = 19


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx", type=Path, help="input WBC reference-motion .onnx model")
    parser.add_argument("-o", "--output", type=Path, help="output .rknn (default: beside ONNX)")
    parser.add_argument("--max-abs-error", type=float, default=0.02)
    parser.add_argument(
        "--target", help="runtime target platform, e.g. rk3588; select a board with --device-id"
    )
    parser.add_argument("--device-id", help="connected device ID; requires --target rk3588")
    parser.add_argument("--build-only", action="store_true",
                        help="export FP16 RKNN without loading a Python Runtime; C++ deployment validation remains pending")
    return parser.parse_args()


def static_shape(item, kind, onnx):
    if item.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise ValueError(f"{kind} '{item.name}' must be float32")
    dims = []
    for dim in item.type.tensor_type.shape.dim:
        if dim.dim_value <= 0:
            raise ValueError(f"{kind} '{item.name}' must have a fully static shape")
        dims.append(dim.dim_value)
    return tuple(dims)


def check_reference_model(path):
    import onnx

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    inputs = list(model.graph.input)
    outputs = list(model.graph.output)
    if len(inputs) != 1 or inputs[0].name != "time_step" or static_shape(inputs[0], "input", onnx) != (1, 1):
        raise ValueError("reference model must have float32 time_step[1,1]")
    if len(outputs) != 6 or {item.name for item in outputs} != set(REFERENCE_OUTPUTS):
        raise ValueError(f"reference model must have outputs {list(REFERENCE_OUTPUTS)}")
    for item in outputs:
        if static_shape(item, "output", onnx) != REFERENCE_OUTPUTS[item.name]:
            raise ValueError(f"reference output {item.name} must be float32 {REFERENCE_OUTPUTS[item.name]}")
    meta = {item.key: item.value for item in model.metadata_props}
    if meta.get("quaternion_order") != "wxyz" or \
            meta.get("time_step_semantics") != "zero_based_frame_index_clamped" or \
            float(meta.get("fps", 0)) != 50 or not 0 < int(meta.get("num_frames", 0)) <= 100000:
        raise ValueError("reference model needs valid 50 Hz frame metadata and clamped wxyz semantics")
    joints = meta["joint_names"].split(",")
    bodies = meta["body_names"].split(",")
    if len(joints) != 21 or len(set(joints)) != 21 or len(bodies) != 23 or len(set(bodies)) != 23 or \
            "base_link" not in bodies:
        raise ValueError("invalid reference joint/body metadata")
    return [("time_step", (1, 1))], [(item.name, REFERENCE_OUTPUTS[item.name]) for item in outputs], meta


def pack_reference_gathers(model):
    import numpy as np
    from onnx import helper, numpy_helper

    count = int(next(item.value for item in model.metadata_props if item.key == "num_frames"))
    tables = {item.name: numpy_helper.to_array(item) for item in model.graph.initializer}
    gathers = [node for node in model.graph.node if node.op_type == "Gather"]
    by_output = {node.output[0]: node for node in gathers if len(node.output) == 1}
    if len(gathers) != 6 or set(by_output) != set(REFERENCE_OUTPUTS):
        raise ValueError("reference compatibility rewrite requires six direct table Gather outputs")
    indices = {node.input[1] for node in gathers}
    if len(indices) != 1:
        raise ValueError("reference Gather branches must share the same frame index")
    packed, widths, table_names = [], [], set()
    for name, shape in REFERENCE_OUTPUTS.items():
        node = by_output[name]
        if node.input[0] not in tables or any(attr.name == "axis" and attr.i != 0 for attr in node.attribute):
            raise ValueError(f"{name}: expected a constant frame table on axis 0")
        table = tables[node.input[0]]
        if table.dtype != np.float32 or table.shape != (count,) + shape[1:] or not np.isfinite(table).all():
            raise ValueError(f"{name}: invalid frame table shape, type or value")
        packed.append(table.reshape(count, -1))
        widths.append(math.prod(shape))
        table_names.add(node.input[0])

    # Avoid Toolkit2 2.3.2 merging parallel Gather branches with different widths (3 vs 4).
    prefix = "wbc_rknn_reference_"
    existing_names = {name for node in model.graph.node for name in list(node.input) + list(node.output)}
    if any(name.startswith(prefix) for name in existing_names):
        raise ValueError("reference graph already uses compatibility rewrite names")
    initializers = [item for item in model.graph.initializer if item.name not in table_names]
    packed_table = np.concatenate(packed, axis=1)
    if np.any(np.abs(packed_table) > 65504):
        raise ValueError("Reference table exceeds finite FP16 range")
    initializers.extend([
        numpy_helper.from_array(packed_table.astype(np.float16), prefix + "table"),
        numpy_helper.from_array(np.asarray(widths, dtype=np.int64), prefix + "widths"),
    ])
    nodes = [node for node in model.graph.node if node.op_type != "Gather"]
    nodes.append(helper.make_node("Gather", [prefix + "table", next(iter(indices))],
                                  [prefix + "frame_fp16"], axis=0, name=prefix + "gather"))
    nodes.append(helper.make_node("Cast", [prefix + "frame_fp16"], [prefix + "frame"],
                                  to=1, name=prefix + "fp32_output_abi"))
    split_names = [prefix + name for name in REFERENCE_OUTPUTS]
    nodes.append(helper.make_node("Split", [prefix + "frame", prefix + "widths"],
                                  split_names, axis=1, name=prefix + "split"))
    for name, shape in REFERENCE_OUTPUTS.items():
        shape_name = prefix + name + "_shape"
        initializers.append(numpy_helper.from_array(np.asarray(shape, dtype=np.int64), shape_name))
        nodes.append(helper.make_node("Reshape", [prefix + name, shape_name], [name], name=prefix + name + "_reshape"))
    del model.graph.node[:]
    model.graph.node.extend(nodes)
    del model.graph.initializer[:]
    model.graph.initializer.extend(initializers)
    del model.graph.value_info[:]
    print(f"packed six reference Gather branches into one Gather ({sum(widths)} floats/frame)")


@contextmanager
def rknn_compatible_model(path):
    import onnx
    from onnx import version_converter

    model = onnx.load(str(path))
    default_opsets = [item.version for item in model.opset_import if item.domain in ("", "ai.onnx")]
    if len(default_opsets) != 1:
        raise ValueError(f"expected one ai.onnx opset, got {default_opsets}")
    target_opset = max(13, min(default_opsets[0], RKNN_MAX_ONNX_OPSET))
    if default_opsets[0] != target_opset:
        model = version_converter.convert_version(model, target_opset)
        print(f"converted ONNX opset {default_opsets[0]} -> {target_opset} for RKNN")
    if any(t.data_type == onnx.TensorProto.FLOAT16 for t in model.graph.initializer):
        from preprocess_wbc_lora_fp16 import normalize_reference_graph
        meta = {item.key: item.value for item in model.metadata_props}
        model = normalize_reference_graph(model, meta)
    pack_reference_gathers(model)
    onnx.checker.check_model(model)
    with tempfile.TemporaryDirectory(prefix="wbc_reference_compatible_") as directory:
        converted_path = Path(directory) / path.name
        onnx.save(model, str(converted_path))
        yield converted_path


def verify_reference(rknn, onnx_path, output_specs, meta, args):
    import numpy as np
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ret = rknn.init_runtime(**runtime_options(args))
    if ret != 0:
        raise RuntimeError(f"rknn.init_runtime failed, ret={ret}")
    count = int(meta["num_frames"])
    names = [name for name, _ in output_specs]
    maximum_errors = np.zeros(len(names), dtype=np.float64)
    error_sums = np.zeros(len(names), dtype=np.float64)
    worst_latency = 0.0
    # Check every frame, plus the clamped boundary behavior used by the controller.
    steps = [-8, -1] + list(range(count)) + [count, count + 8]
    for step in steps:
        time_step = np.array([[step]], dtype=np.float32)
        expected = session.run(names, {"time_step": time_step})
        started = time.perf_counter()
        actual = rknn.inference(inputs=[time_step])
        worst_latency = max(worst_latency, time.perf_counter() - started)
        if actual is None or len(actual) != len(names):
            raise RuntimeError("reference RKNN must return all six outputs")
        for i, (name, shape) in enumerate(output_specs):
            value = np.asarray(actual[i], dtype=np.float32)
            reference = np.asarray(expected[i], dtype=np.float32)
            if value.size != math.prod(shape) or not np.isfinite(value).all() or \
                    reference.shape != shape or not np.isfinite(reference).all():
                raise RuntimeError(f"invalid reference output {name} at frame {step}")
            error = np.abs(value.reshape(shape) - reference)
            maximum_errors[i] = max(maximum_errors[i], float(error.max()))
            error_sums[i] += float(error.mean())
            if error.max() > args.max_abs_error:
                raise RuntimeError(f"reference verification failed: frame={step}, output={name}, max_abs={error.max()}")
        if step % 50 == 0 or step == steps[-1]:
            print(f"  reference frame={step}, maximum errors={maximum_errors.tolist()}")
    for i, name in enumerate(names):
        print(f"{name}: max_abs={maximum_errors[i]:.6g}, mean_abs={error_sums[i] / len(steps):.6g}")
    print(f"reference verification passed: {len(steps)} queries, worst inference ms={worst_latency * 1000:.3f}")
    return {"queries": len(steps), "max_abs_per_output": dict(zip(names, maximum_errors.tolist())),
            "worst_ms": worst_latency * 1000.0}


def main():
    args = parse_args()
    onnx_path = args.onnx.resolve()
    output_path = (args.output or onnx_path.with_suffix(".rknn")).resolve()
    if not onnx_path.is_file() or onnx_path.suffix.lower() != ".onnx":
        raise ValueError(f"ONNX model not found: {onnx_path}")
    if output_path.suffix.lower() != ".rknn":
        raise ValueError("output filename must end with .rknn")
    if not math.isfinite(args.max_abs_error) or args.max_abs_error <= 0:
        raise ValueError("--max-abs-error must be finite and positive")
    runtime_options(args)
    input_specs, output_specs, reference_meta = check_reference_model(onnx_path)
    print("inputs: " + ", ".join(f"{name}{shape}" for name, shape in input_specs))
    print("outputs: " + ", ".join(f"{name}{shape}" for name, shape in output_specs))

    with rknn_compatible_model(onnx_path) as rknn_onnx_path:
        from rknn.api import RKNN

        rknn = RKNN(verbose=True)
        try:
            ret = rknn.config(target_platform="rk3588", float_dtype="float16")
            if ret != 0:
                raise RuntimeError(f"rknn.config failed, ret={ret}")
            ret = rknn.load_onnx(model=str(rknn_onnx_path))
            if ret != 0:
                raise RuntimeError(f"rknn.load_onnx failed, ret={ret}")
            ret = rknn.build(do_quantization=False)
            if ret != 0:
                raise RuntimeError(f"rknn.build failed, ret={ret}")
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix="wbc_rknn_", dir=output_path.parent) as directory:
                candidate = Path(directory) / output_path.name
                ret = rknn.export_rknn(str(candidate))
                if ret != 0:
                    raise RuntimeError(f"rknn.export_rknn failed, ret={ret}")
                verification = None if args.build_only else verify_reference(rknn, onnx_path, output_specs, reference_meta, args)
                candidate.replace(output_path)
                conversion_record(onnx_path, output_path, args, verification)
            print(f"exported: {output_path}; validation={'pending C++ deployment' if args.build_only else 'Python verification completed'}")
        finally:
            rknn.release()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
