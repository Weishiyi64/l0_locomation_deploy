#!/usr/bin/env python3
"""Convert a float32 ONNX model to RK3588 RKNN with RKNN-Toolkit2."""

import argparse
import hashlib
import json
import sys
import tempfile
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a float32 ONNX model to RK3588 RKNN with RKNN-Toolkit2 (non-quantized)."
    )
    parser.add_argument("onnx", type=Path, help="input .onnx model")
    parser.add_argument("-o", "--output", type=Path, help="output .rknn (default: beside ONNX)")
    parser.add_argument("--verify", action="store_true", help="compare RKNN simulator output with ONNX Runtime")
    parser.add_argument("--samples", type=int, default=20, help="random samples used by --verify")
    parser.add_argument("--cosine-threshold", type=float, default=0.999)
    parser.add_argument("--max-abs-error", type=float, default=0.05)
    return parser.parse_args()


def check_onnx_model(path):
    import onnx

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    initializer_names = {item.name for item in model.graph.initializer}
    inputs = [item for item in model.graph.input if item.name not in initializer_names]
    if not inputs:
        raise ValueError("ONNX model has no runtime inputs")
    if not model.graph.output:
        raise ValueError("ONNX model has no outputs")

    def tensor_spec(item, kind):
        if item.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
            raise ValueError(f"{kind} '{item.name}' must be float32")
        shape = []
        for dim in item.type.tensor_type.shape.dim:
            if dim.dim_value > 0:
                shape.append(dim.dim_value)
            elif dim.dim_param.lower() in {"batch", "batch_size"}:
                shape.append(1)
            else:
                dynamic_name = dim.dim_param or "unnamed"
                raise ValueError(
                    f"{kind} '{item.name}' has unsupported dynamic dimension '{dynamic_name}'; "
                    "only batch/batch_size can be fixed automatically"
                )
        return item.name, tuple(shape)

    input_specs = [tensor_spec(item, "input") for item in inputs]
    output_specs = [tensor_spec(item, "output") for item in model.graph.output]
    metadata = {item.key: item.value for item in model.metadata_props}
    return input_specs, output_specs, metadata


def prepare_onnx_for_rknn(path, directory):
    """Extract the RKNN-supported actor MLP.

    The source model deliberately contains validation, angle wrapping and
    output interlocks.  RKNN Toolkit 2.3.x cannot lower IsInf/IsNaN/Atan and
    has been observed to segfault instead of returning an error.  The runtime
    performs those deterministic operations and only sends sanitized history
    to this MLP.
    """
    import copy

    import onnx

    model = onnx.load(str(path))
    actor_nodes = [node for node in model.graph.node if node.name.startswith("/actor/actor.")]
    if [node.op_type for node in actor_nodes] != ["Gemm", "Elu", "Gemm", "Elu", "Gemm", "Elu", "Gemm"]:
        raise ValueError("planner actor must be the verified four-layer Gemm/ELU MLP")

    initializers = {item.name: item for item in model.graph.initializer}
    actor_initializers = []
    for node in actor_nodes:
        for name in node.input[1:]:
            if name not in initializers:
                raise ValueError(f"actor initializer not found: {name}")
            actor_initializers.append(copy.deepcopy(initializers[name]))

    reshape_shape = "__actor_input_shape"
    actor_initializers.append(onnx.helper.make_tensor(reshape_shape, onnx.TensorProto.INT64, [2], [1, 85]))
    nodes = [
        onnx.helper.make_node(
            "Reshape", ["state_history", reshape_shape], ["/Reshape_1_output_0"], name="actor_input_reshape"
        )
    ]
    nodes.extend(copy.deepcopy(actor_nodes))
    nodes[-1].output[0] = "actions"

    graph = onnx.helper.make_graph(
        nodes,
        "planner_actor_rknn",
        [onnx.helper.make_tensor_value_info("state_history", onnx.TensorProto.FLOAT, [1, 5, 17])],
        [onnx.helper.make_tensor_value_info("actions", onnx.TensorProto.FLOAT, [1, 2])],
        actor_initializers,
    )
    actor_model = onnx.helper.make_model(
        graph,
        producer_name=model.producer_name,
        producer_version=model.producer_version,
        opset_imports=copy.deepcopy(model.opset_import),
    )
    actor_model.ir_version = model.ir_version
    actor_model.metadata_props.extend(copy.deepcopy(model.metadata_props))
    onnx.checker.check_model(actor_model)
    converted_path = Path(directory) / f"{path.stem}.actor.onnx"
    onnx.save(actor_model, str(converted_path))
    print("RKNN partition: actor MLP only (validation and velocity mapping stay in C++)")
    return converted_path, [("state_history", (1, 5, 17))], [("actions", (1, 2))]


def preprocess_history(history):
    import numpy as np

    result = np.asarray(history, dtype=np.float32).copy()
    for state in result.reshape(-1, 17):
        state[6] = np.clip(state[6], -1.0, 1.2)
        state[7] = np.clip(state[7], -3.14, 3.14)
        task = state[8:13].copy()
        nav = bool(np.any(task[:3] != 0.0))
        face = bool(np.any(task[3:] != 0.0))
        if nav and not face:
            state[8:13] = [task[0], task[1], np.clip(task[2], 0.3, 1.2), 0.0, 0.0]
        elif face and not nav:
            error = np.arctan2(np.sin(task[3]), np.cos(task[3]))
            turn_speed = np.clip(abs(task[4]), 1.57, 3.14)
            fade = np.clip((1.0 - abs(error)) / 0.7382006, 0.0, 1.0)
            low_speed = min(turn_speed, 0.3)
            magnitude = turn_speed - (turn_speed - low_speed) * np.expm1(3.0 * fade) / 19.085537
            omega_star = 0.0 if abs(error) <= 0.2617994 else np.copysign(magnitude, error)
            state[8:13] = [0.0, 0.0, 0.0, error, omega_star]
            state[13:17] = 0.0
        else:
            state[8:17] = 0.0
        if not nav:
            state[13:17] = 0.0
    return result


def map_velocity(actions, processed_history):
    import numpy as np

    task = np.asarray(processed_history, dtype=np.float32).reshape(-1, 17)[-1, 8:13]
    nav = bool(np.any(task[:3] != 0.0)) and not bool(np.any(task[3:] != 0.0))
    face = bool(np.any(task[3:] != 0.0)) and not bool(np.any(task[:3] != 0.0))
    result = np.zeros((1, 3), dtype=np.float32)
    flat = np.asarray(actions, dtype=np.float32).reshape(-1)
    if nav:
        result[0, 0] = np.clip(task[2] + flat[0], -1.0, 1.2)
    if nav or face:
        result[0, 2] = np.clip(task[4] + flat[1], -3.14, 3.14)
    return result


def verify(rknn, onnx_path, actor_path, input_specs, metadata, args):
    import numpy as np
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    actor_session = ort.InferenceSession(str(actor_path), providers=["CPUExecutionProvider"])
    if [item.name for item in session.get_outputs()] != ["velocity_command", "actions"]:
        raise RuntimeError("source planner outputs must be velocity_command and actions")

    ret = rknn.init_runtime()
    if ret != 0:
        raise RuntimeError(f"rknn.init_runtime failed, ret={ret}")

    minimum_cosine = 1.0
    maximum_error = 0.0

    def compare(raw_history, label):
        nonlocal minimum_cosine, maximum_error
        processed = preprocess_history(raw_history)
        expected_velocity, expected_actions = session.run(None, {"state_history": raw_history})
        extracted_actions = actor_session.run(["actions"], {"state_history": processed})[0]
        if np.max(np.abs(expected_actions - extracted_actions)) > 1e-5:
            raise RuntimeError("actor extraction does not match the source planner")
        actual = rknn.inference(inputs=[processed])
        if len(actual) != 1:
            raise RuntimeError(f"output count mismatch: RKNN={len(actual)}, expected=1")
        actual_actions = np.asarray(actual[0], dtype=np.float32).reshape(1, 2)
        actual_velocity = map_velocity(actual_actions, processed)

        for name, reference, candidate in (
            ("actions", expected_actions, actual_actions),
            ("velocity_command", expected_velocity, actual_velocity),
        ):
            reference = np.asarray(reference, dtype=np.float32).ravel()
            candidate = np.asarray(candidate, dtype=np.float32).ravel()
            if reference.size != candidate.size:
                raise RuntimeError(f"output '{name}' size mismatch: RKNN={candidate.size}, ONNX={reference.size}")
            max_error = float(np.max(np.abs(reference - candidate))) if reference.size else 0.0
            denominator = float(np.linalg.norm(reference) * np.linalg.norm(candidate))
            cosine = float(np.dot(reference, candidate) / denominator) if denominator else 1.0
            minimum_cosine = min(minimum_cosine, cosine)
            maximum_error = max(maximum_error, max_error)
            print(f"  sample={label} output={name} cosine={cosine:.6f} max_abs={max_error:.6g}")
            if cosine < args.cosine_threshold or max_error > args.max_abs_error:
                raise RuntimeError(f"verification failed for output '{name}'")
        return expected_velocity, actual_velocity

    rng = np.random.default_rng(0)
    for sample in range(args.samples):
        state = rng.uniform(-1.0, 1.0, input_specs[0][1]).astype(np.float32)
        state[:, :, 6] = rng.uniform(-1.0, 1.2, (1, 5))
        state[:, :, 7] = rng.uniform(-3.14, 3.14, (1, 5))
        mode = sample % 3
        state[:, :, 8:17] = 0.0
        if mode == 0:
            state[:, :, 8:11] = rng.uniform([0.1, -1.0, 0.3], [2.0, 1.0, 1.2], (1, 5, 3))
            state[:, :, 13:17] = rng.uniform(-3.0, 3.0, (1, 5, 4))
        elif mode == 1:
            state[:, :, 11] = rng.uniform(-3.14, 3.14, (1, 5))
            state[:, :, 12] = rng.uniform(1.57, 3.14, (1, 5))
        compare(state, sample)

    invariants = {}
    if "l0_arrival_contract" in metadata:
        contract = json.loads(metadata["l0_arrival_contract"])
        if input_specs != [("state_history", (1, 5, 17))]:
            raise RuntimeError("planner invariant checks require state_history[1,5,17]")
        cases = {
            "nav_vy_zero": ([1.0, 0.2, 0.5, 0.0, 0.0], [1]),
            "face_vx_vy_zero": ([0.0, 0.0, 0.0, 0.4, 2.0], [0, 1]),
            "hold_all_zero": ([0.0] * 5, [0, 1, 2]),
        }
        for name, (task, zero_indices) in cases.items():
            state = np.zeros((1, 5, 17), dtype=np.float32)
            state[:, :, 5] = -1.0
            state[:, :, 8:13] = task
            if name == "nav_vy_zero":
                state[:, :, 13:17] = [2.0, 0.2, 3.0, 0.2]
            expected, actual = compare(state, name)
            for output in (expected, actual):
                flat = np.asarray(output, dtype=np.float32).ravel()
                if any(abs(float(flat[index])) > 1e-6 for index in zero_indices):
                    raise RuntimeError(f"planner invariant failed: {name}")
            invariants[name] = True
        if contract.get("output_interlocks") is None:
            raise RuntimeError("planner contract is missing output_interlocks")
    return {
        "minimum_cosine": minimum_cosine,
        "maximum_absolute_error": maximum_error,
        "invariants": invariants,
    }


def main():
    args = parse_args()
    onnx_path = args.onnx.resolve()
    output_path = (args.output or onnx_path.with_suffix(".rknn")).resolve()
    if onnx_path.suffix.lower() != ".onnx" or not onnx_path.is_file():
        raise ValueError(f"ONNX model not found: {onnx_path}")
    if output_path.suffix.lower() != ".rknn":
        raise ValueError("output filename must end with .rknn")
    if args.samples <= 0:
        raise ValueError("--samples must be positive")

    source_input_specs, source_output_specs, metadata = check_onnx_model(onnx_path)
    print("inputs: " + ", ".join(f"{name} {shape}" for name, shape in source_input_specs))
    print("source outputs: " + ", ".join(name for name, _ in source_output_specs))

    temporary_directory = tempfile.TemporaryDirectory(prefix="rknn_onnx_")
    rknn = None
    try:
        conversion_path, input_specs, output_specs = prepare_onnx_for_rknn(onnx_path, temporary_directory.name)
        if source_input_specs != input_specs:
            raise ValueError("source planner input must be state_history[1,5,17]")
        print("RKNN output: actions")

        from rknn.api import RKNN

        rknn = RKNN(verbose=False)
        ret = rknn.config(target_platform="rk3588")
        if ret != 0:
            raise RuntimeError(f"rknn.config failed, ret={ret}")
        ret = rknn.load_onnx(
            model=str(conversion_path),
            inputs=[name for name, _ in input_specs],
            input_size_list=[list(shape) for _, shape in input_specs],
        )
        if ret != 0:
            raise RuntimeError(f"rknn.load_onnx failed, ret={ret}")
        ret = rknn.build(do_quantization=False)
        if ret != 0:
            raise RuntimeError(f"rknn.build failed, ret={ret}")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        ret = rknn.export_rknn(str(output_path))
        if ret != 0:
            raise RuntimeError(f"rknn.export_rknn failed, ret={ret}")
        print(f"exported: {output_path}")

        verification_results = {}
        if args.verify:
            verification_results = verify(rknn, onnx_path, conversion_path, input_specs, metadata, args)
            print("verification passed")

        manifest = {
            "format_version": 2,
            "target_platform": "rk3588",
            "inference_partition": "actor_mlp",
            "onnx_sha256": hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
            "rknn_sha256": hashlib.sha256(output_path.read_bytes()).hexdigest(),
            "inputs": [{"name": name, "shape": list(shape)} for name, shape in input_specs],
            "outputs": [{"name": name, "shape": list(shape)} for name, shape in output_specs],
            "metadata": metadata,
            "verification": {
                "performed": bool(args.verify),
                "samples": args.samples if args.verify else 0,
                "cosine_threshold": args.cosine_threshold,
                "max_abs_error": args.max_abs_error,
                **verification_results,
            },
        }
        manifest_path = Path(str(output_path) + ".json")
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        print(f"manifest: {manifest_path}")
    finally:
        if rknn is not None:
            rknn.release()
        temporary_directory.cleanup()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
