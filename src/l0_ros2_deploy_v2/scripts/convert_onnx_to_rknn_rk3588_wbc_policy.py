#!/usr/bin/env python3
"""Build RK3588 FP16 WBC: static two-input policy, or shared 33-input LoRA with --manifest.

LoRA A/B/bias/LayerNorm remain runtime inputs; one shared policy serves all actions.
Use --build-only for the formal board deployment (no Python Runtime initialization).
"""

import argparse
import math
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

from wbc_lora_assets import (asset_path, check_graph, conversion_record, graph_feeds,
                             load_adapter, load_graph_manifest, rknn_build_config, runtime_options, sha256)

EXPECTED_INPUTS = {"policy": 399, "command_short": 240}
EXPECTED_OUTPUT = ("action", 21)
RKNN_MAX_ONNX_OPSET = 19


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx", type=Path, help="input WBC .onnx model")
    parser.add_argument("-o", "--output", type=Path, help="output .rknn (default: beside ONNX)")
    parser.add_argument("--samples", type=int, default=20, help="random verification samples (minimum: 20)")
    parser.add_argument("--manifest", type=Path, help="dynamic policy/probe graph manifest")
    parser.add_argument("--device-id", help="connected device ID; requires --target rk3588")
    parser.add_argument("--verification-data", type=Path, help="recorded NPZ: policy[N,399], command_short/command[N,240]")
    parser.add_argument("--cosine-threshold", type=float, default=0.999)
    parser.add_argument("--max-abs-error", type=float, default=0.05)
    parser.add_argument(
        "--target", help="runtime target platform, e.g. rk3588; select a board with --device-id"
    )
    parser.add_argument("--build-only", action="store_true",
                        help="export FP16 RKNN without loading a Python Runtime; C++ deployment validation remains pending")
    parser.add_argument("--dry-run", action="store_true",
                        help="validate ONNX/manifest/adapters and print the build plan without importing RKNN or writing files")
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


def check_model(path, manifest_path=None):
    import onnx

    if manifest_path is not None:
        graph = check_graph(path, manifest_path)
        return [(x["name"], tuple(x["shape"])) for x in graph["inputs"]], (graph["outputs"][0]["name"], tuple(graph["outputs"][0]["shape"]))

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    initializers = {item.name for item in model.graph.initializer}
    inputs = [item for item in model.graph.input if item.name not in initializers]
    outputs = list(model.graph.output)
    names = {item.name for item in inputs}
    if len(inputs) != 2 or names not in ({"policy", "command_short"}, {"policy", "command"}):
        raise ValueError(f"expected policy and command_short/command inputs, got {[item.name for item in inputs]}")
    specs = []
    for item in inputs:
        shape = static_shape(item, "input", onnx)
        width = EXPECTED_INPUTS["policy" if item.name == "policy" else "command_short"]
        if shape not in ((width,), (1, width)):
            raise ValueError(f"input '{item.name}' must contain {width} floats, got {shape}")
        specs.append((item.name, shape))
    if len(outputs) != 1 or outputs[0].name != EXPECTED_OUTPUT[0]:
        raise ValueError(f"expected only output '{EXPECTED_OUTPUT[0]}', got {[item.name for item in outputs]}")
    output_shape = static_shape(outputs[0], "output", onnx)
    if output_shape not in ((21,), (1, 21)):
        raise ValueError(f"output 'action' must contain 21 floats, got {output_shape}")
    return specs, (outputs[0].name, output_shape)


def replace_mish(model, onnx):
    rewritten = []
    count = 0
    for node in model.graph.node:
        if node.op_type != "Mish" or node.domain not in ("", "ai.onnx"):
            rewritten.append(node)
            continue
        if len(node.input) != 1 or len(node.output) != 1:
            raise ValueError(f"Mish node '{node.name}' must have one input and one output")
        count += 1
        prefix = node.name or f"mish_{count}"
        softplus = f"{node.output[0]}__softplus"
        tanh = f"{node.output[0]}__tanh"
        rewritten.extend(
            [
                onnx.helper.make_node("Softplus", node.input, [softplus], name=f"{prefix}__softplus"),
                onnx.helper.make_node("Tanh", [softplus], [tanh], name=f"{prefix}__tanh"),
                onnx.helper.make_node("Mul", [node.input[0], tanh], node.output, name=f"{prefix}__mul"),
            ]
        )
    if count:
        del model.graph.node[:]
        model.graph.node.extend(rewritten)
    return count


@contextmanager
def rknn_compatible_model(path):
    import onnx
    from onnx import version_converter

    model = onnx.load(str(path))
    default_opsets = [item.version for item in model.opset_import if item.domain in ("", "ai.onnx")]
    if len(default_opsets) != 1:
        raise ValueError(f"expected one ai.onnx opset, got {default_opsets}")
    changed = False
    if default_opsets[0] > RKNN_MAX_ONNX_OPSET:
        model = version_converter.convert_version(model, RKNN_MAX_ONNX_OPSET)
        changed = True
        print(f"converted ONNX opset {default_opsets[0]} -> {RKNN_MAX_ONNX_OPSET} for RKNN")

    mish_count = replace_mish(model, onnx)
    if mish_count:
        changed = True
        print(f"replaced {mish_count} Mish node(s) with NPU-supported Softplus/Tanh/Mul")

    if not changed:
        yield path
        return

    onnx.checker.check_model(model)
    with tempfile.TemporaryDirectory(prefix="wbc_onnx_opset19_") as directory:
        converted_path = Path(directory) / path.name
        onnx.save(model, str(converted_path))
        yield converted_path


def cosine_similarity(reference, candidate):
    import numpy as np

    denominator = float(np.linalg.norm(reference) * np.linalg.norm(candidate))
    if denominator == 0.0:
        return 1.0 if np.array_equal(reference, candidate) else 0.0
    return float(np.dot(reference, candidate) / denominator)


def verify_dynamic(rknn, onnx_path, args):
    import numpy as np
    import onnxruntime as ort
    graph = check_graph(onnx_path, args.manifest)
    root = args.manifest.resolve().parent
    data_path = args.verification_data or asset_path(root, graph["verification_data"])
    observations = {}
    with np.load(data_path, allow_pickle=False) as data:
        for item in graph["inputs"]:
            if item["role"] != "observation":
                continue
            value = data[item["name"]]
            if value.dtype != np.float32 or list(value.shape[1:]) != item["shape"] or not np.isfinite(value).all():
                raise ValueError(f"Invalid recorded input {item['name']}")
            observations[item["name"]] = value.copy()
    counts = {len(x) for x in observations.values()}
    if len(counts) != 1 or min(counts) < 20:
        raise ValueError("Dynamic verification requires at least 20 aligned observation samples")
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ret = rknn.init_runtime(**runtime_options(args))
    if ret != 0:
        raise RuntimeError(f"rknn.init_runtime failed, ret={ret}")
    reports, first_feed, first_output = [], None, None
    for action in graph["actions"]:
        adapter_path = asset_path(root, action["adapter_manifest"])
        parameters = load_adapter(adapter_path, graph, action["action_id"])
        maximum, worst, minimum_cosine = 0.0, 0.0, 1.0
        for i in range(min(counts)):
            feed = graph_feeds(graph, {k: v[i] for k, v in observations.items()}, parameters)
            expected = np.asarray(session.run(["action"], feed)[0], np.float32).reshape(-1)
            started = time.perf_counter()
            actual_outputs = rknn.inference(inputs=[feed[x["name"]] for x in graph["inputs"]])
            worst = max(worst, time.perf_counter() - started)
            if actual_outputs is None or len(actual_outputs) != 1:
                raise RuntimeError("Dynamic RKNN must return one output")
            actual = np.asarray(actual_outputs[0], np.float32).reshape(-1)
            if actual.shape != expected.shape or not np.isfinite(actual).all():
                raise RuntimeError("Invalid dynamic RKNN output")
            error = float(np.max(np.abs(actual - expected)))
            cosine = cosine_similarity(expected, actual) if np.linalg.norm(expected) > 1e-8 else 1.0
            maximum, minimum_cosine = max(maximum, error), min(minimum_cosine, cosine)
            if error > args.max_abs_error or cosine < args.cosine_threshold:
                raise RuntimeError(f"Dynamic RKNN verification failed: {action['action_id']} sample={i} error={error} cosine={cosine}")
            if first_feed is None:
                first_feed, first_output = feed, actual.copy()
        reports.append({"action_id": action["action_id"], "samples": min(counts),
                        "adapter_manifest_sha256": sha256(adapter_path), "max_abs": maximum,
                        "minimum_cosine": minimum_cosine, "worst_ms": worst * 1000.0})
        print(f"dynamic verification: {reports[-1]}")
    if first_feed is None:
        raise ValueError("Graph manifest has no selected adapters")
    repeated = rknn.inference(inputs=[first_feed[x["name"]] for x in graph["inputs"]])
    if repeated is None or not np.allclose(np.asarray(repeated[0]).reshape(-1), first_output, atol=args.max_abs_error, rtol=0):
        raise RuntimeError("A-to-B-to-A RKNN verification failed")
    return {"actions": reports, "a_to_b_to_a": "passed" if len(reports) > 1 else "one_adapter_only"}


def verify(rknn, onnx_path, input_specs, output_spec, args):
    if args.manifest:
        return verify_dynamic(rknn, onnx_path, args)
    import numpy as np
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ret = rknn.init_runtime(**runtime_options(args))
    if ret != 0:
        raise RuntimeError(f"rknn.init_runtime failed, ret={ret}")

    rng = np.random.default_rng(0)
    recorded = None
    if args.verification_data:
        with np.load(args.verification_data, allow_pickle=False) as data:
            recorded = {}
            for name, shape in input_specs:
                key = name
                if name != "policy" and name not in data:
                    key = "command" if name == "command_short" else "command_short"
                value = data[key]
                if value.dtype != np.float32 or value.ndim != 2 or value.shape[1] != shape[-1] or \
                        not np.isfinite(value).all():
                    raise ValueError(f"recorded {name} must be finite float32 [N,{shape[-1]}]")
                recorded[name] = value.copy()
        counts = {len(value) for value in recorded.values()}
        if len(counts) != 1 or min(counts) < 20:
            raise ValueError("recorded policy and command need the same number of frames, at least 20")
        sample_count = min(counts)
    else:
        sample_count = args.samples
        print("Random inputs check conversion only; replay recorded observations before hardware use")
    minimum_cosine = 1.0
    maximum_error = 0.0
    error_sum = np.zeros(EXPECTED_OUTPUT[1], dtype=np.float64)
    joint_maximum = np.zeros(EXPECTED_OUTPUT[1], dtype=np.float64)
    cumulative_error = np.zeros(EXPECTED_OUTPUT[1], dtype=np.float64)
    worst_latency = 0.0
    for sample in range(sample_count):
        feed = {name: recorded[name][sample].reshape(shape) if recorded is not None else
                rng.uniform(-1.0, 1.0, shape).astype(np.float32) for name, shape in input_specs}
        expected = np.asarray(session.run([output_spec[0]], feed)[0], dtype=np.float32).reshape(-1)
        if expected.size != EXPECTED_OUTPUT[1] or not np.isfinite(expected).all():
            raise RuntimeError("ONNX action must contain 21 finite floats")
        started = time.perf_counter()
        actual_outputs = rknn.inference(inputs=[feed[name] for name, _ in input_specs])
        latency = time.perf_counter() - started
        worst_latency = max(worst_latency, latency)
        if actual_outputs is None or len(actual_outputs) != 1:
            raise RuntimeError("RKNN must return exactly one output")
        actual = np.asarray(actual_outputs[0], dtype=np.float32).reshape(-1)
        if actual.size != EXPECTED_OUTPUT[1] or not np.isfinite(actual).all():
            raise RuntimeError(f"RKNN action must contain 21 finite floats, got {actual.shape}")
        error = np.abs(expected - actual)
        max_error = float(np.max(error))
        error_sum += error
        joint_maximum = np.maximum(joint_maximum, error)
        cumulative_error += actual - expected
        cosine = cosine_similarity(expected, actual)
        minimum_cosine = min(minimum_cosine, cosine)
        maximum_error = max(maximum_error, max_error)
        print(f"  sample={sample:02d} cosine={cosine:.6f} max_abs={max_error:.6g} latency_ms={latency * 1000:.3f}")
        if cosine < args.cosine_threshold or max_error > args.max_abs_error:
            raise RuntimeError(f"verification failed at sample {sample}")
    print(f"verification passed: minimum cosine={minimum_cosine:.6f}, maximum abs error={maximum_error:.6g}")
    print(f"mean abs error={error_sum.sum() / (sample_count * 21):.6g}, worst inference ms={worst_latency * 1000:.3f}")
    print(f"per-joint max abs error: {joint_maximum.tolist()}")
    print(f"per-joint mean abs error: {(error_sum / sample_count).tolist()}")
    print(f"per-joint signed cumulative error: {cumulative_error.tolist()}")


def main():
    args = parse_args()
    onnx_path = args.onnx.resolve()
    output_path = (args.output or onnx_path.with_suffix(".rknn")).resolve()
    if not onnx_path.is_file() or onnx_path.suffix.lower() != ".onnx":
        raise ValueError(f"ONNX model not found: {onnx_path}")
    if output_path.suffix.lower() != ".rknn":
        raise ValueError("output filename must end with .rknn")
    if args.samples < 20:
        raise ValueError("--samples must be at least 20")
    if not math.isfinite(args.cosine_threshold) or not -1.0 <= args.cosine_threshold <= 1.0 or \
            not math.isfinite(args.max_abs_error) or args.max_abs_error <= 0:
        raise ValueError("verification thresholds must be finite: cosine in [-1,1], max abs error > 0")

    runtime_options(args)
    input_specs, output_spec = check_model(onnx_path, args.manifest)
    output_specs = [output_spec]
    print("inputs: " + ", ".join(f"{name}{shape}" for name, shape in input_specs))
    print("outputs: " + ", ".join(f"{name}{shape}" for name, shape in output_specs))
    if args.manifest:
        graph = load_graph_manifest(args.manifest)
        root = args.manifest.resolve().parent
        actions = graph.get("actions", [])
        if not actions or len({x["action_id"] for x in actions}) != len(actions):
            raise ValueError("Dynamic graph requires unique, non-empty action entries")
        for action in actions:
            load_adapter(asset_path(root, action["adapter_manifest"]), graph, action["action_id"])
        print(f"shared {graph['kind']}: actions={len(actions)}, rank={graph['rank']}, alpha={graph['alpha']}; "
              "adapter tensors remain runtime inputs (no per-action policy compilation)")
    print(f"build: config={rknn_build_config()} do_quantization=False; output={output_path}")
    if args.dry_run:
        print(f"DRY-RUN: no RKNN build/runtime; after build, validation={'pending formal C++ runtime' if args.build_only else 'Python verification requested'}")
        return 0

    with rknn_compatible_model(onnx_path) as rknn_onnx_path:
        from rknn.api import RKNN

        rknn = RKNN(verbose=True)
        try:
            ret = rknn.config(**rknn_build_config())
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
                verification = None if args.build_only else verify(rknn, onnx_path, input_specs, output_spec, args)
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
