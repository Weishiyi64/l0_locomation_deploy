#!/usr/bin/env python3
"""Convert one qwen_tokenizer ONNX to non-quantized RK3588 RKNN beside the source file.

Only this model is checked and converted; no manifest, other model or NPZ is needed.
No numerical inference, ROS build or hardware launch is performed.
"""
import argparse
import importlib.metadata
import json
import os
import platform
import sys
import tempfile
from pathlib import Path

TOOLKIT_VERSION = "2.3.2"
MODEL_NAME = "qwen_tokenizer"
MODEL_ABI = {
    "inputs": [("input_ids", [1, 129], "int32"), ("padding_mask", [1, 129], "int32")],
    "outputs": [("qwen_embedding", [1, 1024], "float32"), ("head_logits", [1, 47], "float32"),
                ("left_hand_logits", [1, 43], "float32"), ("right_hand_logits", [1, 43], "float32")],
}


def inspect_model(path):
    """Static graph/ABI admission only; this does not run a neural model."""
    import onnx

    model = onnx.load(str(path), load_external_data=False)
    onnx.checker.check_model(model)
    problems = []
    types = {onnx.TensorProto.FLOAT: "float32", onnx.TensorProto.FLOAT16: "float16",
             onnx.TensorProto.INT32: "int32", onnx.TensorProto.INT64: "int64", onnx.TensorProto.BOOL: "bool"}
    interfaces = {}
    for kind, tensors in (("inputs", model.graph.input), ("outputs", model.graph.output)):
        actual = []
        for value in tensors:
            tensor = value.type.tensor_type
            if tensor.elem_type not in types:
                problems.append(f"{value.name}: nonnumeric tensor")
            if not tensor.HasField("shape") or any(not d.HasField("dim_value") or d.dim_value <= 0 for d in tensor.shape.dim):
                problems.append(f"{value.name}: dynamic/unknown shape")
            actual.append((value.name, [d.dim_value for d in tensor.shape.dim], types.get(tensor.elem_type)))
        interfaces[kind] = actual
    if interfaces != MODEL_ABI:
        problems.append(f"expected the native {MODEL_NAME} tensor interface")

    def walk(graph):
        for tensor in graph.initializer:
            if tensor.data_location == onnx.TensorProto.EXTERNAL:
                problems.append("external tensor data is not accepted")
        for node in graph.node:
            if node.domain not in ("", "ai.onnx"):
                problems.append(f"custom operator: {node.domain}::{node.op_type}")
            if node.op_type in ("If", "Loop", "Scan", "NonZero", "RandomUniformLike", "RandomNormalLike"):
                problems.append(f"dynamic/control operator: {node.op_type}")
            for attribute in node.attribute:
                if attribute.type == onnx.AttributeProto.GRAPH:
                    walk(attribute.g)

    walk(model.graph)
    if model.functions:
        problems.append("local functions must be inlined")
    if problems:
        raise ValueError("Not an admitted static neural model: " + "; ".join(sorted(set(problems))))
    return {"model": MODEL_NAME, "inputs": [value.name for value in model.graph.input],
            "outputs": [value.name for value in model.graph.output],
            "operators": sorted({node.op_type for node in model.graph.node}),
            "note": "Static preflight only; Toolkit2 compilation and board validation are still required."}


def compile_one(source, destination):
    from rknn.api import RKNN

    def checked(code, operation):
        if code != 0:
            raise RuntimeError(f"RKNN {operation} failed (return code {code})")

    source, destination = source.resolve(), destination.resolve()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"RKNN output already exists: {destination}")
    # Toolkit2 writes diagnostics to cwd; keep them out of the source and final bundle.
    with tempfile.TemporaryDirectory(prefix=".compiler_", dir=destination.parent) as work:
        previous = Path.cwd()
        try:
            os.chdir(work)
            rknn = RKNN(verbose=True)
            try:
                checked(rknn.config(target_platform="rk3588"), "config")
                checked(rknn.load_onnx(model=str(source)), "load_onnx")
                checked(rknn.build(do_quantization=False), "build")
                checked(rknn.export_rknn(str(destination)), "export_rknn")
                if not destination.is_file() or destination.stat().st_size == 0:
                    raise RuntimeError(f"Compiler reported success but produced no model: {destination}")
            finally:
                rknn.release()
        finally:
            os.chdir(previous)


def compiler_version():
    if platform.system() != "Linux" or platform.machine() != "aarch64":
        raise ValueError("Actual conversion must run on RK3588 Linux/aarch64; use --dry-run for static checks")
    version = importlib.metadata.version("rknn-toolkit2")
    if version != TOOLKIT_VERSION:
        raise ValueError(f"Expected rknn-toolkit2 {TOOLKIT_VERSION}; found {version}")
    return version


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx", type=Path, help="input ONNX model")
    parser.add_argument("-o", "--output", type=Path, help="output .rknn (default: beside ONNX)")
    parser.add_argument("--dry-run", action="store_true", help="check the ONNX graph without writing or importing RKNN")
    args = parser.parse_args(argv)

    source = args.onnx.resolve(strict=True)
    if not source.is_file() or source.suffix.lower() != ".onnx":
        raise ValueError("Input must be an ONNX file")
    output = (args.output or source.with_suffix(".rknn")).absolute()
    if output.suffix.lower() != ".rknn":
        raise ValueError("Output filename must end with .rknn")
    if not args.dry_run and (output.exists() or output.is_symlink()):
        raise FileExistsError(f"RKNN output already exists; choose a new output file: {output}")
    report = inspect_model(source)
    print(json.dumps({"status": "static_preflight_passed_not_compiled", **report,
                      "source": str(source), "output": str(output), "quantized": False,
                      "toolkit2_version": TOOLKIT_VERSION}, ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0
    compiler_version()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".txt2motion_compile_", dir=output.parent) as work:
        candidate = Path(work) / output.name
        compile_one(source, candidate)
        os.link(candidate, output)  # Publish a complete model without replacing an existing file.
    print(f"Exported: {output}; RKNN loading and robot effects remain unverified")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
