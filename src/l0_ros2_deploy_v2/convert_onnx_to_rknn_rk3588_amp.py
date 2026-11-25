#!/usr/bin/env python3
"""Convert a single-input AMP ONNX policy to non-quantized RK3588 RKNN."""

import argparse
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Convert an AMP ONNX policy to RK3588 RKNN.")
    parser.add_argument("onnx", type=Path, help="input AMP .onnx model")
    parser.add_argument("-o", "--output", type=Path, help="output .rknn (default: beside ONNX)")
    parser.add_argument("--verify", action="store_true", help="compare RKNN output with ONNX Runtime")
    parser.add_argument("--samples", type=int, default=5, help="random samples used by --verify")
    parser.add_argument("--cosine-threshold", type=float, default=0.999)
    parser.add_argument("--max-abs-error", type=float, default=0.05)
    return parser.parse_args()


def check_amp_model(path):
    import onnx

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    initializer_names = {item.name for item in model.graph.initializer}
    inputs = [item for item in model.graph.input if item.name not in initializer_names]
    outputs = list(model.graph.output)
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError(f"AMP policy must have 1 input and 1 output, got {len(inputs)} and {len(outputs)}")

    def tensor_spec(item, kind):
        if item.type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
            raise ValueError(f"{kind} '{item.name}' must be float32")
        shape = []
        dynamic_batch = False
        for index, dim in enumerate(item.type.tensor_type.shape.dim):
            if dim.dim_value > 0:
                shape.append(dim.dim_value)
            elif index == 0 and dim.dim_param.lower() in {"batch", "batch_size"}:
                shape.append(1)
                dynamic_batch = True
            else:
                raise ValueError(f"{kind} '{item.name}' must have a fixed shape (dynamic batch is allowed)")
        return item.name, tuple(shape), dynamic_batch

    input_name, input_shape, dynamic_batch = tensor_spec(inputs[0], "input")
    output_name, output_shape, _ = tensor_spec(outputs[0], "output")
    if len(input_shape) != 2 or input_shape[0] != 1:
        raise ValueError(f"AMP input must be [1, observation_size], got {input_shape}")
    if output_name != "actions" or len(output_shape) != 2 or output_shape[0] != 1:
        raise ValueError(f"AMP output must be actions[1, action_size], got {output_name}{output_shape}")
    return (input_name, input_shape), (output_name, output_shape), dynamic_batch


def verify(rknn, onnx_path, input_spec, output_spec, args):
    import numpy as np
    import onnxruntime as ort

    input_name, input_shape = input_spec
    output_name, output_shape = output_spec
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ret = rknn.init_runtime()
    if ret != 0:
        raise RuntimeError(f"rknn.init_runtime failed, ret={ret}")

    rng = np.random.default_rng(0)
    for sample in range(args.samples):
        input_data = rng.uniform(-1.0, 1.0, input_shape).astype(np.float32)
        expected = np.asarray(session.run([output_name], {input_name: input_data})[0], dtype=np.float32)
        outputs = rknn.inference(inputs=[input_data])
        if len(outputs) != 1:
            raise RuntimeError(f"RKNN output count mismatch: expected 1, got {len(outputs)}")
        actual = np.asarray(outputs[0], dtype=np.float32).reshape(output_shape)
        expected = expected.reshape(output_shape)
        max_error = float(np.max(np.abs(expected - actual))) if expected.size else 0.0
        denominator = float(np.linalg.norm(expected) * np.linalg.norm(actual))
        cosine = float(np.sum(expected * actual) / denominator) if denominator else 1.0
        print(f"  sample={sample} cosine={cosine:.6f} max_abs={max_error:.6g}")
        if cosine < args.cosine_threshold or max_error > args.max_abs_error:
            raise RuntimeError(f"verification failed at sample {sample}")


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

    input_spec, output_spec, dynamic_batch = check_amp_model(onnx_path)
    print(f"input: {input_spec[0]} {input_spec[1]}")
    print(f"output: {output_spec[0]} {output_spec[1]}")

    from rknn.api import RKNN

    rknn = RKNN(verbose=True)
    try:
        ret = rknn.config(target_platform="rk3588")
        if ret != 0:
            raise RuntimeError(f"rknn.config failed, ret={ret}")
        load_options = {"model": str(onnx_path)}
        if dynamic_batch:
            load_options.update(inputs=[input_spec[0]], input_size_list=[list(input_spec[1])])
        ret = rknn.load_onnx(**load_options)
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

        if args.verify:
            verify(rknn, onnx_path, input_spec, output_spec, args)
            print("verification passed")
    finally:
        rknn.release()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
