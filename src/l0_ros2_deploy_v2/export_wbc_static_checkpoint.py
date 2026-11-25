#!/usr/bin/env python3
"""Export a verified, fused WBC LoRA checkpoint as a static two-input ONNX."""

import argparse
import copy
import hashlib
import json
import shutil
import tempfile
from pathlib import Path

import numpy as np

from convert_onnx_to_rknn_rk3588_wbc_policy import check_model
from export_wbc_lora import (actor_forward, adapter_state, array, checkpoint,
                             normalization_hash, schema_for, statistics, topology)


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def export(args):
    import onnx
    import onnxruntime as ort
    import torch
    from onnx import numpy_helper

    if args.output.exists():
        raise ValueError("output directory must not exist")
    base_hash = digest(args.base_checkpoint)
    base = checkpoint(args.base_checkpoint)["policy"]
    final_file = checkpoint(args.checkpoint)
    final = final_file["policy"]
    adapter = checkpoint(args.adapter)
    if final_file["lora"].get("base_sha256") != base_hash or adapter.get("base_sha256") != base_hash:
        raise ValueError("checkpoint/adapter Base identity does not match the supplied Base")
    if final_file["lora"].get("num_updates") != adapter.get("num_updates"):
        raise ValueError("checkpoint and adapter were saved at different training steps")
    model = onnx.load(str(args.base_onnx))
    onnx.checker.check_model(model)
    linears, norms, bindings, norm_bindings, base_stats = topology(base, model)
    schema = schema_for(linears, norms, base["actor"])
    merged = adapter_state(adapter, base, base_hash, linears, schema)
    actor = final["actor"]
    if set(actor) != set(base["actor"]):
        raise ValueError("checkpoint actor keys differ from Base topology")
    for name, value in merged.items():
        if name.endswith("actor_std"):
            continue
        if not torch.allclose(value, actor[name], atol=2e-6, rtol=1e-5):
            raise ValueError("LoRA does not reconstruct checkpoint actor: " + name)
    final_stats = statistics(final["vecnorms"])
    if normalization_hash(final_stats) != normalization_hash(base_stats):
        raise ValueError("checkpoint normalization differs from Base ONNX")

    result = copy.deepcopy(model)
    replacements = {}
    nodes = {node.name: node for node in result.graph.node}
    for name, (matmul_name, add_name, _) in zip(linears, bindings):
        matmul, add = nodes[matmul_name], nodes[add_name]
        replacements[matmul.input[1]] = array(actor[name + ".weight"]).T.copy()
        bias_input = next(value for value in add.input if value != matmul.output[0])
        replacements[bias_input] = array(actor[name + ".bias"])
    for name, node_name in zip(norms, norm_bindings):
        node = nodes[node_name]
        replacements[node.input[1]] = array(actor[name + ".weight"])
        replacements[node.input[2]] = array(actor[name + ".bias"])
    expected = set(replacements)
    for initializer in result.graph.initializer:
        if initializer.name in replacements:
            updated = numpy_helper.from_array(replacements[initializer.name], initializer.name)
            if tuple(numpy_helper.to_array(initializer).shape) != tuple(replacements[initializer.name].shape):
                raise ValueError("checkpoint tensor shape mismatch: " + initializer.name)
            initializer.CopyFrom(updated)
            expected.remove(initializer.name)
    if expected:
        raise ValueError("Base ONNX is missing actor initializers: " + ", ".join(sorted(expected)))
    onnx.helper.set_model_props(result, {
        "deploy_source_checkpoint_sha256": digest(args.checkpoint),
        "deploy_source_adapter_sha256": digest(args.adapter),
        "deploy_base_checkpoint_sha256": base_hash,
        "deploy_base_onnx_sha256": digest(args.base_onnx),
    })
    onnx.checker.check_model(result)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wbc_static_", dir=args.output.parent) as temporary:
        stage = Path(temporary) / "export"
        stage.mkdir()
        policy = stage / "checkpoint_2000.onnx"
        onnx.save(result, str(policy))
        check_model(policy)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(policy), options, providers=["CPUExecutionProvider"])
        rng = np.random.default_rng(2000)
        maximum = 0.0
        for index in range(33):
            feeds = {key: np.asarray(mean + std * rng.uniform(-0.25, 0.25, len(mean)), np.float32)
                     for key, (mean, std) in final_stats.items()}
            if index == 0:
                feeds = {key: mean.copy() for key, (mean, _) in final_stats.items()}
            expected_action = actor_forward(actor, linears, norms, final_stats,
                                            feeds["policy"], feeds["command_short"])
            actual = session.run(["action"], feeds)[0]
            error = float(np.max(np.abs(actual - expected_action)))
            if not np.isfinite(actual).all() or error > 1e-3:
                raise ValueError(f"ONNX/checkpoint mismatch at sample {index}: {error}")
            maximum = max(maximum, error)
        shutil.copyfile(args.base_yaml, stage / "checkpoint_2000.yaml")
        (stage / "export_manifest.json").write_text(json.dumps({
            "format": "wbc_static_checkpoint_export_v1",
            "checkpoint_sha256": digest(args.checkpoint),
            "adapter_sha256": digest(args.adapter),
            "base_checkpoint_sha256": base_hash,
            "base_onnx_sha256": digest(args.base_onnx),
            "policy_onnx_sha256": digest(policy),
            "policy_yaml_sha256": digest(stage / "checkpoint_2000.yaml"),
            "cpu_samples": 33,
            "max_abs_error": maximum,
        }, indent=2) + "\n", encoding="utf-8")
        stage.rename(args.output)
    print(f"Exported {args.output / 'checkpoint_2000.onnx'}; 33 CPU samples max abs error {maximum:.6g}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--adapter", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--base-onnx", type=Path, required=True)
    parser.add_argument("--base-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    export(parser.parse_args())
