#!/usr/bin/env python3
"""Export native L0 WBC LoRA as one dynamic ONNX, FP16 adapters and RK3588 probes.

Per-action full checkpoints are optional; Base and compact LoRA remain required.
"""
import argparse
import copy
import json
import math
import shutil
import tempfile
from pathlib import Path

import numpy as np
import yaml

from wbc_lora_assets import (ADAPTER_FORMAT, GRAPH_FORMAT, ID_PATTERN, asset_path,
                             fingerprint, graph_feeds, load_adapter, sha256)

DIMS = [(1024, 639), (1024, 1024), (1024, 1024), (1024, 1423),
        (1024, 1024), (1024, 1024), (21, 1024)]
NORMALIZED_LAYERS = (0, 1, 3, 4, 5)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def checkpoint(path):
    # These are the explicitly selected local training delivery checkpoints.
    import torch
    return torch.load(path, map_location="cpu", weights_only=False)


def array(tensor):
    import torch
    if not isinstance(tensor, torch.Tensor) or tensor.dtype != torch.float32 or not torch.isfinite(tensor).all():
        raise ValueError("Expected a finite float32 training tensor")
    return tensor.detach().cpu().numpy().copy()


def statistics(state):
    result = {}
    for name in ("policy", "command_short"):
        count = state[name + ".count"]
        if not np.isfinite(float(count)) or float(count) <= 0:
            raise ValueError("Invalid normalization count")
        mean = state[name + ".sum"] / count
        std = (state[name + ".ssq"] / count - mean.square()).clamp_min(1e-5).sqrt()
        result[name] = (array(mean), array(std))
    return result


def normalization_hash(stats):
    return fingerprint({k: {"mean": v[0].tolist(), "std": v[1].tolist()} for k, v in stats.items()})


def actor_forward(actor, linear_names, norm_names, stats, policy, command):
    import torch
    from torch.nn import functional as F
    p = (torch.from_numpy(policy) - torch.from_numpy(stats["policy"][0])) / torch.from_numpy(stats["policy"][1])
    c = (torch.from_numpy(command) - torch.from_numpy(stats["command_short"][0])) / torch.from_numpy(stats["command_short"][1])
    value, norm_index = torch.cat((p, c)), 0
    for index, name in enumerate(linear_names):
        if index == 3:
            value = torch.cat((p, value))
        value = F.linear(value, actor[name + ".weight"], actor[name + ".bias"])
        if index in NORMALIZED_LAYERS:
            norm = norm_names[norm_index]
            value = F.mish(F.layer_norm(value, (1024,), actor[norm + ".weight"], actor[norm + ".bias"], eps=1e-5))
            norm_index += 1
    return value.detach().numpy()


def topology(base, model):
    from onnx import numpy_helper
    actor = base["actor"]
    linears = [k[:-7] for k, v in actor.items() if k.endswith(".weight") and v.ndim == 2]
    norms = [k[:-7] for k, v in actor.items() if k.endswith(".weight") and v.ndim == 1]
    if [tuple(actor[k + ".weight"].shape) for k in linears] != DIMS or len(norms) != 5:
        raise ValueError("Unsupported native L0 actor topology")
    initializers = {x.name: numpy_helper.to_array(x) for x in model.graph.initializer}
    matmuls = [n for n in model.graph.node if n.op_type == "MatMul"]
    ln_nodes = [n for n in model.graph.node if n.op_type == "LayerNormalization"]
    if len(matmuls) != 7 or len(ln_nodes) != 5 or len([n for n in model.graph.node if n.op_type == "Mish"]) != 5:
        raise ValueError("Expected the native seven-Linear / five-LayerNorm / five-Mish graph")
    bindings, norm_bindings = [], []
    for name in linears:
        weight = array(actor[name + ".weight"]).T
        matches = [n for n in matmuls if n.input[1] in initializers and
                   np.array_equal(initializers[n.input[1]], weight)]
        if len(matches) != 1:
            raise ValueError(f"Cannot uniquely bind ONNX Linear to checkpoint: {name}")
        node = matches[0]
        consumers = [n for n in model.graph.node if node.output[0] in n.input]
        if len(consumers) != 1 or consumers[0].op_type != "Add":
            raise ValueError("Unsupported Linear bias topology")
        bias = consumers[0]
        other = next(x for x in bias.input if x != node.output[0])
        if other not in initializers or not np.array_equal(initializers[other], array(actor[name + ".bias"])):
            raise ValueError("Base ONNX bias differs from checkpoint")
        bindings.append((node.name, bias.name, node.output[0]))
    if any(not n.name for n in matmuls + ln_nodes) or len({n.name for n in model.graph.node}) != len(model.graph.node):
        raise ValueError("Native graph node names must be unique and non-empty")
    for norm_index, (name, node) in enumerate(zip(norms, ln_nodes)):
        bias_name = bindings[NORMALIZED_LAYERS[norm_index]][1]
        bias_node = next(n for n in model.graph.node if n.name == bias_name)
        if node.input[0] != bias_node.output[0]:
            raise ValueError("LayerNorm/Linear connectivity differs from the native actor")
        for index, suffix in ((1, ".weight"), (2, ".bias")):
            if not np.array_equal(initializers[node.input[index]], array(actor[name + suffix])):
                raise ValueError("Base ONNX LayerNorm differs from checkpoint")
        norm_bindings.append(node.name)
    stats = statistics(base["vecnorms"])
    for name, (mean, std) in stats.items():
        sub = next(n for n in model.graph.node if n.op_type == "Sub" and n.input[0] == name)
        div = next(n for n in model.graph.node if n.op_type == "Div" and n.input[0] == sub.output[0])
        if not np.array_equal(initializers[sub.input[1]], mean) or not np.array_equal(initializers[div.input[1]], std):
            raise ValueError("Base ONNX normalization differs from checkpoint")
    return linears, norms, bindings, norm_bindings, stats


def schema_for(linears, norms, actor):
    result = []
    for i, name in enumerate(linears):
        for suffix, short in ((".lora_A", "a"), (".lora_B", "b"), (".bias", "bias")):
            shape = [16, DIMS[i][1]] if short == "a" else [DIMS[i][0], 16] if short == "b" else [DIMS[i][0]]
            result.append({"name": f"lora_{i}_{short}", "actor_key": name + suffix, "shape": shape})
    for i, name in enumerate(norms):
        for suffix, short in ((".weight", "gamma"), (".bias", "beta")):
            result.append({"name": f"norm_{i}_{short}", "actor_key": name + suffix, "shape": list(actor[name + suffix].shape)})
    return result


def residual_nodes(prefix, value, base_output, index):
    from onnx import helper
    a, b = f"lora_{index}_a", f"lora_{index}_b"
    return [
        helper.make_node("Transpose", [a], [prefix + "at"], perm=[1, 0]),
        helper.make_node("Transpose", [b], [prefix + "bt"], perm=[1, 0]),
        helper.make_node("MatMul", [value, prefix + "at"], [prefix + "low"]),
        helper.make_node("MatMul", [prefix + "low", prefix + "bt"], [prefix + "delta"]),
        helper.make_node("Add", [base_output, prefix + "delta"], [prefix + "sum"]),
    ]


def affine_norm_nodes(prefix, input_value, output_value, index, original=None):
    from onnx import helper
    node = copy.deepcopy(original) if original is not None else helper.make_node(
        "LayerNormalization", [input_value, prefix + "ones", prefix + "zeros"], [prefix + "normalized"], epsilon=1e-5)
    node.input[:] = [input_value, prefix + "ones", prefix + "zeros"]
    node.output[0] = prefix + "normalized"
    return [node, helper.make_node("Mul", [node.output[0], f"norm_{index}_gamma"], [prefix + "scaled"]),
            helper.make_node("Add", [prefix + "scaled", f"norm_{index}_beta"], [output_value])]


def dynamic_model(source, schema, bindings, norm_bindings):
    from onnx import helper, numpy_helper
    model = copy.deepcopy(source)
    linear_by_node = {entry[0]: (i, entry) for i, entry in enumerate(bindings)}
    bias_by_node = {entry[1]: (i, entry) for i, entry in enumerate(bindings)}
    norms = {name: i for i, name in enumerate(norm_bindings)}
    nodes, constants = [], []
    for node in model.graph.node:
        if node.name in linear_by_node:
            i, _ = linear_by_node[node.name]
            nodes.append(node)
            nodes.extend(residual_nodes(f"wbc_lora_{i}_", node.input[0], node.output[0], i))
        elif node.name in bias_by_node:
            i, entry = bias_by_node[node.name]
            node.input[:] = [f"wbc_lora_{i}_sum", f"lora_{i}_bias"]
            nodes.append(node)
        elif node.name in norms:
            i = norms[node.name]
            prefix = f"wbc_norm_{i}_"
            constants += [numpy_helper.from_array(np.ones(1024, np.float32), prefix + "ones"),
                          numpy_helper.from_array(np.zeros(1024, np.float32), prefix + "zeros")]
            nodes.extend(affine_norm_nodes(prefix, node.input[0], node.output[0], i, node))
        else:
            nodes.append(node)
    del model.graph.node[:]
    model.graph.node.extend(nodes)
    used = {x for n in nodes for x in n.input}
    kept = [x for x in model.graph.initializer if x.name in used] + constants
    del model.graph.initializer[:]
    model.graph.initializer.extend(kept)
    del model.graph.value_info[:]
    for p in schema:
        model.graph.input.append(helper.make_tensor_value_info(p["name"], 1, p["shape"]))
    return model


def probe_model(base_actor, linears, index, schema):
    from onnx import helper, numpy_helper
    prefix, value = f"probe_{index}_", "x"
    nodes = [helper.make_node("MatMul", [value, "base_weight"], [prefix + "base"])]
    nodes += residual_nodes(prefix, value, prefix + "base", index)
    nodes += [helper.make_node("Add", [prefix + "sum", f"lora_{index}_bias"], [prefix + "linear"])]
    constants = [numpy_helper.from_array(array(base_actor[linears[index] + ".weight"]).T.copy(), "base_weight")]
    names = [f"lora_{index}_{s}" for s in ("a", "b", "bias")]
    if index in NORMALIZED_LAYERS:
        norm_index = NORMALIZED_LAYERS.index(index)
        names += [f"norm_{norm_index}_{s}" for s in ("gamma", "beta")]
        constants += [numpy_helper.from_array(np.ones(1024, np.float32), prefix + "ones"),
                      numpy_helper.from_array(np.zeros(1024, np.float32), prefix + "zeros")]
        nodes += affine_norm_nodes(prefix, prefix + "linear", prefix + "affine", norm_index)
        nodes += [helper.make_node("Mish", [prefix + "affine"], ["action"])]
    else:
        nodes += [helper.make_node("Identity", [prefix + "linear"], ["action"])]
    inputs = [{"name": "x", "shape": [DIMS[index][1]], "dtype": "float32", "role": "observation"}]
    inputs += [{"name": p["name"], "shape": p["shape"], "dtype": "float32", "role": "adapter"}
               for p in schema if p["name"] in names]
    graph = helper.make_graph(nodes, f"WBC LoRA layer {index}",
        [helper.make_tensor_value_info(x["name"], 1, x["shape"]) for x in inputs],
        [helper.make_tensor_value_info("action", 1, [DIMS[index][0]])], constants)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 19)])
    model.ir_version = 9
    return model, inputs


def adapter_state(adapter, base, base_hash, linears, schema):
    import torch
    if adapter.get("format") != "mimic_lite_lora_adapter_v2" or adapter.get("base_sha256") != base_hash:
        raise ValueError("Adapter format/Base checksum mismatch")
    if adapter["rank"] != 16 or adapter["alpha"] != 16 or adapter["layers"] != linears:
        raise ValueError("Adapter layers/rank/alpha mismatch")
    values = adapter["actor"]
    expected = {p["actor_key"] for p in schema}
    extra = set(values) - expected
    if any(not name.endswith("actor_std") for name in extra) or not expected <= set(values):
        raise ValueError("Unexpected/missing actor adapter parameters")
    for p in schema:
        if list(values[p["actor_key"]].shape) != p["shape"]:
            raise ValueError("Adapter tensor shape mismatch")
        array(values[p["actor_key"]])
    if normalization_hash(statistics(adapter["vecnorms"])) != normalization_hash(statistics(base["vecnorms"])):
        raise ValueError("Adapter normalization differs from the shared Base graph")
    merged = {k: v.detach().cpu().clone() for k, v in base["actor"].items()}
    for p in schema:
        if not p["actor_key"].endswith((".lora_A", ".lora_B")):
            merged[p["actor_key"]] = values[p["actor_key"]].detach().cpu().clone()
    for name in linears:
        merged[name + ".weight"] += values[name + ".lora_B"] @ values[name + ".lora_A"]
    return merged


def export_adapter(directory, action, adapter, graph, parameter_source="lora"):
    payload, tensors = bytearray(), []
    for p in graph["parameters"]:
        payload.extend(b"\0" * ((-len(payload)) % 64))
        value = array(adapter["actor"][p["actor_key"]])
        with np.errstate(over="ignore"):
            value = value.astype("<f2")
        if not np.isfinite(value).all():
            raise ValueError("Adapter value is not representable in FP16")
        data = value.tobytes(order="C")
        tensors.append(dict(p, offset_bytes=len(payload), size_bytes=len(data)))
        payload.extend(data)
    directory.mkdir(parents=True)
    (directory / "weights.bin").write_bytes(payload)
    manifest = {"format": ADAPTER_FORMAT, "action_id": action, "rank": 16, "alpha": 16,
                "parameter_source": parameter_source,
                **{k: graph[k] for k in ("base_checkpoint_sha256", "normalization_sha256", "parameter_schema_sha256")},
                "weights": {"path": "weights.bin", "dtype": "float16-le", "size_bytes": len(payload),
                            "sha256": sha256(directory / "weights.bin")}, "tensors": tensors}
    write_json(directory / "manifest.json", manifest)


def action_config(config, reference_meta, action):
    from prepare_wbc_deploy import joint_values, HISTORY_STEPS, FUTURE_STEPS
    names = config["policy_joint_names"]
    if names != reference_meta["joint_names"].split(",") or names != config["joint_names_simulation"] or len(names) != 21:
        raise ValueError("Reference/policy joint order mismatch")
    robot = {"policy_type": "rl_wbc", "policy_name": "base_lora", "policy_frequency": 50,
             "lowlevel_frequency": 500, "num_actions": 21, "joint_order_mj": names, "joint_order_isaac": names,
             "default_joint_angles": joint_values(config["default_joint_pos"], names, "default_joint_pos"),
             "kps": joint_values(config["joint_kp"], names, "joint_kp"),
             "kds": joint_values(config["joint_kd"], names, "joint_kd"),
             "action_scale": config["action_scale"], "clip_actions": config.get("clip_actions", 1000000.0)}
    if len(robot["action_scale"]) != 21 or not np.isfinite(robot["action_scale"]).all() or min(robot["action_scale"]) <= 0:
        raise ValueError("Invalid action scales")
    if min(robot["kps"] + robot["kds"]) < 0 or not math.isfinite(robot["clip_actions"]) or robot["clip_actions"] <= 0:
        raise ValueError("Invalid gains/clipping")
    return {"RobotCfg": robot, "WbcCfg": {"motion_id": action, "control_dt": 0.02, "start_hold_sec": 1.0,
        "reference_num_frames": int(reference_meta["num_frames"]),
        "reference_root_body_index": reference_meta["body_names"].split(",").index("base_link"),
        "history_steps": HISTORY_STEPS, "future_steps": FUTURE_STEPS, "previous_action_steps": 3,
        "policy_input_name": "policy", "command_input_name": "command_short", "output_name": "action"}}


def fp16_initializers(model):
    """Store native float initializers in FP16 while retaining the FP32 graph ABI."""
    import onnx
    from onnx import helper, numpy_helper

    model = copy.deepcopy(model)
    names = {x.name for x in model.graph.initializer}
    names.update(x for n in model.graph.node for x in list(n.input) + list(n.output))
    prefix = "wbc_fp16_storage_"
    if any(name.startswith(prefix) for name in names):
        raise ValueError("Model already contains FP16 storage casts")
    if any(a.type in (onnx.AttributeProto.GRAPH, onnx.AttributeProto.GRAPHS)
           for node in model.graph.node for a in node.attribute):
        raise ValueError("FP16 storage conversion does not support subgraphs")
    casts = []
    for index, tensor in enumerate(model.graph.initializer):
        if tensor.data_location == onnx.TensorProto.EXTERNAL:
            raise ValueError("External tensor data is not supported")
        if tensor.data_type != onnx.TensorProto.FLOAT:
            continue
        original_name = tensor.name
        values = numpy_helper.to_array(tensor)
        with np.errstate(over="ignore"):
            reduced = values.astype(np.float16)
        if not np.isfinite(values).all() or not np.isfinite(reduced).all():
            raise ValueError(f"Initializer cannot be represented in FP16: {original_name}")
        storage_name = prefix + str(index)
        tensor.CopyFrom(numpy_helper.from_array(reduced, storage_name))
        casts.append(helper.make_node("Cast", [storage_name], [original_name],
                                      name=storage_name + "_cast", to=onnx.TensorProto.FLOAT))
    nodes = casts + list(model.graph.node)
    del model.graph.node[:]
    model.graph.node.extend(nodes)
    onnx.checker.check_model(model, full_check=True)
    return model


def finalize_fp16(stage, graph, report):
    """Convert and verify the final delivery against the existing FP32 oracles."""
    import onnx
    import onnxruntime as ort
    from convert_onnx_to_rknn_rk3588_wbc_reference import check_reference_model
    from wbc_lora_assets import check_graph, read_json

    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    final_report = {"initializer_dtype": "float16", "application_dtype": "float32",
                    "policy_max_abs_error": 0.05, "reference_max_abs_error": 0.02,
                    "graphs": [], "references": []}
    for manifest_name, current in [("base_manifest.json", graph)] + [
            (name, read_json(stage / name)) for name in graph["probes"]]:
        path = asset_path(stage, current["model"]["onnx"])
        onnx.save(fp16_initializers(onnx.load(str(path), load_external_data=False)), str(path))
        current["model"]["sha256"] = sha256(path)
        current["initializer_dtype"] = "float16"
        write_json(stage / manifest_name, current)
        check_graph(path, stage / manifest_name)
        session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        errors, first_feed, first_output = [], None, None
        with np.load(stage / current["verification_data"]) as observations, \
                np.load(stage / current["verification_expected"]) as expected:
            for action in current["actions"]:
                action_id = action["action_id"]
                packed = load_adapter(asset_path(stage, action["adapter_manifest"]), current, action_id)
                maximum = 0.0
                for i, target in enumerate(expected[action_id]):
                    feed = graph_feeds(current, {key: observations[key][i] for key in observations.files}, packed)
                    actual = session.run(["action"], feed)[0]
                    error = float(np.max(np.abs(actual - target)))
                    if not np.isfinite(actual).all() or not np.isfinite(error) or error > 0.05:
                        raise ValueError(f"Final FP16 policy/probe tolerance exceeded: {manifest_name}/{action_id}: {error}")
                    maximum = max(maximum, error)
                    if first_feed is None:
                        first_feed, first_output = feed, actual.copy()
                errors.append({"action_id": action_id, "max_abs": maximum})
        np.testing.assert_array_equal(session.run(["action"], first_feed)[0], first_output)
        final_report["graphs"].append({"manifest": manifest_name, "actions": errors,
                                       "switch_repeatability": "passed"})
        print(f"FP16 verified: {manifest_name}, {len(errors)} actions", flush=True)
    for action in graph["actions"]:
        path = asset_path(stage, action["reference_onnx"])
        original = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        onnx.save(fp16_initializers(onnx.load(str(path), load_external_data=False)), str(path))
        _, _, meta = check_reference_model(path)
        converted = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        count, maximum = int(meta["num_frames"]), 0.0
        for frame in [-1] + list(range(count)) + [count]:
            feed = {"time_step": np.asarray([[frame]], dtype=np.float32)}
            for actual, target in zip(converted.run(None, feed), original.run(None, feed)):
                error = float(np.max(np.abs(actual - target)))
                if not np.isfinite(actual).all() or not np.isfinite(error) or error > 0.02:
                    raise ValueError(f"Final FP16 reference tolerance exceeded: {action['action_id']}/{frame}: {error}")
                maximum = max(maximum, error)
        action["reference_onnx_sha256"] = sha256(path)
        final_report["references"].append({"action_id": action["action_id"], "frames_checked": count + 2,
                                            "max_abs": maximum})
    # Probe catalogs share the same action/reference identities as the policy.
    for name in graph["probes"]:
        probe = read_json(stage / name)
        probe["actions"] = copy.deepcopy(graph["actions"])
        write_json(stage / name, probe)
    report["fp16_delivery"] = final_report


def export_delivery(source, output, actions, samples=33, base_actions=(), fp16=False):
    import onnx
    import onnxruntime as ort
    import torch
    from convert_onnx_to_rknn_rk3588_wbc_reference import check_reference_model
    source, output = Path(source).resolve(), Path(output).resolve()
    index = json.loads((source / "emotion_index.json").read_text(encoding="utf-8"))
    if actions is None:
        actions = [entry["motion"] for entry in index]
    if not actions or len(set(actions)) != len(actions) or any(not ID_PATTERN.fullmatch(x) for x in actions):
        raise ValueError("Select unique valid action IDs")
    if len(set(base_actions)) != len(base_actions) or not set(base_actions) <= set(actions):
        raise ValueError("--base-actions must be a unique subset of --actions")
    base_actions = set(base_actions)
    if samples < 20:
        raise ValueError("At least 20 verification samples are required")
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("Output must be absent or an empty directory")
    entries = {x["motion"]: x for x in index}
    if len(entries) != len(index):
        raise ValueError("Duplicate source action IDs")
    if any(x not in entries for x in actions):
        raise ValueError("Unknown source action ID")
    base_path, onnx_path = source / "base/base_ckpt.pt", source / "base/policy.onnx"
    base_hash = sha256(base_path)
    base = checkpoint(base_path)["policy"]
    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)
    if {x.name: [d.dim_value for d in x.type.tensor_type.shape.dim] for x in model.graph.input} != {
            "policy": [399], "command_short": [240]}:
        raise ValueError("Unexpected Base ONNX interface")
    linears, norms, bindings, norm_bindings, stats = topology(base, model)
    config = yaml.safe_load((source / "base/policy.yaml").read_text(encoding="utf-8"))
    schema = schema_for(linears, norms, base["actor"])
    # Bias and LayerNorm inputs are complete parameters, never zero increments.
    base_adapter = {"actor": {
        p["actor_key"]: torch.zeros(p["shape"], dtype=torch.float32)
        if p["actor_key"].endswith((".lora_A", ".lora_B")) else base["actor"][p["actor_key"]]
        for p in schema}}
    base_parameters = {p["name"]: array(base_adapter["actor"][p["actor_key"]]) for p in schema}
    graph = {"format": GRAPH_FORMAT, "kind": "policy", "rank": 16, "alpha": 16,
             "base_checkpoint_sha256": base_hash, "normalization_sha256": normalization_hash(stats),
             "parameter_schema_sha256": fingerprint(schema), "parameters": schema,
             "target_precision": "float16", "application_dtype": "float32",
             "source_onnx_sha256": sha256(onnx_path),
             "inputs": [{"name": x.name, "shape": [d.dim_value for d in x.type.tensor_type.shape.dim],
                         "dtype": "float32", "role": "observation"} for x in model.graph.input] +
                       [{"name": p["name"], "shape": p["shape"], "dtype": "float32", "role": "adapter"} for p in schema],
             "outputs": [{"name": "action", "shape": [21], "dtype": "float32"}],
             "verification_data": "verification_inputs.npz", "actions": [], "probes": []}
    rng = np.random.default_rng(0)
    observations = {k: np.asarray(mean + std * rng.uniform(-0.25, 0.25, (samples, len(mean))), np.float32)
                    for k, (mean, std) in stats.items()}
    for k, (mean, _) in stats.items():
        observations[k][0] = mean
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wbc_lora_", dir=output.parent) as temporary:
        stage = Path(temporary)
        dynamic = dynamic_model(model, schema, bindings, norm_bindings)
        onnx.checker.check_model(dynamic)
        onnx.save(dynamic, str(stage / "base_lora.onnx"))
        graph["model"] = {"onnx": "base_lora.onnx", "sha256": sha256(stage / "base_lora.onnx"), "rknn": "base_lora.rknn", "rknn_record": "base_lora.rknn.json"}
        np.savez(stage / graph["verification_data"], **observations)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(stage / "base_lora.onnx"), options, providers=["CPUExecutionProvider"])
        original_session = ort.InferenceSession(str(onnx_path), options, providers=["CPUExecutionProvider"])
        report = {"samples_per_action": samples, "inputs": "synthetic normalized-range conversion samples, not robot rollout",
                  "base_checkpoint_sha256": base_hash, "base_actions": sorted(base_actions), "actions": [], "probes": [],
                  "rknn_conversion": "not_run", "rk3588_gate": "not_run"}
        adapters, actors, expected_actions = {}, {}, {}
        for action in actions:
            entry = entries[action]
            parameter_source = "base" if action in base_actions else "lora"
            checkpoint_verification = "not_applicable"
            expected_source = "base/policy.onnx"
            if action in base_actions:
                # Explicit opt-in only. Missing/bad tuned assets never imply Base mode.
                adapter, final = base_adapter, base
            else:
                adapter_path = asset_path(source, entry["adapter"])
                if sha256(adapter_path) != entry["adapter_sha256"]:
                    raise ValueError(f"Source adapter checksum mismatch: {adapter_path}")
                adapter = checkpoint(adapter_path)
                merged = adapter_state(adapter, base, base_hash, linears, schema)
                # Use an FP32 merged actor as the oracle for the dynamic graph
                # and FP16 payload, without retaining a full checkpoint per action.
                final = {"actor": merged}
                expected_source = "base_plus_lora_actor"
                checkpoint_verification = "not_provided"
                if entry.get("checkpoint") is not None:
                    final_path = asset_path(source, entry["checkpoint"])
                    checkpoint_verification = "missing"
                    if final_path.exists():
                        # Existing optional checkpoints must still pass all checks.
                        if sha256(final_path) != entry.get("checkpoint_sha256"):
                            raise ValueError(f"Source checkpoint checksum mismatch: {final_path}")
                        final = checkpoint(final_path)["policy"]
                        for key, value in merged.items():
                            if key.endswith("actor_std"):
                                continue
                            if key not in final["actor"] or not torch.allclose(value, final["actor"][key], atol=2e-6, rtol=1e-5):
                                raise ValueError(f"Adapter does not reconstruct final actor: {key}")
                        if normalization_hash(statistics(final["vecnorms"])) != graph["normalization_sha256"]:
                            raise ValueError("Final checkpoint normalization mismatch")
                        expected_source = "final_checkpoint_actor"
                        checkpoint_verification = "passed"
            adapter_dir = stage / "adapters" / action
            export_adapter(adapter_dir, action, adapter, graph, parameter_source)
            parameters = {p["name"]: array(adapter["actor"][p["actor_key"]]) for p in schema}
            packed = load_adapter(adapter_dir / "manifest.json", graph, expected_action_id=action)
            if action in base_actions:
                for p in schema:
                    np.testing.assert_array_equal(packed[p["name"]], base_parameters[p["name"]].astype("<f2").astype(np.float32))
            maximum, packed_maximum, expected_rows = 0.0, 0.0, []
            for i in range(samples):
                obs = {k: v[i] for k, v in observations.items()}
                expected = actor_forward(final["actor"], linears, norms, stats, obs["policy"], obs["command_short"])
                if action in base_actions:
                    original = original_session.run(["action"], obs)[0]
                    np.testing.assert_allclose(expected, original, atol=2e-5, rtol=1e-5)
                    expected = original  # Base expectations come from the original Base ONNX.
                expected_rows.append(expected.copy())
                actual = session.run(["action"], graph_feeds(graph, obs, parameters))[0]
                reduced = session.run(["action"], graph_feeds(graph, obs, packed))[0]
                np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=1e-5)
                if not np.isfinite(reduced).all() or np.max(np.abs(reduced - expected)) > 0.05:
                    raise ValueError("FP16 adapter payload exceeded policy precision tolerance")
                maximum = max(maximum, float(np.max(np.abs(actual - expected))))
                packed_maximum = max(packed_maximum, float(np.max(np.abs(reduced - expected))))
            reference = asset_path(source, entry["motion_onnx"])
            _, _, meta = check_reference_model(reference)
            action_dir = stage / "actions" / action
            action_dir.mkdir(parents=True)
            shutil.copyfile(reference, action_dir / "reference_motion.onnx")
            cfg = action_config(config, meta, action)
            (action_dir / "rl_config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
            graph["actions"].append({"action_id": action, "parameter_source": parameter_source,
                "adapter_manifest": f"adapters/{action}/manifest.json",
                "config": f"actions/{action}/rl_config.yaml", "config_sha256": sha256(action_dir / "rl_config.yaml"),
                "reference_onnx": f"actions/{action}/reference_motion.onnx", "reference_onnx_sha256": sha256(reference),
                "reference_rknn": f"actions/{action}/reference_motion.rknn",
                "reference_rknn_record": f"actions/{action}/reference_motion.rknn.json"})
            report["actions"].append({"action_id": action, "parameter_source": parameter_source,
                "expected_source": expected_source, "full_checkpoint_verification": checkpoint_verification,
                "fp32_max_abs": maximum, "fp16_payload_max_abs": packed_maximum})
            adapters[action], actors[action] = packed, final["actor"]
            expected_actions[action] = np.stack(expected_rows)
            print(f"Exported {len(expected_actions)}/{len(actions)}: {action}", flush=True)
        np.savez(stage / "verification_expected.npz", **expected_actions)
        graph["verification_expected"] = "verification_expected.npz"
        graph["verification_data_sha256"] = sha256(stage / graph["verification_data"])
        graph["verification_expected_sha256"] = sha256(stage / graph["verification_expected"])
        base_max = 0.0
        for i in range(samples):
            obs = {k: v[i] for k, v in observations.items()}
            actual = session.run(["action"], graph_feeds(graph, obs, base_parameters))[0]
            expected = original_session.run(["action"], obs)[0]
            np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=1e-5)
            base_max = max(base_max, float(np.max(np.abs(actual - expected))))
        report["zero_adapter_base_max_abs"] = base_max
        obs = {k: v[1] for k, v in observations.items()}
        first = session.run(["action"], graph_feeds(graph, obs, adapters[actions[0]]))[0]
        for action in actions[1:]:
            session.run(["action"], graph_feeds(graph, obs, adapters[action]))
        again = session.run(["action"], graph_feeds(graph, obs, adapters[actions[0]]))[0]
        if not np.array_equal(first, again):
            raise ValueError("A-to-B-to-A inference is not repeatable")
        report["a_to_b_to_a"] = "passed" if len(actions) > 1 else "only_one_adapter_selected"
        report["base_switches"] = []
        for base_action in sorted(base_actions):
            for tuned_action in (x for x in actions if x not in base_actions):
                for first_action, middle_action in ((tuned_action, base_action), (base_action, tuned_action)):
                    first = session.run(["action"], graph_feeds(graph, obs, adapters[first_action]))[0]
                    session.run(["action"], graph_feeds(graph, obs, adapters[middle_action]))
                    again = session.run(["action"], graph_feeds(graph, obs, adapters[first_action]))[0]
                    np.testing.assert_array_equal(first, again)
                    report["base_switches"].append({"actions": [first_action, middle_action, first_action],
                                                   "cpu_same_input": "passed", "rk3588_resident": "not_run"})
        for label, index in (("encoder", 0), ("decoder", 3), ("output", 6)):
            probe, inputs = probe_model(base["actor"], linears, index, schema)
            onnx.checker.check_model(probe)
            filename = f"probe_{label}.onnx"
            onnx.save(probe, str(stage / filename))
            probe_graph = copy.deepcopy(graph)
            probe_graph.update(kind="probe", inputs=inputs,
                outputs=[{"name": "action", "shape": [DIMS[index][0]], "dtype": "float32"}],
                model={"onnx": filename, "sha256": sha256(stage / filename), "rknn": f"probe_{label}.rknn",
                       "rknn_record": f"probe_{label}.rknn.json"},
                verification_data=f"probe_{label}_inputs.npz", probes=[])
            probe_inputs = rng.uniform(-1, 1, (samples, DIMS[index][1])).astype(np.float32)
            np.savez(stage / probe_graph["verification_data"], x=probe_inputs)
            probe_session = ort.InferenceSession(str(stage / filename), options, providers=["CPUExecutionProvider"])
            probe_errors, probe_expected = [], {}
            from torch.nn import functional as F
            for action in actions:
                maximum, expected_rows = 0.0, []
                for value in probe_inputs:
                    actor = actors[action]
                    expected = F.linear(torch.from_numpy(value), actor[linears[index] + ".weight"],
                                        actor[linears[index] + ".bias"])
                    if index in NORMALIZED_LAYERS:
                        norm = norms[NORMALIZED_LAYERS.index(index)]
                        expected = F.mish(F.layer_norm(expected, (1024,), actor[norm + ".weight"],
                                                      actor[norm + ".bias"], eps=1e-5))
                    expected_rows.append(expected.detach().numpy().copy())
                    actual = probe_session.run(["action"], graph_feeds(probe_graph, {"x": value}, adapters[action]))[0]
                    error = float(np.max(np.abs(actual - expected.detach().numpy())))
                    if not np.isfinite(actual).all() or error > 0.05:
                        raise ValueError(f"FP16 adapter probe verification failed: {label}/{action}")
                    maximum = max(maximum, error)
                probe_errors.append({"action_id": action, "fp16_payload_max_abs": maximum})
                probe_expected[action] = np.stack(expected_rows)
            probe_graph["verification_expected"] = f"probe_{label}_expected.npz"
            np.savez(stage / probe_graph["verification_expected"], **probe_expected)
            probe_graph["verification_data_sha256"] = sha256(stage / probe_graph["verification_data"])
            probe_graph["verification_expected_sha256"] = sha256(stage / probe_graph["verification_expected"])
            write_json(stage / f"probe_{label}_manifest.json", probe_graph)
            graph["probes"].append(f"probe_{label}_manifest.json")
            report["probes"].append({"name": label, "linear_index": index, "onnx_checker": "passed",
                                     "cpu_verification": probe_errors})
        if fp16:
            finalize_fp16(stage, graph, report)
        write_json(stage / "base_manifest.json", graph)
        write_json(stage / "validation_report.json", report)
        # Hash only the completed deployment assets, never the source checkpoints.
        checksums = [f"{sha256(path)}  {path.relative_to(stage).as_posix()}\n"
                     for path in sorted(stage.rglob("*")) if path.is_file()]
        (stage / "ASSETS.sha256").write_text("".join(checksums), encoding="utf-8")
        if output.exists():
            output.rmdir()
        stage.rename(output)
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--actions", nargs="+")
    selection.add_argument("--all-actions", action="store_true", help="export every action in emotion_index.json")
    parser.add_argument("--base-actions", nargs="+", default=[], help="subset of --actions using complete Base parameters")
    parser.add_argument("--samples", type=int, default=33)
    parser.add_argument("--fp16", action="store_true", help="also convert and verify Base/reference/probe FP16 storage")
    args = parser.parse_args()
    export_delivery(args.source, args.output, args.actions, args.samples, args.base_actions, args.fp16)


if __name__ == "__main__":
    main()
