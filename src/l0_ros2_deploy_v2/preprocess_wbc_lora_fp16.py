#!/usr/bin/env python3
"""Stage an FP16 WBC delivery for the existing RK3588 build-only converters.

Never changes the delivery, loads checkpoints, runs inference or compiles RKNN.
The shared policy and adapters are copied byte-for-byte. Reference table/Cast
paths are materialized as FP32 tables with six direct Gather outputs, preserving
the delivered FP16 values and the original frame-index calculation.
"""

import argparse
import copy
import importlib.metadata
import json
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path

import numpy as np
import yaml

from convert_onnx_to_rknn_rk3588_wbc_reference import (
    REFERENCE_OUTPUTS, check_reference_model, pack_reference_gathers,
)
from wbc_lora_assets import ID_PATTERN, asset_path, check_graph, load_adapter, read_json, sha256


def load_embedded_model(path):
    import onnx

    model = onnx.load(str(path), load_external_data=False)
    if any(t.data_location == onnx.TensorProto.EXTERNAL for t in model.graph.initializer):
        raise ValueError(f"External ONNX tensor data is not supported: {path}")
    if any(a.type in (onnx.AttributeProto.GRAPH, onnx.AttributeProto.GRAPHS)
           for n in model.graph.node for a in n.attribute):
        raise ValueError(f"Subgraphs are not supported by this preprocessing contract: {path}")
    onnx.checker.check_model(model, full_check=True)
    return model


def check_fp16_policy(path, graph):
    """Check the fixed Base matrices without evaluating the policy."""
    import onnx
    from onnx import numpy_helper

    model = load_embedded_model(path)
    shapes = {p["name"]: p["shape"] for p in graph["parameters"]}
    expected = sorted((shapes[f"lora_{i}_a"][1], shapes[f"lora_{i}_b"][0]) for i in range(7))
    matrices = []
    for tensor in model.graph.initializer:
        if tensor.data_type not in (onnx.TensorProto.FLOAT, onnx.TensorProto.FLOAT16):
            continue
        values = numpy_helper.to_array(tensor)
        if not np.isfinite(values).all():
            raise ValueError(f"Non-finite policy initializer: {tensor.name}")
        if len(tensor.dims) == 2:
            if tensor.data_type != onnx.TensorProto.FLOAT16:
                raise ValueError(f"Expected an FP16 Base matrix: {tensor.name}; finish training-side export first")
            matrices.append(tuple(tensor.dims))
    if sorted(matrices) != expected:
        raise ValueError("Expected the seven native FP16 Base matrix initializers; optimized/fused graphs are unsupported")


def normalize_reference(path):
    """Only rewrite constant-table -> Gather -> Cast/Identity output paths."""
    model = load_embedded_model(path)
    _, _, meta = check_reference_model(path)
    return normalize_reference_graph(model, meta), meta


def normalize_reference_graph(model, meta):
    import onnx
    from onnx import helper, numpy_helper

    count = int(meta["num_frames"])
    nodes = list(model.graph.node)
    producers = {name: (i, node) for i, node in enumerate(nodes) for name in node.output if name}
    tensors = {t.name: t for t in model.graph.initializer}
    allowed_types = {onnx.TensorProto.FLOAT: np.float32, onnx.TensorProto.FLOAT16: np.float16}

    def unwrap(value):
        casts, visited = [], set()
        while value in producers:
            index, node = producers[value]
            if node.op_type not in ("Identity", "Cast"):
                break
            if index in visited or node.domain not in ("", "ai.onnx") or \
                    len(node.input) != 1 or len(node.output) != 1:
                raise ValueError(f"Unsupported or cyclic reference wrapper: {value}")
            visited.add(index)
            attrs = {a.name: helper.get_attribute_value(a) for a in node.attribute}
            if node.op_type == "Identity":
                if attrs:
                    raise ValueError("Unexpected reference Identity attributes")
            else:
                if set(attrs) != {"to"} or attrs["to"] not in allowed_types:
                    raise ValueError("Reference table/output Cast must only convert FP16/FP32")
                casts.append(allowed_types[attrs["to"]])
            value = node.input[0]
        return value, list(reversed(casts))

    tables, gather_indices, frame_index = {}, set(), None
    for name, shape in REFERENCE_OUTPUTS.items():
        value, output_casts = unwrap(name)
        if value not in producers:
            raise ValueError(f"{name}: expected a Gather producer")
        index, node = producers[value]
        attrs = {a.name: helper.get_attribute_value(a) for a in node.attribute}
        if node.domain not in ("", "ai.onnx") or node.op_type != "Gather" or \
                len(node.input) != 2 or len(node.output) != 1 or \
                set(attrs) - {"axis"} or attrs.get("axis", 0) != 0:
            raise ValueError(f"{name}: expected one axis-0 table Gather")
        gather_indices.add(index)
        if frame_index is None:
            frame_index = node.input[1]
        if frame_index != node.input[1]:
            raise ValueError("All reference outputs must share the same frame-index tensor")
        table_name, input_casts = unwrap(node.input[0])
        if table_name not in tensors or tensors[table_name].data_type != onnx.TensorProto.FLOAT16:
            raise ValueError(f"{name}: expected an embedded FP16 reference table")
        table = numpy_helper.to_array(tensors[table_name])
        if table.shape != (count,) + shape[1:] or not np.isfinite(table).all():
            raise ValueError(f"{name}: invalid reference table shape or value")
        # Float Cast commutes with Gather. Materialize only the declared data
        # casts; leave all computation of the frame index unchanged.
        for dtype in input_casts + output_casts:
            table = table.astype(dtype)
        if table.dtype != np.float32:
            raise ValueError(f"{name}: the table/output path does not produce the required FP32 interface")
        tables[name] = table
    if len(gather_indices) != 6 or sum(n.op_type == "Gather" for n in nodes) != 6:
        raise ValueError("Expected six distinct native reference table Gather branches")

    # Keep the exact original subgraph feeding the common frame index.
    retained_nodes, retained_values, pending = set(), set(), [frame_index]
    graph_inputs = {v.name for v in model.graph.input}
    while pending:
        value = pending.pop()
        if not value or value in retained_values:
            continue
        retained_values.add(value)
        if value in producers:
            index, node = producers[value]
            if index in gather_indices:
                raise ValueError("Reference frame index must not depend on a trajectory Gather")
            retained_nodes.add(index)
            pending.extend(node.input)
        elif value not in tensors and value not in graph_inputs:
            raise ValueError(f"Unresolved reference frame-index input: {value}")

    prefix = "wbc_preprocess_reference_"
    names = set(tensors) | set(producers) | graph_inputs
    if any(name.startswith(prefix) for name in names):
        raise ValueError("Reference already uses preprocessing names; supply the original FP16 delivery")
    retained = [node for i, node in enumerate(nodes) if i in retained_nodes]
    initializers = [t for t in model.graph.initializer if t.name in retained_values]
    for name in REFERENCE_OUTPUTS:
        table_name = prefix + name + "_table"
        initializers.append(numpy_helper.from_array(tables[name], table_name))
        retained.append(helper.make_node("Gather", [table_name, frame_index], [name],
                                         axis=0, name=prefix + name + "_gather"))
    del model.graph.node[:]
    model.graph.node.extend(retained)
    del model.graph.initializer[:]
    model.graph.initializer.extend(initializers)
    del model.graph.value_info[:]
    onnx.checker.check_model(model, full_check=True)
    # Exercise the existing converter's graph rewrite, without RKNN or inference.
    packed = copy.deepcopy(model)
    pack_reference_gathers(packed)
    onnx.checker.check_model(packed, full_check=True)
    return model


def prepare(source, output, dry_run=False):
    import onnx

    source, output = source.resolve(), output.resolve()
    if not source.is_dir() or output.exists() or output.is_symlink():
        raise ValueError("Source must be a directory and output must not exist")
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError("Source and output must be separate, non-nested directories")
    manifest_path = source / "base_manifest.json"
    manifest_hash = sha256(manifest_path)
    graph = read_json(manifest_path)
    model_path = asset_path(source, graph["model"]["onnx"])
    graph = check_graph(model_path, manifest_path)
    if graph["kind"] != "policy":
        raise ValueError("Preprocessing requires a full shared policy bundle, not a probe")
    check_fp16_policy(model_path, graph)
    actions = graph.get("actions", [])
    ids = [a["action_id"] for a in actions]
    if not ids or len(ids) != len(set(ids)) or any(not ID_PATTERN.fullmatch(x) for x in ids):
        raise ValueError("Expected unique, valid action IDs")

    input_hashes = {"base_manifest.json": manifest_hash}
    output_hashes, reserved, references = {}, set(), []

    def reserve(relative):
        path = Path(relative)
        if not relative or path.is_absolute() or ".." in path.parts or path == Path("."):
            raise ValueError(f"Invalid bundle path: {relative}")
        if any(path == old or path in old.parents or old in path.parents for old in reserved):
            raise ValueError(f"Overlapping bundle destinations: {relative}")
        reserved.add(path)

    reserve("base_manifest.json")
    reserve("PREPROCESSING.json")
    for model in [graph["model"]] + [
            {"rknn": a["reference_rknn"], "rknn_record": a["reference_rknn_record"]} for a in actions]:
        if not model["rknn"].endswith(".rknn") or model["rknn_record"] != model["rknn"] + ".json":
            raise ValueError("Expected adjacent RKNN model/conversion record paths")
        reserve(model["rknn"])
        reserve(model["rknn_record"])

    with ExitStack() as stack:
        stage = None
        if not dry_run:
            output.parent.mkdir(parents=True, exist_ok=True)
            temporary = stack.enter_context(tempfile.TemporaryDirectory(prefix=".wbc_preprocess_", dir=output.parent))
            stage = Path(temporary) / "bundle"
            stage.mkdir()

        def include(relative, expected=None, copy_file=True):
            reserve(relative)
            path = asset_path(source, relative)
            digest = sha256(path)
            if expected is not None and digest != expected:
                raise ValueError(f"Delivery checksum mismatch: {relative}; do not regenerate stale hashes on the board")
            input_hashes[relative] = digest
            if copy_file:
                output_hashes[relative] = digest
                if stage:
                    import shutil
                    destination = stage / relative
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, destination)
                    if sha256(destination) != digest:
                        raise ValueError(f"Delivery changed while copying: {relative}")
            return path

        include(graph["model"]["onnx"], graph["model"]["sha256"])
        for action in actions:
            action_id = action["action_id"]
            adapter_path = include(action["adapter_manifest"])
            load_adapter(adapter_path, graph, action_id)
            adapter = read_json(adapter_path)
            asset_path(adapter_path.parent, adapter["weights"]["path"])
            weights_relative = str(Path(action["adapter_manifest"]).parent / adapter["weights"]["path"])
            include(weights_relative, adapter["weights"]["sha256"])
            config_path = include(action["config"], action["config_sha256"])
            reference = include(action["reference_onnx"], action["reference_onnx_sha256"], copy_file=False)
            normalized, meta = normalize_reference(reference)
            config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            robot, wbc = config["RobotCfg"], config["WbcCfg"]
            joints = meta["joint_names"].split(",")
            if wbc["motion_id"] != action_id or robot["policy_type"] != "rl_wbc" or \
                    robot["joint_order_mj"] != joints or robot["joint_order_isaac"] != joints or \
                    robot["num_actions"] != 21 or robot["policy_frequency"] != 50 or \
                    wbc["reference_num_frames"] != int(meta["num_frames"]) or \
                    wbc["reference_root_body_index"] != meta["body_names"].split(",").index("base_link"):
                raise ValueError(f"Action/config/reference contract mismatch: {action_id}")
            record = {"action_id": action_id, "reference": action["reference_onnx"],
                      "source_sha256": action["reference_onnx_sha256"],
                      "source_table_dtype": "float16", "prepared_table_dtype": "float32",
                      "frame_index_subgraph": "preserved", "num_frames": int(meta["num_frames"])}
            if stage:
                destination = stage / action["reference_onnx"]
                destination.parent.mkdir(parents=True, exist_ok=True)
                onnx.save_model(normalized, str(destination), save_as_external_data=False)
                check_reference_model(destination)
                prepared_hash = sha256(destination)
                output_hashes[action["reference_onnx"]] = prepared_hash
                action["reference_onnx_sha256"] = prepared_hash
                record["prepared_sha256"] = prepared_hash
            references.append(record)

        # Diagnostics belong to the training archive, not the build-only bundle.
        graph["probes"] = []
        for key in ("verification_data", "verification_expected", "verification_data_sha256",
                    "verification_expected_sha256"):
            graph.pop(key, None)
        for relative, digest in input_hashes.items():
            if sha256(asset_path(source, relative)) != digest:
                raise ValueError(f"Delivery changed during preprocessing: {relative}")
        report = {"format": "wbc_lora_preprocess_v1", "source_manifest_sha256": manifest_hash,
                  "source_directory": str(source), "prepared_directory": str(output),
                  "preprocessor_sha256": sha256(Path(__file__).resolve()),
                  "onnx_version": importlib.metadata.version("onnx"),
                  "numpy_version": np.__version__, "numerical_verification": "not_run",
                  "source_files": input_hashes, "references": references, "dry_run": dry_run}
        if stage:
            (stage / "base_manifest.json").write_text(json.dumps(graph, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            check_graph(stage / graph["model"]["onnx"], stage / "base_manifest.json")
            output_hashes["base_manifest.json"] = sha256(stage / "base_manifest.json")
            report["prepared_files"] = output_hashes
            (stage / "PREPROCESSING.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            if output.exists() or output.is_symlink():
                raise ValueError("Output appeared during preprocessing; refusing to replace it")
            stage.rename(output)
    print(f"{'DRY-RUN checked' if dry_run else 'Prepared'} {len(actions)} actions: {output}")
    print("Source delivery unchanged; no checkpoint loading, inference or RKNN compilation")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="complete training FP16 delivery directory (read-only)")
    parser.add_argument("output", type=Path, help="new prepared bundle directory outside source")
    parser.add_argument("--dry-run", action="store_true", help="check the entire delivery and reference rewrite without writing files")
    args = parser.parse_args()
    if args.output.is_symlink():
        parser.error("output must not be a symlink")
    prepare(args.source, args.output, args.dry_run)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
