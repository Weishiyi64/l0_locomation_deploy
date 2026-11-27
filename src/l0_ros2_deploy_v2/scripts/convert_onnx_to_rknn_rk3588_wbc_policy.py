#!/usr/bin/env python3
"""Convert a complete WBC ONNX bundle into a ready-to-install RK3588 policy directory.

Accept the bundle directory or its shared policy ONNX. Reference normalization,
all RKNN builds and runtime packaging are automatic; source files stay unchanged.
No training export, Python numerical inference, ROS build or hardware launch.
"""
import argparse
import importlib.metadata
import hashlib
import json
import os
import platform
import re
import shutil
import sys
import tempfile
from contextlib import ExitStack, contextmanager
from pathlib import Path

import numpy as np
import yaml

from convert_onnx_to_rknn_rk3588_wbc_reference import (
    check_reference_model, load_embedded_model, normalize_reference,
    rknn_compatible_model as reference_compatible_model, rknn_build_config,
)

RKNN_MAX_ONNX_OPSET = 19
GRAPH_FORMAT = "wbc_lora_graph_v1"
ADAPTER_FORMAT = "wbc_lora_adapter_v1"
ID_PATTERN = re.compile(r"[A-Za-z0-9_-]{1,32}")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 4 * 1024 * 1024:
        raise ValueError(f"Manifest too large: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a manifest object: {path}")
    return value


def asset_path(root, relative):
    root = Path(root).resolve()
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError(f"Expected a relative asset path: {relative}")
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Asset escapes deployment root: {relative}")
    return path


def tensor_shape(value):
    if not isinstance(value, list) or not value or any(type(x) is not int or x <= 0 for x in value):
        raise ValueError(f"Invalid static tensor shape: {value}")
    return tuple(value)


def load_graph_manifest(path):
    manifest = read_json(path)
    if manifest.get("format") != GRAPH_FORMAT or manifest.get("kind") not in ("policy", "probe"):
        raise ValueError("Unsupported WBC LoRA graph manifest")
    parameters = manifest.get("parameters")
    if not isinstance(parameters, list) or len(parameters) != 31:
        raise ValueError("Expected the complete 31-tensor WBC adapter schema")
    for key in ("base_checkpoint_sha256", "normalization_sha256", "parameter_schema_sha256", "source_onnx_sha256"):
        if not isinstance(manifest.get(key), str) or not re.fullmatch(r"[0-9a-f]{64}", manifest[key]):
            raise ValueError(f"Missing/invalid required graph fingerprint: {key}")
    if manifest.get("target_precision") != "float16":
        raise ValueError("The target RKNN precision must be float16")
    if fingerprint(parameters) != manifest.get("parameter_schema_sha256"):
        raise ValueError("Adapter schema fingerprint mismatch")
    names = [p["name"] for p in parameters]
    if len(set(names)) != len(names) or manifest.get("rank") != 16 or manifest.get("alpha") != 16:
        raise ValueError("Invalid parameter names or unsupported rank/alpha")
    dimensions = [(1024, 639), (1024, 1024), (1024, 1024), (1024, 1423), (1024, 1024), (1024, 1024), (21, 1024)]
    expected = {}
    for i, (out_width, in_width) in enumerate(dimensions):
        expected.update({f"lora_{i}_a": [16, in_width], f"lora_{i}_b": [out_width, 16], f"lora_{i}_bias": [out_width]})
    for i in range(5):
        expected.update({f"norm_{i}_gamma": [1024], f"norm_{i}_beta": [1024]})
    if {p["name"]: p["shape"] for p in parameters} != expected:
        raise ValueError("Adapter schema does not match the native L0 31-tensor ABI")
    for param in parameters:
        tensor_shape(param["shape"])
        if not isinstance(param.get("actor_key"), str) or not param["actor_key"]:
            raise ValueError("Missing actor parameter mapping")
    inputs = manifest.get("inputs", [])
    if not inputs or len({x["name"] for x in inputs}) != len(inputs):
        raise ValueError("Invalid graph inputs")
    for item in inputs + manifest.get("outputs", []):
        tensor_shape(item["shape"])
        if item.get("dtype") != "float32":
            raise ValueError("The application ABI must remain float32")
    obs = {x["name"]: x["shape"] for x in inputs if x.get("role") == "observation"}
    if manifest["kind"] == "policy":
        if obs != {"policy": [399], "command_short": [240]} or len(inputs) != 33:
            raise ValueError("Policy graph requires 399/240 observations and 31 dynamic parameters")
        if manifest["outputs"] != [{"name": "action", "shape": [21], "dtype": "float32"}]:
            raise ValueError("Invalid WBC policy output")
    declared = {p["name"]: p for p in parameters}
    for item in inputs:
        if item["role"] == "adapter":
            if item["name"] not in declared or item["shape"] != declared[item["name"]]["shape"]:
                raise ValueError(f"Graph parameter does not match schema: {item['name']}")
        elif item["role"] != "observation":
            raise ValueError("Unknown graph input role")
    return manifest


def load_adapter(path, graph, expected_action_id=None):
    path = Path(path).resolve()
    manifest = read_json(path)
    if manifest.get("format") != ADAPTER_FORMAT or not ID_PATTERN.fullmatch(manifest.get("action_id", "")):
        raise ValueError("Invalid adapter format or action ID")
    if expected_action_id is not None and manifest["action_id"] != expected_action_id:
        raise ValueError("Adapter action ID does not match its catalog entry")
    for key in ("base_checkpoint_sha256", "parameter_schema_sha256", "normalization_sha256", "rank", "alpha"):
        if manifest.get(key) != graph.get(key):
            raise ValueError(f"Adapter/graph mismatch: {key}")
    weights = asset_path(path.parent, manifest["weights"]["path"])
    info = manifest["weights"]
    if info.get("dtype") != "float16-le" or weights.stat().st_size != info.get("size_bytes"):
        raise ValueError("Adapter dtype/size mismatch")
    if sha256(weights) != info.get("sha256"):
        raise ValueError("Adapter payload checksum mismatch")
    records = manifest.get("tensors", [])
    if len(records) != len(graph["parameters"]):
        raise ValueError("Incomplete adapter tensor set")
    payload = weights.read_bytes()
    result, previous_end = {}, 0
    for spec, record in zip(graph["parameters"], records):
        shape = tensor_shape(spec["shape"])
        size = int(np.prod(shape)) * 2
        offset = record.get("offset_bytes")
        if any(record.get(k) != spec[k] for k in ("name", "actor_key", "shape")):
            raise ValueError(f"Adapter tensor mismatch: {spec['name']}")
        if type(offset) is not int or offset < previous_end or offset % 64 or record.get("size_bytes") != size:
            raise ValueError("Invalid/overlapping adapter tensor range")
        if offset + size > len(payload):
            raise ValueError("Truncated adapter tensor")
        value = np.frombuffer(payload, dtype="<f2", count=size // 2, offset=offset).astype(np.float32).reshape(shape)
        if not np.isfinite(value).all():
            raise ValueError("Non-finite adapter data")
        result[spec["name"]] = value
        previous_end = offset + size
    if previous_end != len(payload):
        raise ValueError("Unexpected trailing adapter data")
    return result


def check_graph(path, manifest_path):
    import onnx

    path, manifest_path = Path(path).resolve(), Path(manifest_path).resolve()
    graph = load_graph_manifest(manifest_path)
    if path != asset_path(manifest_path.parent, graph["model"]["onnx"]) or sha256(path) != graph["model"]["sha256"]:
        raise ValueError("ONNX path/checksum does not match graph manifest")
    model = onnx.load(str(path), load_external_data=False)
    if any(t.data_location == onnx.TensorProto.EXTERNAL for t in model.graph.initializer):
        raise ValueError("External ONNX tensor data is not supported")
    onnx.checker.check_model(model)
    initializer_names = {t.name for t in model.graph.initializer}
    def specs(items):
        values = []
        for item in items:
            tensor = item.type.tensor_type
            shape = [d.dim_value for d in tensor.shape.dim]
            if tensor.elem_type != onnx.TensorProto.FLOAT:
                raise ValueError("WBC application inputs/outputs must be float32")
            tensor_shape(shape)
            values.append({"name": item.name, "shape": shape, "dtype": "float32"})
        return values
    expected_inputs = [{k: x[k] for k in ("name", "shape", "dtype")} for x in graph["inputs"]]
    if specs(model.graph.input) != expected_inputs or specs(model.graph.output) != graph["outputs"]:
        raise ValueError("ONNX I/O does not match the manifest ABI")
    if initializer_names.intersection(x["name"] for x in graph["inputs"]):
        raise ValueError("Dynamic adapter inputs must not be ONNX initializers")
    return graph


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


def include_text_assets(source, graph, include):
    """Keep the optional JSON-only tracker with its matching shared Base."""
    if not (source / "deployment.json").is_file():
        if (source / "text").exists():
            raise ValueError("Text tracking assets require deployment.json")
        return
    layout = read_json(include("deployment.json"))
    if layout.get("format") != "wbc_motion_deployment_v1" or \
            layout.get("model_manifest") != "base_manifest.json" or \
            layout.get("catalogs") != ["base_manifest.json", "text/tracker.json"] or \
            layout.get("text_action") != "UpperBody_Enhanced" or \
            layout.get("standing_config") != "text/standing.yaml":
        raise ValueError("Expected the combined Base/fixed/text deployment layout")
    catalog_path = include("text/tracker.json")
    catalog = read_json(catalog_path)
    if catalog.get("format") != "wbc_motion_catalog_v1":
        raise ValueError("Invalid text tracker catalog")
    for key in ("base_checkpoint_sha256", "normalization_sha256", "parameter_schema_sha256", "rank", "alpha"):
        if catalog.get(key) != graph[key]:
            raise ValueError(f"Text tracker/shared Base mismatch: {key}")
    actions = catalog.get("actions", [])
    if len(actions) != 1 or actions[0].get("action_id") != layout["text_action"] or \
            actions[0].get("json_only") is not True or actions[0].get("parameter_source") != "lora" or \
            any(a["action_id"] == layout["text_action"] for a in graph["actions"]):
        raise ValueError("Expected one distinct JSON-only UpperBody_Enhanced LoRA")
    action = actions[0]
    adapter_path = include(str(Path("text") / action["adapter_manifest"]), action["adapter_manifest_sha256"])
    load_adapter(adapter_path, graph, action["action_id"])
    adapter = read_json(adapter_path)
    weights = asset_path(adapter_path.parent, adapter["weights"]["path"])
    include(str(weights.relative_to(source)), adapter["weights"]["sha256"])
    config_path = include(str(Path("text") / action["config"]), action["config_sha256"])
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config["WbcCfg"]["motion_id"] != action["action_id"]:
        raise ValueError("Text tracker/config identity mismatch")
    include(layout["standing_config"])


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
    reserve("ASSETS.sha256")
    graph["model"].pop("rknn_record", None)
    for action in actions:
        action.pop("reference_rknn_record", None)
    for relative in [graph["model"]["rknn"]] + [a["reference_rknn"] for a in actions]:
        if not relative.endswith(".rknn"):
            raise ValueError("Expected RKNN model paths")
        reserve(relative)

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
                      "source_table_dtype": meta["source_table_dtype"], "prepared_table_dtype": "float32",
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

        include_text_assets(source, graph, include)

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
            checksums = [f"{sha256(path)}  {path.relative_to(stage).as_posix()}\n"
                         for path in sorted(stage.rglob("*")) if path.is_file()]
            (stage / "ASSETS.sha256").write_text("".join(checksums), encoding="utf-8")
            if output.exists() or output.is_symlink():
                raise ValueError("Output appeared during preprocessing; refusing to replace it")
            stage.rename(output)
    print(f"{'DRY-RUN checked' if dry_run else 'Prepared'} {len(actions)} actions: {output}")
    print("Source delivery unchanged; reference values and frame indexing preserved")
    return report


def install_lora(source, output, model_hashes):
    """Validate the converted source; stage only RKNN runtime assets for colcon."""
    if output.exists() or output.is_symlink():
        raise ValueError("install-lora requires a new output directory; use a versioned bundle name for updates")
    source, output = source.resolve(strict=True), output.resolve()
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError("installation output and source delivery must not be nested")
    manifest_path = source / "base_manifest.json"
    manifest = read_json(manifest_path)
    graph = check_graph(asset_path(source, manifest["model"]["onnx"]), manifest_path)
    if graph["kind"] != "policy":
        raise ValueError("install-lora requires the full shared policy, not a probe")
    files = {manifest_path: sha256(manifest_path)}

    def include(relative, expected=None):
        path = asset_path(source, relative)
        if path.suffix.lower() not in (".json", ".onnx", ".rknn", ".yaml", ".bin"):
            raise ValueError(f"Not a deployment asset (checkpoints are forbidden): {relative}")
        if not path.is_file():
            raise ValueError(f"Missing deployment asset: {path}")
        actual = sha256(path)
        if expected is not None and actual != expected:
            raise ValueError(f"Deployment asset checksum mismatch: {path}")
        files[path] = actual
        return path

    if (source / "PREPROCESSING.json").is_file():
        include("PREPROCESSING.json")

    def model(onnx, rknn, expected):
        if Path(onnx).suffix != ".onnx" or Path(rknn).suffix != ".rknn":
            raise ValueError("Expected ONNX source and RKNN runtime model paths")
        include(onnx, expected)
        if rknn not in model_hashes or asset_path(source, rknn).stat().st_size == 0:
            raise ValueError(f"Missing converted RKNN: {rknn}")
        include(rknn, model_hashes[rknn])

    shared = graph["model"]
    model(shared["onnx"], shared["rknn"], shared["sha256"])
    seen = set()
    for action in graph["actions"]:
        action_id = action["action_id"]
        if not ID_PATTERN.fullmatch(action_id) or action_id in seen:
            raise ValueError("Invalid or duplicate action ID")
        seen.add(action_id)
        adapter = include(action["adapter_manifest"])
        load_adapter(adapter, graph, action_id)
        info = read_json(adapter)
        weights = asset_path(adapter.parent, info["weights"]["path"])
        include(str(weights.relative_to(source)), info["weights"]["sha256"])
        config = include(action["config"], action["config_sha256"])
        if yaml.safe_load(config.read_text(encoding="utf-8"))["WbcCfg"]["motion_id"] != action_id:
            raise ValueError(f"Action/config identity mismatch: {action_id}")
        check_reference_model(asset_path(source, action["reference_onnx"]))
        model(action["reference_onnx"], action["reference_rknn"], action["reference_onnx_sha256"])
    if not seen:
        raise ValueError("No actions in deployment bundle")
    include_text_assets(source, graph, include)
    # Integrity checks finish here; the worker loads the converted package directly.
    # Keep source ONNX models in model_sources for rebuilding.
    files = {path: digest for path, digest in files.items() if path.suffix != ".onnx"}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wbc_lora_stage_", dir=output.parent) as temporary:
        staged = Path(temporary) / "bundle"
        staged.mkdir()
        for path, digest in files.items():
            destination = staged / path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
            if sha256(destination) != digest:
                raise ValueError(f"Source changed during installation: {path}")
        (staged / "INSTALLATION.json").write_text(json.dumps({
            "format": "wbc_lora_installation_v1", "validation_status": "pending_formal_runtime",
            "asset_profile": "rknn_runtime_only",
            "files": {str(path.relative_to(source)): digest for path, digest in files.items()},
        }, indent=2) + "\n", encoding="utf-8")
        if output.exists():
            raise ValueError("Installation destination appeared during staging")
        staged.rename(output)
    print(f"Installed {len(seen)} actions / one shared WBC Base at {output}")
    print("Asset checks passed; colcon build and formal C++ runtime validation remain required")


def compile_model(source, output, manifest=None):
    from rknn.api import RKNN

    if output.exists() or output.is_symlink():
        raise ValueError(f"RKNN output already exists: {output}")
    if manifest:
        check_graph(source, manifest)
    else:
        check_reference_model(source)
    compatible = rknn_compatible_model if manifest else reference_compatible_model
    output.parent.mkdir(parents=True, exist_ok=True)
    with compatible(source) as model, tempfile.TemporaryDirectory(prefix=".compiler_", dir=output.parent) as work:
        # Toolkit2 writes diagnostics to cwd; keep them inside this build's scratch directory.
        previous = Path.cwd()
        try:
            os.chdir(work)
            rknn = RKNN(verbose=False)
            try:
                result = rknn.config(**rknn_build_config())
                if result != 0:
                    raise RuntimeError(f"RKNN config failed: {result}")
                result = rknn.load_onnx(model=str(model))
                if result != 0:
                    raise RuntimeError(f"RKNN load_onnx failed: {result}")
                result = rknn.build(do_quantization=False)
                if result != 0:
                    raise RuntimeError(f"RKNN build failed: {result}")
                result = rknn.export_rknn(str(output))
                if result != 0 or not output.is_file() or output.stat().st_size == 0:
                    raise RuntimeError(f"RKNN export failed: {result}")
            finally:
                rknn.release()
        finally:
            os.chdir(previous)
    return sha256(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="complete bundle directory, base_manifest.json or shared policy ONNX")
    parser.add_argument("-o", "--output", type=Path, help="new runtime bundle directory (default: SOURCE_DIRECTORY_rknn)")
    parser.add_argument("--dry-run", action="store_true", help="check assets and reference normalization without compiling/writing")
    parser.add_argument("--manifest", type=Path, help="optional base_manifest.json beside the shared ONNX")
    parser.add_argument("--build-only", action="store_true", help="accepted for compatibility; this command always builds without inference")
    args = parser.parse_args()

    source = args.source.resolve(strict=True)
    root = source if source.is_dir() else source.parent
    manifest = root / "base_manifest.json"
    if not manifest.is_file():
        raise ValueError("WBC requires a complete ONNX bundle with base_manifest.json, adapters, configs and references beside the shared policy")
    if args.manifest and args.manifest.resolve(strict=True) != manifest:
        raise ValueError("--manifest must be base_manifest.json in the selected bundle")
    graph = read_json(manifest)
    if source.is_file() and source not in (manifest, asset_path(root, graph["model"]["onnx"])):
        raise ValueError("Select the bundle directory or its shared policy ONNX; use the reference converter for a lone reference")
    output = (args.output or root.with_name(root.name + "_rknn")).absolute()
    if output.is_symlink() or output.exists():
        raise ValueError(f"Output directory already exists; choose a new version: {output}")
    output = output.resolve()
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Output must be outside the source bundle and must not contain it")

    # Validate delivered checksum lists before making any transformed copies.
    for name, directory in (("ASSETS.sha256", root), ("SOURCE.sha256", Path(__file__).resolve().parent.parent)):
        checksum_list = root / name
        if checksum_list.is_file():
            for line in checksum_list.read_text().splitlines():
                if not line.strip():
                    continue
                digest, relative = line.split(maxsplit=1)
                if sha256(asset_path(directory, relative.removeprefix('*'))) != digest:
                    raise ValueError(f"Checksum mismatch in {checksum_list}: {relative}")
    if args.dry_run:
        prepare(root, output, dry_run=True)
        print(f"DRY-RUN: build one shared policy + {len(graph['actions'])} references, then install at {output}")
        return 0
    if platform.system() != "Linux" or platform.machine() != "aarch64":
        raise ValueError("Actual conversion must run on the RK3588 Linux/aarch64 board; use --dry-run for static checks")
    if importlib.metadata.version("rknn-toolkit2").split('+')[0] != "2.3.2":
        raise ValueError("Expected rknn-toolkit2 2.3.2")

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wbc_convert_", dir=output.parent) as work:
        prepared = Path(work) / "prepared"
        prepare(root, prepared)
        prepared_manifest = prepared / "base_manifest.json"
        graph = read_json(prepared_manifest)
        models = [(graph["model"]["onnx"], graph["model"]["rknn"], prepared_manifest)]
        models.extend((action["reference_onnx"], action["reference_rknn"], None) for action in graph["actions"])
        model_hashes = {}
        for index, (onnx, rknn, policy_manifest) in enumerate(models, 1):
            print(f"[{index}/{len(models)}] Convert {onnx}", flush=True)
            model_hashes[rknn] = compile_model(asset_path(prepared, onnx), asset_path(prepared, rknn), policy_manifest)
        install_lora(prepared, output, model_hashes)
    print(f"Ready-to-install WBC bundle: {output}; RKNN loading and robot effects remain unverified")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
