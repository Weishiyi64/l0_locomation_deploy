"""Versioned WBC LoRA graph and adapter assets shared by export/conversion tools."""
import hashlib
import json
import re
from pathlib import Path

import numpy as np

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


def graph_feeds(graph, observations, parameters):
    return {item["name"]: np.ascontiguousarray(
        observations[item["name"]] if item["role"] == "observation" else parameters[item["name"]],
        dtype=np.float32).reshape(item["shape"]) for item in graph["inputs"]}


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


def runtime_options(args):
    if getattr(args, "build_only", False) and (args.target or getattr(args, "device_id", None)):
        raise ValueError("--build-only cannot be combined with --target/--device-id")
    options = {"target": args.target} if args.target else {}
    if getattr(args, "device_id", None):
        if not args.target:
            raise ValueError("--device-id requires --target rk3588")
        options["device_id"] = args.device_id
    return options


def rknn_build_config():
    # Toolkit2 2.3.2 exNorm Add fusion reads dynamic FP16 beta as FP32.
    # Keep the affine Add separate; shared policy and reference converters use
    # this same default, and conversion provenance records the exact setting.
    return {"target_platform": "rk3588", "float_dtype": "float16",
            "disable_rules": ["fuse_exnorm_add"]}


def conversion_record(onnx_path, output_path, args, verification):
    import importlib.metadata
    import sys
    try:
        toolkit = importlib.metadata.version("rknn-toolkit2")
    except importlib.metadata.PackageNotFoundError:
        toolkit = "unknown"
    build_only = getattr(args, "build_only", False)
    record = {"format": "wbc_rknn_conversion_v1", "source_sha256": sha256(onnx_path),
              "rknn_sha256": sha256(output_path), "precision": "float16",
              "do_quantization": False, "toolkit_version": toolkit,
              "rknn_config": rknn_build_config(),
              "target_platform": "rk3588", "converter_sha256": sha256(Path(sys.argv[0]).resolve()),
              "asset_helper_sha256": sha256(Path(__file__).resolve()),
              "verification_backend": "not_run" if build_only else ("target" if args.target else "toolkit_simulator"),
              "validation_status": "converted_pending_runtime" if build_only else "python_verified",
              "target": args.target, "device_id": getattr(args, "device_id", None),
              "verification": verification, "max_abs_error": args.max_abs_error}
    if getattr(args, "manifest", None):
        record["graph_manifest_sha256"] = sha256(args.manifest)
    Path(str(output_path) + ".json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    return record
