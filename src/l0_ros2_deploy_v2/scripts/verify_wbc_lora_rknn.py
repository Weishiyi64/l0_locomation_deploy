#!/usr/bin/env python3
"""Manually verify a converted WBC LoRA graph on RK3588 without ROS or HPM."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from wbc_lora_assets import asset_path, graph_feeds, load_adapter, load_graph_manifest, read_json, sha256


def verify_cases(runtime, root, graph, max_abs_error):
    """Use one caller-owned initialized RKNN context; never reload it between adapters."""
    arrays = {}
    for key in ("verification_data", "verification_expected"):
        path = asset_path(root, graph[key])
        if sha256(path) != graph[key + "_sha256"]:
            raise ValueError(f"Verification data checksum mismatch: {key}")
        with np.load(path, allow_pickle=False) as source:
            arrays[key] = {name: source[name].copy() for name in source.files}
    observations = arrays["verification_data"]
    expected_outputs = arrays["verification_expected"]
    for item in graph["inputs"]:
        if item["role"] == "observation":
            value = observations[item["name"]]
            if value.dtype != np.float32 or list(value.shape[1:]) != item["shape"] or not np.isfinite(value).all():
                raise ValueError(f"Invalid recorded observation: {item['name']}")
    counts = {len(observations[x["name"]]) for x in graph["inputs"] if x["role"] == "observation"}
    if len(counts) != 1 or min(counts) < 20:
        raise ValueError("At least 20 aligned observations are required")
    count = min(counts)
    first_feed, first_output, reports = None, None, []
    for action in graph["actions"]:
        parameters = load_adapter(asset_path(root, action["adapter_manifest"]), graph, action["action_id"])
        expected = expected_outputs[action["action_id"]]
        if expected.dtype != np.float32 or list(expected.shape) != [count] + graph["outputs"][0]["shape"]:
            raise ValueError("Invalid expected output shape/dtype")
        if not np.isfinite(expected).all():
            raise ValueError("Non-finite expected output")
        maximum, total, worst_ms = 0.0, 0.0, 0.0
        for i in range(count):
            feed = graph_feeds(graph, {k: v[i] for k, v in observations.items()}, parameters)
            started = time.perf_counter()
            outputs = runtime.inference(inputs=[feed[x["name"]] for x in graph["inputs"]])
            elapsed = (time.perf_counter() - started) * 1000
            if outputs is None or len(outputs) != 1:
                raise RuntimeError("RKNN inference did not return one output")
            actual = np.asarray(outputs[0], np.float32).reshape(-1)
            reference = expected[i].reshape(-1)
            if actual.shape != reference.shape or not np.isfinite(actual).all():
                raise RuntimeError("Invalid/non-finite RKNN output")
            error = np.abs(actual - reference)
            maximum, total, worst_ms = max(maximum, float(error.max())), total + float(error.mean()), max(worst_ms, elapsed)
            if error.max() > max_abs_error:
                raise RuntimeError(f"{action['action_id']}: sample {i} max_abs={error.max()} > {max_abs_error}")
            if first_feed is None:
                first_feed, first_output = feed, actual.copy()
        reports.append({"action_id": action["action_id"], "samples": count, "max_abs": maximum,
                        "mean_abs": total / count, "worst_ms": worst_ms})
    if first_feed is None:
        raise ValueError("No adapters selected")
    repeated = runtime.inference(inputs=[first_feed[x["name"]] for x in graph["inputs"]])
    if repeated is None or len(repeated) != 1 or not np.isfinite(repeated[0]).all() or \
            not np.allclose(np.asarray(repeated[0]).reshape(-1), first_output, atol=max_abs_error, rtol=0):
        raise RuntimeError("A-to-B-to-A output mismatch")
    return {"actions": reports, "a_to_b_to_a": "passed" if len(reports) > 1 else "one_adapter_only",
            "runtime_initializations_in_this_run": 1, "max_abs_error": max_abs_error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--rknn", type=Path, help="defaults to the manifest RKNN path")
    parser.add_argument("--max-abs-error", type=float, default=0.05)
    parser.add_argument("--report", type=Path, help="optional JSON report path")
    args = parser.parse_args()
    if not math.isfinite(args.max_abs_error) or args.max_abs_error <= 0:
        raise ValueError("Invalid precision threshold")
    path = args.manifest.resolve()
    root, graph = path.parent, load_graph_manifest(path)
    model_path = (args.rknn or asset_path(root, graph["model"]["rknn"])).resolve()
    record = read_json(Path(str(model_path) + ".json"))
    if record.get("source_sha256") != graph["model"]["sha256"] or \
            record.get("rknn_sha256") != sha256(model_path) or record.get("precision") != "float16":
        raise ValueError("RKNN conversion record does not match this FP16 graph/model")
    from rknnlite.api import RKNNLite
    runtime = RKNNLite()
    try:
        ret = runtime.load_rknn(str(model_path))
        if ret != 0:
            raise RuntimeError(f"load_rknn failed: {ret}")
        ret = runtime.init_runtime()
        if ret != 0:
            raise RuntimeError(f"init_runtime failed: {ret}")
        report = verify_cases(runtime, root, graph, args.max_abs_error)
        report.update(backend="RKNNLite_on_target", manifest_sha256=sha256(path),
                      rknn_sha256=sha256(model_path), precision="float16",
                      operator_placement="inspect compiler/runtime profiling separately")
        print(json.dumps(report, indent=2, allow_nan=False))
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    finally:
        runtime.release()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
