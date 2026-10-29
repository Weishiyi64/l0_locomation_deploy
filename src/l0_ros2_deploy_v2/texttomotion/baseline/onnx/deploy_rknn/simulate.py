"""Validate neural components with the explicit Toolkit2 PC simulator.

This rebuilds the prepared ONNX graphs without INT8 quantization, then calls
RKNN.init_runtime() without a target.  It is not a real-board verification and
does not silently substitute ONNX Runtime if Toolkit2 fails.
"""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from .convert import ROOT, checked_file, check_return, inspect_model, sha256, require_toolkit_version
from .pipeline import _validated_tensors
from .verify import compare_output


@contextmanager
def _working_directory(path):
    """Contain SDK debug ONNX files and restore the caller's working directory."""
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def simulate(manifest_path, report_path, *, atol=0.005, rtol=0.02):
    manifest_path, report_path = Path(manifest_path).resolve(), Path(report_path).resolve()
    if report_path.exists():
        raise FileExistsError(f"Choose a new simulator report: {report_path}")
    if not np.isfinite(atol) or not np.isfinite(rtol) or atol < 0 or rtol < 0:
        raise ValueError("Numerical tolerances must be finite and nonnegative")
    report = {"status": "running", "backend": "RKNN-Toolkit2 PC simulator",
              "board_verified": False, "full_pipeline_verified": False,
              "quantized": False, "cases": [], "atol": atol, "rtol": rtol}
    session = None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        report["manifest_sha256"] = sha256(manifest_path)
        if manifest.get("format") != "rknn_static_neural_v1" or manifest.get("target_platform") != "rk3588":
            raise ValueError("Simulator requires a prepared RK3588 ONNX package, not a compiled .rknn package")
        models = manifest["models"]
        if set(models) != {"qwen_tokenizer", "inbetweener"}:
            raise ValueError("Expected exactly qwen_tokenizer and inbetweener models")
        cases = manifest.get("validation_cases", [])
        if not cases or {case["model"] for case in cases} != set(models):
            raise ValueError("Validation cases must cover both neural models")
        paths = {name: checked_file(manifest_path.parent, record) for name, record in models.items()}
        for path in paths.values():
            inspect_model(path)
        for case in cases:
            checked_file(manifest_path.parent, case)
        report["toolkit2_version"] = require_toolkit_version()
        from rknn.api import RKNN
        log_root = ROOT / "tmp/deploy_rknn"
        log_root.mkdir(parents=True, exist_ok=True)
        log_dir = Path(tempfile.mkdtemp(prefix="simulator_", dir=log_root))
        report["log_directory"] = str(log_dir)
        for name, record in models.items():
            report["active_model"] = name
            workdir = log_dir / name
            workdir.mkdir()
            # Toolkit2 emits intermediate check*.onnx into cwd even with an
            # explicit verbose_file. Keep these implementation artifacts in tmp.
            with _working_directory(workdir):
                session = RKNN(verbose=True, verbose_file=str(log_dir / f"{name}.log"))
                check_return(session.config(target_platform="rk3588"), "config")
                check_return(session.load_onnx(model=str(paths[name])), "load_onnx")
                check_return(session.build(do_quantization=False), "build")
                # target=None is the PC simulator, not the RK3588 board.
                check_return(session.init_runtime(), "init_runtime simulator")
                for case in (item for item in cases if item["model"] == name):
                    report["active_case"] = case["file"]
                    with np.load(checked_file(manifest_path.parent, case), allow_pickle=False) as data:
                        feed = _validated_tensors(
                            {spec["name"]: data["input__" + spec["name"]] for spec in record["inputs"]}, record["inputs"])
                        result = session.inference(inputs=[feed[spec["name"]] for spec in record["inputs"]],
                                                   data_format=["nchw"] * len(feed))
                        if result is None or len(result) != len(record["outputs"]):
                            raise RuntimeError("Unexpected RKNN simulator output count")
                        # Compare raw outputs before any dtype normalization.
                        errors = {}
                        for spec, actual in zip(record["outputs"], result):
                            report["active_output"] = spec["name"]
                            errors[spec["name"]] = compare_output(spec["name"], actual,
                                data["output__" + spec["name"]], atol=atol, rtol=rtol)
                    report["cases"].append({"model": name, "file": case["file"], "max_abs_errors": errors})
                releasing, session = session, None
                releasing.release()
        report.pop("active_model", None)
        report.pop("active_case", None)
        report.pop("active_output", None)
        report["status"] = "simulator_numerical_parity_passed"
    except Exception as exc:
        report.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if session is not None:
            try:
                session.release()
            except Exception as exc:
                report.update(status="failed", release_error=str(exc))
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if report["status"] != "simulator_numerical_parity_passed":
        raise RuntimeError(f"Simulator cleanup failed; see {report_path}")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "rknn/prepared/manifest.json")
    parser.add_argument("--report", type=Path, default=ROOT / "tmp/deploy_rknn/simulator_report.json")
    parser.add_argument("--atol", type=float, default=0.005)
    parser.add_argument("--rtol", type=float, default=0.02)
    args = parser.parse_args(argv)
    print(json.dumps(simulate(args.manifest, args.report, atol=args.atol, rtol=args.rtol),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
