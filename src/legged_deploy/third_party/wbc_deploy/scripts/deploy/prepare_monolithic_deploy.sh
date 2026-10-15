#!/usr/bin/env bash
set -euo pipefail

show_help() {
  cat <<'EOF'
Build an unchanged-ROS2-controller deployment bundle from MimicLite ONNX files.

Usage:
  prepare_monolithic_deploy.sh POLICY.onnx MOTION.onnx OUTPUT_DIR

Output:
  OUTPUT_DIR/policy.onnx
  OUTPUT_DIR/motion.onnx
  OUTPUT_DIR/rl_config.yaml

The native policy and motion adapters are temporary build artifacts and are
removed automatically. The deployment bundle contains only the three files
listed above.

The generated TensorRT filenames expected by rl_config.yaml are:
  policy.trt
  motion.trt

Set MIMIC_LITE_LOWLEVEL_FREQUENCY to override the ROS2 low-level controller
frequency (default: 500 Hz). Sim2Sim physics frequency is configured separately.
If the policy YAML has no policy_frequency, set MIMIC_LITE_POLICY_FREQUENCY
to the policy frequency used during training.
EOF
}

if [[ $# -ne 3 ]]; then
  show_help >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
PYTHON_BIN="${WBC_DEPLOY_PYTHON:-${MIMIC_LITE_PYTHON:-${REPO_ROOT}/.venv/bin/python}}"
if [[ ! -x "${PYTHON_BIN}" && -z "${WBC_DEPLOY_PYTHON:-}${MIMIC_LITE_PYTHON:-}" ]]; then
  PYTHON_BIN="$(command -v python3 || true)"
fi
POLICY_SOURCE="$(realpath -- "$1")"
POLICY_CONFIG_SOURCE="${POLICY_SOURCE%.onnx}.yaml"
MOTION_SOURCE="$(realpath -- "$2")"
OUTPUT_DIR="$(realpath -m -- "$3")"
LOWLEVEL_FREQUENCY="${MIMIC_LITE_LOWLEVEL_FREQUENCY:-500}"
POLICY_FREQUENCY="${MIMIC_LITE_POLICY_FREQUENCY:-}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python environment not found: ${PYTHON_BIN}" >&2
  exit 1
fi
if [[ ! -f "${POLICY_SOURCE}" ]]; then
  echo "Policy ONNX not found: ${POLICY_SOURCE}" >&2
  exit 1
fi
if [[ ! -f "${POLICY_CONFIG_SOURCE}" ]]; then
  echo "Policy config not found: ${POLICY_CONFIG_SOURCE}" >&2
  echo "Expected the YAML exported by play.py next to the policy ONNX." >&2
  exit 1
fi
if [[ ! -f "${MOTION_SOURCE}" ]]; then
  echo "Motion ONNX not found: ${MOTION_SOURCE}" >&2
  exit 1
fi

mkdir -p -- "${OUTPUT_DIR}"
BUILD_TEMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "${BUILD_TEMP_DIR}"' EXIT
NATIVE_POLICY="${BUILD_TEMP_DIR}/native_policy_deploy.onnx"
NATIVE_MOTION="${BUILD_TEMP_DIR}/native_motion_deploy.onnx"

"${PYTHON_BIN}" "${SCRIPT_DIR}/prepare_robot_deploy_onnx.py" \
  --policy "${POLICY_SOURCE}" \
  --motion "${MOTION_SOURCE}" \
  --policy-output "${NATIVE_POLICY}" \
  --motion-output "${NATIVE_MOTION}" \
  --opset 18

POLICY_FREQUENCY_ARGS=()
if [[ -n "${POLICY_FREQUENCY}" ]]; then
  POLICY_FREQUENCY_ARGS=(--policy-frequency "${POLICY_FREQUENCY}")
fi

"${PYTHON_BIN}" "${SCRIPT_DIR}/build_monolithic_robot_policy.py" \
  --policy "${NATIVE_POLICY}" \
  --policy-config "${POLICY_CONFIG_SOURCE}" \
  --lowlevel-frequency "${LOWLEVEL_FREQUENCY}" \
  "${POLICY_FREQUENCY_ARGS[@]}" \
  --motion "${NATIVE_MOTION}" \
  --policy-output "${OUTPUT_DIR}/policy.onnx" \
  --motion-output "${OUTPUT_DIR}/motion.onnx" \
  --config-output "${OUTPUT_DIR}/rl_config.yaml"

"${PYTHON_BIN}" - "${OUTPUT_DIR}" <<'PY'
from pathlib import Path
import sys
import onnx
import onnxruntime as ort

output = Path(sys.argv[1])
expected = {
    "policy.onnx": (["obs"], ["actions"]),
    "motion.onnx": (
        ["time_step"],
        ["root_pos", "root_quat", "contact_mask", "body_quat_w"],
    ),
}
for filename, (inputs, outputs) in expected.items():
    path = output / filename
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    actual_inputs = [item.name for item in session.get_inputs()]
    actual_outputs = [item.name for item in session.get_outputs()]
    if actual_inputs != inputs or actual_outputs != outputs:
        raise RuntimeError(
            f"Unexpected ABI for {path}: inputs={actual_inputs}, outputs={actual_outputs}"
        )
    print(f"Validated {path}")
PY

echo
echo "Deployment bundle prepared: ${OUTPUT_DIR}"
echo "Convert on the target Jetson:"
echo "  trtexec --onnx=${OUTPUT_DIR}/policy.onnx --saveEngine=${OUTPUT_DIR}/policy.trt --fp16"
echo "  trtexec --onnx=${OUTPUT_DIR}/motion.onnx --saveEngine=${OUTPUT_DIR}/motion.trt --fp16"
