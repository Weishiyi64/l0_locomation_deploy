#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This script builds the Jetson-specific TensorRT engine and must run on ARM64." >&2
  exit 2
fi

if [[ $# -gt 0 ]]; then
  policy_dir="$1"
else
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  source_policy_dir="${script_dir}/../policy/l0_v3/text_motion_wbc_stg25_dynamic"
  install_policy_dir="${script_dir}/../../share/legged_deploy/policy/l0_v3/text_motion_wbc_stg25_dynamic"
  if [[ -d "${source_policy_dir}" ]]; then
    policy_dir="$(cd "${source_policy_dir}" && pwd)"
  else
    policy_dir="$(cd "${install_policy_dir}" && pwd)"
  fi
fi
onnx_path="${policy_dir}/policy.onnx"
trt_path="${policy_dir}/policy.trt"
trtexec_bin="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

[[ -f "${onnx_path}" ]] || { echo "Missing ${onnx_path}" >&2; exit 1; }
[[ -x "${trtexec_bin}" ]] || { echo "Missing executable ${trtexec_bin}" >&2; exit 1; }

if [[ -f "${trt_path}" && "${trt_path}" -nt "${onnx_path}" ]]; then
  echo "TensorRT engine is current: ${trt_path}"
  exit 0
fi

"${trtexec_bin}" --onnx="${onnx_path}" --saveEngine="${trt_path}" --fp16
echo "Built fixed text-motion WBC engine: ${trt_path}"
