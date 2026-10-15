#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  build_trt_engines.sh [--force] POLICY_DIR [POLICY_DIR ...]

Run this script on the target Jetson. Each policy directory must contain
policy.onnx and motion.onnx. TensorRT engines are hardware and TensorRT-version
specific, so they must not be copied from an x86 development computer.

Set TRTEXEC=/absolute/path/to/trtexec if trtexec is not in PATH or in one of
the standard Jetson locations.
EOF
}

force=false
if [[ "${1:-}" == "--force" ]]; then
  force=true
  shift
fi

if [[ "$#" -eq 0 ]]; then
  usage >&2
  exit 2
fi

trtexec_bin="${TRTEXEC:-}"
if [[ -z "$trtexec_bin" ]]; then
  trtexec_bin="$(command -v trtexec || true)"
fi
if [[ -z "$trtexec_bin" ]]; then
  for candidate in /usr/src/tensorrt/bin/trtexec /usr/local/tensorrt/bin/trtexec; do
    if [[ -x "$candidate" ]]; then
      trtexec_bin="$candidate"
      break
    fi
  done
fi
if [[ -z "$trtexec_bin" || ! -x "$trtexec_bin" ]]; then
  echo "ERROR: trtexec was not found. Install TensorRT tools or set TRTEXEC." >&2
  exit 1
fi

for policy_dir_arg in "$@"; do
  if [[ ! -d "$policy_dir_arg" ]]; then
    echo "ERROR: policy directory does not exist: $policy_dir_arg" >&2
    exit 1
  fi
  policy_dir="$(realpath "$policy_dir_arg")"

  for model_name in policy motion; do
    onnx_path="$policy_dir/$model_name.onnx"
    engine_path="$policy_dir/$model_name.trt"
    if [[ ! -f "$onnx_path" ]]; then
      echo "ERROR: missing model: $onnx_path" >&2
      exit 1
    fi

    if [[ "$force" == false && -f "$engine_path" && "$engine_path" -nt "$onnx_path" ]]; then
      echo "Up to date: $engine_path"
      continue
    fi

    echo "Building: $engine_path"
    "$trtexec_bin" \
      --onnx="$onnx_path" \
      --saveEngine="$engine_path" \
      --fp16
  done
done

echo "TensorRT engine generation completed."
