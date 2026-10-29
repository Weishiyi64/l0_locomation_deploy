#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash compile.sh [--output NEW_DIRECTORY] [--build-only] [--dry-run]'
  echo 'Input: onnx/manifest.json beside this script; default output: compiled/.'
  echo 'Requires rknn-toolkit2==2.3.2. Set RKNN_PYTHON to its Python interpreter.'
  echo 'The onnx/ bundle contains the compiler; training project/checkpoints are not required.'
  echo 'Board compilation only: never initializes a simulator or inference runtime.'
  echo 'On the supplied Python 3.10.12 board environment: RKNN_PYTHON=python3.10 bash compile.sh'
  exit 0
fi
if [[ ! -f "$SCRIPT_DIR/onnx/deploy_rknn/convert.py" ]]; then
  echo 'Missing ONNX compiler bundle. Run export.sh in the original project first.' >&2
  exit 2
fi
DEFAULT_PYTHON=python3
export PYTHONPATH="$SCRIPT_DIR/onnx${PYTHONPATH:+:$PYTHONPATH}"
export RKNN_WORK_DIR="${RKNN_WORK_DIR:-$SCRIPT_DIR/../tmp/onnx_rknn}"
exec "${RKNN_PYTHON:-$DEFAULT_PYTHON}" "$SCRIPT_DIR/convert_onnx_to_rknn_rk3588.py" "$@"
