#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash compile.sh [--output NEW_DIRECTORY] [--build-only] [--dry-run]'
  echo 'Input: manifest.json beside this script (handoff), or project onnx/manifest.json.'
  echo 'Default output: compiled/ beside this script.'
  echo 'Requires rknn-toolkit2==2.3.2. Set RKNN_PYTHON to its Python interpreter.'
  echo 'The onnx/ bundle contains the compiler; training project/checkpoints are not required.'
  echo 'Board compilation only: never initializes a simulator or inference runtime.'
  echo 'On the supplied Python 3.10.12 board environment: RKNN_PYTHON=python3.10 bash compile.sh'
  exit 0
fi
DEFAULT_PYTHON=python3
export RKNN_WORK_DIR="${RKNN_WORK_DIR:-$SCRIPT_DIR/../tmp/deploy_onnx_rknn}"
exec "${RKNN_PYTHON:-$DEFAULT_PYTHON}" "$SCRIPT_DIR/convert_onnx_to_rknn_rk3588.py" "$@"
