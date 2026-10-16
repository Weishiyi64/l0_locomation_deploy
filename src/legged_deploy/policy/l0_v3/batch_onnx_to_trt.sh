#!/usr/bin/env bash
# Batch convert missing ONNX engines on the target Jetson.
# Place this script anywhere under the policy tree; it scans its own directory recursively.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TRTEXEC="${TRTEXEC:-}"
if [[ -z "$TRTEXEC" ]]; then
  TRTEXEC="$(command -v trtexec || true)"
fi
if [[ -z "$TRTEXEC" && -x /usr/src/tensorrt/bin/trtexec ]]; then
  TRTEXEC=/usr/src/tensorrt/bin/trtexec
fi
if [[ -z "$TRTEXEC" || ! -x "$TRTEXEC" ]]; then
  echo '[ERROR] trtexec not found. Set TRTEXEC=/absolute/path/to/trtexec' >&2
  exit 1
fi

printf '[INFO] Scan root: %s\n[INFO] trtexec: %s\n' "$ROOT" "$TRTEXEC"
count=0; skipped=0; failed=0; built=0

# -P avoids following symlinked subdirectories; -type f avoids symlinked ONNX inputs.
while IFS= read -r -d '' onnx; do
  ((count+=1))
  case "$onnx" in
    */text_motion_wbc_stg25_dynamic/*)
      echo "[SKIP protected text-motion policy] $onnx"
      ((skipped+=1))
      continue
      ;;
  esac

  engine="${onnx%.onnx}.trt"
  if [[ -s "$engine" ]]; then
    echo "[SKIP existing] $engine"
    ((skipped+=1))
    continue
  fi

  tmp="${engine}.building.$$"
  log="${engine}.build.log"
  echo "[BUILD] $onnx -> $engine"
  rm -f -- "$tmp"
  if "$TRTEXEC" --onnx="$onnx" --saveEngine="$tmp" --fp16 >"$log" 2>&1 && [[ -s "$tmp" ]]; then
    mv -f -- "$tmp" "$engine"
    echo "[OK] $engine"
    ((built+=1))
  else
    echo "[FAILED] $onnx (see $log)" >&2
    rm -f -- "$tmp"
    ((failed+=1))
  fi
done < <(find -P "$ROOT" -type f -name '*.onnx' -print0)

printf '\n[SUMMARY] ONNX=%d  built=%d  skipped=%d  failed=%d\n' "$count" "$built" "$skipped" "$failed"
if (( failed > 0 )); then exit 1; fi
