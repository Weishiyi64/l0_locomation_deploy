#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin="${TEXT_MOTION_PYTHON:-/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python}"

if [[ ! -x "${python_bin}" ]]; then
  python_bin="$(command -v python3)"
fi

if [[ "${1:-}" == "--preview" ]]; then
  shift
  exec "${python_bin}" -B "${script_dir}/preview_motion.py" --ros-sync "$@"
fi

live=false
wbc_input=false
output=""
manual_pose=false
export_only=false
arguments=()
while (( $# )); do
  case "$1" in
    --live)
      live=true
      shift
      ;;
    --wbc-input)
      wbc_input=true
      shift
      ;;
    --output)
      if (( $# < 2 )); then
        echo "--output 需要文件路径" >&2
        exit 2
      fi
      output="$2"
      arguments+=("$1" "$2")
      shift 2
      ;;
    --export-fp16)
      if (( $# < 2 )); then echo "--export-fp16 需要输出模型路径" >&2; exit 2; fi
      export_only=true
      arguments+=("$1" "$2")
      shift 2
      ;;
    --start-pose|--end-pose)
      manual_pose=true
      if (( $# < 2 )); then
        echo "$1 需要文件路径或 JSON 数组" >&2
        exit 2
      fi
      arguments+=("$1" "$2")
      shift 2
      ;;
    *)
      arguments+=("$1")
      shift
      ;;
  esac
done

if [[ "$live" == true || "$wbc_input" == true ]]; then
  if [[ "$live" == true && "$wbc_input" == true ]]; then
    echo "--live 和 --wbc-input 只能选一个" >&2
    exit 2
  fi
  if [[ "$export_only" == true ]]; then
    echo "自动采集姿态不能和 --export-fp16 同时使用" >&2
    exit 2
  fi
  if [[ "$manual_pose" == true ]]; then
    echo "自动采集姿态会传入 start/end pose；不要手动指定姿态" >&2
    exit 2
  fi
  if [[ -z "$output" ]]; then
    echo "自动采集姿态需要 --output" >&2
    exit 2
  fi
  pose_file="$(mktemp /tmp/l0_text_pose_XXXXXX.json)"
  trap 'rm -f -- "$pose_file"' EXIT
  /usr/bin/python3 "${script_dir}/capture_init_pose.py" --output "$pose_file"
  "${python_bin}" -B "${script_dir}/text_motion_onnx.py" "${arguments[@]}" \
    --start-pose "$pose_file" --end-pose "$pose_file"
  if [[ "$live" == true ]]; then
    /usr/bin/python3 "${script_dir}/text_motion_publish.py" --input "$output"
  else
    echo "WBC 输入 JSON 已生成；未通过文本叠加控制器播放"
  fi
else
  exec "${python_bin}" -B "${script_dir}/text_motion_onnx.py" "${arguments[@]}"
fi
