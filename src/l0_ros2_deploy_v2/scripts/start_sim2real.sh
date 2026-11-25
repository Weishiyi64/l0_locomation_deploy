#!/usr/bin/env bash
# Environment setup + exec only. sim2real owns configuration, preflight and shutdown.
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo '用法：bash scripts/start_sim2real.sh [--workspace 工作空间] [--dry-run] [name:=value ...]'
  echo '配置优先级：命令行 > deploy_config/wbc-production.env > sim2real.launch.py 默认值。'
  echo '--dry-run 只解析配置并检查资产，不运行 launch 或硬件脚本。'
  exit 0
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/wbc_runtime_common.bash"
options=()
launch_args=()
while (($#)); do
  case "$1" in
    --workspace)
      (($# >= 2)) || wbc_die '--workspace 缺少路径'
      options+=("$1" "$2"); shift 2 ;;
    --dry-run) options+=("$1"); shift ;;
    *:=*) launch_args+=("$1"); shift ;;
    *) wbc_die "未知参数：$1" ;;
  esac
done
wbc_options "${options[@]}"
wbc_ros_environment
if $wbc_dry_run; then
  launch_dir=$(ros2 pkg prefix legged_controllers)/share/legged_controllers/launch
  exec /usr/bin/python3 -B "$launch_dir/deployment_config.py" "${launch_args[@]}"
fi
# Replace this shell: no extra session, tee pipe, signal forwarding or child wait.
exec ros2 launch legged_controllers sim2real.launch.py "${launch_args[@]}"
