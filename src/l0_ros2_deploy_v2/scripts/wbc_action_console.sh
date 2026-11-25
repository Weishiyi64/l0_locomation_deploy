#!/usr/bin/env bash
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo '用法：bash scripts/wbc_action_console.sh [--workspace 工作空间] [--dry-run]'
  echo '输入编号或完整动作 ID 发布一次请求；list 显示列表，q 退出。'
  echo '读取 sim2real 的实际运行快照；支持直接 ros2 launch 和 start_sim2real.sh。'
  echo '--dry-run 仅显示动作，不创建 ROS 节点或发布消息。'
  exit 0
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/wbc_runtime_common.bash"
wbc_options "$@"
wbc_current_run
args=(--bundle "$wbc_installed" --actions "$wbc_actions" --run-dir "$wbc_run_dir")
if $wbc_dry_run; then
  args+=(--dry-run)
else
  exec 9>"$wbc_run_dir/.action_console.lock"
  flock -n 9 || wbc_die '本次运行已打开一个动作控制台'
fi
exec /usr/bin/python3 -B "$wbc_script_dir/wbc_action_console.py" "${args[@]}"
