#!/usr/bin/env bash
# Shared environment/path checks only; sourcing this file starts no ROS nodes.

wbc_die() { printf '错误：%s\n' "$*" >&2; exit 1; }

wbc_options() {
  wbc_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
  wbc_workspace=$(cd -- "$wbc_script_dir/../../.." && pwd -P)
  wbc_dry_run=false
  while (($#)); do
    case "$1" in
      --workspace)
        (($# >= 2)) || wbc_die '--workspace 缺少路径'
        wbc_workspace=$(cd -- "$2" && pwd -P) || exit 1
        shift 2 ;;
      --dry-run) wbc_dry_run=true; shift ;;
      *) wbc_die "未知参数：$1；使用 --help 查看用法" ;;
    esac
  done
  wbc_repo="$wbc_workspace/src/l0_ros2_deploy_v2"
  wbc_run_root="$wbc_workspace/runs/wbc_production"
  export PYTHONDONTWRITEBYTECODE=1
}

wbc_ros_environment() {
  [[ -f /opt/ros/humble/setup.bash ]] || wbc_die '缺少 ROS Humble 环境'
  [[ -f "$wbc_workspace/install/setup.bash" ]] || wbc_die '工作空间尚未构建'
  source /opt/ros/humble/setup.bash
  source "$wbc_workspace/install/setup.bash"
  wbc_prefix=$(ros2 pkg prefix legged_deploy)
  [[ "$(realpath "$wbc_prefix")" == "$(realpath "$wbc_workspace/install")/"* ]] ||
    wbc_die 'legged_deploy 来自其他工作空间'
}

wbc_environment() {
  # Read an already resolved run snapshot, never a second user configuration.
  local config=$1
  wbc_ros_environment
  [[ -f "$config" ]] || wbc_die "缺少运行快照：$config"
  unset robot_type amp_policy wbc_bundle wbc_actions wbc_mode wbc_enabled
  source "$config"
  wbc_mode=${wbc_mode:-lora}
  [[ ${wbc_enabled:-true} == true ]] || wbc_die '本次运行未启用 WBC'
  [[ $wbc_mode == base || $wbc_mode == lora ]] || wbc_die '此控制台/录包入口需要 base 或 lora 包'
  [[ ${robot_type:-} =~ ^[A-Za-z0-9_-]+$ ]] || wbc_die 'robot_type 无效'
  [[ ${amp_policy:-} =~ ^[A-Za-z0-9_-]+$ ]] || wbc_die 'amp_policy 无效'
  [[ ${wbc_bundle:-} =~ ^[A-Za-z0-9_-]+$ ]] || wbc_die 'wbc_bundle 无效'
  [[ -n ${wbc_actions:-} ]] || wbc_die 'wbc_actions 为空'
  wbc_installed="$wbc_prefix/share/legged_deploy/policy/$robot_type/$wbc_bundle"
  wbc_actions=$(/usr/bin/python3 -B - "$wbc_installed" "$wbc_actions" "$wbc_mode" <<'PY'
import json, re, sys
from pathlib import Path
import yaml
root = Path(sys.argv[1])
selected = yaml.safe_load(sys.argv[2])
manifest = json.loads((root / 'base_manifest.json').read_text())
expected = 'wbc_base_graph_v1' if sys.argv[3] == 'base' else 'wbc_lora_graph_v1'
if manifest.get('format') != expected:
    raise SystemExit('模式与部署包不一致：' + expected)
if selected == 'all':
    selected = [a['action_id'] for a in manifest['actions']]
if (not isinstance(selected, list) or not selected or
        any(not isinstance(a, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', a) for a in selected) or
        len(selected) != len(set(selected))):
    raise SystemExit('wbc_actions 必须是非空、无重复的动作 ID 列表')
available = {a['action_id'] for a in manifest['actions']}
if not set(selected) <= available:
    raise SystemExit('配置包含未安装动作：' + repr(set(selected) - available))
for name in ('INSTALLATION.json', manifest['model']['rknn']):
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise SystemExit('缺少运行资产：' + str(root / name))
print(json.dumps(selected))
PY
  ) || wbc_die '模型配置检查失败'
}

wbc_current_run() {
  if $wbc_dry_run && [[ ! -f "$wbc_run_root/current_run_dir" ]]; then
    wbc_ros_environment
    local preview
    preview=$(bash "$wbc_script_dir/start_sim2real.sh" --workspace "$wbc_workspace" --dry-run) || return 1
    mapfile -t fields < <(/usr/bin/python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["robot_type"], d["wbc_lora_bundle"], d["wbc_policies"], sep="\n")' <<< "$preview")
    robot_type=${fields[0]}; wbc_bundle=${fields[1]}; wbc_actions=${fields[2]}
    wbc_installed="$wbc_prefix/share/legged_deploy/policy/$robot_type/$wbc_bundle"
    wbc_run_dir="$wbc_run_root/preview"
    return
  fi
  wbc_run_dir=$(/usr/bin/python3 -B - "$wbc_run_root" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
run = Path((root / 'current_run_dir').read_text().strip()).resolve(strict=True)
if run.parent != root or not (run / 'environment.env').is_file():
    raise SystemExit('无效的 current_run_dir；先启动 sim2real.launch.py')
if (run / 'LOGS.sha256').exists():
    raise SystemExit('本次记录已封存；请重新启动生成新的运行目录')
print(run)
PY
  ) || wbc_die '找不到可写的当前运行记录'
  # Both direct ros2 launch and the wrapper create this snapshot before starting nodes.
  wbc_environment "$wbc_run_dir/environment.env"
  cmp -s "$wbc_run_dir/base_manifest.json" "$wbc_installed/base_manifest.json" ||
    wbc_die '安装 manifest 与运行快照不一致，请重新启动部署'
  cmp -s "$wbc_run_dir/INSTALLATION.json" "$wbc_installed/INSTALLATION.json" ||
    wbc_die '安装清单与运行快照不一致，请重新启动部署'
  export ROS_LOG_DIR="$wbc_run_dir/ros"
}

wbc_print_command() { printf '%q ' "$@"; printf '\n'; }

wbc_logged_command() {
  local log=$1 child logger log_fd result
  shift
  exec {log_fd}> >(trap '' INT TERM; exec tee "$log")
  logger=$!
  setsid "$@" >&$log_fd 2>&1 &
  child=$!
  exec {log_fd}>&-
  # Let ROS and its existing sequential launcher shut down their own children.
  trap 'kill -INT -- "-$child" 2>/dev/null || true' INT TERM
  while true; do
    if wait "$child"; then result=0; else result=$?; fi
    kill -0 "$child" 2>/dev/null || break
  done
  wait "$logger" || true
  trap - INT TERM
  return "$result"
}
