#!/usr/bin/env bash
# One optional rosbag group per terminal, independent of deployment/actions.
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h || $# == 0 ]]; then
  echo '用法：bash scripts/record_wbc_rosbag.sh A|B|C|D [--workspace 工作空间] [--dry-run]'
  echo 'A 策略输入输出；B 电机与 IMU；C 动作与安全事件；D ROS 日志。'
  echo '每组独立终端；Ctrl+C 停止并写盘。不启动 sim2real/HPM，不发送动作。'
  exit 0
fi
group=$1
shift
source "$(dirname -- "${BASH_SOURCE[0]}")/wbc_runtime_common.bash"
wbc_options "$@"
case "$group" in
  A|a) label=policy; topics=(/control/policy/observation /control/policy/command/rl /control/policy/command/wbc) ;;
  B|b) label=actuator; topics=(/actuator/motor/state /actuator/motor/command /imu/data) ;;
  C|c) label=events; topics=(/algorithm/brain/wbc_action_cmd /system/state_machine/wbc_action_fdb
    /control/policy/wbc/active /control/policy/wbc/done /hardware_status/motors_ready
    /system/safety/emergency_stop /system/safety/hardware_fault /actuator/motor/enable /joy) ;;
  D|d) label=diagnostics; topics=(/rosout) ;;
  *) wbc_die '录包组必须是 A、B、C 或 D' ;;
esac
wbc_current_run
qos="$wbc_workspace/deploy_config/wbc-record-qos.yaml"
[[ -f "$qos" ]] || qos="$wbc_script_dir/wbc-record-qos.yaml"
stamp=$(date +%Y%m%d_%H%M%S_%N)
bag_path="$wbc_run_dir/bags/${label}_$stamp"
cmd=(ros2 bag record --storage sqlite3 --node-name "record_wbc_$label"
  --output "$bag_path" --max-cache-size 16777216 --max-bag-size 268435456
  --max-bag-duration 300 --qos-profile-overrides-path "$qos" "${topics[@]}")
wbc_print_command "${cmd[@]}"
$wbc_dry_run && exit 0
exec 9>"$wbc_run_dir/.record_$label.lock"
flock -n 9 || wbc_die "本次运行的 $label 组已在录制"
mkdir -p -- "$wbc_run_dir/bags"
cp -- "$qos" "$wbc_run_dir/record_${label}_$stamp.qos.yaml"
printf '录包目录：%s\n按 Ctrl+C 停止并等待 metadata 写入。\n' "$bag_path"
if wbc_logged_command "$wbc_run_dir/record_${label}_$stamp.log" "${cmd[@]}"; then result=0; else result=$?; fi
printf '%s\n' "$result" > "$wbc_run_dir/record_${label}_$stamp.exitcode"
[[ -f "$bag_path/metadata.yaml" ]] || wbc_die "录包未正常封存，请检查日志：$bag_path"
printf '录包已停止：%s\n' "$bag_path"
exit "$result"
