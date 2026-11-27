#!/usr/bin/env bash
# Run on the board in the same ROS workspace as sim2real.
set -eo pipefail
if [[ ${1:-} == --help ]]; then
  echo '用法：record_crouch_off.sh [--workspace 工作空间]；就绪后在动作控制台选择 L0_crouch_off。'
  exit 0
fi
dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
source "$dir/wbc_runtime_common.bash"
wbc_options "$@"
wbc_current_run
[[ "$robot_type" == l0_v5 ]] || wbc_die '录制仅适用于 L0 V5' 
exec /usr/bin/python3 -B "$dir/record_crouch_off.py" "$wbc_run_dir"
