#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 || ( $# -eq 2 && "$2" != "--dry-run" ) ]]; then
  echo "用法: bash $0 <机器人IPv4地址> [--dry-run]" >&2
  exit 2
fi

robot_ip="$1"
if [[ ! "$robot_ip" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  echo "无效的 IPv4 地址: $robot_ip" >&2
  exit 2
fi
IFS=. read -r -a octets <<< "$robot_ip"
for octet in "${octets[@]}"; do
  if (( 10#$octet > 255 )); then
    echo "无效的 IPv4 地址: $robot_ip" >&2
    exit 2
  fi
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_root="$(cd "$script_dir/../.." && pwd)"
sources=()
for repo in l0_ros2_deploy_v2 luvbot_ros_interfaces odin_ros_driver_rev1 hpm_transport_ros2; do
  if [[ ! -d "$source_root/$repo" ]]; then
    echo "源码目录不存在: $source_root/$repo" >&2
    exit 1
  fi
  # Keep each directory name so --delete stays inside the listed repositories.
  sources+=("$source_root/$repo")
done

hpm_legacy_interfaces="$source_root/hpm_transport_ros2/src/robot_interfaces"
if [[ -f "$hpm_legacy_interfaces/package.xml" && ! -f "$hpm_legacy_interfaces/COLCON_IGNORE" ]]; then
  echo "HPM 仍会构建自带的 robot_interfaces，与部署包重名，请先完成共享接口迁移。" >&2
  exit 1
fi

options=(
  -avzP --delete
  # Keep any existing HPM Git checkout metadata on the robot.
  '--filter=H .git'
  '--filter=P /hpm_transport_ros2/.git'
  '--filter=P /hpm_transport_ros2/.git/***'
  '--filter=R **/.git/***'
)
# Deploy only l0_v5 assets; keep every robot's assets in the local source tree.
for resource_dir in "$source_root/l0_ros2_deploy_v2/legged_description/"* \
                    "$source_root/l0_ros2_deploy_v2/legged_deploy/policy/"*; do
  [[ -d "$resource_dir" ]] || continue
  case "$resource_dir" in
    */legged_description/l0_v5_description|*/legged_deploy/policy/l0_v5) continue ;;
  esac
  relative_dir="${resource_dir#"$source_root/"}"
  options+=("--filter=H /$relative_dir" "--filter=R /$relative_dir/***")
done
options+=(
  # Keep board-built SDK/plug-in shared libraries. Ship the vendor OpenSSL archives and headers.
  '--exclude=/odin_ros_driver_rev1/module/sdk_api/lib/aarch64/*.so*'
  --exclude=.DS_Store
  --exclude=.vscode/
  --exclude=.idea/
  --exclude=__pycache__/
  '--exclude=*.pyc'
  '--exclude=*.pyo'
  --exclude=build/
  --exclude=install/
  --exclude=log/
  # These HPM files are generated locally or collected on the robot.
  '--exclude=/hpm_transport_ros2/**/compile_commands.json'
  --exclude=/hpm_transport_ros2/.ota_deploy_staging/
  --exclude=/hpm_transport_ros2/.sync_backups/
  --exclude=/hpm_transport_ros2/motor_log/
  --exclude=/hpm_transport_ros2/set_zero_logs/
  '--exclude=/hpm_transport_ros2/set_zero_capture_*/'
  '--exclude=/hpm_transport_ros2/set_zero_reenable_backup_*/'
)
if [[ "${2:-}" == "--dry-run" ]]; then
  options+=(--dry-run --itemize-changes)
fi

# Bundle links in src must have their target on the receiving board first.
ws_root="$(dirname "$source_root")"
dest="/userdata_b/forlinx/stop_workspace/l0_deploy_v2_ws"
extra_options=()
[[ "${2:-}" != --dry-run ]] || extra_options+=(--dry-run --itemize-changes)
for directory in assets zy_assets; do
  [[ -d "$ws_root/$directory" ]] || continue
  rsync -avz "${extra_options[@]}" "$ws_root/$directory/" "forlinx@${robot_ip}:$dest/$directory/"
done
rsync "${options[@]}" "${sources[@]}" \
  "forlinx@${robot_ip}:/userdata_b/forlinx/stop_workspace/l0_deploy_v2_ws/src/"
