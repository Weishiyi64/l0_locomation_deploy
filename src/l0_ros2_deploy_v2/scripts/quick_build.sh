#!/usr/bin/env bash
set -e

if (( $# != 0 )); then
  printf '用法：bash %s\n' "$0" >&2
  exit 2
fi

if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 ]]; then
  echo '请在 RK3588 机器人的 Linux 终端运行此脚本。' >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir/../../.."
source /opt/ros/humble/setup.bash

bash src/odin_ros_driver_rev1/script/build_ros2.sh

exec colcon build --base-paths src --packages-up-to hpm_system legged_real l0_v5_description \
  --packages-skip odin_ros_driver_rev1 --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --ament-cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
