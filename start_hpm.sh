#!/usr/bin/env bash
set -e

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source install/setup.bash
exec ros2 launch hpm_system hpm_system.launch.py "$@"
