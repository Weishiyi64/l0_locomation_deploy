#!/usr/bin/env bash
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/src/l0_ros2_deploy_v2/scripts/start_sim2real.sh" "$@"
