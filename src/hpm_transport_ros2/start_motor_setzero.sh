#!/usr/bin/env bash
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
set -e

workspace=$(realpath -- "$(dirname -- "${BASH_SOURCE[0]}")/../..")
connect_timeout_s=10
usage() {
    echo "用法：$0 [--server]。整组标零请调用 /set_zero/calibrate Action。"
}

if (($#)); then
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --server) if (($# != 1)); then usage >&2; exit 2; fi ;;
        *) usage >&2; exit 2 ;;
    esac
fi

cd "$workspace"
source install/setup.bash

system_pid=""
calibration_pid=""
system_log=$(mktemp "${TMPDIR:-/tmp}/start_motor_setzero.XXXXXX")
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    # Keep communications alive until calibration has finished its cleanup.
    if [[ -n "$calibration_pid" ]] && kill -0 "$calibration_pid" 2>/dev/null; then
        kill -INT "$calibration_pid" 2>/dev/null || true
        wait "$calibration_pid" || true
    fi
    if [[ -n "$system_pid" ]] && kill -0 "$system_pid" 2>/dev/null; then
        kill -INT "$system_pid" 2>/dev/null || true
        wait "$system_pid" || true
    fi
    rm -f "$system_log"
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "工作区：$workspace，启动 set_zero Action 服务"
# Separate sessions let this wrapper stop calibration before communications.
# tee -i keeps logging alive during Ctrl+C until calibration has stopped.
setsid ros2 launch hpm_system hpm_system.launch.py > >(tee -i "$system_log") 2>&1 &
system_pid=$!

echo "等待板卡 UDP connected，最多 ${connect_timeout_s} 秒……"
connect_attempt=0
while ! grep -Fq '[hpm_system]: UDP connected to ' "$system_log"; do
    if ! kill -0 "$system_pid" 2>/dev/null; then
        echo "通信进程已退出，未启动标零。" >&2
        exit 1
    fi
    if ((connect_attempt >= connect_timeout_s * 10)); then
        echo "等待 UDP connected 超时，未启动标零。" >&2
        exit 1
    fi
    sleep 0.1
    connect_attempt=$((connect_attempt + 1))
done
if ! kill -0 "$system_pid" 2>/dev/null; then
    echo "通信进程已退出，未启动标零。" >&2
    exit 1
fi

setsid ros2 launch set_zero set_zero_server.launch.py &
calibration_pid=$!
wait "$calibration_pid"
