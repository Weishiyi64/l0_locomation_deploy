#!/bin/bash

# Shared process-group lifecycle for the real and simulation sequential launchers.
MANAGED_PIDS=()
MANAGED_NAMES=()
STARTED_PID=""
CLEANUP_STARTED=false
SIGINT_GRACE_SEC=3
SIGTERM_GRACE_SEC=2

function process_group_alive() {
    local pid="$1"
    [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null
}

function signal_process_group() {
    local pid="$1"
    local signal="$2"
    kill "-$signal" -- "-$pid" 2>/dev/null || true
}

function wait_for_process_group_exit() {
    local pid="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))

    while process_group_alive "$pid" && (( SECONDS < deadline )); do
        sleep 0.2
    done

    ! process_group_alive "$pid"
}

function start_managed_process() {
    local name="$1"
    shift

    setsid "$@" &
    STARTED_PID=$!
    MANAGED_PIDS+=("$STARTED_PID")
    MANAGED_NAMES+=("$name")
}

function cleanup() {
    local exit_code=$?

    if [[ "$CLEANUP_STARTED" == "true" ]]; then
        return "$exit_code"
    fi
    CLEANUP_STARTED=true
    trap '' INT TERM
    trap - EXIT

    if [[ ${#MANAGED_PIDS[@]} -gt 0 ]]; then
        echo ""
        echo "Cleaning up launched ROS process groups..."
        echo "Repeated Ctrl+C/SIGTERM will be ignored until cleanup completes."
    fi

    for ((idx=${#MANAGED_PIDS[@]}-1; idx>=0; idx--)); do
        local pid="${MANAGED_PIDS[$idx]}"
        local name="${MANAGED_NAMES[$idx]}"

        if process_group_alive "$pid"; then
            echo "Stopping $name (PGID: $pid) with SIGINT..."
            signal_process_group "$pid" INT
            if ! wait_for_process_group_exit "$pid" "$SIGINT_GRACE_SEC"; then
                echo "Stopping $name (PGID: $pid) with SIGTERM..."
                signal_process_group "$pid" TERM
                if ! wait_for_process_group_exit "$pid" "$SIGTERM_GRACE_SEC"; then
                    echo "Force stopping $name (PGID: $pid) with SIGKILL..."
                    signal_process_group "$pid" KILL
                    wait_for_process_group_exit "$pid" 2 || true
                fi
            fi
        fi

        wait "$pid" 2>/dev/null || true
    done

    return "$exit_code"
}

trap cleanup EXIT
trap 'cleanup; exit 0' INT
trap 'cleanup; exit 0' TERM
