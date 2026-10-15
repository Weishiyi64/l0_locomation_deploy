#!/usr/bin/env bash

# Hardware Readiness Check Script (ROS2)
# This script waits for all hardware nodes to publish their ready status
# ROS1 used rostopic echo; ROS2 uses Python rclpy (preferred) or ros2 topic echo

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PREFIX="[READY_CHECK]"

# Ensure ROS2 env when run as child of launch Node (sequential_launch_real.sh)
if [ -z "${ROS_DISTRO}" ]; then
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash 2>/dev/null || true
fi
# Source workspace: SCRIPT_DIR = install/.../lib/legged_controllers, parent = install
_ws_setup="${SCRIPT_DIR}/../../../setup.bash"
if [ -f "${_ws_setup}" ]; then
    source "${_ws_setup}" 2>/dev/null || true
fi

# Timeouts (seconds)
IMU_TIMEOUT=20
MOTORS_TIMEOUT=20
BATTERY_TIMEOUT=20

# Battery enabled flag (passed as argument)
BATTERY_ENABLED="${1:-true}"
IMU_ENABLED="${2:-true}"
# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}${LOG_PREFIX} INFO:${NC} $1"
}

log_success() {
    echo -e "${GREEN}${LOG_PREFIX} SUCCESS:${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}${LOG_PREFIX} WARNING:${NC} $1"
}

log_error() {
    echo -e "${RED}${LOG_PREFIX} ERROR:${NC} $1"
}

# Function to wait for a topic to publish with timeout (ROS2)
# Prefer Python rclpy - ros2 topic echo often fails when check runs after nodes start
wait_for_topic() {
    local topic_name="$1"
    local timeout="$2"
    local description="$3"

    log_info "Waiting for $description ($topic_name)..."

    # Method 1: Python rclpy subscriber (reliable when launched as Node child)
    local python_script="${SCRIPT_DIR}/wait_for_hardware_ready.py"
    if [[ -x "$python_script" || -f "$python_script" ]]; then
        if python3 "$python_script" "$topic_name" "$timeout" "$description" 2>/dev/null; then
            log_success "$description is ready!"
            return 0
        else
            log_error "$description timeout after ${timeout}s"
            return 1
        fi
    fi

    # Method 2: ros2 topic echo fallback (may miss messages if published before subscribe)
    local start_time=$(date +%s)
    local elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        local msg
        msg=$(timeout 5 ros2 topic echo "$topic_name" std_msgs/msg/Bool -n 1 2>/dev/null) || true
        if [[ -n "$msg" ]] && echo "$msg" | grep -qE "data:\s*(true|True)"; then
            log_success "$description is ready!"
            return 0
        fi
        sleep 0.5
        elapsed=$(($(date +%s) - start_time))
        if [[ $((elapsed % 2)) -eq 0 && $elapsed -gt 0 ]]; then
            log_info "Still waiting for $description... (${elapsed}s/${timeout}s)"
        fi
    done
    log_error "$description timeout after ${timeout}s"
    return 1
}

# Main health check
main() {
    log_info "Hardware readiness check starting..."
    echo "========================================================"

    local check_failed=0

    # Check 1: IMU
    if [[ "$IMU_ENABLED" == "true" ]]; then
        if ! wait_for_topic "/hardware_status/imu_ready" "$IMU_TIMEOUT" "IMU node"; then
            check_failed=1
        fi
    else
        log_info "IMU check skipped (imu_enabled=false)"
    fi

    echo ""

    # Check 2: Motors
    if ! wait_for_topic "/hardware_status/motors_ready" "$MOTORS_TIMEOUT" "Motors node"; then
        check_failed=1
    fi

    echo ""

    # Check 3: Battery
    if [[ "$BATTERY_ENABLED" == "true" ]]; then
        if ! wait_for_topic "/hardware_status/battery_ready" "$BATTERY_TIMEOUT" "Battery node"; then
            check_failed=1
        fi
    else
        log_info "Battery check skipped (battery_enabled=false)"
    fi

    echo ""
    echo "========================================================"

    if [[ $check_failed -eq 0 ]]; then
        log_success "All hardware nodes are ready!"
        return 0
    else
        log_error "Hardware readiness check FAILED"
        log_error "One or more hardware nodes did not become ready in time"
        return 1
    fi
}

# Handle script arguments
case "${1:-}" in
    --help|-h)
        echo "Usage: $0 [BATTERY_ENABLED]"
        echo ""
        echo "Arguments:"
        echo "  BATTERY_ENABLED   true or false (default: true)"
        echo ""
        echo "This script waits for all hardware nodes to publish ready status:"
        echo "  - /hardware_status/imu_ready (timeout: ${IMU_TIMEOUT}s)"
        echo "  - /hardware_status/motors_ready (timeout: ${MOTORS_TIMEOUT}s)"
        echo "  - /hardware_status/battery_ready (timeout: ${BATTERY_TIMEOUT}s, optional)"
        exit 0
        ;;
    *)
        main
        exit $?
        ;;
esac
