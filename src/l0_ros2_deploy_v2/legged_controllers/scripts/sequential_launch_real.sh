#!/bin/bash

# Sequential launcher script for real hardware
# Start HPM separately before this script. This script runs the hardware bridge,
# waits for readiness, and then starts the main controller.
#
# joint_csv_blend_sec: per-sequence YAML sequence.joint_csv_blend_sec (LeggedController).

function showHelp(){
    echo
    echo "This script expects HPM to be running, then starts the hardware bridge and main controller"
    echo "Each step waits for the previous one to complete"
    echo
    echo "Usage: sequential_launch_real.sh [robot_type] [bag_enabled] [subpolicy] [bag_name] [rviz] [battery_enabled] [imu_enabled] [head_script_enabled] [head_script_config] [joystick_imitation_enabled] [joystick_imitation_config_path] [arm_swing_enabled] [arm_swing_stop_vx_threshold] [arm_swing_high_vx_threshold] [arm_swing_vx_hysteresis] [arm_swing_blend_sec] [safety_orientation_roll_limit_deg] [safety_orientation_pitch_limit_deg] [safety_orientation_consecutive_violation_count] [joystick_sequence_enabled] [joystick_sequence_config_path] [joystick_back_emergency_sequence_enabled] [joystick_back_emergency_sequence_path] [emergency_damping_kd] [radar_control_enabled] [nav_planner_model_path] [planner_depth_input_enabled] [raw_depth_preview_enabled] [wbc_enabled] [wbc_policies] [wbc_mode] [wbc_lora_bundle] [hpm_mode]"
    echo "Body joy is fixed at /dev/input/js0 (see joy_teleop.launch.py). Defaults live in legged_controllers/launch/sim2real.launch.py — use:"
    echo "  ros2 launch legged_controllers sim2real.launch.py"
    echo "Optional final positional argument after hpm_mode: text_motion_enabled (default false)."
    echo "Example (manual): sequential_launch_real.sh l0_v4 false l0_v3_amp_07090 l0_v4_real false true true false /path/head.yaml true /path/to/joystick_imitation_actions.yaml true 0.05 0.8 0.03 0.35 30.0 30.0 3 false /path/to/joystick_sequence_actions.yaml true /path/to/walking_back_emergency.yaml 5.0 true /path/planner_09090.onnx false true"
    echo
}

if [ "$1" = "-h" ]; then
    showHelp
    exit 0
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MANAGED_PIDS=()
MANAGED_NAMES=()
STARTED_PID=""
CLEANUP_STARTED=false
CLEANUP_FAILED=false
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

        # SIGKILL cannot release a task blocked in an uninterruptible kernel wait.
        # Never follow the bounded group shutdown with an unbounded wait on it.
        local state
        state=$(ps -o stat= -p "$pid" 2>/dev/null) || state=""
        if [[ -z "$state" || "$state" == Z* ]]; then
            wait "$pid" 2>/dev/null || true
        else
            echo "ERROR: $name PID=$pid remains after shutdown deadline (state=$state); not waiting indefinitely" >&2
            CLEANUP_FAILED=true
        fi
        if process_group_alive "$pid"; then
            echo "ERROR: process group $pid still exists after shutdown deadline" >&2
            CLEANUP_FAILED=true
        fi
    done

    if [[ "$CLEANUP_FAILED" == true ]]; then return 1; fi
    return "$exit_code"
}

trap 'cleanup; exit $?' EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# Parse arguments.
SCRIPT_ARGS=()
while [[ $# -gt 0 ]]; do
    if [[ "$1" == "--ros-args" ]]; then
        break
    fi
    SCRIPT_ARGS+=("$1")
    shift
done
set -- "${SCRIPT_ARGS[@]}"

if [[ $# -ne 19 && $# -ne 21 && $# -ne 23 && $# -ne 24 && $# -ne 25 && $# -ne 26 && $# -ne 28 && $# -ne 29 && $# -ne 30 && $# -ne 33 && $# -ne 34 ]]; then
    echo "ERROR: sequential_launch_real.sh requires 19, 21, 23, 24, 25, 26, 28, 29, 30, 33, or 34 positional arguments (got $#)."
    echo "Launch via: ros2 launch legged_controllers sim2real.launch.py"
    echo "Or run: $0 -h"
    exit 1
fi

ROBOT_TYPE=$1
BAG_ENABLED=$2
SUBPOLICY=$3
BAG_NAME=$4
RVIZ=$5
BATTERY_ENABLED=$6
IMU_ENABLED=$7
HEAD_SCRIPT_ENABLED=$8
HEAD_SCRIPT_CONFIG=$9
JOYSTICK_IMITATION_ENABLED=${10}
JOYSTICK_IMITATION_CONFIG_PATH=${11}
ARM_SWING_ENABLED=${12}
ARM_SWING_STOP_VX_THRESHOLD=${13}
ARM_SWING_HIGH_VX_THRESHOLD=${14}
ARM_SWING_VX_HYSTERESIS=${15}
ARM_SWING_BLEND_SEC=${16}
SAFETY_ORIENTATION_ROLL_LIMIT_DEG=${17}
SAFETY_ORIENTATION_PITCH_LIMIT_DEG=${18}
SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT=${19}
JOYSTICK_SEQUENCE_ENABLED=${20:-"false"}
JOYSTICK_SEQUENCE_CONFIG_PATH=${21:-""}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED=${22:-"true"}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH=${23:-"action_sequences/walking_back_emergency.yaml"}
EMERGENCY_DAMPING_KD=${24:-"5.0"}
RADAR_CONTROL_ENABLED=${25:-"false"}
NAV_PLANNER_MODEL_PATH=${26:-""}
PLANNER_DEPTH_INPUT_ENABLED=${27:-"false"}
RAW_DEPTH_PREVIEW_ENABLED=${28:-"false"}
WBC_ENABLED=${29:-"false"}
WBC_POLICIES=${30:-"[]"}
WBC_MODE=${31:-"static"}
WBC_LORA_BUNDLE=${32:-"wbc_lora"}
HPM_MODE=${33:-"real"}
TEXT_MOTION_ENABLED=${34:-"false"}
if [[ "$HPM_MODE" != "real" && "$HPM_MODE" != "isolated" ]]; then
    echo "ERROR: hpm_mode must be real or isolated"
    exit 1
fi
IMU_READY_TOPIC=/imu/data
if [[ "$HPM_MODE" == "isolated" ]]; then
    IMU_READY_TOPIC=/wbc_isolated/imu/data
fi

if [[ "$RADAR_CONTROL_ENABLED" == "true" && -z "$NAV_PLANNER_MODEL_PATH" ]]; then
    echo "ERROR: radar/planner control requires nav_planner_model_path."
    echo "Launch via sim2real.launch.py or provide nav_planner_model_path."
    exit 1
fi
if [[ "$RADAR_CONTROL_ENABLED" == "true" && ! -f "$NAV_PLANNER_MODEL_PATH" ]]; then
    echo "ERROR: radar/planner is enabled, but no planner policy was found for robot_type=$ROBOT_TYPE: $NAV_PLANNER_MODEL_PATH"
    exit 1
fi

echo "=========================================="
echo "Sequential Launch Starting (Real Hardware)"
echo "Robot Type: $ROBOT_TYPE"
echo "Subpolicy: $SUBPOLICY"
echo "Bag Enabled: $BAG_ENABLED"
echo "Bag Name: $BAG_NAME"
echo "RViz: $RVIZ"
echo "Battery: $BATTERY_ENABLED"
echo "IMU Enabled: $IMU_ENABLED"
echo "Head Script Enabled: $HEAD_SCRIPT_ENABLED"
echo "Head Script Config: $HEAD_SCRIPT_CONFIG"
echo "Joystick Imitation Enabled: $JOYSTICK_IMITATION_ENABLED"
echo "Joystick Imitation Config Path: $JOYSTICK_IMITATION_CONFIG_PATH"
echo "Arm Swing: enabled=$ARM_SWING_ENABLED stop_vx=$ARM_SWING_STOP_VX_THRESHOLD high_vx=$ARM_SWING_HIGH_VX_THRESHOLD blend=$ARM_SWING_BLEND_SEC"
echo "Joystick Sequence Enabled: $JOYSTICK_SEQUENCE_ENABLED"
echo "Joystick Sequence Config Path: $JOYSTICK_SEQUENCE_CONFIG_PATH"
echo "Joystick Back Emergency Sequence Enabled: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED"
echo "Joystick Back Emergency Sequence Path: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH"
echo "Emergency Damping Kd: $EMERGENCY_DAMPING_KD"
echo "WBC Enabled: $WBC_ENABLED"
echo "WBC Policies: $WBC_POLICIES"
echo "Safety Orientation Roll Limit Deg: $SAFETY_ORIENTATION_ROLL_LIMIT_DEG"
echo "Safety Orientation Pitch Limit Deg: $SAFETY_ORIENTATION_PITCH_LIMIT_DEG"
echo "Safety Orientation Consecutive Violation Count: $SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT"
echo "Radar Control Enabled: $RADAR_CONTROL_ENABLED"
echo "Navigation Planner Model: $NAV_PLANNER_MODEL_PATH"
echo "Planner Depth Input Enabled: $PLANNER_DEPTH_INPUT_ENABLED"
echo "Raw Depth Preview Enabled: $RAW_DEPTH_PREVIEW_ENABLED"
echo "HPM mode: $HPM_MODE; WBC mode: $WBC_MODE"
echo "=========================================="

# Start first and stop last, so the adapter can deliver its normal shutdown disable.
if [[ "$HPM_MODE" == "isolated" ]]; then
    start_managed_process "isolated HPM endpoint" ros2 launch legged_real isolated_hpm.launch.py \
        "robot_type:=$ROBOT_TYPE" "subpolicy:=$SUBPOLICY"
fi

# Step 1: Launch Hardware Bridge
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 1: Launching hardware bridge...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"
echo "Starting legged_real with robot_type=$ROBOT_TYPE subpolicy:=$SUBPOLICY rviz:=$RVIZ imu_enabled:=$IMU_ENABLED"
echo "Starting legged_real with head_script_enabled=$HEAD_SCRIPT_ENABLED head_script_config=$HEAD_SCRIPT_CONFIG"

# Launch hardware bridge in background and wait for it to be ready
# Pass robot_type, subpolicy, rviz, and head config.
start_managed_process "hardware bridge" ros2 launch legged_real legged_real.launch.py \
    robot_type:=$ROBOT_TYPE \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    text_motion_enabled:=$TEXT_MOTION_ENABLED \
    "hpm_mode:=$HPM_MODE" \
    rviz:=$RVIZ \
    imu_enabled:=$IMU_ENABLED \
    head_script_enabled:=$HEAD_SCRIPT_ENABLED \
    head_script_config:=$HEAD_SCRIPT_CONFIG \
    joystick_imitation_enabled:=$JOYSTICK_IMITATION_ENABLED \
    joystick_imitation_config_path:=$JOYSTICK_IMITATION_CONFIG_PATH \
    joystick_sequence_enabled:=$JOYSTICK_SEQUENCE_ENABLED \
    joystick_sequence_config_path:=$JOYSTICK_SEQUENCE_CONFIG_PATH \
    joystick_back_emergency_sequence_enabled:=$JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED \
    joystick_back_emergency_sequence_path:=$JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH \
    arm_swing_enabled:=$ARM_SWING_ENABLED \
    arm_swing_stop_vx_threshold:=$ARM_SWING_STOP_VX_THRESHOLD \
    arm_swing_high_vx_threshold:=$ARM_SWING_HIGH_VX_THRESHOLD \
    arm_swing_vx_hysteresis:=$ARM_SWING_VX_HYSTERESIS \
    arm_swing_blend_sec:=$ARM_SWING_BLEND_SEC \
    safety_orientation_roll_limit_deg:=$SAFETY_ORIENTATION_ROLL_LIMIT_DEG \
    safety_orientation_pitch_limit_deg:=$SAFETY_ORIENTATION_PITCH_LIMIT_DEG \
    safety_orientation_consecutive_violation_count:=$SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT \
    emergency_damping_kd:=$EMERGENCY_DAMPING_KD \
    radar_control_enabled:=$RADAR_CONTROL_ENABLED
HW_BRIDGE_PID=$STARTED_PID

# Wait a bit for hardware bridge to initialize
echo "Waiting for hardware bridge to initialize..."
sleep 3

# Check if hardware bridge is still running
if process_group_alive "$HW_BRIDGE_PID"; then
    echo "Hardware bridge started successfully (PID: $HW_BRIDGE_PID)"
else
    echo "WARNING: Hardware bridge may have failed to start"
fi

# Launch battery monitor if enabled
if [[ "$BATTERY_ENABLED" == "true" ]]; then
    echo ""
    echo "Launching battery monitor..."
    echo "----------------------------------------"
    start_managed_process "battery monitor" ros2 launch kvms_battery battery.launch.py
    BATTERY_PID=$STARTED_PID

    # Wait for battery node to initialize
    sleep 1

    # Check if battery node is still running
    if process_group_alive "$BATTERY_PID"; then
        echo "Battery monitor started successfully (PID: $BATTERY_PID)"
    else
        echo "WARNING: Battery monitor may have failed to start"
    fi
else
    echo ""
    echo "Battery monitor disabled - skipping"
fi

# Find the hardware readiness check script
READY_CHECK_SCRIPT="$SCRIPT_DIR/check_hardware_ready.bash"

if [[ -f "$READY_CHECK_SCRIPT" ]]; then
    echo "Running hardware readiness check: $READY_CHECK_SCRIPT"

    # Run the check with battery and imu enabled flag
    # Waiting on an owned background job is interruptible in Bash. A foreground
    # readiness command can otherwise defer the launcher's cleanup trap.
    start_managed_process "hardware readiness check" bash "$READY_CHECK_SCRIPT" "$BATTERY_ENABLED" "$IMU_ENABLED" "$IMU_READY_TOPIC"
    READY_CHECK_PID=$STARTED_PID
    if wait "$READY_CHECK_PID"; then
        echo "SUCCESS: All hardware nodes are ready!"
        echo "Hardware readiness check completed."
    else
        echo "ERROR: Hardware readiness check failed"
        echo "Shutting down launched nodes..."
        cleanup
        echo "Cannot proceed without all hardware nodes ready - EXITING"
        exit 1
    fi
else
    echo "WARNING: Hardware readiness check script not found at $READY_CHECK_SCRIPT"
    echo "Proceeding without readiness verification (NOT RECOMMENDED)"
fi

echo ""

# Step 2: Launch Main Controller
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 2: Launching main controller...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"
echo "Starting one_start.launch with:"
echo "  robot_type: $ROBOT_TYPE"
echo "  bag_enabled: $BAG_ENABLED"
echo "  subpolicy: $SUBPOLICY"
echo "  bag_name: $BAG_NAME"
echo "  rviz: $RVIZ"
echo "  joystick_imitation_enabled: $JOYSTICK_IMITATION_ENABLED  joystick_imitation_config_path: $JOYSTICK_IMITATION_CONFIG_PATH"
echo "  joystick_sequence_enabled: $JOYSTICK_SEQUENCE_ENABLED  joystick_sequence_config_path: $JOYSTICK_SEQUENCE_CONFIG_PATH"
echo "  joystick_back_emergency_sequence_enabled: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED  joystick_back_emergency_sequence_path: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH"
echo "  radar_control_enabled: $RADAR_CONTROL_ENABLED"
echo "  nav_planner_model_path: $NAV_PLANNER_MODEL_PATH"
echo "  planner_depth_input_enabled: $PLANNER_DEPTH_INPUT_ENABLED"
echo "  raw_depth_preview_enabled: $RAW_DEPTH_PREVIEW_ENABLED"

# Launch main controller and wait for it while keeping cleanup control over all child launches
start_managed_process "main controller" ros2 launch legged_controllers one_start.launch.py \
    robot_type:=$ROBOT_TYPE \
    bag_enabled:=$BAG_ENABLED \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    "wbc_policies:=$WBC_POLICIES" \
    "wbc_mode:=$WBC_MODE" \
    "wbc_lora_bundle:=$WBC_LORA_BUNDLE" \
    bag_name:=$BAG_NAME \
    rviz:=$RVIZ \
    joystick_imitation_enabled:=$JOYSTICK_IMITATION_ENABLED \
    joystick_imitation_config_path:=$JOYSTICK_IMITATION_CONFIG_PATH \
    joystick_sequence_enabled:=$JOYSTICK_SEQUENCE_ENABLED \
    joystick_sequence_config_path:=$JOYSTICK_SEQUENCE_CONFIG_PATH \
    radar_control_enabled:=$RADAR_CONTROL_ENABLED \
    nav_planner_model_path:=$NAV_PLANNER_MODEL_PATH \
    planner_depth_input_enabled:=$PLANNER_DEPTH_INPUT_ENABLED \
    raw_depth_preview_enabled:=$RAW_DEPTH_PREVIEW_ENABLED
CONTROLLER_PID=$STARTED_PID

CONTROLLER_EXIT=0
wait "$CONTROLLER_PID" || CONTROLLER_EXIT=$?

echo ""
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSequential launch completed (Real Hardware)\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

exit "$CONTROLLER_EXIT"
