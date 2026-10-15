#!/bin/bash

# Sequential launcher script for real hardware
# This script runs tasks in sequence: CAN setup -> Hardware bridge -> Main controller
#
# joint_csv_blend_sec: per-sequence YAML sequence.joint_csv_blend_sec (LeggedController).

function showHelp(){
    echo
    echo "This script runs CAN setup, then hardware bridge, then main controller in sequence"
    echo "Each step waits for the previous one to complete"
    echo
    echo "Usage: sequential_launch_real.sh [robot_type] [bag_enabled] [subpolicy] [bag_name] [rviz] [battery_enabled] [imu_enabled] [head_script_enabled] [head_script_config] [head_topic_config] [emotion_sequence_enabled] [emotion_sequence_map_path] [keyboard_action_enabled] [keyboard_action_config_path] [joystick_imitation_enabled] [joystick_imitation_config_path] [arm_swing_enabled] [arm_swing_stop_vx_threshold] [arm_swing_high_vx_threshold] [arm_swing_vx_hysteresis] [arm_swing_blend_sec] [safety_orientation_roll_limit_deg] [safety_orientation_pitch_limit_deg] [safety_orientation_consecutive_violation_count] [joystick_sequence_enabled] [joystick_sequence_config_path] [joystick_back_emergency_sequence_enabled] [joystick_back_emergency_sequence_path] [emergency_damping_kd] [policy_root_path] [imitation_entry_blend_sec]"
    echo "Body joy is fixed at /dev/input/js0 (see joy_teleop.launch.py). Defaults live in legged_controllers/launch/sim2real.launch.py — use:"
    echo "  ros2 launch legged_controllers sim2real.launch.py"
    echo "Example (manual): sequential_launch_real.sh l0_v3 false l0_v3_amp_07090 l0_v3_amp_07090_real false true true false /path/head.yaml /path/head_topic.yaml false '' true /path/to/keyboard_action_map.yaml true /path/to/joystick_imitation_actions.yaml true 0.05 0.8 0.03 0.35 30.0 30.0 3 false /path/to/joystick_sequence_actions.yaml true /path/to/walking_back_emergency.yaml 5.0"
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

if [[ $# -ne 24 && $# -ne 26 && $# -ne 28 && $# -ne 29 && $# -ne 30 && $# -ne 31 ]]; then
    echo "ERROR: sequential_launch_real.sh requires 24, 26, 28, 29, 30, or 31 positional arguments (got $#)."
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
HEAD_TOPIC_CONFIG=${10}
EMOTION_SEQUENCE_ENABLED=${11}
EMOTION_SEQUENCE_MAP_PATH=${12}
KEYBOARD_ACTION_ENABLED=${13}
KEYBOARD_ACTION_CONFIG_PATH=${14}
JOYSTICK_IMITATION_ENABLED=${15}
JOYSTICK_IMITATION_CONFIG_PATH=${16}
ARM_SWING_ENABLED=${17}
ARM_SWING_STOP_VX_THRESHOLD=${18}
ARM_SWING_HIGH_VX_THRESHOLD=${19}
ARM_SWING_VX_HYSTERESIS=${20}
ARM_SWING_BLEND_SEC=${21}
SAFETY_ORIENTATION_ROLL_LIMIT_DEG=${22}
SAFETY_ORIENTATION_PITCH_LIMIT_DEG=${23}
SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT=${24}
JOYSTICK_SEQUENCE_ENABLED=${25:-"false"}
JOYSTICK_SEQUENCE_CONFIG_PATH=${26:-""}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED=${27:-"true"}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH=${28:-"action_sequences/walking_back_emergency.yaml"}
EMERGENCY_DAMPING_KD=${29:-"5.0"}
POLICY_ROOT_PATH=${30:-""}
IMITATION_ENTRY_BLEND_SEC=${31:-"0.8"}

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
echo "Head Topic Config: $HEAD_TOPIC_CONFIG"
echo "Emotion Sequence Enabled: $EMOTION_SEQUENCE_ENABLED"
echo "Emotion Sequence Map Path: $EMOTION_SEQUENCE_MAP_PATH"
echo "Keyboard Action Enabled: $KEYBOARD_ACTION_ENABLED"
echo "Keyboard Action Config Path: $KEYBOARD_ACTION_CONFIG_PATH"
echo "Joystick Imitation Enabled: $JOYSTICK_IMITATION_ENABLED"
echo "Joystick Imitation Config Path: $JOYSTICK_IMITATION_CONFIG_PATH"
echo "Arm Swing: enabled=$ARM_SWING_ENABLED stop_vx=$ARM_SWING_STOP_VX_THRESHOLD high_vx=$ARM_SWING_HIGH_VX_THRESHOLD blend=$ARM_SWING_BLEND_SEC"
echo "Joystick Sequence Enabled: $JOYSTICK_SEQUENCE_ENABLED"
echo "Joystick Sequence Config Path: $JOYSTICK_SEQUENCE_CONFIG_PATH"
echo "Joystick Back Emergency Sequence Enabled: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED"
echo "Joystick Back Emergency Sequence Path: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH"
echo "Emergency Damping Kd: $EMERGENCY_DAMPING_KD"
echo "Policy Root Path: $POLICY_ROOT_PATH"
echo "Imitation Entry Blend Sec: $IMITATION_ENTRY_BLEND_SEC"
echo "Safety Orientation Roll Limit Deg: $SAFETY_ORIENTATION_ROLL_LIMIT_DEG"
echo "Safety Orientation Pitch Limit Deg: $SAFETY_ORIENTATION_PITCH_LIMIT_DEG"
echo "Safety Orientation Consecutive Violation Count: $SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT"
echo "=========================================="

# Step 0: Auto-configure Candlelight udev rules
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 0: Auto-configuring Candlelight...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

CANDLELIGHT_SCRIPT="$SCRIPT_DIR/auto_configure_candlelight.bash"

if [[ -f "$CANDLELIGHT_SCRIPT" ]]; then
    echo "Running Candlelight auto-configuration script: $CANDLELIGHT_SCRIPT"
    echo "Robot type: $ROBOT_TYPE"

    if bash "$CANDLELIGHT_SCRIPT" --robot-type "$ROBOT_TYPE"; then
        echo "SUCCESS: Candlelight udev rules configured successfully!"
    else
        echo "ERROR: Candlelight auto-configuration failed"
        echo "Cannot proceed without proper Candlelight setup - EXITING"
        exit 1
    fi

    echo "Candlelight configuration phase completed."
else
    echo "WARNING: Candlelight auto-config script not found at $CANDLELIGHT_SCRIPT"
    echo "Skipping automatic Candlelight configuration"
    echo "Please ensure udev rules are manually configured"
fi

echo ""

# Step 1: CAN Interface Setup
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 1: Setting up CAN interfaces...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

# Find the CAN setup script using relative path
CAN_SCRIPT="$SCRIPT_DIR/canable_setup.bash"

if [[ -f "$CAN_SCRIPT" ]]; then
    echo "Running CAN setup script: $CAN_SCRIPT"
    echo "Robot type: $ROBOT_TYPE"

    # Check if all interfaces are ready first
    bash "$CAN_SCRIPT" --status | grep "Summary:" | grep -q "All interfaces ready"
    if [[ $? -eq 0 ]]; then
        echo "SUCCESS: All CAN interfaces are already ready!"
    else
        echo "Setting up CAN interfaces for robot: $ROBOT_TYPE (battery: $BATTERY_ENABLED)..."
        # Run the setup script
        if ENABLE_BATTERY="$BATTERY_ENABLED" bash "$CAN_SCRIPT" --robot-type "$ROBOT_TYPE"; then
            echo "SUCCESS: CAN setup completed successfully!"
        else
            echo "ERROR: CAN setup failed after all internal retry cycles"
            echo "Cannot proceed without proper CAN setup - EXITING"
            exit 1
        fi
    fi

    echo "CAN setup phase completed."
else
    echo "ERROR: CAN setup script not found at $CAN_SCRIPT"
    echo "Cannot proceed without CAN setup script - EXITING"
    exit 1
fi

# Step 2: Launch Hardware Bridge
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 2: Launching hardware bridge...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"
echo "Starting legged_real with robot_type=$ROBOT_TYPE subpolicy:=$SUBPOLICY rviz:=$RVIZ imu_enabled:=$IMU_ENABLED"
echo "Starting legged_real with head_script_enabled=$HEAD_SCRIPT_ENABLED head_script_config=$HEAD_SCRIPT_CONFIG"

# Launch hardware bridge in background and wait for it to be ready
# Pass robot_type, subpolicy, rviz, and head config.
start_managed_process "hardware bridge" ros2 launch legged_real legged_real.launch.py \
    robot_type:=$ROBOT_TYPE \
    subpolicy:=$SUBPOLICY \
    rviz:=$RVIZ \
    imu_enabled:=$IMU_ENABLED \
    head_script_enabled:=$HEAD_SCRIPT_ENABLED \
    head_script_config:=$HEAD_SCRIPT_CONFIG \
    head_topic_config:=$HEAD_TOPIC_CONFIG \
    emotion_sequence_enabled:=$EMOTION_SEQUENCE_ENABLED \
    emotion_sequence_map_path:=$EMOTION_SEQUENCE_MAP_PATH \
    keyboard_action_enabled:=$KEYBOARD_ACTION_ENABLED \
    keyboard_action_config_path:=$KEYBOARD_ACTION_CONFIG_PATH \
    joystick_imitation_enabled:=$JOYSTICK_IMITATION_ENABLED \
    joystick_imitation_config_path:=$JOYSTICK_IMITATION_CONFIG_PATH \
    imitation_entry_blend_sec:=$IMITATION_ENTRY_BLEND_SEC \
    policy_root_path:=$POLICY_ROOT_PATH \
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
    emergency_damping_kd:=$EMERGENCY_DAMPING_KD
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
    if bash "$READY_CHECK_SCRIPT" "$BATTERY_ENABLED" "$IMU_ENABLED"; then
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

# Step 3: Launch Main Controller
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 3: Launching main controller...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"
echo "Starting one_start.launch with:"
echo "  robot_type: $ROBOT_TYPE"
echo "  bag_enabled: $BAG_ENABLED"
echo "  subpolicy: $SUBPOLICY"
echo "  bag_name: $BAG_NAME"
echo "  rviz: $RVIZ"
echo "  emotion_sequence_enabled: $EMOTION_SEQUENCE_ENABLED  emotion_sequence_map_path: $EMOTION_SEQUENCE_MAP_PATH"
echo "  keyboard_action_enabled: $KEYBOARD_ACTION_ENABLED  keyboard_action_config_path: $KEYBOARD_ACTION_CONFIG_PATH"
echo "  joystick_imitation_enabled: $JOYSTICK_IMITATION_ENABLED  joystick_imitation_config_path: $JOYSTICK_IMITATION_CONFIG_PATH"
echo "  joystick_sequence_enabled: $JOYSTICK_SEQUENCE_ENABLED  joystick_sequence_config_path: $JOYSTICK_SEQUENCE_CONFIG_PATH"
echo "  joystick_back_emergency_sequence_enabled: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED  joystick_back_emergency_sequence_path: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH"

# Launch main controller and wait for it while keeping cleanup control over all child launches
start_managed_process "main controller" ros2 launch legged_controllers one_start.launch.py \
    robot_type:=$ROBOT_TYPE \
    bag_enabled:=$BAG_ENABLED \
    subpolicy:=$SUBPOLICY \
    bag_name:=$BAG_NAME \
    rviz:=$RVIZ \
    emotion_sequence_enabled:=$EMOTION_SEQUENCE_ENABLED \
    emotion_sequence_map_path:=$EMOTION_SEQUENCE_MAP_PATH \
    keyboard_action_enabled:=$KEYBOARD_ACTION_ENABLED \
    keyboard_action_config_path:=$KEYBOARD_ACTION_CONFIG_PATH \
    joystick_imitation_enabled:=$JOYSTICK_IMITATION_ENABLED \
    joystick_imitation_config_path:=$JOYSTICK_IMITATION_CONFIG_PATH \
    joystick_sequence_enabled:=$JOYSTICK_SEQUENCE_ENABLED \
    joystick_sequence_config_path:=$JOYSTICK_SEQUENCE_CONFIG_PATH
CONTROLLER_PID=$STARTED_PID

CONTROLLER_EXIT=0
wait "$CONTROLLER_PID" || CONTROLLER_EXIT=$?

echo ""
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSequential launch completed (Real Hardware)\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

exit "$CONTROLLER_EXIT"
