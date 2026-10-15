#!/bin/bash

# Sequential launcher script for simulation
# This script runs tasks in sequence: Mujoco bridge -> Main controller -> Mujoco
# Note: mujoco_sim is launched last to ensure other components are ready
#
# joint_csv_blend_sec: per-sequence YAML sequence.joint_csv_blend_sec (LeggedController).

function showHelp(){
    echo
    echo "This script runs legged_mujoco_sim, then main controller, then mujoco in sequence"
    echo "Each step waits for the previous one to complete"
    echo
    echo "Usage: sequential_launch_sim.sh [robot_type] [bag_enabled] [subpolicy] [bag_name] [rviz] [head_script_enabled] [head_script_config] [head_topic_config] [emotion_sequence_enabled] [emotion_sequence_map_path] [keyboard_action_enabled] [keyboard_action_config_path] [joystick_imitation_enabled] [joystick_imitation_config_path] [arm_swing_enabled] [arm_swing_stop_vx_threshold] [arm_swing_high_vx_threshold] [arm_swing_vx_hysteresis] [arm_swing_blend_sec] [joystick_sequence_enabled] [joystick_sequence_config_path] [joystick_back_emergency_sequence_enabled] [joystick_back_emergency_sequence_path] [emergency_damping_kd] [policy_root_path] [imitation_entry_blend_sec]"
    echo "Example: sequential_launch_sim.sh l0_v3 false l0_v3_amp_07090 l0_v3_amp_07090_mujoco false false /path/head.yaml /path/head_topic.yaml false '' true /path/to/keyboard_action_map.yaml true /path/to/joystick_imitation_actions.yaml true 0.05 0.8 0.03 0.35 false /path/to/joystick_sequence_actions.yaml true /path/to/walking_back_emergency.yaml 5.0"
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

# Parse arguments
SCRIPT_ARGS=()
while [[ $# -gt 0 ]]; do
    if [[ "$1" == "--ros-args" ]]; then
        break
    fi
    SCRIPT_ARGS+=("$1")
    shift
done
set -- "${SCRIPT_ARGS[@]}"

if [[ $# -gt 26 ]]; then
    echo "ERROR: sequential_launch_sim.sh accepts at most 26 positional arguments (got $#)."
    echo "Launch via: ros2 launch legged_controllers sim2mujoco.launch.py"
    echo "Or run: $0 -h"
    exit 1
fi

ROBOT_TYPE=${1:-"l0_v3"}
BAG_ENABLED=${2:-"false"}
SUBPOLICY=${3:-"l0_v3_amp_07090"}
BAG_NAME=${4:-"${SUBPOLICY}_mujoco"}
RVIZ=${5:-"false"}
HEAD_SCRIPT_ENABLED=${6:-"false"}
HEAD_SCRIPT_CONFIG=${7:-""}
HEAD_TOPIC_CONFIG=${8:-""}
EMOTION_SEQUENCE_ENABLED=${9:-"false"}
EMOTION_SEQUENCE_MAP_PATH=${10:-""}
KEYBOARD_ACTION_ENABLED=${11:-"false"}
KEYBOARD_ACTION_CONFIG_PATH=${12:-""}
JOYSTICK_IMITATION_ENABLED=${13:-"false"}
JOYSTICK_IMITATION_CONFIG_PATH=${14:-""}
ARM_SWING_ENABLED=${15:-"true"}
ARM_SWING_STOP_VX_THRESHOLD=${16:-"0.05"}
ARM_SWING_HIGH_VX_THRESHOLD=${17:-"0.8"}
ARM_SWING_VX_HYSTERESIS=${18:-"0.03"}
ARM_SWING_BLEND_SEC=${19:-"0.35"}
JOYSTICK_SEQUENCE_ENABLED=${20:-"false"}
JOYSTICK_SEQUENCE_CONFIG_PATH=${21:-""}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED=${22:-"true"}
JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH=${23:-"action_sequences/walking_back_emergency.yaml"}
EMERGENCY_DAMPING_KD=${24:-"5.0"}
POLICY_ROOT_PATH=${25:-""}
IMITATION_ENTRY_BLEND_SEC=${26:-"0.8"}

# Avoid AddressSanitizer new-delete-type-mismatch with rclcpp/rcutils (ROS2 Humble)
# when legged_mujoco or mujoco_simulate are built with -fsanitize=address
export ASAN_OPTIONS="${ASAN_OPTIONS:+$ASAN_OPTIONS:}new_delete_type_mismatch=0"

echo "=========================================="
echo "Sequential Launch Starting (Simulation)"
echo "Robot Type: $ROBOT_TYPE"
echo "Bag Enabled: $BAG_ENABLED"
echo "Subpolicy: $SUBPOLICY"
echo "Bag Name: $BAG_NAME"
echo "RViz: $RVIZ"
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
echo "=========================================="

# Step 1: Launch Legged Mujoco Simulation Bridge
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 1: Launching legged_mujoco_sim...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

echo "Starting legged_mujoco_sim with robot_type=$ROBOT_TYPE"

start_managed_process "legged mujoco simulation bridge" ros2 launch legged_mujoco legged_mujoco_sim.launch.py \
    robot_type:=$ROBOT_TYPE \
    subpolicy:=$SUBPOLICY \
    rviz:=$RVIZ \
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
    arm_swing_enabled:=$ARM_SWING_ENABLED \
    arm_swing_stop_vx_threshold:=$ARM_SWING_STOP_VX_THRESHOLD \
    arm_swing_high_vx_threshold:=$ARM_SWING_HIGH_VX_THRESHOLD \
    arm_swing_vx_hysteresis:=$ARM_SWING_VX_HYSTERESIS \
    arm_swing_blend_sec:=$ARM_SWING_BLEND_SEC \
    joystick_sequence_enabled:=$JOYSTICK_SEQUENCE_ENABLED \
    joystick_sequence_config_path:=$JOYSTICK_SEQUENCE_CONFIG_PATH \
    joystick_back_emergency_sequence_enabled:=$JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED \
    joystick_back_emergency_sequence_path:=$JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH \
    emergency_damping_kd:=$EMERGENCY_DAMPING_KD
MUJOCO_BRIDGE_PID=$STARTED_PID

# Wait a bit for legged_mujoco_sim to initialize
echo "Waiting for legged_mujoco_sim to initialize..."
sleep 1

# Check if legged_mujoco_sim is still running
if process_group_alive "$MUJOCO_BRIDGE_PID"; then
    echo "Legged mujoco simulation bridge started successfully (PID: $MUJOCO_BRIDGE_PID)"
else
    echo "WARNING: Legged mujoco simulation bridge may have failed to start"
fi

echo ""

# Step 2: Launch Main Controller
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 2: Launching main controller...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

echo "Starting one_start.launch with:"
echo "  robot_type: $ROBOT_TYPE"
echo "  subpolicy: $SUBPOLICY"
echo "  bag_enabled: $BAG_ENABLED"
echo "  bag_name: $BAG_NAME"
echo "  rviz: $RVIZ"
echo "  emotion_sequence_enabled: $EMOTION_SEQUENCE_ENABLED  emotion_sequence_map_path: $EMOTION_SEQUENCE_MAP_PATH"
echo "  keyboard_action_enabled: $KEYBOARD_ACTION_ENABLED  keyboard_action_config_path: $KEYBOARD_ACTION_CONFIG_PATH"
echo "  joystick_imitation_enabled: $JOYSTICK_IMITATION_ENABLED  joystick_imitation_config_path: $JOYSTICK_IMITATION_CONFIG_PATH"
echo "  joystick_sequence_enabled: $JOYSTICK_SEQUENCE_ENABLED  joystick_sequence_config_path: $JOYSTICK_SEQUENCE_CONFIG_PATH"
echo "  joystick_back_emergency_sequence_enabled: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_ENABLED  joystick_back_emergency_sequence_path: $JOYSTICK_BACK_EMERGENCY_SEQUENCE_PATH"

# Launch main controller in background
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
    policy_root_path:=$POLICY_ROOT_PATH \
    joystick_sequence_enabled:=$JOYSTICK_SEQUENCE_ENABLED \
    joystick_sequence_config_path:=$JOYSTICK_SEQUENCE_CONFIG_PATH
CONTROLLER_PID=$STARTED_PID

# Wait a bit for controller to initialize
echo "Waiting for main controller to initialize..."
sleep 1

# Check if controller is still running
if process_group_alive "$CONTROLLER_PID"; then
    echo "Main controller started successfully (PID: $CONTROLLER_PID)"
else
    echo "WARNING: Main controller may have failed to start"
fi

echo ""

# Step 3: Launch Mujoco Simulation
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 3: launching mujoco...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

# Wait 3 seconds before launching mujoco_sim
sleep 3

echo "Starting mujoco_sim.launch with robot_type=$ROBOT_TYPE"

# Launch mujoco_sim and wait for it while keeping cleanup control over all child launches
start_managed_process "mujoco simulation" ros2 launch mujoco mujoco_sim.launch.py robot_type:=$ROBOT_TYPE
MUJOCO_PID=$STARTED_PID

MUJOCO_EXIT=0
wait "$MUJOCO_PID" || MUJOCO_EXIT=$?

echo ""
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSequential launch completed (Simulation)\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

exit "$MUJOCO_EXIT"
