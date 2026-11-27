#!/bin/bash

# Sequential launcher script for simulation
# Start the virtual motor backend before the bridge starts its enable deadline.

function showHelp(){
    echo
    echo "This script starts MuJoCo feedback, then the hardware bridge, then policy nodes"
    echo "Feedback and motor readiness are checked before advancing"
    echo "Usage: sequential_launch_sim.sh [robot_type] [bag_enabled] [subpolicy] [bag_name] [rviz] [imitation_policies] [arm_swing_enabled] [arm_swing_stop_vx_threshold] [arm_swing_high_vx_threshold] [arm_swing_vx_hysteresis] [arm_swing_blend_sec] [emergency_damping_kd] [wbc_enabled] [wbc_policies] [wbc_lora_bundle] [wbc_frontend_enabled] [txt2motion_enabled]"
    echo
    echo
}

if [ "$1" = "-h" ]; then
    showHelp
    exit 0
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/launch_processes.bash" || exit 1

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

if [[ $# -gt 17 ]]; then
    echo "ERROR: sequential_launch_sim.sh accepts at most 17 positional arguments (got $#)."
    echo "Launch via: ros2 launch legged_controllers sim2mujoco.launch.py"
    echo "Or run: $0 -h"
    exit 1
fi

ROBOT_TYPE=${1:-"atom_v3_smallfeet"}
BAG_ENABLED=${2:-"false"}
SUBPOLICY=${3:-"l0_v1_amp_03250_stable"}
BAG_NAME=${4:-"${SUBPOLICY}_mujoco"}
RVIZ=${5:-"false"}
IMITATION_POLICIES=${6:-"[]"}
ARM_SWING_ENABLED=${7:-"true"}
ARM_SWING_STOP_VX_THRESHOLD=${8:-"0.05"}
ARM_SWING_HIGH_VX_THRESHOLD=${9:-"0.8"}
ARM_SWING_VX_HYSTERESIS=${10:-"0.03"}
ARM_SWING_BLEND_SEC=${11:-"0.35"}
EMERGENCY_DAMPING_KD=${12:-"5.0"}
WBC_ENABLED=${13:-"false"}
WBC_POLICIES=${14:-"[]"}
WBC_LORA_BUNDLE=${15:-""}
WBC_FRONTEND_ENABLED=${16:-"$WBC_ENABLED"}
TXT2MOTION_ENABLED=${17:-"false"}

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
echo "Arm Swing: enabled=$ARM_SWING_ENABLED stop_vx=$ARM_SWING_STOP_VX_THRESHOLD high_vx=$ARM_SWING_HIGH_VX_THRESHOLD blend=$ARM_SWING_BLEND_SEC"
echo "Emergency Damping Kd: $EMERGENCY_DAMPING_KD"
echo "WBC Enabled: $WBC_ENABLED"
echo "WBC Policies: $WBC_POLICIES"
echo "Imitation Policies: $IMITATION_POLICIES"
echo "WBC LoRA Bundle: $WBC_LORA_BUNDLE"
echo "Fixed WBC Frontend: $WBC_FRONTEND_ENABLED"
echo "Text Frontend: $TXT2MOTION_ENABLED"
echo "=========================================="

# Step 1: Load MuJoCo before starting the hardware enable deadline.
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 1: Loading MuJoCo and waiting for motor feedback...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

echo "Starting mujoco_sim.launch with robot_type=$ROBOT_TYPE"
start_managed_process "mujoco simulation" ros2 launch mujoco mujoco_sim.launch.py robot_type:=$ROBOT_TYPE
MUJOCO_PID=$STARTED_PID

# Loading meshes and creating the software-rendered window can exceed 20 seconds.
echo "Waiting up to 60s for the first MuJoCo motor state before starting the hardware bridge..."
start_managed_process "mujoco feedback readiness" timeout --foreground --kill-after=2s 60s \
    ros2 topic echo /actuator/motor/state luvbot_hardware_interfaces/msg/MotorStateArray \
    --once --qos-reliability best_effort > /dev/null
SIM_READY_PID=$STARTED_PID
if wait "$SIM_READY_PID" && process_group_alive "$MUJOCO_PID"; then
    echo "MuJoCo motor feedback is online."
else
    echo "ERROR: MuJoCo did not produce motor feedback; shutting down launched nodes"
    exit 1
fi

echo ""

# Step 2: Enable motors through the same bridge/readiness check used on real hardware.
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 2: Launching legged_mujoco_sim...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

echo "Starting legged_mujoco_sim with robot_type=$ROBOT_TYPE"

start_managed_process "legged mujoco simulation bridge" ros2 launch legged_mujoco legged_mujoco_sim.launch.py \
    robot_type:=$ROBOT_TYPE \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    rviz:=$RVIZ \
    arm_swing_enabled:=$ARM_SWING_ENABLED \
    arm_swing_stop_vx_threshold:=$ARM_SWING_STOP_VX_THRESHOLD \
    arm_swing_high_vx_threshold:=$ARM_SWING_HIGH_VX_THRESHOLD \
    arm_swing_vx_hysteresis:=$ARM_SWING_VX_HYSTERESIS \
    arm_swing_blend_sec:=$ARM_SWING_BLEND_SEC \
    emergency_damping_kd:=$EMERGENCY_DAMPING_KD
MUJOCO_BRIDGE_PID=$STARTED_PID

READY_CHECK_SCRIPT="$SCRIPT_DIR/check_hardware_ready.py"
if [[ ! -f "$READY_CHECK_SCRIPT" ]]; then
    echo "ERROR: Hardware readiness check script not found: $READY_CHECK_SCRIPT"
    exit 1
fi

echo "Waiting for virtual motor enable, feedback and IMU readiness..."
start_managed_process "hardware readiness check" python3 "$READY_CHECK_SCRIPT" false true
READY_CHECK_PID=$STARTED_PID
if wait "$READY_CHECK_PID" && process_group_alive "$MUJOCO_BRIDGE_PID"; then
    echo "SUCCESS: Virtual motor and IMU readiness check completed."
else
    echo "ERROR: Virtual motor readiness check failed; shutting down launched nodes"
    exit 1
fi

echo ""

# Step 3: Launch Main Controller
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 3: Launching main controller...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

echo "Starting one_start.launch with:"
echo "  robot_type: $ROBOT_TYPE"
echo "  subpolicy: $SUBPOLICY"
echo "  bag_enabled: $BAG_ENABLED"
echo "  bag_name: $BAG_NAME"
echo "  rviz: $RVIZ"

# ros2 launch rejects name:= with an empty value; keep downstream defaults instead.
WBC_OPTIONAL_ARGS=()
if [[ -n "$WBC_LORA_BUNDLE" ]]; then
    WBC_OPTIONAL_ARGS+=("wbc_lora_bundle:=$WBC_LORA_BUNDLE")
fi

# Launch main controller in background
start_managed_process "main controller" ros2 launch legged_controllers one_start.launch.py \
    robot_type:=$ROBOT_TYPE \
    bag_enabled:=$BAG_ENABLED \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    "wbc_policies:=$WBC_POLICIES" \
    "imitation_policies:=$IMITATION_POLICIES" \
    wbc_frontend_enabled:=$WBC_FRONTEND_ENABLED \
    txt2motion_enabled:=$TXT2MOTION_ENABLED \
    "${WBC_OPTIONAL_ARGS[@]}" \
    bag_name:=$BAG_NAME \
    rviz:=$RVIZ
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

MUJOCO_EXIT=0
wait "$MUJOCO_PID" || MUJOCO_EXIT=$?

echo ""
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSequential launch completed (Simulation)\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

exit "$MUJOCO_EXIT"
