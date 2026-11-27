#!/bin/bash

# Sequential launcher script for real hardware
# Start HPM separately before this script. This script runs the hardware bridge,
# waits for readiness, and then starts the main controller.

function showHelp(){
    echo
    echo "This script expects HPM to be running, then starts the hardware bridge and main controller"
    echo "Each step waits for the previous one to complete"
    echo "Usage: sequential_launch_real.sh [robot_type] [bag_enabled] [subpolicy] [bag_name] [rviz] [battery_enabled] [imu_enabled] [imitation_policies] [arm_swing_enabled] [arm_swing_stop_vx_threshold] [arm_swing_high_vx_threshold] [arm_swing_vx_hysteresis] [arm_swing_blend_sec] [safety_orientation_roll_limit_deg] [safety_orientation_pitch_limit_deg] [safety_orientation_consecutive_violation_count] [emergency_damping_kd] [radar_control_enabled] [nav_planner_model_path] [wbc_enabled] [wbc_policies] [nav_check_sensor_timestamps] [nav_reference_receive_timeout] [wbc_lora_bundle] [wbc_frontend_enabled] [txt2motion_enabled]"
    echo
    echo "Body joy is fixed at /dev/input/js0 (see joy_teleop.launch.py). Defaults live in legged_controllers/launch/sim2real.launch.py — use:"
    echo "  ros2 launch legged_controllers sim2real.launch.py"
    echo
}

if [ "$1" = "-h" ]; then
    showHelp
    exit 0
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/launch_processes.bash" || exit 1

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

if [[ $# -lt 16 || $# -gt 26 ]]; then
    echo "ERROR: sequential_launch_real.sh requires 16 to 26 positional arguments (got $#)."
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
IMITATION_POLICIES=${8:-"[]"}
ARM_SWING_ENABLED=${9}
ARM_SWING_STOP_VX_THRESHOLD=${10}
ARM_SWING_HIGH_VX_THRESHOLD=${11}
ARM_SWING_VX_HYSTERESIS=${12}
ARM_SWING_BLEND_SEC=${13}
SAFETY_ORIENTATION_ROLL_LIMIT_DEG=${14}
SAFETY_ORIENTATION_PITCH_LIMIT_DEG=${15}
SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT=${16}
EMERGENCY_DAMPING_KD=${17:-"5.0"}
RADAR_CONTROL_ENABLED=${18:-"false"}
NAV_PLANNER_MODEL_PATH=${19:-""}
WBC_ENABLED=${20:-"false"}
WBC_POLICIES=${21:-"[]"}
NAV_CHECK_SENSOR_TIMESTAMPS=${22:-"true"}
NAV_REFERENCE_RECEIVE_TIMEOUT=${23:-"false"}
WBC_LORA_BUNDLE=${24:-""}
WBC_FRONTEND_ENABLED=${25:-"false"}
TXT2MOTION_ENABLED=${26:-"false"}

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
echo "Arm Swing: enabled=$ARM_SWING_ENABLED stop_vx=$ARM_SWING_STOP_VX_THRESHOLD high_vx=$ARM_SWING_HIGH_VX_THRESHOLD blend=$ARM_SWING_BLEND_SEC"
echo "Emergency Damping Kd: $EMERGENCY_DAMPING_KD"
echo "WBC Enabled: $WBC_ENABLED"
echo "WBC Policies: $WBC_POLICIES"
echo "Imitation Policies: $IMITATION_POLICIES"
echo "WBC LoRA Bundle: $WBC_LORA_BUNDLE"
echo "Fixed WBC Frontend: $WBC_FRONTEND_ENABLED"
echo "Text Frontend: $TXT2MOTION_ENABLED"
echo "Safety Orientation Roll Limit Deg: $SAFETY_ORIENTATION_ROLL_LIMIT_DEG"
echo "Safety Orientation Pitch Limit Deg: $SAFETY_ORIENTATION_PITCH_LIMIT_DEG"
echo "Safety Orientation Consecutive Violation Count: $SAFETY_ORIENTATION_CONSECUTIVE_VIOLATION_COUNT"
echo "Radar Control Enabled: $RADAR_CONTROL_ENABLED"
echo "Navigation Planner Model: $NAV_PLANNER_MODEL_PATH"
echo "Navigation Sensor Timestamp Checks: $NAV_CHECK_SENSOR_TIMESTAMPS"
echo "Navigation Reference Timeout From Receipt: $NAV_REFERENCE_RECEIVE_TIMEOUT"
echo "HPM motor and IMU nodes must already be running"
echo "=========================================="

# Step 1: Launch Hardware Bridge
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSTEP 1: Launching hardware bridge...\033[0m"
echo -e "\033[1;35m==========================================\033[0m"
echo "Starting legged_real with robot_type=$ROBOT_TYPE subpolicy:=$SUBPOLICY rviz:=$RVIZ imu_enabled:=$IMU_ENABLED"

# Launch hardware bridge in background and wait for it to be ready
# Pass robot_type, subpolicy, rviz, and head config.
start_managed_process "hardware bridge" ros2 launch legged_real legged_real.launch.py \
    robot_type:=$ROBOT_TYPE \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    rviz:=$RVIZ \
    imu_enabled:=$IMU_ENABLED \
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

# The hardware bridge is already sending zero-gain MIT while enable/feedback settle.
READY_CHECK_SCRIPT="$SCRIPT_DIR/check_hardware_ready.py"
if [[ ! -f "$READY_CHECK_SCRIPT" ]]; then
    echo "ERROR: Hardware readiness check script not found: $READY_CHECK_SCRIPT"
    exit 1
fi

echo "Running hardware readiness check: $READY_CHECK_SCRIPT"
start_managed_process "hardware readiness check" python3 "$READY_CHECK_SCRIPT" "$BATTERY_ENABLED" "$IMU_ENABLED"
READY_CHECK_PID=$STARTED_PID
if wait "$READY_CHECK_PID"; then
    echo "SUCCESS: Hardware readiness check completed."
else
    echo "ERROR: Hardware readiness check failed; shutting down launched nodes"
    exit 1
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
echo "  radar_control_enabled: $RADAR_CONTROL_ENABLED"
echo "  nav_planner_model_path: $NAV_PLANNER_MODEL_PATH"

# ros2 launch rejects name:= with an empty value; keep downstream defaults instead.
WBC_OPTIONAL_ARGS=()
if [[ -n "$WBC_LORA_BUNDLE" ]]; then
    WBC_OPTIONAL_ARGS+=("wbc_lora_bundle:=$WBC_LORA_BUNDLE")
fi

# Launch main controller and wait for it while keeping cleanup control over all child launches
start_managed_process "main controller" ros2 launch legged_controllers one_start.launch.py \
    robot_type:=$ROBOT_TYPE \
    bag_enabled:=$BAG_ENABLED \
    subpolicy:=$SUBPOLICY \
    wbc_enabled:=$WBC_ENABLED \
    wbc_frontend_enabled:=$WBC_FRONTEND_ENABLED \
    txt2motion_enabled:=$TXT2MOTION_ENABLED \
    "wbc_policies:=$WBC_POLICIES" \
    "imitation_policies:=$IMITATION_POLICIES" \
    "${WBC_OPTIONAL_ARGS[@]}" \
    bag_name:=$BAG_NAME \
    rviz:=$RVIZ \
    radar_control_enabled:=$RADAR_CONTROL_ENABLED \
    nav_planner_model_path:=$NAV_PLANNER_MODEL_PATH \
    nav_check_sensor_timestamps:=$NAV_CHECK_SENSOR_TIMESTAMPS \
    nav_reference_receive_timeout:=$NAV_REFERENCE_RECEIVE_TIMEOUT
CONTROLLER_PID=$STARTED_PID

CONTROLLER_EXIT=0
wait "$CONTROLLER_PID" || CONTROLLER_EXIT=$?

echo ""
echo -e "\033[1;35m==========================================\033[0m"
echo -e "\033[1;35mSequential launch completed (Real Hardware)\033[0m"
echo -e "\033[1;35m==========================================\033[0m"

exit "$CONTROLLER_EXIT"
