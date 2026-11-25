# Agent Context

Read this reference only for tasks involving package ownership, controller
composition, launch wiring, or related architecture. The always-applicable
rules live in `../AGENTS.md`.

## Project Map

This repository is a ROS2/ament workspace slice for legged robot deployment.

- `legged_controllers/`: shared controller library. It owns
  `LeggedController`, state-machine integration, joystick and head control,
  policy mux, safety checks, CSV/action-sequence playback, and command
  composition.
- `legged_env_mujoco/legged_mujoco/`: MuJoCo-side hardware layer using the
  shared controller.
- `legged_env_mujoco/mujoco/`: MuJoCo simulator launch/runtime package.
- `legged_env_real/legged_real/`: real-robot hardware layer using the shared
  controller.
- `legged_deploy/`: RL controller launch, configuration, and policy assets.
- `robot_interfaces/`: temporary exact copy of the HPM motor interfaces used by
  the real-robot path; keep it byte-for-byte aligned with the HPM source package.
- `luvbot_control_interfaces/` (sibling package): policy and navigation message definitions.
  It also owns `ControlWbcMotion.srv`; deploy requires the matching WBC interface revision.
- `legged_description/`: robot descriptions and display launches.

The v2 workspace is `~/zy_workspace/l0_deploy_v2_ws`, with this repository at
`src/l0_ros2_deploy_v2` and matching `luvbot_ros_interfaces` sources alongside it.
The v1 repository and its dependencies belong in the separate
`~/zy_workspace/l0_deploy_v1_ws` workspace. Each workspace owns its build,
install, and log directories. Source only the selected deployment workspace.
The v2 controller has no eye-animation input or serial transport dependency;
HPM owns the real motor and IMU transport in its own workspace.

## Controller Architecture

- Arm motion is driven through Joint CSV/action sequences and policy commands.
  Do not restore the old hand/arm YAML script overlay path.
- Head control remains separate. `HeadScriptController` handles configured
  scripts and the focused face-follow position overlay while walking.
- `JointCommandComposer` priority is:
  1. Joint CSV/action sequence command when enabled and allowed for the joint.
  2. Head overlay when auxiliary overlays are allowed.
  3. Policy/default standby command.
  4. Position handoff and emergency overrides.
- Arm/head/leg classification comes from `JointGroups`. Keep it available for
  CSV/action-sequence behavior even when removing an overlay.
- Prefer existing helpers such as `ParameterManager`, `ControllerFileUtils`,
  `JointGroups`, `PolicyMux`, `SequenceRuntime`, and `JointCommandComposer`.
- WBC uses `PolicyObservation` and `PolicyCommand`, with a dedicated worker: RKNN on RK3588 and static FP32 OpenVINO on x86 simulation.
  `LeggedControllerWbc.cpp` stages policy handoffs for the control loop; callbacks
  must not mutate the shared `JointCommandTransition` directly. Face following
  pauses during WBC and policy handoff. See the root README for deployment.
- Real hardware accepts `l0_v3`, `l0_v4` (RS05 arms), and `l0_v5` (LS
  PJ2S-48XX22A arms). Shared mappings accept bus 0; v5 stores the physical
  HPM bus directly, while LeggedReal sends only joint indices for HPM routing.
  The v5 mechanical calibration and joint limits are
  inherited from v4 and require joint-by-joint hardware verification. Its AMP
  policy directory links to the v4 assets. WBC uses the `L0_AfraidToLeave`
  native two-input policy RKNN and a reference-motion RKNN queried every control cycle; the former v4
  upper-body JSON trajectory path has been removed. See the root README for WBC deployment.
  WBC preloads the `wbc_policies` list from sim2real (sim2mujoco defaults empty),
  resolves `policy/<robot_type>/<policy>` or `policy/<robot_type>/wbc/<policy>`, and selects one cached motion by WbcCfg.motion_id.
  One worker runs only the active motion; AMP pauses during WBC execution.
  Observation joint names/order and model ABI remain checked.

## Launch Flow

- Simulation entry point:
  `ros2 launch legged_controllers sim2mujoco.launch.py`
- Real-robot entry point:
  `ros2 launch legged_controllers sim2real.launch.py`
- Both entry points call a `sequential_launch_*.sh` script, which starts the
  environment bridge and `one_start.launch.py`.
- `one_start.launch.py` starts RViz, joystick, RL-controller support, and
  action-sequence helpers. Head/topic/sequence parameters flow through the
  simulation or real environment launch.

When changing a launch argument, inspect and update the consumers across this
chain as needed:

- `legged_controllers/launch/sim2mujoco.launch.py`
- `legged_controllers/launch/sim2real.launch.py`
- `legged_controllers/scripts/sequential_launch_sim.sh`
- `legged_controllers/scripts/sequential_launch_real.sh`
- `legged_env_mujoco/legged_mujoco/launch/legged_mujoco_sim.launch.py`
- `legged_env_real/legged_real/launch/legged_real.launch.py`
- `legged_controllers/launch/one_start.launch.py` when it consumes the argument
