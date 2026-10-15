# AGENTS.md

This file applies to the whole `luvbot_deploy_ros2` tree unless a more
specific `AGENTS.md` is added in a subdirectory.

To better fulfill the requirements, here are some key points you need to understand: the primary target for modifications within `luvbot_deploy_ros2` is the `atom_ros2_deploy_baseline` package. When compiling and verifying your changes, please execute the build process from the `luvbot_deploy_ros2` directory, rather than directly from the `atom_ros2_deploy_baseline` package directory.

## Project Map

This is a ROS2/ament workspace slice for legged robot deployment.

- `legged_controllers/`: shared controller library. It owns `LeggedController`,
  state machine integration, joystick handling, head control, policy mux,
  safety checks, CSV/action-sequence playback, and command composition.
- `legged_env_mujoco/legged_mujoco/`: MuJoCo-side hardware layer using the
  shared controller.
- `legged_env_mujoco/mujoco/`: MuJoCo simulator launch/runtime package.
- `legged_env_real/legged_real/`: real robot hardware layer using the shared
  controller.
- `legged_env_real/hardware/`: CAN, actuator, IMU, and battery support.
- `legged_deploy/`: RL controller launch/config/policy assets.
- `legged_msgs/`: ROS2 message definitions.
- `legged_description/`: robot descriptions and display launches.
- `serial-ros2/`: serial support package.

The parent workspace is normally `/home/luvbot/zy_workspace/luvbot_deploy_ros2`.
Some dependencies, especially `luvbot_eye`, live as sibling packages outside
this repository directory. If CMake cannot find them, source the parent
workspace install setup before building.

## Safety Boundaries

- Do not run real hardware launches or setup scripts unless the user explicitly
  asks for it. In particular, avoid `sequential_launch_real.sh`,
  `legged_real.launch.py`, `canable_setup.bash`, and
  `auto_configure_candlelight.bash` during routine validation.
- Simulation launch commands are safer, but still start ROS processes; only run
  them when needed and stop all spawned process groups before finishing.
- Do not change robot policy files, calibration values, safety limits, udev
  rules, CAN mappings, or actuator configs unless the task clearly targets them.
- Preserve user changes in the working tree. Never use `git reset --hard` or
  checkout files back unless the user explicitly requested that exact action.

## Current Controller Architecture

- Arm motion is driven through Joint CSV/action sequences and policy commands.
  Do not reintroduce the old hand/arm YAML script overlay path.
- Head control is still separate: `HeadScriptController` and
  `HeadTopicController` may overlay head joints while walking.
- `JointCommandComposer` priority is:
  1. Joint CSV/action sequence command, when enabled and allowed for the joint.
  2. Head overlay, when auxiliary overlays are allowed.
  3. Policy/default standby command.
  4. Release handoff, emergency, and soft-release overrides.
- Arm/head/leg classification comes from `JointGroups`; keep it available for
  CSV/action-sequence and soft-release behavior even when a specific overlay is
  removed.

## Launch Flow

- Simulation entry point:
  `ros2 launch legged_controllers sim2mujoco.launch.py`
- Real robot entry point:
  `ros2 launch legged_controllers sim2real.launch.py`
- Both entry points call a `sequential_launch_*.sh` script, which then starts
  the environment bridge and `one_start.launch.py`.
- `one_start.launch.py` starts RViz/joy/RL controller support and action
  sequence helpers. The actual controller parameters for head/topic/sequence
  behavior are passed through the sim/real environment launch.

When changing a launch argument, update the whole chain together:

- `legged_controllers/launch/sim2mujoco.launch.py`
- `legged_controllers/launch/sim2real.launch.py`
- `legged_controllers/scripts/sequential_launch_sim.sh`
- `legged_controllers/scripts/sequential_launch_real.sh`
- `legged_env_mujoco/legged_mujoco/launch/legged_mujoco_sim.launch.py`
- `legged_env_real/legged_real/launch/legged_real.launch.py`
- `legged_controllers/launch/one_start.launch.py`, only if it actually
  consumes the argument.

## Coding Guidelines

- Match the existing C++14 style and module boundaries. Keep controller logic in
  focused source files instead of growing `LeggedController.cpp` monolithically.
- Prefer existing helpers such as `ParameterManager`, `ControllerFileUtils`,
  `JointGroups`, `PolicyMux`, `SequenceRuntime`, and `JointCommandComposer`.
- Keep real hardware details in `legged_real`/hardware packages; keep shared
  policy/state-machine behavior in `legged_controllers`.
- Use structured YAML/CSV parsing helpers already in the codebase. Avoid ad hoc
  string parsing for config formats.
- For launch files, keep default values at the highest user-facing entry point
  when possible, and pass only arguments that downstream launches consume.
- Do not leave generated build outputs, `.pyc` files, `__pycache__`, logs, or
  temporary test files in the working tree.
