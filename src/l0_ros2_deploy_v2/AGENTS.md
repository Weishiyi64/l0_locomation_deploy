# AGENTS.md

## Scope

These instructions apply only to this deployment repository and its
descendants. They do not apply to sibling repositories such as
`odin_ros_driver_rev1/`.

- The local development environment is macOS, while the deployment target is
  the robot's Linux/ROS2 environment. Do not compile, launch, or run ROS tests
  locally unless the user explicitly asks; use static inspection and
  non-runtime checks only. The user will deploy to the robot for build and
  runtime verification.
- If the user explicitly requests a build, run it from the parent
  `l0_deploy_v2_ws` workspace, not from this package directory. Keep matching
  `luvbot_ros_interfaces` sources in that workspace and build dependencies with
  `--packages-up-to`.
- Read `docs/agent-context.md` only when a task touches package ownership,
  controller composition, launch wiring, or related architecture.

## Working Style

Use Ponytail Full for coding, debugging, refactoring, review, and design unless
the user requests another mode.

- Understand the affected path and callers, then make the smallest correct
  change at the shared root cause.
- Reuse existing code, the C++ standard library, ROS2/ament features, and
  installed dependencies. Avoid speculative abstractions, one-use helpers,
  unnecessary configuration, new dependencies, and unrelated refactors.
- Use the smallest useful static check for non-trivial logic. Do not substitute
  a local macOS build or runtime test for the user's robot-side verification.
- Do not create unit-test, integration-test, fixture, mock, or other test files
  unless the user explicitly asks for them. The user will move the changed
  program to the robot's Linux/ROS2 environment and perform build and runtime
  testing there; provide the necessary robot-side verification steps instead.
- Do not leave generated build outputs, `.pyc` files, `__pycache__`, logs, or
  temporary test files in the working tree.

## Safety Boundaries

- Do not run real-hardware launches or setup scripts unless the user explicitly
  asks. This includes `sequential_launch_real.sh`, `legged_real.launch.py`, and
  `setup_rk3588_once.bash`.
- Do not start simulation locally unless the user explicitly asks. Stop every
  spawned process group before finishing.
- Do not change policy files, calibration values, safety limits, udev rules,
  CAN mappings, or actuator configuration unless the task explicitly targets
  them.
- Preserve hardware interlocks, actuator limits, timeouts, watchdogs,
  emergency-stop behavior, trust-boundary validation,
  data-loss prevention, real-time constraints, thread safety, and calibration
  controls.
- Policy inference changes must preserve tensor shape/type checks,
  observation/action ordering, output bounds, timing, and a deterministic safe
  fallback for invalid or late output.
- Preserve user changes. Never reset, discard, or overwrite unrelated work.

## Code Rules

- Match the existing C++14 style and module boundaries. Keep shared
  policy/state-machine behavior in `legged_controllers` and hardware-specific
  behavior in `legged_env_real` or `hardware`.
- Keep focused controller logic out of the `LeggedController.cpp` monolith.
- Use the repository's structured YAML/CSV helpers instead of ad hoc parsers.
- Keep launch defaults at the highest user-facing entry point and pass only
  arguments that downstream launches consume.
