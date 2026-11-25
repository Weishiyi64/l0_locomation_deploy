# Luvbot ROS interfaces

Canonical ROS 2 Humble interface definitions shared by independently built
Luvbot nodes.  This repository intentionally contains no node implementation,
launch files, robot mappings, hardware protocols, or policy configuration.

## Packages

- `luvbot_hardware_interfaces`: controller/arbiter/actuator contracts.
- `luvbot_control_interfaces`: controller/policy/navigation contracts.

Both packages are released together.  Consumers must pin a release tag (for
example `v1.0.0`) in their `.repos` file and must not depend on a moving branch.

## Version policy

- Major: change/remove/reorder any existing field or reinterpret a constant.
- Minor: add a new independent message, service, or action.
- Patch: documentation or build-only change that preserves the wire format.

Published interfaces are immutable.  A breaking protocol change gets a new
major release and a new type/topic during migration.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select luvbot_hardware_interfaces luvbot_control_interfaces
colcon test --packages-select \
  luvbot_hardware_interfaces luvbot_control_interfaces
colcon test-result --verbose
```

See [CONTRACT.md](CONTRACT.md) for topic ownership, QoS, units, and failure
semantics and [DEVELOPMENT.md](DEVELOPMENT.md) for the independent-node build
and release workflow.
