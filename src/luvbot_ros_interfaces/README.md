# Luvbot ROS interfaces

Canonical ROS 2 Humble interface definitions for external boundaries between
independently built Luvbot modules. Deployment-internal interfaces belong to
`l0_ros2_deploy_v2/robot_interfaces`. This repository intentionally contains no node implementation,
launch files, robot mappings, hardware protocols, or policy configuration.

## Packages

- `luvbot_hardware_interfaces`: controller/arbiter/actuator contracts.
- `luvbot_control_interfaces`: external navigation, voice and perception contracts.

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

Version 1.1.0 adds `SetCustomWakeWord.srv`. Voice servers and cloud clients
must both use `luvbot_control_interfaces/srv/SetCustomWakeWord`; see the
migration and timeout rules in `CONTRACT.md`. No existing message is changed.

Deployment-internal `PolicyObservation`, `PolicyCommand`, `SwitchPolicy`,
`ControlWbcMotion`, `MotionTransaction` and `RequestMotion` are maintained by
`robot_interfaces` in the deployment repository. Their topic/service/Action names
and fields are preserved, but the ROS type package changes. Rebuild and restart
all affected producers and consumers together; old and new type packages cannot
communicate with each other. External navigation, voice and face interfaces keep
their existing types.
