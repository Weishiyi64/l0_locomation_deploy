# Luvbot ROS 2 interface contract

Target platform: ROS 2 Humble on Ubuntu 22.04, amd64 and arm64.

Node implementations use relative names.  The leading `/` below denotes the
canonical system graph after launch-time namespacing/remapping.

## Hardware boundary

| Name | Type | Producer | Consumer | Rate | QoS | Units / frames | Timeout and failure rule | Owner |
|---|---|---|---|---:|---|---|---|---|
| `/actuator/motor/command` | `luvbot_hardware_interfaces/msg/MotorCommand` | command arbiter | real or simulated actuator driver | 200–500 Hz | best effort, volatile, keep last 1 | rad, rad/s, Nm; actuator space | `valid_until` and driver watchdog apply; reject expired, out-of-order, incomplete, duplicate, unknown, or non-finite commands | controls + actuator driver |
| `/actuator/motor/state` | `luvbot_hardware_interfaces/msg/MotorStateArray` | real or simulated actuator driver | hardware adapter and arbiter | 200–500 Hz | best effort, volatile, keep last 5 | rad, rad/s, Nm, °C; actuator space | consumer-configured feedback timeout; stale, missing, invalid, error, or unexpected disabled mode is a hardware fault | actuator driver |
| `/actuator/motor/set_enabled` | `luvbot_hardware_interfaces/srv/SetMotorEnabled` | actuator driver | hardware adapter and arbiter | event | service default: reliable, volatile | one `MotorRoute` | caller timeout 500 ms; request is idempotent and success requires transport acknowledgement | actuator driver |
| `/imu/data` | `sensor_msgs/msg/Imu` | real or simulated IMU driver | hardware adapter | device rate | sensor data profile | SI; `header.frame_id` is IMU/body frame | adapter default startup/receive timeout 5 s; stale or invalid IMU is a hardware fault | sensing |
| `/imu/euler_deg` | `geometry_msgs/msg/Vector3Stamped` | real IMU driver | debugging and rosbag tools | device rate | sensor data profile | degrees; `header.frame_id` matches `/imu/data` | diagnostic only; must not be used as a control input | sensing |
| `/battery/state` | `sensor_msgs/msg/BatteryState` | battery driver | system monitor | 1–10 Hz | sensor data profile | standard ROS SI fields; `percentage` is 0..1 | monitoring policy; unavailable numeric data is NaN | power |
| `/odom` | `nav_msgs/msg/Odometry` | localization or simulator | navigation/policy | 20–100 Hz | sensor data profile | SI; parent `odom`, child robot base | consumer-configured timeout; empty/inconsistent frame IDs are rejected | localization |

Motor position is rad, velocity rad/s, torque Nm, and temperature degrees C.
`MotorRoute.bus_index` is the platform bus identifier; zero is a valid bus
index, not an invalid-route sentinel. Supported buses are platform-defined.
Motor type constant values are stable wire values and must never be reused.

## Control boundary

| Name | Type | Producer | Consumer | Rate | QoS | Units / frames | Timeout and failure rule | Owner |
|---|---|---|---|---:|---|---|---|---|
| `/control/policy/observation` | `luvbot_control_interfaces/msg/PolicyObservation` | controller | policy/planner | controller rate | best effort, volatile, keep last 1 | SI; `base_pose` uses `header.frame_id`; joints are named | consumer-configured timeout; reject invalid quaternion or inconsistent joint arrays | controls |
| `/control/policy/command/rl` | `luvbot_control_interfaces/msg/PolicyCommand` | RL policy | controller | policy rate | best effort, volatile, keep last 1 | target position rad; named joints | `valid_until` applies; reject expired, out-of-order, non-finite, unknown, duplicate, or incomplete joints | RL policy |
| `/control/policy/command/imitation` | `luvbot_control_interfaces/msg/PolicyCommand` | imitation policy | controller | policy rate | best effort, volatile, keep last 1 | target position rad; named joints | same as RL command | imitation policy |
| `/control/policy/switch` | `luvbot_control_interfaces/srv/SwitchPolicy` | controller client | policy server | event | service default: reliable, volatile | policy identifier, not a path | caller-configured timeout; response never exposes a process-local file path | controls + policy |
| `/navigation/goal` | `luvbot_control_interfaces/msg/NavigationGoal` | mission/UI | goal converter | event | reliable, volatile, keep last 1 | pose in `header.frame_id`; speed m/s | no historical replay; reject empty/mismatched frame, non-finite pose, or invalid speed | navigation |
| `/navigation/planner_command` | `luvbot_control_interfaces/msg/NavigationPlannerCommand` | goal converter | planner | 5 Hz default | best effort, volatile, keep last 1 | local x/y m, speeds m/s and rad/s, heading rad | `valid_until` applies; reject invalid phase-specific fields | navigation |
| `/control/cmd_vel` | `geometry_msgs/msg/TwistStamped` | planner/teleop | policy/controller | producer rate | best effort, volatile, keep last 1 | base-frame m/s and rad/s | consumer timeout; stale command means zero velocity | navigation + controls |

Quaternion fields always use ROS `geometry_msgs/Quaternion` ordering and names:
`x`, `y`, `z`, `w`.  Array position `i` always refers to joint name `i`; arrays
must have equal non-zero length and contain at most 64 entries.

## Ownership and changes

Every wire-format PR requires approval from the system interface owner and at
least one affected producer or consumer owner.  The PR must list affected
topics, repositories, migration order, rollback, and evidence from the mock
contract tests.

Replace each role in the Owner column with the repository's actual GitHub team
or named maintainer when the organization is connected.
