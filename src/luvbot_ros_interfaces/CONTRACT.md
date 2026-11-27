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
| `/control/navigation/follow_path` | `luvbot_control_interfaces/action/FollowNavigationPath` | navigation coordinator (client) | motion-control bridge (server) | reference updates; feedback about 5 Hz | ROS 2 Action defaults | PathReference with a shared header and ordered XYZ points (XY used for control), plus final absolute yaw in `device0/odom`; m, rad, m/s, rad/s | source `path.header.stamp + reference_timeout`; old retransmissions cannot renew expiry; cancel/expiry stops execution | navigation + controls |
| `/navigation/goal` | `luvbot_control_interfaces/msg/NavigationGoal` | mission/UI | goal converter | event | reliable, volatile, keep last 1 | pose in `header.frame_id`; speed m/s | no historical replay; reject empty/mismatched frame, non-finite pose, or invalid speed | navigation |
| `/navigation/planner_command` | `luvbot_control_interfaces/msg/NavigationPlannerCommand` | goal converter | planner | 5 Hz default | best effort, volatile, keep last 1 | local x/y m, speeds m/s and rad/s, heading rad | `valid_until` applies; reject invalid phase-specific fields | navigation |
| `/control/cmd_vel` | `geometry_msgs/msg/TwistStamped` | planner/teleop | policy/controller | producer rate | best effort, volatile, keep last 1 | base-frame m/s and rad/s | consumer timeout; stale command means zero velocity | navigation + controls |

Quaternion fields always use ROS `geometry_msgs/Quaternion` ordering and names:
`x`, `y`, `z`, `w`.  Array position `i` always refers to joint name `i`; arrays
must have equal non-zero length and contain at most 64 entries.

Policy and motion coordination are internal to `l0_ros2_deploy_v2` and use its
`robot_interfaces` package: `PolicyObservation`, `PolicyCommand`, `SwitchPolicy`,
`ControlWbcMotion`, `MotionTransaction` and `RequestMotion`. They are not external
contracts in this repository. Migration preserves fields and endpoint names but
changes ROS type identity; rebuild and restart all participating processes
together. Brain WBC/text JSON topics and external navigation interfaces are unchanged.

## External remaining-path navigation

The deployment uses `FollowNavigationPath`; the `NavigationGoal` and
`NavigationPlannerCommand` messages above remain available as legacy schemas,
but are not runtime inputs to the current deployment. Its former
`/navigation_goal` Float32 array subscription has been removed.
Navigation crops passed points and keeps the final task endpoint as the last
`PathReference.points` entry. `PathReference` has one `std_msgs/Header` and a
`geometry_msgs/Point[] points` array; no per-point headers or orientations.
The shared frame is `device0/odom`; xyz are metres and z is height, never a
control parameter. Controls validate finite xyz and use XY for execution;
z still participates in same-revision execution-content comparisons.
Controls use the first eight original points, repeat the endpoint
if fewer remain, and never resample. Final yaw is absolute in `device0/odom`,
not a relative turn or a continuously recomputed bearing to the endpoint.
Zero points along +X, positive yaw is counterclockwise viewed from +Z.

A nonempty unique task_id groups Action UUIDs. Start path_revision at 1 and
increment for changes, including prefix cropping. Identical execution content
may be revalidated with a newer generation stamp at the same revision; merely
retransmitting an old stamp does not extend its lifetime. New references use
new Action UUIDs; replaced UUIDs terminate with result REPLACED while the
business task may continue. Finished task IDs must not be reused.

This Goal schema replaces the previous `nav_msgs/Path` field and is wire
incompatible with it. Navigation clients and the deployment server must update
and rebuild against the same interface sources, then restart together. The
Action endpoint, Result and Feedback fields are unchanged. Rollback requires
restoring both the old interface and its matching client/server binaries.

Feedback reports task_id, revision, HOLD/NAV/FACE, remaining distance, heading
error and stop reason. Result codes are SUCCEEDED=0, REPLACED=1,
REFERENCE_EXPIRED=2, CANCELED=3 and STOPPED=4. Controls compute heading error
from final yaw minus the odometry-derived base yaw; heading tolerance is
15 degrees. See the Action definition and deployment's
`legged_controllers/docs/external_navigation_path.md` for lifecycle and
robot-side verification.

## Ownership and changes

Every wire-format PR requires approval from the system interface owner and at
least one affected producer or consumer owner.  The PR must list affected
topics, repositories, migration order, rollback, and evidence from the mock
contract tests.

Replace each role in the Owner column with the repository's actual GitHub team
or named maintainer when the organization is connected.

## Custom wake word service (1.1.0)

`/wake/set_custom_keyword` uses
`luvbot_control_interfaces/srv/SetCustomWakeWord`. The voice module is the
server; `luvbot_cloud_bridge` is the client. The request has one nonempty
`string keyword`. The response contains only `uint8 result` and
`string evaluation` (plain text, not JSON). Only `result=0` means success;
all other values are voice error codes whose meanings are not yet defined.
The bridge sends `CommandReply.code=result` and `CommandReply.message=evaluation`
verbatim for every received service response, without an additional `data` field.
`evaluation` describes the result, including the failure reason for nonzero codes;
it is not used to decide success. Bridge-local errors without a service response
(unavailable, busy, timeout, invalid request or exception) retain code 1001 and
their local error message.
No resource ID, resource path or separate response-message field is exposed.

The default client response deadline is 10 seconds (configurable). Calls are
asynchronous, reliable/volatile service QoS. Timeout means the outcome is
unconfirmed, not that the voice server canceled the operation. The bridge
retains one outstanding call and rejects further rename requests until that
response retires or the bridge restarts. It does not retry automatically.
If a late response arrives, the bridge sends its actual result/evaluation as the
final command reply and updates the cached result for the same command ID.

This is a new shared service type, not wire-compatible with the former
`wheeltec_mic_msg/srv/SetCustomWakeWord`, even when fields look similar.
The earlier `bool success` response must also be rebuilt and migrated on both
sides; do not mix it with the new result-code response.
Deploy the voice server and bridge with the same shared interface version,
stop the old endpoint, and restart both after rebuilding. Existing policy,
navigation and hardware message definitions are unchanged. Rollback requires
restoring the matching old voice server and bridge together. The interface
packages advance together to 1.1.0; tagging/publishing remains a release task.

## Mapping workflow additions (1.2.0)

The two packages advanced together to 1.2.0 for these additive interfaces.
Existing navigation, motion, voice and hardware schemas remain unchanged.
No release tag is created by this source change. Producers and consumers must
build against these matching sources before enabling the workflow.

| Name | Type | Server / producer | Client / consumer | Contract |
|---|---|---|---|---|
| `/control/radar/map_mode` | `luvbot_control_interfaces/srv/MapControl` | deployment radar-mode node | brain | action 1 calls Odin SLAM (3); action 2 switches to Relocation (4) and loads the Odin map selected by map_id |
| `/algorithm/mapping/control` | same | mapping module (separately implemented) | brain | action 1 starts mapping for the requested map; server success means start accepted/executed, not mapping complete |
| `/algorithm/mapping/status` | `luvbot_control_interfaces/msg/ExploreStatus` | mapping and segmentation | brain | reliable, volatile, keep-last 10. Case-sensitive `node_name`: `Mapping` or `Segmentation`. State 0/1/2/3 = IDLE/RUNNING/FINISHED/FAILED. |

MapControl preserves ACTION_MAPPING=1, ACTION_RELOCATION=2, ACTION_NOMAP=3,
ACTION_RECORY=4 (spelling retained), ACTION_DELETE_MAP=5, ACTION_OTHER=6,
MAP_INDEX_1=1 and MAP_INDEX_2=2. Requests contain uint8 action, string map_id,
and uint8 index. Rebuild all clients and servers against the same schema.
The brain converts the server's map.start.params.mapIndex ("1" or "2") to
the numeric index and passes the server's original mapId as map_id.
The mapping producer uses map_id for the filename
and payload.mapId. Its allowed characters are ASCII letters, digits,
hyphens, and underscores, with a length of 1 to 128.
Response accepted=true with error_code=ERROR_NONE(0) means this server's step
succeeded, not that a complete mapping/localization workflow succeeded.
ERROR_INVALID_REQUEST=1 covers invalid/unsupported actions or maps;
ERROR_SERVICE_DOWN=2 covers unavailable/busy/timed-out downstream services;
ERROR_STEP_FAILED=3 covers missing or unreadable map files and explicit downstream rejection.
Deployment v1 implements actions 1/2 only. Both servers have distinct names;
the brain invokes them separately, not as broadcast RPC.

Deployment checks the map before switching and allows up to 10 seconds for the
Odin operation, including the mode switch and map load for action 2. Timeout does
not cancel Odin, and the server rejects further operations until that request retires.
The brain waits up to 10 seconds for service discovery and another 10 seconds
for a response, and stops on failure without automatic retry or rollback.

ExploreStatus has string node_name, uint16 cost_time (task duration in seconds,
saturating at 65535), uint8 state, uint16 error_code (0=no error) and string
error_msg. Mapping publishes RUNNING after start, then FINISHED or FAILED.
Publish Mapping/FINISHED with error_code=0 after saving the Odin loop-closure map.
Publish Segmentation/FINISHED with error_code=0 after atomically saving the final
segmented map JSON. The brain waits for both successful FINISHED statuses in either
order. A FAILED status or nonzero error code from either stage fails the workflow.
This v1 assumes one task and no old producer/restart replay; status has no map/task
identity and must not be used for concurrent jobs or automatic restart recovery.

The mapping module directly writes `/userdata/forlinx/config/<map_id>.json`
with payload.mapId equal to map_id and payload.mapIndex equal to index.
While in SLAM the mapping module also saves the Odin map to
`/userdata/forlinx/auto_exploration/map/<map_id>.bin`. Deployment passes that path
to Odin load_map after switching to Relocation. Success confirms that the map
loaded. Relocalization readiness requires separate verification.
After both stages finish successfully, the bridge verifies both fields and
uploads the saved JSON, using the same server ID in the outer event.
Files use the agreed full cmd/payload format including Base64 PNG, dimensions,
resolution, origin, robotPose and roomList. The producer atomically replaces the
JSON before reporting Segmentation/FINISHED. The brain notifies the bridge to
upload the whole JSON in map.build.done.mapData.

Perception remains JSON std_msgs/String on `/algorithm/em/mode_command` and
`/algorithm/em/mode_state`, with matching request_id/last_request_id and
active target mode. Parallel perception/SLAM preparation follows the agreed
integration workflow and requires compatibility confirmation by perception.
Mapped navigation readiness is not inferred from a successful Relocation mode call.

For rollout, update these interface sources, deployment and the separately owned
mapping producer together, verify on Linux/ROS2 Humble and follow this repository's
interface-owner/consumer review and release process. Roll back new executables
and their new types together; existing consumers need no schema migration.

## Output volume services

`luvbot_control_interfaces/srv/GetVolumeState` reads the current output volume
and mute state with an empty request. `success` requires both reads to succeed.

`luvbot_control_interfaces/srv/SetVolume` sets the absolute output volume to
0–100 percent by default. In absolute mode, `volume=-1` leaves the volume
unchanged. With `relative=true`, the signed value is a percentage adjustment.
Set `set_mute=true` to apply the requested `mute` value. The response field
`saved` reports whether the settings were persisted. A persistence failure can
return `success=false` after the live volume has already changed.

Both responses report the actual volume as a percentage, or `-1` when it is
unavailable. `muted` is meaningful when `mute_known=true` and indicates whether
all channels of the configured mute control are off.
