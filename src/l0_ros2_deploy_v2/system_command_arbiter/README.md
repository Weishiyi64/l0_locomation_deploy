# Motor command arbiter

This package owns the L0_v3 joint/motor mapping and the safety boundary between
the `system` and `actuator` domains. Device transport belongs to the external
embedded node.

## Production interfaces

- `/system/command_arbiter/motor_command_candidate`: logical joint commands.
- `/actuator/motor/command`: the single, arbitrated final motor command.
- `/actuator/motor/state`: raw motor feedback.
- `/actuator/motor/set_enabled`: idempotent single-motor enable/disable.

The standalone arbiter and mock use shared `luvbot_hardware_interfaces` types.
The final command publisher uses Best Effort, Volatile, Keep Last 1. Motor
state uses Best Effort, Volatile, Keep Last 5. Candidates are rejected when
expired, out of order, incomplete, non-finite, from an unknown source, or unsafe.
The shared output is MIT, so the arbiter accepts hybrid candidates.
Final MIT frames carry a timestamp and sequence number. The mock applies its
configured `command_validity_ms` (20 ms by default) to that timestamp and also
enforces its receive watchdog.

The shared mapping used by `LeggedReal` confirms a feedback turn with three
consecutive valid samples. A position already inside the configured joint
limits (with the existing 0.1 rad tolerance) needs no correction; otherwise only
an unambiguous integer-turn correction that lands inside those limits is
accepted. During initial acquisition and damping-only standby, an invalidated
turn can be reselected. `LeggedReal` continues sending damping commands during
acquisition so motors that reply only to MIT commands can supply feedback;
unconfirmed feedback still counts toward the existing missing-feedback timeout.

The first command with nonzero kp, target velocity, or feedforward torque locks
the turns. Invalid feedback thereafter latches a fault instead of silently
changing the motor target by a full revolution. Inspect the raw feedback and
mapping before recovery: `LeggedReal` clears the turn state on explicit disable
publication (and for selected motors during zeroing); the standalone arbiter
requires restart. Do not reset a fault while the robot depends on position hold.
`LeggedReal` logs the raw position, selected offset, converted position, and
confirmation state when a turn changes or becomes ready.

`MotorRoute.motor_type` is the zero-based motor index in the configured joint
order. The mapping keeps the protocol family separately for local calibration
and model selection.

Start the normal arbiter (it does not generate commands on its own):

```bash
ros2 launch system_command_arbiter system_command_arbiter.launch.py
```

## Build and safe mock test

Build from the parent workspace, not from this package directory:

```bash
cd /path/to/luvbot_deploy_ros2
colcon build --symlink-install --packages-up-to system_command_arbiter legged_real
source install/setup.bash
ros2 launch system_command_arbiter motor_pilot_mock.launch.py
```

The mock launch uses only `/debug/actuator/motor/*` and never opens hardware.
It runs the complete enable, feedback, two-second hold, and disable sequence.

## Real single-motor pilot

The standalone pilot requires a compatible driver implementing `SetMotorEnabled`.
HPM direct deployment uses `MotorEnableCommand` through `LeggedReal`. The
standalone pilot is disarmed by default:

```bash
ros2 launch system_command_arbiter motor_pilot.launch.py
```

After mechanically securing the robot, pass the index, bus and CAN ID of the
verified motor. The example selects index 4 in the default l0_v3 mapping:

```bash
ros2 launch system_command_arbiter motor_pilot.launch.py \
  allow_actuation:=true pilot_motor_type:=4 pilot_bus_index:=0 pilot_motor_id:=5
```

## `my_ws` handoff

`my_ws` should source this workspace's install environment before building so
that it can depend on `luvbot_hardware_interfaces`. Its actuator node owns transport,
publishes `MotorStateArray`, serves `SetMotorEnabled`, and enforces its command
watchdog. Robot joint mapping remains in this package's `l0_v3.yaml`.

## L0 v4 / v5 HPM mappings

- `config/l0_v4.yaml`: 21 enabled joints, including six RS05 arm motors.
- `config/l0_v5.yaml`: 21 enabled joints, with MR PA35-10-8 arm motors.
  Joint indices, protocol families, CAN buses and motor IDs match the default
  HPM `hpm_motor/config/motors.yaml` at `ad8ecd80`.

The shared mapping/arbiter contract accepts bus 0. The v5 mapping stores the
physical HPM bus directly, with no offset. Bus ranges remain platform-defined;
existing v3/v4 bus assignments are unchanged. HPM selects the physical device
from the joint index in `route.motor_type` and owns CAN-FD framing, bit rates
and physical routing.

| Group | HPM / deployment bus | Joint indices | CAN IDs |
| --- | --- | --- | --- |
| Left leg | 0 | 0–5 | 0x01–0x06 |
| Right leg | 3 | 6–11 | 0x11–0x16 |
| Left arm | 5 | 12–14 | 0x21–0x23 |
| Right arm | 2 | 15–17 | 0x31–0x33 |
| Head | 1 | 18–20 | 0x41–0x43 |

The HPM routing table specifies protocol, model and route. The deployment
mapping supplies reduction, zero offsets, directions and joint limits.
Verify these mechanical values on the robot before commanding motion.

The `legged_controllers sim2real.launch.py` entry point defaults to
`robot_type:=l0_v5`.

Before movement, also align HPM's RS06 encoding ranges and LS torque/current
units with the actual motor specification. Adding a mapping does not resolve
those transport issues. In HPM revision `321e2ae5`, `motor.hpp` encodes all RS
motors with torque +/-5.5, kp 0..500 and kd 0..5, while the deployment RS06
profile uses torque +/-36, kp 0..5000 and kd 0..100. For LS, HPM uses torque
+/-90 and kd 0..5, while PJ2S-48XX22A uses torque +/-23 and kd 0..50 here.
These are unresolved encoding/unit differences, not values to copy between
profiles without the motor specification. Startup still auto-enables all
configured motors; this mapping update does not fix the reported limit strike.

On Linux, build from the parent workspace using a
fresh build/install directory, verify that the installed v5 AMP link resolves,
then validate feedback direction/zero and emergency-stop behavior with the
robot supported. These changes have only been checked statically on macOS.
