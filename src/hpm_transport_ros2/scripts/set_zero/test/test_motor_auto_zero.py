# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Offline fake drive/service checks. No ROS graph or physical device is used."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import math
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from set_zero import motor_auto_zero as h


def plan(module="left_arm", index=None):
    motors, targets = h.load_plan(h.motor_config_path(), PACKAGE / "config/motor_zero_calibration.yaml",
                                 PACKAGE / "config/l0_v5.urdf", module if index is None else None, index)
    # Keep legacy fixed-feedforward scenarios explicit; adaptive cases enable it separately.
    for target in targets:
        target.settings["adaptive_torque_enabled"] = False
        target.settings.update(dict.fromkeys(h.FEEDFORWARD, 0.0))  # Explicit legacy test baseline.
        target.settings.update(
            arrival_tolerance_rad=0.01, hold_error_rad=0.05, final_mode="damping", zero_max_attempts=3,
            return_overspeed_kd=0.0,  # Explicit baseline; speed damping cases enable it separately.
            brake_kd=0.2, brake_max_torque_nm=0.5, brake_speed_ratio=1.5,
            brake_speed_margin_rad_s=0.2, brake_position_error_rad=math.radians(10),
            brake_confirm_s=0.1, brake_velocity_rad_s=0.02, brake_settle_s=0.2,
            brake_timeout_s=2.0, max_brake_attempts=3,
            brake_overspeed_confirm_s=0.05, brake_emergency_speed_ratio=2.0,
            motion_settle_position_rad=math.radians(0.1),
        )
    return motors, targets


def commissioned(module="left_arm", index=None):
    motors, targets = plan(module, index)
    for target in targets:
        target.settings.update(
            rate_hz=100.0, feedback_timeout_s=0.25, ls_zero_retry_until_success=False,
            stall_window_s=0.2, stall_confirm_s=0.1, position_epsilon_rad=0.001,
            stall_velocity_rad_s=0.02, stop_settle_s=0.05, arrival_settle_s=0.05,
            seek_speed_rad_s=0.1, return_speed_rad_s=1.5,
            bck_kp=1.0, kd=0.5, bck_kd=0.5, hold_kp=1.0, hold_kd=0.1,
            contact_torque_nm=0.02, max_torque_nm=2.0, max_temperature_c=80.0,
            max_seek_time_s=8.0, max_seek_travel_rad=0.8, max_return_time_s=8.0,
            max_feedback_step_rad=0.1,
        )
    return motors, targets


class Log:
    def __init__(self):
        self.events = []

    def write(self, event, **data):
        self.events.append((event, data))

    def check(self):
        pass


class Future:
    def __init__(self, result, ready=True):
        self.value, self.ready = result, ready

    def done(self):
        return self.ready

    def result(self):
        return self.value


def id_reply(motor, ids=None, result=0):
    return SimpleNamespace(result=result, motor_name=motor.name, motor_type=motor.protocol,
                           model=motor.model, bus_index=motor.bus, target_id=motor.motor_id,
                           detected_ids=[motor.motor_id] if ids is None else ids)


class Drive:
    """A bounded fake drive, asynchronous SETZERO and fresh sparse state snapshots."""
    def __init__(self, targets, log, offline=None, zero_timeout=False, disable_on_zero=False):
        self.now = 0.0
        self.feedback, self.enabled, self.positions, self.commands = {}, {}, {}, {}
        self.targets = {t.motor.index: t for t in targets}
        self.log, self.offline, self.zero_timeout = log, offline, zero_timeout
        self.disable_on_zero = disable_on_zero
        self.calls, self.zero_calls, self.command_batches = [], [], []
        self.pending = None
        self.zeroed = set()
        self.stop_positions = {t.motor.index: t.settings["seek_direction"] * 0.06 for t in targets}
        self.seq = 0
        self.dropped = set()
        self.stuck = set()
        self.stop_after_complete = True
        self.completed_at = None
        self.competitor = False

    def clock(self):
        return self.now

    def ready(self):
        return True

    def exclusive(self):
        if self.competitor:
            raise h.HomingError("Other motor publisher")

    def normal_mode(self):
        return Future(SimpleNamespace(values=[SimpleNamespace(type=1, bool_value=False),
                                               SimpleNamespace(type=1, bool_value=True),
                                               SimpleNamespace(type=4, string_value="new")]))

    def query(self, motor):
        self.calls.append(("query", motor.index, self.now))
        return Future(id_reply(motor, result=4 if motor.index == self.offline else 0))

    def enable(self, motors, enabled):
        for motor in motors:
            self.calls.append(("enable" if enabled else "disable", motor.index, self.now))
            self.enabled[motor.index] = enabled
            self.positions.setdefault(motor.index, 0.0)

    def command(self, commands):
        self.command_batches.append((self.now, list(commands)))
        self.commands = {motor.index: cmd for motor, cmd in commands}

    def zero(self, motor):
        p = self.targets[motor.index].settings
        assert self.enabled[motor.index] or (self.disable_on_zero and motor.index in self.zeroed)
        if motor.index not in self.zero_calls:
            assert self.commands[motor.index][1:] == (p["seek_direction"] * p["seek_speed_rad_s"], 0, p["kd"], p["seek_torque_ff_nm"])
        else:
            assert self.commands[motor.index][1] == 0 and self.commands[motor.index][2] == 0
        if motor.index in self.stuck:
            self.stop_positions[motor.index] = self.positions[motor.index]
        self.calls.append(("zero_request", motor.index, self.now))
        self.zero_calls.append(motor.index)
        response = SimpleNamespace(motor_types=[motor.index], results=[0], messages=["matched feedback"])
        future = Future(response, False)
        self.pending = (motor.index, self.now + 0.002, future)
        return future

    def spin(self, timeout):
        dt = max(0.001, timeout)
        self.now += dt
        self.seq += 1
        if self.pending and self.now >= self.pending[1] and not self.zero_timeout:
            index, _, future = self.pending
            self.zeroed.add(index)
            self.stop_positions[index] -= self.positions[index]
            self.positions[index] = 0.0
            if self.disable_on_zero:
                self.enabled[index] = False
            future.ready = True
            self.calls.append(("zero_ack", index, self.now))
            self.pending = None
        for index, q in list(self.positions.items()):
            desired, velocity, kp, kd, tau_ff = self.commands.get(index, (q, 0.0, 0.0, 0.0, 0.0))
            contact = False
            if self.enabled[index]:
                if kp and index in self.zero_calls:
                    q = desired
                elif kd and velocity:
                    stop = self.stop_positions[index]
                    q += velocity * dt
                    if index in self.stuck:
                        q = self.stop_positions[index] if index in self.zeroed else 0.0
                    elif velocity < 0:
                        q = max(stop, q)
                    else:
                        q = min(stop, q)
                    contact = q == stop or index in self.stuck
            dq = (q - self.positions[index]) / dt
            self.positions[index] = q
            if index not in self.dropped:
                self.feedback[index] = h.Feedback(self.now, self.seq, q, dq, 0.05 if contact else 0.001, 25.0, self.enabled[index])
        if self.completed_at is None and any(e == "CALIBRATION_PASS" for e, _ in self.log.events):
            self.completed_at = self.now
        if self.stop_after_complete and self.completed_at is not None and self.now - self.completed_at >= 0.5:
            self.stop_after_complete = False
            raise KeyboardInterrupt


def test_selection_order_and_offsets():
    _, left = plan()
    assert [t.motor.index for t in left] == [14, 13, 12]
    assert [t.settings["seek_direction"] for t in left] == [1, -1, 1]
    assert [math.degrees(t.zero_target) for t in left] == pytest.approx([-90, 90, -150])
    assert [t.motor.bus for t in left] == [5, 5, 5]
    assert [t.motor.motor_id for t in left] == [35, 34, 33]
    _, right = plan("right_arm")
    assert [t.motor.index for t in right] == [17, 16, 15]
    assert [t.settings["seek_direction"] for t in right] == [-1, 1, -1]
    assert [math.degrees(t.zero_target) for t in right] == pytest.approx([90, -90, 150])
    assert all(t.settings["direction_confirmed"] for t in left + right)
    for target in left + right:
        assert not h.motion_errors(target)
        p = target.settings
        assert p["contact_torque_nm"] < p["kd"] * p["seek_speed_rad_s"] <= p["max_torque_nm"]
    assert all(not h.motion_errors(t) for t in commissioned("right_arm")[1])
    for module, expected in {"left_leg": [5, 4, 3, 2, 1, 0],
                             "right_leg": [11, 10, 9, 8, 7, 6], "neck": [20, 19, 18]}.items():
        _, targets = plan(module)
        assert [t.motor.index for t in targets] == expected
        profiles = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")["joints"]
        assert all(t.settings["direction_confirmed"] is profiles[t.motor.name]["direction_confirmed"]
                   for t in targets)
    assert [t.motor.index for t in plan(index=14)[1]] == [14]
    assert [t.motor.index for t in plan(index=7)[1]] == [7]
    with pytest.raises(ValueError):
        plan(index=99)


def test_fixture_contact_angle_is_independent_of_motor_seek_sign():
    _, targets = commissioned(index=3)
    target = targets[0]
    for direction, home in ((1, 0.0), (-1, 0.0), (1, 135.0)):
        target.settings.update(seek_direction=direction, home_angle_deg=home,
                               seek_torque_ff_nm=direction * 0.3)
        assert not h.motion_errors(target)
        assert target.zero_target == pytest.approx(-math.radians(home))
        run = h.MotorRun(target, 0.0)
        command = run.step(0.01, h.Feedback(0.01, 1, 0.0, 0.0, 0.0, 25.0))
        assert command[1] == pytest.approx(direction * target.settings['seek_speed_rad_s'])
    for home in (None, True, '0', math.nan, math.inf, -math.inf):
        target.settings['home_angle_deg'] = home
        assert 'home_angle_deg must be finite' in h.motion_errors(target)
    target.settings['home_angle_deg'] = math.degrees(target.limits['p_max'] + 1)
    assert 'zero target exceeds protocol position range' in h.motion_errors(target)
    target.settings.update(home_angle_deg=0.0, seek_direction=0)
    assert 'seek_direction must be -1 or +1' in h.motion_errors(target)


def test_config_validation_and_precedence(tmp_path):
    motors, selected = commissioned()
    assert all(not h.motion_errors(t) for t in selected)
    _, placeholders = plan("left_leg")
    for target in placeholders:
        target.settings["direction_confirmed"] = False  # Fixture, independent of field confirmations.
    assert all(any("placeholder" in e for e in h.motion_errors(t)) for t in placeholders)
    for key, value in [("seek_speed_rad_s", None), ("max_torque_nm", math.nan),
                       ("home_angle_deg", math.nan), ("seek_direction", True),
                       ("seek_speed_rad_s", 0.001), ("contact_torque_nm", 2.0), ("kd", 0.02),
                       ("acceleration_rad_s2", 0.3)]:
        target = deepcopy(selected[0])
        # These low-speed/gain cases assume no feedforward helps reach contact torque.
        target.settings["seek_torque_ff_nm"] = 0.0
        target.settings[key] = value
        assert h.motion_errors(target), (key, value)
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    for name in document["modules"]["left_arm"]:
        document["joints"][name].pop("seek_speed_rad_s", None)
    document["defaults"]["seek_speed_rad_s"] = 0.02
    document["joints"]["left_elbow_roll_joint"]["seek_speed_rad_s"] = 0.03
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(document))
    _, targets = h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", "left_arm")
    assert [t.settings["seek_speed_rad_s"] for t in targets] == [0.03, 0.02, 0.02]
    _, targets = h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", "left_arm",
                             overrides={"seek_speed_rad_s": 0.04})
    assert all(t.settings["seek_speed_rad_s"] == 0.04 for t in targets)
    document["defaults"]["bck_kp"] = 1.0
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="per joint"):
        h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", "left_arm")
    with pytest.raises(SystemExit):
        h.parse_args(["--module", "left_arm", "--kd", "0.5"])
    with pytest.raises(SystemExit):
        h.parse_args(["--motor-index", "17", "--acceleration-rad-s2", "0.3"])
    # Field tuning can make gains equal; use explicit independent joint fixtures.
    document["defaults"].pop("bck_kp")
    names = document["modules"]["left_leg"]
    for i, name in enumerate(names):
        document["joints"][name].update(bck_kp=1.0 + i, kd=0.1 * (i + 1), bck_kd=0.2 * (i + 1))
    path.write_text(yaml.safe_dump(document))
    _, leg = h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", "left_leg")
    assert [t.settings["bck_kp"] for t in leg] == [1.0 + i for i in range(len(names))]
    assert [t.settings["kd"] for t in leg] == pytest.approx([0.1 * (i + 1) for i in range(len(names))])
    assert [t.settings["bck_kd"] for t in leg] == pytest.approx([0.2 * (i + 1) for i in range(len(names))])
    path.write_text("defaults: {}\ndefaults: {}\n")
    with pytest.raises(ValueError, match="Duplicate"):
        h.read_yaml(path)


@pytest.mark.parametrize("effort", ["2.4", None])
def test_torque_threshold_uses_protocol_range_without_urdf_effort(tmp_path, effort):
    import xml.etree.ElementTree as ET
    tree = ET.parse(PACKAGE / "config/l0_v5.urdf")
    limit = tree.getroot().find("joint[@name='right_shoulder_pitch_joint']/limit")
    if effort is None:
        limit.attrib.pop("effort")
    else:
        limit.set("effort", effort)
    path = tmp_path / "test.urdf"
    tree.write(path)
    _, targets = h.load_plan(h.motor_config_path(), PACKAGE / "config/motor_zero_calibration.yaml",
                             path, motor_index=15, overrides={"max_torque_nm": 3.0})
    target = targets[0]
    assert not h.motion_errors(target)
    assert target.zero_target == pytest.approx(math.radians(150))
    target.settings["max_torque_nm"] = 3.01
    assert "torque threshold exceeds protocol range" in h.motion_errors(target)


@pytest.mark.parametrize("limit", [0.5, 2.0])
def test_temperature_cutoff_can_be_disabled_without_disabling_other_guards(limit):
    _, targets = plan(index=17)
    target = targets[0]
    target.settings["max_torque_nm"] = limit
    assert target.settings["max_temperature_c"] is None
    assert not h.motion_errors(target)
    run = h.MotorRun(target, 0.0)
    run.step(0.01, h.Feedback(0.01, 1, 0.0, 0.0, 0.0, 60.0))
    assert run.phase == "SEEK"
    with pytest.raises(h.HomingError, match="TORQUE_LIMIT") as caught:
        run.step(0.02, h.Feedback(0.02, 2, 0.0, 0.0, limit + 0.1, 60.0))
    assert caught.value.diagnostics["feedback"]["tau_nm"] == limit + 0.1
    assert caught.value.diagnostics["max_torque_nm"] == limit
    with pytest.raises(h.HomingError, match="INVALID_FEEDBACK"):
        run.step(0.03, h.Feedback(0.03, 3, 0.0, 0.0, 0.0, math.nan))
    target.settings["max_temperature_c"] = 45.0
    with pytest.raises(h.HomingError, match="TEMPERATURE_LIMIT") as caught:
        run.step(0.04, h.Feedback(0.04, 4, 0.0, 0.0, 0.0, 60.0))
    assert caught.value.diagnostics["feedback"]["temperature_c"] == 60.0
    del target.settings["max_temperature_c"]
    assert any("max_temperature_c" in error for error in h.motion_errors(target))


@pytest.mark.parametrize("zero_code", [2047, 2048])
def test_quantized_mr_feedback_completes_calibration(zero_code):
    motors, targets = plan(index=17)
    targets[0].settings.update(kd=0.3, bck_kd=0.3, hold_kd=0.2)  # Keep margin for either zero code.
    assert not h.motion_errors(targets[0])
    # Position-window speeds do not inherit MIT velocity quantization floors.
    low_velocity = deepcopy(targets[0])
    low_velocity.settings["stall_velocity_rad_s"] = 0.01
    assert not h.motion_errors(low_velocity)

    class QuantizedDrive(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            for index, f in list(self.feedback.items()):
                limits = self.targets[index].limits
                low, span = limits["v_min"], limits["v_max"] - limits["v_min"]
                raw = zero_code if f.dq == 0 else int((f.dq - low) / span * 4095)
                dq = raw / 4095 * span + low
                self.feedback[index] = h.Feedback(f.received_at, f.sequence, f.q, dq, f.tau, f.temperature, f.enabled)

    log = Log()
    io = QuantizedDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.preflight(True)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and io.zero_calls == [17] * 3
        assert any(event == "MOTOR_PASS" for event, _ in log.events)
    finally:
        runner.shutdown()
    assert not io.enabled[17]


@pytest.mark.parametrize("key", h.FEEDFORWARD)
def test_feedforward_yaml_validation_and_legacy_default(tmp_path, key):
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    profile = document["joints"]["right_elbow_roll_joint"]
    # Keep both feedforward signs reachable regardless of field speed tuning.
    profile.update(max_torque_nm=2.0, kd=0.3, bck_kd=0.3, seek_speed_rad_s=0.47, contact_torque_nm=0.05)
    for name in h.FEEDFORWARD:
        profile.pop(name, None)
    path = tmp_path / "ff.yaml"
    def load():
        path.write_text(yaml.safe_dump(document))
        return h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", motor_index=17)[1][0]
    assert all(load().settings[name] == 0.0 for name in h.FEEDFORWARD)
    for value in (-0.05, 0.05):
        profile[key] = value
        target = load()
        assert target.settings[key] == value and not h.motion_errors(target)
    for value in (None, True, "0.1", math.nan, math.inf, -math.inf, 3.1, -3.1, 2.5):
        profile[key] = value
        assert any(key in error for error in h.motion_errors(load()))
    profile[key] = 0.0
    document["defaults"][key] = 0.1
    with pytest.raises(ValueError, match="per joint"):
        load()


def test_feedforward_is_in_static_and_runtime_total_torque_budget():
    motors, targets = plan(index=17)
    target = targets[0]
    target.settings.update(kd=0.3, bck_kd=0.3, seek_speed_rad_s=0.47, max_torque_nm=0.5,
                           seek_torque_ff_nm=-0.4)
    assert any("exceeds torque budget" in error for error in h.motion_errors(target))
    target.settings["seek_torque_ff_nm"] = -0.3
    assert not h.motion_errors(target)
    # The same command is below the budget at rest but over it with opposing velocity.
    io = Drive(targets, Log())
    runner = h.Runner(io, motors, targets, io.log, clock=io.clock)
    run = h.MotorRun(target, 0)
    runner.runs.append(run)
    io.feedback[17] = h.Feedback(0, 1, 0, 0, 0, 25)
    runner.pump(force=True)
    assert io.commands[17][-1] == -0.3
    assert run.published_commands[-1]["tau_ff_nm"] == -0.3
    io.feedback[17] = h.Feedback(io.now, 2, 0, 0.3, 0, 25)
    with pytest.raises(h.HomingError, match="COMMAND_TORQUE_BUDGET") as caught:
        runner.pump(force=True)
    assert caught.value.diagnostics["command"]["tau_ff_nm"] == -0.3
    assert caught.value.diagnostics["torque_estimate"]["total_nm"] == pytest.approx(-0.531)
    assert len(io.command_batches) == 1


def test_diagnostics_keep_published_commands_but_exclude_rejected_or_failed_publish():
    motors, targets = plan(index=17)
    targets[0].settings.update(kd=0.5, bck_kd=0.5, seek_speed_rad_s=1.0, max_torque_nm=0.5)
    log = Log()
    io = Drive(targets, log)
    run = h.MotorRun(targets[0], 0.0)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.runs.append(run)
    io.now = 0.01
    io.feedback[17] = h.Feedback(io.now, 1, 0.0, 0.0, 0.0, 25.0)
    runner.pump(force=True)
    assert len(run.published_commands) == 1
    previous = run.published_commands[0]
    assert previous["dq_rad_s"] < 0 and previous["kp"] == 0
    assert previous["feedback"]["sequence"] == 1

    io.feedback[17] = h.Feedback(io.now, 2, 0.0, 3.0, 0.0, 25.0)
    with pytest.raises(h.HomingError, match="COMMAND_TORQUE_BUDGET") as caught:
        runner.pump(force=True)
    assert caught.value.diagnostics["command"]["published"] is False
    assert caught.value.diagnostics["recent_ros_commands"] == [previous]
    assert len(io.command_batches) == 1

    io.feedback[17] = h.Feedback(io.now, 3, 0.0, 0.0, 0.0, 25.0)
    def failed_publish(commands):
        raise RuntimeError("publish failed")
    io.command = failed_publish
    with pytest.raises(RuntimeError, match="publish failed"):
        runner.pump(force=True)
    assert list(run.published_commands) == [previous]


def test_dry_run_and_missing_configuration_never_create_ros_or_logs(monkeypatch, tmp_path, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError("must not create runtime")
    monkeypatch.setattr(h, "make_ros_io", forbidden)
    monkeypatch.setattr(h, "Logger", forbidden)
    args = ["--motor-index", "14", "--log-dir", str(tmp_path / "logs")]
    assert h.main([*args, "--dry-run"]) == 0
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    document["joints"]["left_elbow_roll_joint"]["max_torque_nm"] = None
    incomplete = tmp_path / "incomplete.yaml"
    incomplete.write_text(yaml.safe_dump(document))
    assert h.main([*args, "--calibration-config", str(incomplete)]) == 2
    assert not (tmp_path / "logs").exists()
    assert "motion_errors" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        h.parse_args(["--module", "left_arm", "--motor-index", "14"])


def test_id_inventory_metadata_and_offline_gate():
    motors, targets = commissioned()
    motor = targets[0].motor
    assert h.validate_id_reply(motor, id_reply(motor), {35}, {35}) == []
    with pytest.raises(h.HomingError):
        h.validate_id_reply(motor, id_reply(motor, []), {35}, {35})
    mismatch = id_reply(motor)
    mismatch.bus_index = 2
    with pytest.raises(h.HomingError, match="configuration mismatch"):
        h.validate_id_reply(motor, mismatch, {35}, {35})
    ls = next(m for m in motors if m.name == "neck_yaw_joint")
    assert h.validate_id_reply(ls, id_reply(ls, [65, 66, 67]), {65}, {65, 66, 67}) == [66, 67]
    for ids in ([66, 67], [65, 99]):
        with pytest.raises(h.HomingError):
            h.validate_id_reply(ls, id_reply(ls, ids), {65}, {65, 66, 67})
    log = Log()
    io = Drive(targets, log, offline=13)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    with pytest.raises(h.HomingError):
        runner.preflight(True)
    runner.shutdown()
    assert not runner.owned and not io.zero_calls
    assert all(call[0] == "query" for call in io.calls)


def test_full_sequence_dispatches_and_keeps_all_completed_motors_in_pure_damping():
    motors, targets = plan()
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.preflight(True)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and runner.completed == [14, 13, 12]
        assert io.zero_calls == [14] * 3 + [13] * 3 + [12] * 3
        sent = [d for e, d in log.events if e == "ZERO_SENT"]
        assert len(sent) == 9 and all(not d["device_verified"] for d in sent)
        assert not any(c[0] == "disable" for c in io.calls)
        assert next(d for e, d in log.events if e == "CALIBRATION_PASS")["state"] == "DAMPING"
        for target in targets:
            index, settings = target.motor.index, target.settings
            assert len([c for c in io.calls if c[0] == "enable" and c[1] == index]) > 9
            command = io.commands[index]
            assert command == (0, 0, 0, settings["damping_kd"], 0)
            stamps = [t for t, batch in io.command_batches if any(m.index == index for m, _ in batch)]
            assert max(b-a for a,b in zip(stamps, stamps[1:])) <= 1 / runner.rate + 1e-6
        assert len([t for t, _ in io.command_batches if t >= io.completed_at]) >= 49
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())



@pytest.mark.parametrize("yaml_mode,cli_mode,expected", [
    (True, None, True), (False, None, False), ("missing", None, True),
    (True, False, False), (False, True, True), ("false", None, None), (None, None, None),
])
def test_write_zero_yaml_default_overrides_and_cleanup(monkeypatch, tmp_path, yaml_mode, cli_mode, expected):
    import json
    saved = {}
    actual_runner = h.Runner

    def create_io(selected, *args):
        io = Drive(selected, Log())
        def close():
            assert not any(io.enabled.values())
            saved["closed"] = True
        io.close = close
        saved["io"] = io
        return io

    def create_runner(io, motors, selected, logger, timeout):
        runner = actual_runner(io, motors, selected, logger, timeout, io.clock)
        original_spin = io.spin
        def spin(timeout):
            original_spin(timeout)
            if runner.all_done and not saved.get("interrupted"):
                saved["interrupted"] = True
                raise KeyboardInterrupt
        io.spin = spin
        return runner

    monkeypatch.setattr(h, "make_ros_io", create_io)
    monkeypatch.setattr(h, "Runner", create_runner)
    assert h.parse_args(["--motor-index", "17"]).write_zero is None
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    document["joints"]["right_elbow_roll_joint"]["zero_max_attempts"] = 3  # Explicit test scenario, independent of live tuning.
    assert document["write_zero"] is True
    if yaml_mode == "missing":
        document.pop("write_zero")
    else:
        document["write_zero"] = yaml_mode
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(document))
    args = ["--motor-index", "17", "--calibration-config", str(path), "--log-dir", str(tmp_path)]
    if cli_mode is not None:
        args.append("--write-zero" if cli_mode else "--no-write-zero")
    if expected is None:
        assert h.main(args) == 2
        assert "io" not in saved and not list(tmp_path.glob("*.jsonl"))
        return
    assert h.main(args) == 0
    assert saved["closed"] and saved["io"].zero_calls == ([17] * 3 if expected else [])
    records = [json.loads(line) for line in next(tmp_path.glob("*.jsonl")).read_text().splitlines()]
    assert next(r for r in records if r["event"] == "RUN_START")["write_zero"] is expected
    if expected:
        assert any(r["event"] == "CALIBRATION_PASS" for r in records)
        assert not any(r["event"] == "CONTACT_CANDIDATE" for r in records)
    else:
        candidate = next(r for r in records if r["event"] == "CONTACT_CANDIDATE")
        assert not candidate["zero_written"] and not candidate["mechanical_stop_confirmed"]
        assert not any(r["event"] in {"ZERO_REQUEST", "MOTOR_PASS", "CALIBRATION_PASS"} for r in records)
    assert records[-1]["event"] == "RUN_END" and records[-1]["exit_code"] == 0


@pytest.mark.parametrize("setting,expected", [("", None), ("true", True), ("false", False)])
def test_launch_write_zero_inherits_or_overrides_yaml(monkeypatch, setting, expected):
    import importlib.util
    pytest.importorskip("launch_ros")
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import TextSubstitution
    from launch.utilities import perform_substitutions
    spec = importlib.util.spec_from_file_location("zero_launch_test", PACKAGE / "launch/motor_auto_zero.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "FindPackageShare", lambda name: TextSubstitution(text=str(PACKAGE)))
    context = LaunchContext()
    for declaration in module.generate_launch_description().entities:
        if isinstance(declaration, DeclareLaunchArgument):
            context.launch_configurations[declaration.name] = perform_substitutions(context, declaration.default_value)
    assert context.launch_configurations["write_zero"] == ""
    context.launch_configurations.update(motor_index="17", write_zero=setting)
    saved = {}
    monkeypatch.setattr(module, "Node", lambda **kwargs: saved.update(kwargs) or object())
    monkeypatch.setattr(module, "OnProcessExit", lambda **kwargs: None)
    monkeypatch.setattr(module, "RegisterEventHandler", lambda handler: None)
    module.start(context)  # Construct arguments only; no ROS node or launch service runs.
    assert h.parse_args(saved["arguments"]).write_zero is expected




@pytest.mark.parametrize("index", range(21))
def test_seek_preload_changes_only_for_send_only_protocols(index):
    _, targets = commissioned(index=index)
    target = targets[0]
    run = h.MotorRun(target, 0)
    direction = target.settings["seek_direction"]
    target.settings["seek_torque_ff_nm"] = direction * 0.15
    for now, sequence, q in [(0.01, 1, 0.2), (0.02, 2, 0.2 + direction * 0.01)]:
        command = run.step(now, h.Feedback(now, sequence, q, 0, 0, 25))
        assert run.phase == "SEEK" and command[0] == q and command[1] * direction > 0
        assert command[1] == direction * target.settings["seek_speed_rad_s"]
        assert command[2:] == (0.0, target.settings["kd"], target.settings["seek_torque_ff_nm"])
    run.contact_q = q
    for phase in ("STOP", "ZERO_REQUEST", "ZERO_PREPARE", "ZERO_PENDING"):
        run.transition(phase, 0.03)
        command = run.step(0.04, h.Feedback(0.04, 3, q, 0, 0, 25))
        assert command[1:] == (direction * target.settings["seek_speed_rad_s"], 0.0, target.settings["kd"], target.settings["seek_torque_ff_nm"])
    run.zero_sent = True
    for phase in ("ZERO_PENDING",):
        run.zero_attempts = 2
        run.transition(phase, 0.05)
        command = run.step(0.06, h.Feedback(0.06, 4, 0, 0, 0, 25))
        assert command == ((0.0, direction * target.settings["seek_speed_rad_s"], 0.0, target.settings["kd"], target.settings["seek_torque_ff_nm"])
                           if target.motor.protocol == "LS" else (0.0, 0.0, 0.0, 0.0, 0.0))
    command = run.step(0.07, h.Feedback(0.07, 5, 0, -direction * 0.1, 0, 25))
    assert command[1:] == ((direction * target.settings["seek_speed_rad_s"], 0.0, target.settings["kd"], target.settings["seek_torque_ff_nm"])
                           if target.motor.protocol == "LS" else (0, 0, 0, 0))


@pytest.mark.parametrize("service_timeout", [0.3, 10.0])
def test_zero_timeout_never_retries_or_starts_next_motor(service_timeout):
    motors, targets = commissioned()
    log = Log()
    io = Drive(targets, log, zero_timeout=True)
    runner = h.Runner(io, motors, targets, log, timeout=service_timeout, clock=io.clock)
    runner.preflight(True)
    with pytest.raises(h.HomingError, match="ZERO_SEND_TIMEOUT"):
        runner.calibrate(write_zero=True)
    requested_at = next(t for kind, _, t in io.calls if kind == "zero_request")
    assert io.clock() - requested_at <= service_timeout + 0.02
    runner.shutdown()
    assert io.zero_calls == [14]
    assert set(runner.owned) == {14}
    assert not io.enabled[14]
    assert io.commands[14] == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_dispatched_write_with_old_coordinates_blocks_return():
    motors, targets = commissioned(index=13)
    log = Log()
    class OffsetDrive(Drive):
        def spin(self, timeout):
            pending = self.pending
            super().spin(timeout)
            if pending and self.pending is None:
                self.positions[13] = self.stop_positions[13] = 0.3
                self.feedback[13] = h.Feedback(self.now, self.seq, 0.3, 0, 0, 25, self.enabled[13])
    io = OffsetDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(h.HomingError, match="RETURN_READY_TIMEOUT"):
            runner.calibrate(write_zero=True)
        assert not runner.completed and io.zero_calls == [13] * 3
        assert not any(e == "RETURN_START" for e, _ in log.events)
    finally:
        runner.shutdown()



@pytest.mark.parametrize("fault", ["moving", "stale", "repeat_ack_timeout", "service_failure", "motor_fault"])
def test_repeated_zero_does_not_bypass_feedback_or_service_guards(fault):
    motors, targets = commissioned(index=13)
    targets[0].settings.update(zero_max_attempts=3, zero_tolerance_rad=0.02)
    log = Log()
    class FaultDrive(Drive):
        def zero(self, motor):
            if fault == "repeat_ack_timeout" and self.zero_calls:
                self.zero_timeout = True
            future = super().zero(motor)
            if fault == "service_failure":
                future.value.results = [4]
            return future
        def spin(self, timeout):
            super().spin(timeout)
            if self.zero_calls and self.pending is None:
                if fault == "motor_fault":
                    raise h.HomingError("invalid state / motor fault 1")
                q = 0.1 * self.now if fault == "moving" else 0.0
                self.positions[13] = self.stop_positions[13] = q
                if fault != "stale":
                    self.feedback[13] = h.Feedback(self.now, self.seq, q,
                                                   0.1 if fault == "moving" else 0, 0, 25, self.enabled[13])
                else:
                    self.dropped.add(13)
    io = FaultDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    reason = {"moving": "RETURN_READY_TIMEOUT", "stale": "FEEDBACK_TIMEOUT",
              "repeat_ack_timeout": "ZERO_SEND_TIMEOUT", "service_failure": "ZERO_SEND_FAILED",
              "motor_fault": "motor fault"}[fault]
    try:
        with pytest.raises(h.HomingError, match=reason):
            runner.calibrate(write_zero=True)
        assert len(io.zero_calls) == ({"repeat_ack_timeout": 2, "moving": 3, "stale": 3, "motor_fault": 1}.get(fault, 1))
        assert not runner.completed
    finally:
        runner.shutdown()
    assert not io.enabled[13]


def test_zero_attempt_config_bounds():
    _, targets = plan(index=13)
    target = targets[0]
    for value in (0, -1, 3.0, True, None, "3"):
        target.settings["zero_max_attempts"] = value
        assert any("zero_max_attempts" in e for e in h.motion_errors(target))
    for value in (1, 2, 3, 4, 5):
        target.settings["zero_max_attempts"] = value
        assert not h.motion_errors(target)


@pytest.mark.parametrize("phase", ["ZERO_PENDING"])
def test_zero_stage_position_excursions_no_longer_abort(phase):
    _, targets = commissioned(index=17)
    run = h.MotorRun(targets[0], 0)
    run.contact_q, run.zero_sent = 2.8, True
    run.transition(phase, 0)
    for i, q in enumerate((2.8, 0.0, -2.672), 1):
        command = run.step(i / 100, h.Feedback(i / 100, i, q, 0, 0, 25))
        assert command == (0, 0, 0, 0, 0)


@pytest.mark.parametrize("index,protocol,count", [(17, "MR", 3), (0, "RS", 3), (18, "LS", 1)])
def test_protocol_write_count_and_feedback_acceptance(index, protocol, count):
    motors, targets = commissioned(index=index)
    assert targets[0].motor.protocol == protocol
    targets[0].settings["zero_max_attempts"] = 3
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.completed == [index] and io.zero_calls == [index] * count
        accepted = [d for e, d in log.events if e == ("ZERO_CONFIRMED" if protocol == "LS" else "ZERO_SENT")]
        assert len(accepted) == count
        assert all(d["device_verified"] is (protocol == "LS") for d in accepted)
        enables = [i for kind, i, _ in io.calls if kind == "enable"]
        assert all(i == index for i in enables)
        if protocol == "LS":
            assert len(enables) == 2  # Initial enable, then one after the ACK.
        else:
            assert len(enables) > 3 * count
        # Fresh normal feedback is emitted while each service is pending, but
        # acceptance still waits for the service reply (dedicated ACK for LS).
        replies = [at for kind, _, at in io.calls if kind == "zero_ack"]
        assert len(replies) == count
    finally:
        runner.shutdown()


@pytest.mark.parametrize("enable_failure", [False, True])
def test_single_reenable_restores_return_or_aborts(enable_failure):
    motors, targets = commissioned(index=14)
    log = Log()
    class ResetDrive(Drive):
        def enable(self, motors, enabled):
            if enabled and self.zero_calls:
                assert runner.current.phase in {"ZERO_PENDING", "CATCH"}
                if enable_failure:
                    raise h.HomingError("reenable publish failed")
            super().enable(motors, enabled)
    io = ResetDrive(targets, log, disable_on_zero=True)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        if enable_failure:
            with pytest.raises(h.HomingError, match="reenable publish failed"):
                runner.calibrate(write_zero=True)
            assert not runner.completed
            assert not any(e == "RETURN_START" for e, _ in log.events)
        else:
            with pytest.raises(KeyboardInterrupt):
                runner.calibrate(write_zero=True)
            assert runner.completed == [14]
            assert io.positions[14] == pytest.approx(targets[0].zero_target)
            assert len([i for kind, i, _ in io.calls if kind == "enable"]) > 9
            requests = [d for e, d in log.events if e == "ZERO_SENT"]
            assert len(requests) == 3 and all(d["immediate_enable_count"] == 3 and d["continuous_reenable"] for d in requests)
            assert runner.runs[0].zero_enable_count > 9
        stamps = [t for t, _ in io.command_batches]
        assert max(b - a for a, b in zip(stamps, stamps[1:])) <= 0.010001
    finally:
        runner.shutdown()
    assert not io.enabled[14]


def test_ls_normal_feedback_cannot_replace_dedicated_ack():
    motors, targets = commissioned(index=18)
    log = Log()
    io = Drive(targets, log, zero_timeout=True)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(h.HomingError, match="LS_ZERO_SERVICE_TIMEOUT"):
            runner.calibrate(write_zero=True)
        assert io.zero_calls == [18] and not runner.completed
        assert io.now - io.feedback[18].received_at < 0.02  # MIT feedback is still flowing.
        assert not any(e == "ZERO_WRITE_ACCEPTED" for e, _ in log.events)
        assert all(i == 18 for kind, i, _ in io.calls if kind == "enable")
    finally:
        runner.shutdown()


def test_direct_speed_rejects_quantized_feedback_above_torque_budget():
    motors, targets = plan(index=17)
    targets[0].settings.update(kd=0.5, bck_kd=0.5, seek_speed_rad_s=1.0, max_torque_nm=0.5)
    log = Log()
    io = Drive(targets, log)
    io.now = 0.01
    io.feedback[17] = h.Feedback(0.01, 1, 0, 65 / 4095, 0, 25)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.runs.append(h.MotorRun(targets[0], 0))
    with pytest.raises(h.HomingError, match="COMMAND_TORQUE_BUDGET") as caught:
        runner.pump()
    assert caught.value.diagnostics["torque_estimate"]["total_nm"] == pytest.approx(-0.5079365079365079)
    assert not io.command_batches and not io.zero_calls


@pytest.mark.parametrize("with_history", [False, True])
def test_torque_budget_diagnostics_and_rejected_command(with_history):
    motors, targets = plan(index=17)
    targets[0].settings.update(kd=0.2, bck_kd=0.2, seek_speed_rad_s=1.0, max_torque_nm=0.5)
    log = Log()
    io = Drive(targets, log)
    run = h.MotorRun(targets[0], 0.0)
    feedback = h.Feedback(0.11, 2, 0.001, 3.0, 0.02, 25.0)
    if with_history:
        run.step(0.10, h.Feedback(0.10, 1, 0.0, 0.0, 0.01, 25.0))
        run.step(0.11, feedback)
        run.step(0.12, feedback)  # Repeated snapshot must preserve the preceding fresh one.
    io.now = 0.13
    io.feedback[17] = feedback
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.runs.append(run)
    with pytest.raises(h.HomingError, match="COMMAND_TORQUE_BUDGET") as caught:
        runner.pump()
    details = caught.value.diagnostics
    assert not io.command_batches  # Offending command was blocked before publish.
    assert details["command"] == pytest.approx({
        "q_rad": 0.001, "dq_rad_s": -1.0, "kp": 0.0, "kd": 0.2,
        "tau_ff_nm": 0.0, "published": False})
    assert details["feedback"]["dq_rad_s"] == 3.0
    assert details["feedback"]["tau_nm"] == 0.02  # Distinct from the command estimate.
    assert details["feedback"]["age_s"] == pytest.approx(0.02)
    assert details["torque_estimate"] == pytest.approx({
        "position_term_nm": 0.0, "velocity_term_nm": -0.8,
        "total_nm": -0.8, "limit_nm": 0.5})
    if with_history:
        assert details["previous_feedback"]["sequence"] == 1
        assert details["feedback_interval_s"] == pytest.approx(0.01)
        assert details["position_delta_rad"] == pytest.approx(0.001)
        assert details["position_rate_from_ros_receive_rad_s"] == pytest.approx(0.1)
    else:
        assert details["previous_feedback"] is None
        assert details["position_rate_from_ros_receive_rad_s"] is None


@pytest.mark.parametrize("at_stop", [True, False])
@pytest.mark.parametrize("write_zero", [True, False])
def test_stationary_start_uses_same_contact_and_return_flow(at_stop, write_zero):
    motors, targets = commissioned(index=17)
    log = Log()
    io = Drive(targets, log)
    if at_stop:
        io.positions[17] = targets[0].settings["seek_direction"] * 0.06
    else:
        io.stuck.add(17)  # Accepted by design: a stationary obstruction also meets the stall test.
    initial_q = io.positions.get(17, 0.0)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.preflight(True)
    started = io.clock()
    if write_zero:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert io.zero_calls == [17] * 3
        assert runner.completed == [17] and runner.all_done
        assert io.positions[17] == pytest.approx(targets[0].zero_target)
        event = next(data for name, data in log.events if name == "MOTOR_PASS")
    else:
        runner.calibrate(write_zero=False)
        assert not io.zero_calls and not runner.completed
        event = next(data for name, data in log.events if name == "CONTACT_CANDIDATE")
        assert event["travel_rad"] == 0.0
    assert event["contact_position_rad"] == initial_q
    assert io.clock() - started >= targets[0].settings["stall_window_s"] + targets[0].settings["stall_confirm_s"]
    runner.shutdown()
    assert not io.enabled[17]


def test_stationary_start_still_requires_contact_torque_and_seek_timeout():
    _, targets = commissioned(index=17)
    run = h.MotorRun(targets[0], 0.0)
    for i in range(401):
        now = i / 100
        run.step(now, h.Feedback(now, i, 0.0, -0.01587677, 0.001, 25.0))
    assert run.phase == "SEEK" and not run.zero_sent  # No 3-second startup-motion error.
    with pytest.raises(h.HomingError, match="SEEK_TIME_OR_TRAVEL_LIMIT"):
        run.step(8.01, h.Feedback(8.01, 402, 0.0, -0.01587677, 0.001, 25.0))


@pytest.mark.parametrize("failure", ["feedback", "competitor"])
def test_feedback_loss_and_competing_controller(failure):
    motors, targets = commissioned(index=14)
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.preflight(True)
    if failure == "feedback":
        io.dropped.add(14)
    else:
        io.competitor = True
    with pytest.raises(h.HomingError):
        runner.calibrate(write_zero=True)
    runner.shutdown()
    assert not io.zero_calls and not io.enabled[14]


def test_no_false_contact_on_slow_motion_and_stale_samples():
    _, targets = commissioned(index=14)
    run = h.MotorRun(targets[0], 0.0)
    velocity = targets[0].settings["seek_direction"] * 0.02
    for i in range(101):
        now = i / 100
        run.step(now, h.Feedback(now, i, velocity * now, velocity, 0.1, 25.0))
    assert run.phase == "SEEK"
    with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
        run.step(1.3, run.last)


@pytest.mark.parametrize("change,reason", [("reverse", "WRONG_DIRECTION"), ("jump", "POSITION_JUMP"),
                                           ("torque", "TORQUE"), ("heat", "TEMPERATURE"), ("nan", "INVALID")])
def test_feedback_faults(change, reason):
    _, targets = commissioned(index=14)
    run = h.MotorRun(targets[0], 0.0)
    run.step(0.0, h.Feedback(0, 0, 0, 0, 0, 25))
    q, tau, temp = 0.0, 0.0, 25.0
    if change == "reverse": q = -targets[0].settings["seek_direction"] * 0.02
    if change == "jump": q = -1.0
    if change == "torque": tau = 3.0
    if change == "heat": temp = 100.0
    if change == "nan": q = math.nan
    with pytest.raises(h.HomingError, match=reason):
        run.step(0.01, h.Feedback(0.01, 1, q, 0, tau, temp))


def test_hold_failure_and_trajectory_bounds():
    _, targets = commissioned(index=14)
    target = targets[0]
    duration = h.trajectory_duration(abs(target.zero_target), target.settings)
    speed = target.settings["return_speed_rad_s"]
    assert duration == pytest.approx(abs(target.zero_target) / speed)
    for end in (abs(target.zero_target), -abs(target.zero_target)):
        velocity = math.copysign(speed, end)
        assert h.trajectory(0, end, 0, duration) == pytest.approx((0, velocity))
        assert h.trajectory(0, end, duration / 2, duration) == pytest.approx((end / 2, velocity))
        assert h.trajectory(0, end, duration, duration) == (end, 0)
        assert h.trajectory(0, end, duration + 1, duration) == (end, 0)
    assert h.trajectory(0, 1, -0.1, 1) == (0, 0)
    assert h.trajectory(0.2, 0.2, 0, h.trajectory_duration(0, target.settings)) == (0.2, 0)
    run = h.MotorRun(target, 0)
    run.phase = "HOLD"
    command = run.step(0.01, h.Feedback(0.01, 1, target.zero_target + 0.1, 0, 0, 25))
    assert run.phase == "RECOVER" and command[2] == target.settings["bck_kp"]
    run = h.MotorRun(target, 0)  # A separate ordinary HOLD scenario.
    run.phase = "HOLD"
    assert run.step(0.02, h.Feedback(0.02, 2, target.zero_target, 0, 0, 25)) == (
        target.zero_target, 0.0, target.settings["hold_kp"], target.settings["hold_kd"], target.settings["hold_torque_ff_nm"])
    with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
        run.step(0.3, run.last)


def test_slow_log_output_does_not_block_control(tmp_path, monkeypatch):
    from threading import Event
    entered, release = Event(), Event()
    def slow_print(*args, **kwargs):
        entered.set()
        assert release.wait(2.0)
    monkeypatch.setattr("builtins.print", slow_print)
    log = h.Logger(tmp_path)
    try:
        log.write("TEST")
        assert entered.wait(1.0)
        motors, targets = commissioned(index=14)
        io = Drive(targets, Log())
        runner = h.Runner(io, motors, targets, log, clock=io.clock)
        runner.runs.append(h.MotorRun(targets[0], io.clock()))
        io.enable([targets[0].motor], True)
        for _ in range(50):
            runner.pump()
        assert len(io.command_batches) == 50
        assert not release.is_set()
        log.error = OSError("disk full")
        with pytest.raises(h.HomingError, match="LOG_WRITE_FAILED"):
            runner.pump()
        assert len(io.command_batches) == 50
        log.error = None
    finally:
        release.set()
        log.close()
    assert '"event": "TEST"' in log.path.read_text()


def test_shutdown_keeps_trying_disable_after_command_failure():
    motors, targets = commissioned(index=14)
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.owned[14] = targets[0].motor
    def broken(commands):
        raise OSError("transport lost")
    io.command = broken
    runner.shutdown()
    assert [c[0] for c in io.calls] == ["disable"] * 10
    assert log.events[-1][1]["disable_confirmed"] is False
    assert log.events[-1][1]["errors"]


@pytest.mark.parametrize("failure", [None, "spin", "disable"])
def test_shutdown_retries_are_spaced_and_send_no_mit_after_disable(monkeypatch, failure):
    motors, targets = commissioned()
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    owned = [t.motor for t in targets[:2]]
    runner.owned = {m.index: m for m in owned}
    runner.runs = [h.MotorRun(t, io.clock()) for t in targets[:2]]
    events = []

    def command(batch):
        assert not runner.runs
        events.append(("neutral", batch, io.clock()))

    def enable(batch, enabled):
        assert enabled is False and batch == owned
        events.append(("disable", batch, io.clock()))
        if failure == "disable" and len(events) == 2:
            raise OSError("first disable publish failed")

    def spin(timeout):
        assert timeout == 0.0
        if failure == "spin":
            raise h.HomingError("feedback callback failed")
        # Model continuously ready callbacks: spin returns without any delay.

    def sleep(duration):
        io.now += duration

    io.command, io.enable, io.spin = command, enable, spin
    monkeypatch.setattr(h.time, "sleep", sleep)
    assert runner.shutdown() is (failure is None)
    assert [e[0] for e in events] == ["neutral"] + ["disable"] * 10
    assert events[0][1] == [(m, (0.0,) * 5) for m in owned]
    assert [e[2] for e in events[1:]] == pytest.approx([i * 0.05 for i in range(10)])
    assert io.clock() == pytest.approx(0.5)  # Includes final publish drain time.
    assert log.events[-1][1]["disable_confirmed"] is False


def test_limits_reference_rebound_and_zero_confirmation():
    _, targets = commissioned(index=14)
    target = targets[0]
    for key, value in [("bck_kp", 99999.0), ("kd", 999.0), ("hold_kd", 999.0),
                       ("return_speed_rad_s", 100.0), ("max_torque_nm", 100.0)]:
        bad = deepcopy(target)
        bad.settings[key] = value
        assert h.motion_errors(bad)
    run = h.MotorRun(target, 0)
    run.phase, run.contact_q = "ZERO_PREPARE", -0.06
    with pytest.raises(h.HomingError, match="CONTACT_REFERENCE_MOVED"):
        run.step(0.1, h.Feedback(0.1, 1, -0.04, 0, 0, 25))


@pytest.mark.parametrize("phase", ["SEEK", "ZERO_PENDING", "MOVE", "ALL_HOLD"])
@pytest.mark.parametrize("cause", ["sigint", "sigterm", "fault"])
def test_main_full_result_files_and_cleanup_without_ros(monkeypatch, tmp_path, phase, cause):
    import signal
    motors, targets = commissioned(index=13)
    for target in targets:
        target.settings.update(seek_torque_ff_nm=0.1 * target.settings["seek_direction"],
                               move_torque_ff_nm=0.1, hold_torque_ff_nm=0.1)
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    for target in targets:
        document["joints"][target.motor.name] = target.settings
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(document))
    actual_logger, actual_runner = h.Logger, h.Runner
    saved = {}

    class SpyLogger(actual_logger):
        def __init__(self, directory):
            super().__init__(directory)
            self.events = []
            saved["log"] = self

        def write(self, event, **data):
            super().write(event, **data)
            self.events.append((event, data))

    def create_io(selected, motion, system_node, args):
        io = Drive(selected, saved["log"])
        io.stop_after_complete = False
        original_spin = io.spin

        def spin(timeout):
            original_spin(timeout)
            runner = saved["runner"]
            reached = (runner.all_done if phase == "ALL_HOLD" else
                       runner.current is not None and runner.current.target.motor.index == 13 and
                       runner.current.phase == phase and
                       (phase != "ZERO_PENDING" or runner.current.zero_sent))
            if reached and "cutoff" not in saved:
                saved["cutoff"] = len(io.command_batches)
                saved["owned"] = list(runner.owned)
                if cause == "fault":
                    raise h.HomingError("injected feedback callback fault", diagnostics={"test_detail": 0.3})
                signal.raise_signal(signal.SIGINT if cause == "sigint" else signal.SIGTERM)

        def close():
            # Stop/disable must happen BEFORE node/context destruction.
            assert not any(io.enabled.values())
            assert [i for kind, i, _ in io.calls if kind == "disable"] == saved["owned"] * 10
            saved["closed"] = True

        io.spin, io.close = spin, close
        saved["io"] = io
        return io

    def create_runner(io, motors, selected, logger, timeout):
        saved["runner"] = actual_runner(io, motors, selected, logger, timeout, io.clock)
        return saved["runner"]

    monkeypatch.setattr(h, "Logger", SpyLogger)
    monkeypatch.setattr(h, "make_ros_io", create_io)
    monkeypatch.setattr(h, "Runner", create_runner)
    expected_code = 1 if cause == "fault" else 0 if phase == "ALL_HOLD" else 130
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    assert h.main(["--motor-index", "13", "--write-zero", "--calibration-config", str(path),
                   "--log-dir", str(tmp_path / "logs")]) == expected_code
    assert all(signal.getsignal(sig) == handler for sig, handler in previous_handlers.items())
    assert saved["closed"] and not any(saved["io"].enabled.values())
    expected_owned = [13]
    assert saved["owned"] == expected_owned  # Includes previous holds; excludes untouched motors.
    cleanup_batches = saved["io"].command_batches[saved["cutoff"]:]
    assert len(cleanup_batches) == 1
    for _, batch in cleanup_batches:
        assert [motor.index for motor, _ in batch] == expected_owned
        assert all(cmd == (0.0, 0.0, 0.0, 0.0, 0.0) for _, cmd in batch)
    import json
    records = [json.loads(line) for line in saved["log"].path.read_text().splitlines()]
    assert [r["index"] for r in records if r["event"] == "MOTOR_PASS"] == (
        [13] if phase == "ALL_HOLD" else [])
    assert records[-1]["event"] == "RUN_END" and records[-1]["exit_code"] == expected_code
    exit_record = next(r for r in records if r["event"] == ("RUN_FAIL" if cause == "fault" else "OPERATOR_STOP"))
    assert exit_record["zero_unknown"] == (phase == "ZERO_PENDING")
    if cause == "fault":
        assert exit_record["diagnostics"] == {"test_detail": 0.3}
        if phase != "ALL_HOLD":
            assert next(r for r in records if r["event"] == "MOTOR_FAIL")["diagnostics"] == {"test_detail": 0.3}
    assert next(r for r in records if r["event"] == "SHUTDOWN")["disable_confirmed"] is False


def test_real_ros_message_types_with_no_ros_graph(monkeypatch):
    """Only generated message objects are real; Node/init/publish are local fakes."""
    msg = pytest.importorskip("luvbot_hardware_interfaces.msg")
    srv = pytest.importorskip("luvbot_hardware_interfaces.srv")
    import rclpy
    import rclpy.node
    from builtin_interfaces.msg import Time
    _, targets = commissioned(index=14)
    publications, requests = {}, []

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, cls, topic, qos):
            def publish(message):
                assert isinstance(message, cls)
                publications[topic] = message
            return SimpleNamespace(publish=publish, get_subscription_count=lambda: 1)

        def create_subscription(self, *args):
            return None

        def create_client(self, cls, name):
            def call(request):
                assert isinstance(request, cls.Request)
                requests.append((name, request))
                return Future(None)
            return SimpleNamespace(call_async=call, service_is_ready=lambda: True)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

        def count_publishers(self, topic):
            return 1

        def destroy_node(self):
            pass

    monkeypatch.setattr(rclpy.node, "Node", Node)
    monkeypatch.setattr(rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(rclpy, "shutdown", lambda: None)
    io = h.make_ros_io(targets, True, "/hpm_system", [])
    motor = targets[0].motor
    io.enable([motor], True)
    io.command([(motor, (0.0, -0.1, 0.0, 0.5, -0.2))])
    assert list(publications[h.ENABLE_TOPIC].motor_types) == [14]
    command = publications[h.COMMAND_TOPIC]
    assert command.control_mode == msg.MotorCommand.CONTROL_MODE_MIT
    assert command.commands[0].route.motor_id == 35
    assert command.commands[0].dq == -0.1
    assert command.commands[0].kp == 0.0 and command.commands[0].tau == pytest.approx(-0.2)
    io.command([(motor, (targets[0].zero_target, 0.0, 5.0, 0.5, 0.3))])
    assert publications[h.COMMAND_TOPIC].commands[0].kp == 5.0
    assert publications[h.COMMAND_TOPIC].commands[0].tau == pytest.approx(0.3)
    io.query(motor)
    io.zero(motor)
    assert requests[0][1].operation == srv.MotorCalibrateId.Request.QUERY
    assert list(requests[1][1].motor_types) == [14]
    assert requests[1][0] == h.ZERO_SERVICE
    _, ls_targets = commissioned(index=18)
    ls_io = h.make_ros_io(ls_targets, True, "/hpm_system", [])
    assert ls_io.ready()
    ls_io.zero(ls_targets[0].motor)
    assert requests[-1][0] == h.LS_ZERO_SERVICE
    assert list(requests[-1][1].motor_types) == [18]
    assert isinstance(srv.MotorSetZero.Response.TIMEOUT, int)
    ls_io.normal_mode()
    assert list(requests[-1][1].names) == ["motor_id_calibration_mode", "launch_motor", "ls_firmware_version"]
    ls_io.close()
    state = msg.MotorState()
    state.route.motor_type, state.route.bus_index, state.route.motor_id = 14, 5, 35
    state.valid = True
    state.enable_state = msg.MotorState.ENABLE_ENABLED
    state.q, state.dq, state.tau = 0.1, 0.0, 0.02
    state.mos_temperature, state.rotor_temperature = math.nan, 25.0
    array = msg.MotorStateArray(sequence_id=1, feedback_mask=1 << 14, states=[state])
    io.on_state(array)
    first = io.feedback[14]
    assert first.q == 0.1 and first.temperature == 25.0 and first.enabled is True
    state.q = 1.0
    io.on_state(array)
    assert io.feedback[14] is first  # Repeated sequence must not refresh age.
    array.sequence_id = 2
    state.route.bus_index = 2
    with pytest.raises(h.HomingError, match="route mismatch"):
        io.on_state(array)
    io.close()
