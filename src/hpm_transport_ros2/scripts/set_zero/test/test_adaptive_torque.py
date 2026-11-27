# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Offline load model and guards; these checks do not certify physical stability."""
from copy import deepcopy
import math

import pytest

from test_motor_auto_zero import PACKAGE, Drive, Log, commissioned, h


def adaptive_run():
    motors, targets = commissioned(index=13)
    t = targets[0]
    t.settings.update(adaptive_torque_enabled=True, adaptive_ki=1.0,
                      adaptive_torque_limit_nm=1.0, adaptive_torque_rate_nm_s=0.25,
                      bck_kp=5.0, kd=0.2, bck_kd=0.2, hold_kp=1.0, hold_kd=0.2,
                      return_speed_rad_s=0.47, max_return_time_s=25.0,
                      arrival_settle_s=0.3, arrival_tolerance_rad=math.radians(10),
                      hold_error_rad=math.radians(10), seek_torque_ff_nm=-0.1,
                      move_torque_ff_nm=-0.6, hold_torque_ff_nm=0.8)
    run = h.MotorRun(t, 0.0)
    run.move_start = 0.0
    run.duration = h.trajectory_duration(abs(t.zero_target), t.settings)
    run.transition("MOVE", 0.0)
    return motors, t, run


@pytest.mark.parametrize("load", [-0.6, 0.6])
def test_opposite_loads_return_then_release_adaptive_torque_into_pure_damping(load):
    _, t, run = adaptive_run()
    q = v = torque = 0.0
    dt = 0.002
    old_i = 0.0
    for sequence in range(1, 12501):
        now = sequence * dt
        command = run.step(now, h.Feedback(now, sequence, q, v, torque, 25.0))
        desired, velocity, kp, kd, ff = command
        if run.phase == "DAMPING":
            assert abs(q - t.zero_target) <= t.settings["arrival_tolerance_rad"]
            assert abs(v) <= t.settings["arrival_velocity_rad_s"]
            break
        assert abs(ff) <= 1.0 and abs(ff - old_i) <= 0.25 * dt + 1e-12
        old_i = ff
        torque = kp * (desired - q) + kd * (velocity - v) + ff
        assert abs(torque) < t.settings["max_torque_nm"]
        v += (torque + load - 0.5 * v) / 0.05 * dt
        q += v * dt
    assert run.phase == "DAMPING"
    assert command == (0, 0, 0, t.settings["damping_kd"], 0)
    assert run.adaptive_torque == 0 and run.adaptive_sample is None
    # Moving away after completion must not restart a position or integral loop.
    for extra in range(1, 101):
        now += dt
        command = run.step(now, h.Feedback(now, sequence + extra, q + extra * 0.005, 0.1, 0, 25))
        assert command == (0, 0, 0, t.settings["damping_kd"], 0)
        assert run.adaptive_torque == 0 and run.phase == "DAMPING"



def test_bounds_unwind_duplicates_handoff_and_missing_feedback():
    _, t, run = adaptive_run()
    t.settings.update(adaptive_ki=10.0, adaptive_torque_limit_nm=0.1)
    f = h.Feedback(0.01, 1, 0, 0, 0, 25)
    assert run.position_command(0.1, 0, 0, 0, f)[4] == 0
    for i in range(2, 100):
        f = h.Feedback(i / 100, i, 0, 0, 0, 25)
        run.position_command(0.1, 0, 0, 0, f)
    assert run.adaptive_torque == pytest.approx(0.1) and run.adaptive_limited
    assert run.position_command(-0.1, 0, 0, 0, f)[4] == pytest.approx(0.1)
    run.position_command(-0.1, 0, 0, 0, h.Feedback(0.5, 100, 0, 0, 0, 25))
    assert run.adaptive_torque == pytest.approx(0.1)  # Reordered timestamp.
    run.transition("HOLD", 0.99)
    assert run.adaptive_torque == pytest.approx(0.1)
    run.position_command(-0.1, 0, 0, 0, h.Feedback(1.0, 101, 0, 0, 0, 25))
    assert run.adaptive_torque == pytest.approx(0.0975)
    before = run.adaptive_torque
    with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
        run.step(2.0, f)
    assert run.adaptive_torque == before


def test_anti_windup_does_not_replace_return_budget_braking():
    motors, t, run = adaptive_run()
    t.settings["max_torque_nm"] = 1.0
    run.position_command(1, 0, 2, 0, h.Feedback(0.01, 1, 0, 0, 0, 25))
    run.position_command(1, 0, 2, 0, h.Feedback(0.02, 2, 0, 0, 0, 25))
    assert run.adaptive_torque == 0 and run.adaptive_limited
    run.adaptive_torque = 0.1
    run.position_command(-0.1, 1, 0, 2, h.Feedback(0.03, 3, 0, 0, 0, 25))
    assert run.adaptive_torque < 0.1  # Unwind even if the PD term remains over budget.
    t.settings["bck_kp"] = 20  # Exceed budget even with the paced 8-degree reference.
    io = Drive([t], Log())
    runner = h.Runner(io, motors, [t], io.log, clock=io.clock)
    runner.runs = [run]
    io.now = 2
    runner.last_pump = io.now
    io.feedback[t.motor.index] = h.Feedback(2, 4, 0, 0, 0, 25)
    runner.pump(force=True)
    assert run.phase == "BRAKE"
    assert io.commands[t.motor.index][2] == io.commands[t.motor.index][4] == 0


def test_adaptive_profiles_validation_and_legacy_default(tmp_path):
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    for index in range(21):
        _, targets = h.load_plan(h.motor_config_path(), PACKAGE / "config/motor_zero_calibration.yaml",
                                 PACKAGE / "config/l0_v5.urdf", motor_index=index)
        assert targets[0].settings["adaptive_torque_enabled"] is True
        # Check all compensation budgets independently of placeholder stop angles.
        checked = targets[0]
        assert all(checked.settings[k] > 0 for k in h.ADAPTIVE)
        assert checked.settings["adaptive_torque_limit_nm"] <= min(
            checked.settings["max_torque_nm"], -checked.limits["t_min"], checked.limits["t_max"])
        assert targets[0].settings["arrival_tolerance_rad"] == pytest.approx(math.radians(10))
        assert targets[0].settings["hold_error_rad"] == pytest.approx(math.radians(10))
    _, t, _ = adaptive_run()
    assert not h.motion_errors(t)
    for key in h.ADAPTIVE:
        for value in (None, True, "1", math.nan, math.inf, -1, 0):
            broken = deepcopy(t)
            broken.settings[key] = value
            assert any(key in error for error in h.motion_errors(broken))
    t.settings["adaptive_torque_limit_nm"] = t.settings["max_torque_nm"] + 0.1
    assert any("adaptive torque limit" in e for e in h.motion_errors(t))
    profile = document["joints"]["left_shoulder_roll_joint"]
    for key in h.ADAPTIVE + ("adaptive_torque_enabled",):
        profile.pop(key)
    path = tmp_path / "legacy.yaml"
    import yaml
    path.write_text(yaml.safe_dump(document))
    _, targets = h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", motor_index=13)
    assert not targets[0].settings["adaptive_torque_enabled"]
    document["defaults"]["adaptive_ki"] = 1.0
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="per joint"):
        h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf", motor_index=13)


def test_handoff_over_limit_keeps_recovery_support():
    _, t, run = adaptive_run()
    run.adaptive_torque = 0.9
    run.last = h.Feedback(1, 1, t.zero_target - 0.1, 0, 0, 25)
    run.transition("HOLD", 1)
    assert run.phase == "RECOVER" and run.adaptive_torque == 0.9
    assert run.hold_transfer_limited
    run.transition("MOVE", 2)
    assert run.adaptive_torque == 0 and run.adaptive_sample is None
    run.last = None
    command = run.step(2.01, h.Feedback(2.01, 2, t.zero_target + math.radians(11), 0, 0, 25))
    assert run.phase == "MOVE" and run.return_replans == 1 and command[1] < 0


@pytest.mark.parametrize("module", ["left_arm", None])
def test_module_freezes_ki_only_after_all_motors_pass_and_keeps_support(module):
    motors, targets = commissioned("left_arm")
    if module is None:
        targets = targets[:1]
    for t in targets:
        t.settings.update(final_mode="position", adaptive_torque_enabled=True,
                          adaptive_ki=1.0, adaptive_torque_limit_nm=1.0,
                          adaptive_torque_rate_nm_s=0.25)
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.module = module
    observed = []
    original_pump = runner.pump

    def pump(*args, **kwargs):
        if not runner.all_done:
            assert not any(r.adaptive_ki_frozen for r in runner.runs)
        elif not observed:
            observed.extend((r, r.adaptive_torque) for r in runner.runs)
        return original_pump(*args, **kwargs)

    runner.pump = pump
    with pytest.raises(KeyboardInterrupt):
        runner.calibrate(write_zero=True)
    assert runner.all_done and len(observed) == len(targets)
    passed = next(data for event, data in log.events if event == "CALIBRATION_PASS")
    expected = [t.motor.index for t in targets] if module else []
    assert passed["adaptive_ki_frozen_indices"] == expected
    for run, retained in observed:
        assert run.adaptive_ki_frozen == (module is not None)
        # Test persistent position error with fresh feedback; frozen Ki must neither
        # add nor unwind compensation, while the single-motor path keeps adapting.
        run.phase = "HOLD"
        run.adaptive_torque = retained = 0.2
        run.adaptive_sample = None
        for seq in range(1, 11):
            f = h.Feedback(run.started + seq * 0.01, seq, run.position_target - 0.02, 0, 0, 25)
            cmd = run.position_command(run.position_target, 0, 1, 0.1, f)
            if module:
                assert cmd[4] == retained
        assert (run.adaptive_torque == retained) if module else (run.adaptive_torque > retained)
        # Freezing Ki does not bypass feedback-loss protection.
        with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
            run.step(f.received_at + run.p["feedback_timeout_s"] + 0.01, f)
