# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Torque continuity at the reference endpoint, without motor/ROS access."""
import math
import pytest
from test_motor_auto_zero import commissioned, h


def trailing(direction=1, *, error=0.0444575, adaptive=True, compensation=0.2):
    _, targets = commissioned(index=1)
    t = targets[0]
    t.settings.update(home_angle_deg=-direction * 20, bck_kp=9, kd=0.5, bck_kd=0.5,
                      return_speed_rad_s=0.3, return_overspeed_kd=0,
                      adaptive_torque_enabled=adaptive, adaptive_ki=0,
                      adaptive_torque_limit_nm=1, max_torque_nm=9,
                      arrival_tolerance_rad=math.radians(1), move_torque_ff_nm=0.1)
    run = h.MotorRun(t, 0)
    q = t.zero_target - direction * error
    f = h.Feedback(0.99, 1, q, direction * 0.1, 0.1, 25)
    run.begin_return(0.99, f)
    run.return_reference = t.zero_target - direction * 0.001
    run.adaptive_torque = direction * compensation
    return run, h.Feedback(1.0, 2, q, direction * 0.1, 0.1, 25)


@pytest.mark.parametrize('direction', [-1, 1])
def test_endpoint_preserves_velocity_support_while_actual_joint_still_lags(direction):
    run, f = trailing(direction)
    before = (run.p['bck_kp'] * (run.position_target - f.q) +
              run.p['kd'] * (direction * 0.3 - f.dq) + run.adaptive_torque)
    command = run.step(1.0, f)
    q, dq, kp, kd, ff = command
    assert run.phase == 'MOVE'  # Reference arrival is not physical arrival.
    assert q == run.position_target and dq == 0
    assert run.endpoint_handoff_nm == pytest.approx(direction * 0.15)
    assert kp * (q - f.q) + kd * (dq - f.dq) + ff == pytest.approx(before)
    run.step(1.002, h.Feedback(1.002, 3, f.q, f.dq, 0.1, 25))
    assert run.adaptive_torque == pytest.approx(direction * 0.35)
    # Brake/resume must not repeatedly add another endpoint contribution.
    run.begin_brake(1.003, 'TEST')
    run.begin_return(1.004, f, resume=True)
    run.return_reference = run.position_target - direction * 0.0001
    run.step(1.006, h.Feedback(1.006, 4, f.q, f.dq, 0.1, 25))
    assert run.adaptive_torque == pytest.approx(direction * 0.35)


@pytest.mark.parametrize('direction', [-1, 1])
def test_endpoint_handoff_obeys_existing_integral_budget(direction):
    run, f = trailing(direction, compensation=0.95)
    run.step(1.0, f)
    assert run.adaptive_torque == pytest.approx(direction * 1.0)
    assert run.endpoint_handoff_nm == pytest.approx(direction * 0.05)


@pytest.mark.parametrize('adaptive,error', [(False, 0.044), (True, 0.005), (True, -0.005)])
def test_no_handoff_for_fixed_feedforward_or_joint_already_at_target(adaptive, error):
    run, f = trailing(error=error, adaptive=adaptive)
    cmd = run.step(1.0, f)
    assert run.endpoint_handoff_nm == 0
    if not adaptive:
        assert cmd[4] == 0.1


def test_handoff_retains_command_torque_braking():
    run, f = trailing()
    run.p['max_torque_nm'] = 0.4
    run.step(1.0, f)
    assert run.phase == 'BRAKE' and run.brake_reason == 'COMMAND_TORQUE_BUDGET'


def test_new_deliberate_move_resets_handoff_state():
    run, f = trailing()
    run.step(1.0, f)
    assert run.endpoint_handoff_done
    run.begin_return(2.0, h.Feedback(2.0, 3, f.q, 0, 0, 25))
    assert not run.endpoint_handoff_done and run.endpoint_handoff_nm == 0
