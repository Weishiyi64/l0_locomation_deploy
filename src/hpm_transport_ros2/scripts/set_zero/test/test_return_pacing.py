# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Slow forward tracking paces its reference; a real fall still brakes."""
import math

import pytest

from test_motor_auto_zero import commissioned, h


def returning(index=11, start=0):
    _, targets = commissioned(index=index)
    target = targets[0]
    target.settings.update(bck_kp=1, kd=0.3, bck_kd=0.3, return_speed_rad_s=0.47, max_torque_nm=3, direction_confirmed=True,
                           adaptive_torque_enabled=True, arrival_settle_s=0.3)
    run = h.MotorRun(target, 0)
    run.begin_return(0, h.Feedback(0, 1, start, 0, 0, 25))
    return run


@pytest.mark.parametrize('index,start', [(11, -0.48504638671875), (14, 0.0)])
def test_slow_tracking_keeps_gains_and_integral_then_arrives_without_braking(index, start):
    run = returning(index, start)
    goal = run.target.zero_target
    direction = 1 if goal > start else -1
    previous_ref = start
    limited = False
    for i in range(1, 5001):
        at = i * 0.002
        distance = min(abs(goal - start), at * 0.25)
        q = start + direction * distance
        dq = direction * 0.25 if distance < abs(goal - start) else 0.0
        f = h.Feedback(at, i + 1, q, dq, 0.2, 25)
        command = run.step(at, f)
        assert run.brake_count == 0
        if run.phase == 'DAMPING':
            break
        assert run.phase == 'MOVE' and command[2:4] == (1, 0.3)
        assert direction * (command[0] - previous_ref) >= -1e-12
        assert abs(command[0] - q) <= run.p['return_max_lead_rad'] + 1e-9
        assert abs(command[4]) <= run.p['adaptive_torque_limit_nm']
        limited |= run.return_reference_limited
        previous_ref = command[0]
    else:
        pytest.fail('Slow but controlled tracking did not arrive')
    assert limited and run.return_started_at == 0


def test_logged_first_tracking_error_paces_instead_of_braking():
    start, final_q, at = -0.48504638671875, -0.00514984130859375, 1.415654
    run = returning(start=start)
    for i in range(1, 709):
        now = min(i * 0.002, at)
        q = start + (final_q - start) * now / at
        command = run.step(now, h.Feedback(now, i + 1, q, 0.4452018737792969, 0.2, 25))
    assert run.phase == 'MOVE' and run.brake_count == 0 and run.return_reference_limited
    assert command[0] - final_q == pytest.approx(math.radians(8))
    assert command[1:4] == (0.47, 1, 0.3)


@pytest.mark.parametrize('index,direction', [(11, 1), (14, -1)])
def test_reference_does_not_chase_a_reverse_fall_and_tracking_guard_still_brakes(index, direction):
    run = returning(index)
    for i in range(1, 201):
        at = i * 0.002
        run.step(at, h.Feedback(at, i + 1, 0, 0, 0.2, 25))
    ref = run.return_reference
    assert ref == pytest.approx(direction * run.p['return_max_lead_rad'])
    for i in range(1, 102):
        at = 0.4 + i * 0.002
        q = -direction * min(0.05, 0.4 * (at - 0.4))
        command = run.step(at, h.Feedback(at, i + 201, q, -direction * 0.4, 0.2, 25))
        assert direction * (run.return_reference - ref) >= -1e-12
        if run.phase == 'BRAKE':
            break
    assert run.phase == 'BRAKE' and run.brake_reason == 'RETURN_TRACKING_ERROR'
    assert command[2] == command[4] == 0


def test_stuck_motor_keeps_bounded_support_but_does_not_reset_return_timeout():
    run = returning()
    run.p['max_return_time_s'] = 1
    for i in range(1, 501):
        at = i * 0.002
        command = run.step(at, h.Feedback(at, i + 1, 0, 0, 0.2, 25))
        assert run.phase == 'MOVE' and run.brake_count == 0
    assert command[0] == pytest.approx(run.p['return_max_lead_rad'])
    assert command[2] == 1 and command[4] > 0
    with pytest.raises(h.HomingError, match='RETURN_TIMEOUT'):
        run.step(1.01, h.Feedback(1.01, 999, 0, 0, 0.2, 25))


@pytest.mark.parametrize('fault', ['overspeed', 'torque', 'stale', 'command_budget'])
def test_pacing_keeps_hard_guards(fault):
    run = returning()
    now, q, dq, tau = 0.01, 0, 0, 0
    if fault == 'overspeed':
        dq = 2.0
    elif fault == 'torque':
        tau = 3.1
    elif fault == 'stale':
        now = 0.3
    else:
        now = 0.3
        run.p['bck_kp'] = 100
    f = h.Feedback(0.01 if fault == 'stale' else now, 2, q, dq, tau, 25)
    if fault in ('torque', 'stale'):
        with pytest.raises(h.HomingError, match='TORQUE_LIMIT' if fault == 'torque' else 'FEEDBACK_TIMEOUT'):
            run.step(now, f)
    else:
        command = run.step(now, f)
        assert run.phase == 'BRAKE'
        assert run.brake_reason == ('RETURN_OVERSPEED' if fault == 'overspeed' else 'COMMAND_TORQUE_BUDGET')
        assert command[2] == command[4] == 0


@pytest.mark.parametrize('value', [0, -0.1, math.nan, math.inf, True, math.radians(10)])
def test_return_lead_requires_positive_finite_value_below_tracking_guard(value):
    run = returning()
    run.p['return_max_lead_rad'] = value
    assert any('return_max_lead' in e for e in h.motion_errors(run.target))


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('reference,position,speed', [
    (0.21858183869783565, 0.12684059143066406, 0.874237060546875),
    (0.34288422248846484, 0.2695121765136719, 0.8253974914550781),
    (0.46987458491835227, 0.3835735321044922, 0.8840045928955078),
])
def test_logged_forward_catchup_keeps_returning_without_repeated_brakes(direction, reference, position, speed):
    _, targets = commissioned(index=4)
    target = targets[0]
    target.settings.update(home_angle_deg=-direction * 50, return_speed_rad_s=0.4,
                           bck_kp=4, kd=0.3, bck_kd=0.3, max_torque_nm=9,
                           arrival_tolerance_rad=math.radians(10))
    run = h.MotorRun(target, 0)
    start = direction * (position - speed * 0.06)
    run.begin_return(0, h.Feedback(0, 1, start, direction * speed, 0.6, 25))
    run.return_reference = direction * (reference - 0.4 * 0.06)
    for i in range(1, 31):
        at = i * 0.002
        q = start + direction * speed * at
        command = run.step(at, h.Feedback(at, i + 1, q, direction * speed, 0.6, 25))
        assert run.phase == 'MOVE' and run.brake_count == 0
        assert command[1:4] == (direction * 0.4, 4, 0.3)
        assert run.return_catchup_active
        assert run.return_speed_limit_rad_s >= speed
        assert run.return_emergency_speed_rad_s == pytest.approx(1.6)


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('fault', ['reverse', 'ahead', 'near_goal', 'no_progress', 'excess_catchup', 'emergency'])
def test_catchup_does_not_relax_reverse_ahead_endpoint_or_emergency_braking(direction, fault):
    _, targets = commissioned(index=4)
    target = targets[0]
    target.settings.update(home_angle_deg=-direction * 50, return_speed_rad_s=0.4,
                           bck_kp=4, kd=0.3, bck_kd=0.3, max_torque_nm=9,
                           arrival_tolerance_rad=math.radians(10))
    run = h.MotorRun(target, 0)
    position, reference, speed = 0.12684, 0.21858, 0.9
    if fault == 'reverse':
        speed = -speed
    elif fault == 'ahead':
        position, reference = 0.3, 0.2
    elif fault == 'near_goal':
        position, reference = 0.79, 0.85
    elif fault == 'excess_catchup':
        speed = 1.5
    elif fault == 'emergency':
        speed = 1.61
    run.begin_return(0, h.Feedback(0, 1, direction * position, 0, 0, 25))
    run.return_reference = direction * reference
    for i in range(1, 32):
        at = i * 0.002
        q = direction * (position + (0 if fault == 'no_progress' else speed * at))
        command = run.step(at, h.Feedback(at, i + 1, q, direction * speed, 0.6, 25))
        if run.phase == 'BRAKE':
            break
    assert run.phase == 'BRAKE' and run.brake_reason == 'RETURN_OVERSPEED'
    assert command[2] == command[4] == 0
    if fault == 'emergency':
        assert i == 1
