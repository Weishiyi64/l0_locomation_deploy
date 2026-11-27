# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Noisy MIT velocity must not prevent position-based brake release / arrival."""
import math

import pytest

from test_motor_auto_zero import commissioned, h


def returning():
    _, targets = commissioned(index=14)
    target = targets[0]
    target.settings.update(return_speed_rad_s=0.47, bck_kp=5, kd=0.3, bck_kd=0.3,
                           arrival_settle_s=0.3, final_mode='position',
                           adaptive_torque_enabled=True)
    run = h.MotorRun(target, 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    return run


def test_logged_one_frame_overspeed_does_not_brake_but_sustained_speed_does():
    run = returning()
    run.step(0.037, h.Feedback(0.037, 2, -0.00171661, -1.222221, 0.1, 25))
    assert run.phase == 'MOVE' and run.brake_count == 0
    # Re-publishing one feedback sample cannot advance the confirmation clock.
    run.step(0.10, run.last)
    assert run.phase == 'MOVE'
    run.step(0.11, h.Feedback(0.11, 3, -0.002, -0.01587677, 0, 25))
    assert run.overspeed_at is None
    for i in range(7):
        at = 0.12 + i * 0.01
        run.step(at, h.Feedback(at, 4 + i, -0.002 - i * 0.01, -1.1, 0, 25))
        if run.phase == 'BRAKE':
            break
    assert run.phase == 'BRAKE' and run.brake_reason == 'RETURN_OVERSPEED'
    assert at == pytest.approx(0.17)


@pytest.mark.parametrize('velocity', [-2, 2])
def test_severe_overspeed_still_brakes_in_first_frame(velocity):
    run = returning()
    command = run.step(0.01, h.Feedback(0.01, 2, 0, velocity, 0, 25))
    assert run.phase == 'BRAKE' and command[2] == command[4] == 0


@pytest.mark.parametrize('phase', ['BRAKE', 'MOVE', 'RECOVER'])
def test_position_still_with_logged_velocity_noise_releases_or_arrives(phase):
    run = returning()
    center = -0.003 if phase == 'BRAKE' else run.target.zero_target
    if phase == "MOVE":
        run.begin_return(0, h.Feedback(0, 1, center, 0, 0, 25))
    else:
        run.transition(phase, 0)
    run.last = None
    noise = [-0.206352, -0.01587677, 0.174606, 0.07936859, -0.1428604]
    duration = run.p['brake_settle_s'] if phase == 'BRAKE' else run.p['arrival_settle_s']
    for i in range(1, round(duration / 0.002) + 3):
        at = i * 0.002
        q = center + (i % 2) * 0.0003814697
        run.step(at, h.Feedback(at, i, q, noise[i % len(noise)], 0.01, 25))
    assert run.phase == ('MOVE' if phase == 'BRAKE' else 'HOLD')
    assert run.brake_count == 0


@pytest.mark.parametrize('motion', ['creep', 'oscillation'])
def test_mean_velocity_and_full_range_both_required_for_brake_release(motion):
    run = returning()
    run.begin_brake(0, 'TEST')
    # Slow unidirectional creep fits the position span but exceeds average speed.
    run.p['brake_velocity_rad_s'] = 0.001
    for i in range(1, 51):
        at = i * 0.01
        q = at * 0.003 if motion == 'creep' else 0.01 * math.sin(40 * math.pi * at)
        run.step(at, h.Feedback(at, i + 1, q, 0, 0, 25))
        assert run.phase == 'BRAKE'


def test_sparse_repeated_or_discontinuous_feedback_cannot_confirm_settling():
    run = returning()
    run.begin_brake(0, 'TEST')
    for seq, at in enumerate([0.01, 0.03, 0.05, 0.07, 0.4], 2):
        f = h.Feedback(at, seq, 0, 0, 0, 25)
        run.step(at, f)
        run.step(at + 0.01, f)
        assert run.phase == 'BRAKE'
    assert len(run.window) == 1
    with pytest.raises(h.HomingError, match='FEEDBACK_TIMEOUT'):
        run.step(0.7, f)


def test_near_goal_on_first_frame_still_waits_for_full_arrival_window():
    run = returning()
    run.begin_return(0, h.Feedback(0, 2, run.target.zero_target, 0, 0, 25))
    run.step(0.01, h.Feedback(0.01, 3, run.target.zero_target, 0, 0, 25))
    assert run.phase == 'MOVE'
    for i in range(2, 33):
        at = i * 0.01
        run.step(at, h.Feedback(at, i + 2, run.target.zero_target, 0, 0, 25))
    assert run.phase == 'HOLD'


def test_low_control_rate_collects_five_samples_instead_of_timing_out_forever():
    run = returning()
    run.p['rate_hz'] = 10
    run.begin_brake(0, 'TEST')
    for i in range(1, 6):
        at = i * 0.1
        run.step(at, h.Feedback(at, i + 1, -0.003, 0.14, 0, 25))
        assert run.phase == ('BRAKE' if i < 5 else 'MOVE')


@pytest.mark.parametrize('key,value', [('motion_settle_position_rad', 0),
    ('motion_settle_position_rad', float('nan')), ('brake_emergency_speed_ratio', 1),
    ('brake_overspeed_confirm_s', 0), ('brake_overspeed_confirm_s', 0.25)])
def test_motion_feedback_settings_are_validated(key, value):
    run = returning()
    run.p[key] = value
    assert h.motion_errors(run.target)
