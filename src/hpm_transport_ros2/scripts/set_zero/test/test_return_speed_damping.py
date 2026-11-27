# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Bounded return-speed damping, without a ROS graph or actuator commands."""
import math

import pytest

from test_motor_auto_zero import commissioned, h


def configured():
    _, targets = commissioned(index=4)
    target = targets[0]
    target.settings.update(bck_kp=4, kd=0.3, bck_kd=0.3, return_speed_rad_s=0.3,
                           return_overspeed_kd=1.0, return_overspeed_torque_limit_nm=0.5)
    run = h.MotorRun(target, 0)
    run.phase = 'MOVE'
    return run


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('speed', [0.2, 0.8, 1.3])
def test_additional_damping_opposes_excess_speed_and_respects_its_own_cap(sign, speed):
    run = configured()
    run.p.update(adaptive_torque_enabled=False, move_torque_ff_nm=0.2)
    f = h.Feedback(0.01, 1, 0, sign * speed, 0, 25)
    command = run.position_command(sign * 0.1, sign * 0.3, 4, 0.3, f)
    assert command[:3] == (sign * 0.1, sign * 0.3, 4)
    assert command[4] == 0.2  # Fixed feedforward is unchanged.
    extra = (command[3] - 0.3) * (command[1] - f.dq)
    assert extra == pytest.approx(run.return_speed_damping_nm)
    assert extra * f.dq <= 0
    assert abs(extra) <= 0.5 + 1e-12
    assert abs(extra) == pytest.approx(min(0.5, max(0, speed - 0.3)))


@pytest.mark.parametrize('phase', ['MOVE', 'HOLD', 'RECOVER'])
def test_damping_is_optional_and_does_not_change_other_position_phases(phase):
    run = configured()
    run.phase = phase
    f = h.Feedback(0.01, 1, 0, 0.8, 0, 25)
    run.p['return_overspeed_kd'] = 0
    assert run.position_command(0.1, 0.3, 4, 0.3, f)[3] == 0.3
    run.p['return_overspeed_kd'] = 1
    if phase != 'MOVE':
        assert run.position_command(0.1, 0.3, 4, 0.3, f)[3] == 0.3
    # Reverse motion keeps the original controller and braking rules.
    f = h.Feedback(0.02, 2, 0, -0.8, 0, 25)
    assert run.position_command(0.1, 0.3, 4, 0.3, f)[3] == 0.3


@pytest.mark.parametrize('sign', [-1, 1])
def test_endpoint_velocity_noise_does_not_raise_return_damping(sign):
    run = configured()
    run.p.update(adaptive_torque_enabled=False, return_overspeed_kd=0.1,
                 move_torque_ff_nm=0.2)
    for i, speed in enumerate((0.4603, 0.7460, 0.8730), 1):
        f = h.Feedback(i * 0.002, i, sign * 0.001, sign * speed, 0, 25)
        command = run.position_command(0.0, 0.0, 4, 0.3, f)
        assert command == (0.0, 0.0, 4, 0.3, 0.2)
        assert run.return_speed_damping_nm == 0.0


@pytest.mark.parametrize('sign', [-1, 1])
def test_speed_damping_prevents_integral_push_but_allows_compensation_to_unwind(sign):
    run = configured()
    run.p['adaptive_torque_enabled'] = True
    run.adaptive_sample = (1, 0)
    run.adaptive_torque = sign * 0.4
    f = h.Feedback(0.01, 2, 0, sign * 0.8, 0, 25)
    run.position_command(sign * 0.1, sign * 0.3, 4, 0.3, f)
    assert run.adaptive_limited and run.adaptive_torque == sign * 0.4
    f = h.Feedback(0.02, 3, 0, sign * 0.8, 0, 25)
    run.position_command(-sign * 0.1, sign * 0.3, 4, 0.3, f)
    assert abs(run.adaptive_torque) < 0.4


@pytest.mark.parametrize('key,value', [
    ('return_overspeed_kd', None), ('return_overspeed_kd', True),
    ('return_overspeed_kd', -1), ('return_overspeed_kd', math.nan),
    ('return_overspeed_kd', 5),
    ('return_overspeed_torque_limit_nm', 0), ('return_overspeed_torque_limit_nm', True),
    ('return_overspeed_torque_limit_nm', math.inf), ('return_overspeed_torque_limit_nm', 3),
])
def test_invalid_speed_damping_cannot_bypass_preflight(key, value):
    run = configured()
    assert not h.motion_errors(run.target)
    run.p[key] = value
    assert h.motion_errors(run.target)


@pytest.mark.parametrize('sign', [-1, 1])
def test_speed_damping_keeps_emergency_overspeed_braking_immediate(sign):
    run = configured()
    run.p.update(home_angle_deg=-sign * 50)
    run.position_target = run.target.zero_target
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    command = run.step(0.002, h.Feedback(0.002, 2, sign * 0.002, sign * 1.31, 0, 25))
    assert run.phase == 'BRAKE' and run.brake_reason == 'RETURN_OVERSPEED'
    assert command[2] == command[4] == 0


@pytest.mark.parametrize('sign', [-1, 1])
def test_stiction_model_avoids_stop_restart_cycle_with_bounded_speed_damping(sign):
    # A deterministic friction model, not a validation of real motor dynamics.
    results = []
    for enabled in (0.0, 1.0):
        run = configured()
        run.p.update(home_angle_deg=-sign * 50, adaptive_torque_enabled=True,
                     return_overspeed_kd=enabled, arrival_tolerance_rad=math.radians(10),
                     arrival_settle_s=0.3, final_mode='position', max_torque_nm=9,
                     max_return_time_s=25, brake_timeout_s=10, max_brake_attempts=10)
        run.position_target, run.final_phase = run.target.zero_target, 'HOLD'
        run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
        position = velocity = peak_speed = 0.0
        for i in range(1, 12501):
            now = i * 0.002
            command = run.step(now, h.Feedback(now, i + 1, position, velocity, 0, 25))
            q, dq, kp, kd, ff = command
            torque = kp * (q - position) + kd * (dq - velocity) + ff
            assert abs(run.return_speed_damping_nm) <= 0.5 + 1e-12
            if abs(velocity) < 0.0001 and abs(torque) <= 1.0:
                velocity = 0.0
            else:
                friction = math.copysign(0.4, velocity if abs(velocity) > 0.0001 else torque)
                next_velocity = velocity + (torque - friction - 0.05 * velocity) / 0.06 * 0.002
                velocity = 0.0 if next_velocity * velocity < 0 and abs(torque) < 1 else next_velocity
            position += velocity * 0.002
            peak_speed = max(peak_speed, abs(velocity))
            if run.phase == 'HOLD':
                break
        assert run.phase == 'HOLD'
        results.append((run.brake_count, peak_speed))
    assert results[0][0] > 0 and results[1][0] == 0
    assert results[1][1] < results[0][1]
