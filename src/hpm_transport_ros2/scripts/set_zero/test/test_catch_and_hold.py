# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
import pytest

from test_motor_auto_zero import Drive, Log, commissioned, h


@pytest.mark.parametrize("velocity", [-65.0, -1.0, -0.01, 0.01, 1.0, 65.0])
def test_pure_brake_uses_configured_kd_with_separate_torque_cap(velocity):
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.p.update(brake_kd=0.4, brake_max_torque_nm=0.5)
    command = run.brake_command(h.Feedback(0.01, 1, 0, velocity, 0, 25))
    q, dq, kp, kd, ff = command
    assert q == dq == kp == ff == 0
    assert kd == pytest.approx(min(0.4, 0.5 / abs(velocity)))
    assert abs(-kd * velocity) <= 0.5 + 1e-12
    assert (-kd * velocity) * velocity < 0


@pytest.mark.parametrize("velocity", [-6.0, 6.0])
def test_overspeed_switches_directly_from_normal_return_to_brake(velocity):
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    command = run.step(0.01, h.Feedback(0.01, 2, 0, velocity, 0, 25))
    assert run.phase == "BRAKE" and run.brake_reason == "RETURN_OVERSPEED"
    assert command[:3] == (0, 0, 0) and command[4] == 0
    assert abs(command[3] * velocity) <= run.p['brake_max_torque_nm'] + 1e-12


def test_tracking_error_uses_current_trajectory_and_requires_confirmation():
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    run.step(0.01, h.Feedback(0.01, 2, 0, 0, 0, 25))
    assert run.phase == 'MOVE'  # Far from final target, close to current trajectory.
    for seq, at in enumerate([0.2, 0.25], 3):
        run.step(at, h.Feedback(at, seq, 0, 0, 0, 25))
        assert run.phase == 'MOVE'
    # Slow/stuck forward tracking waits at the lead bound without braking.
    run.step(0.31, h.Feedback(0.31, 5, 0, 0, 0, 25))
    assert run.phase == 'MOVE'
    # A real backward deviation must still persist for brake_confirm_s.
    run.step(0.32, h.Feedback(0.32, 6, -0.04, 0, 0, 25))
    run.step(0.40, h.Feedback(0.40, 7, -0.045, 0, 0, 25))
    assert run.phase == 'MOVE'
    command = run.step(0.43, h.Feedback(0.43, 8, -0.045, 0, 0, 25))
    assert run.phase == 'BRAKE' and run.brake_reason == 'RETURN_TRACKING_ERROR'
    assert command == (0, 0, 0, run.p['brake_kd'], 0)


def test_brake_waits_for_continuous_stability_then_replans_and_preserves_compensation():
    _, targets = commissioned(index=13)
    targets[0].settings['adaptive_torque_enabled'] = True
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    run.adaptive_torque = 0.3
    run.begin_brake(0, 'TEST')
    # Real position movement prevents release even with an instant low velocity.
    for seq, at in enumerate([0.01, 0.05, 0.10, 0.15, 0.20], 2):
        command = run.step(at, h.Feedback(at, seq, at * 0.1, 0.01588, 0, 25))
        assert run.phase == 'BRAKE' and command[2] == command[4] == 0
        assert run.adaptive_torque == 0
    for seq in range(7, 31):
        at = 0.20 + (seq - 6) * 0.01
        command = run.step(at, h.Feedback(at, seq, 0.02, 0.01588, 0, 25))
        if run.phase == 'MOVE':
            break
    assert run.phase == 'MOVE' and run.move_start == 0.02
    assert run.adaptive_torque == pytest.approx(0.3) and command[4] == pytest.approx(0.3)
    assert run.return_started_at == 0 and run.brake_count == 1


def test_brake_timeout_retry_bound_and_total_return_timeout_remain_active():
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    run.begin_brake(0, 'TEST')
    with pytest.raises(h.HomingError, match='BRAKE_TIMEOUT'):
        run.step(2.1, h.Feedback(2.1, 2, 0, 0.2, 0, 25))
    for attempt in range(1, run.p['max_brake_attempts']):
        run.begin_return(attempt, h.Feedback(attempt, attempt+2, 0, 0, 0, 25), resume=True)
        run.begin_brake(attempt, 'TEST')
    with pytest.raises(h.HomingError, match='BRAKE_RETRY_LIMIT'):
        run.begin_brake(3, 'TEST')
    at = run.p['max_return_time_s'] + 0.1
    with pytest.raises(h.HomingError, match='RETURN_TIMEOUT'):
        run.step(at, h.Feedback(at, 99, 0, 0.2, 0, 25))


def test_braking_does_not_bypass_measured_torque_or_fresh_feedback_guards():
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.begin_brake(0, 'TEST')
    with pytest.raises(h.HomingError, match='TORQUE_LIMIT'):
        run.step(0.01, h.Feedback(0.01, 1, 0, -65, 2.1, 25))
    f = h.Feedback(0.01, 2, 0, 0, 0, 25)
    run.step(0.01, f)
    run.step(0.21, f)
    assert run.phase == 'BRAKE'  # Reusing one frame cannot confirm standstill.
    with pytest.raises(h.HomingError, match='FEEDBACK_TIMEOUT'):
        run.step(0.3, f)


def test_return_budget_excess_sends_brake_instead_of_oversized_pd_command():
    motors, targets = commissioned(index=13)
    t = targets[0]
    t.settings.update(bck_kp=100, brake_position_error_rad=10)
    io = Drive(targets, Log())
    runner = h.Runner(io, motors, targets, io.log, clock=io.clock)
    run = h.MotorRun(t, 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    runner.runs = [run]
    io.now = runner.last_pump = 0.1
    io.feedback[13] = h.Feedback(0.1, 2, 0, 0.1, 0, 25)
    runner.pump(force=True, spin=False)
    assert run.phase == 'BRAKE' and run.brake_reason == 'COMMAND_TORQUE_BUDGET'
    assert io.commands[13] == (0, 0, 0, t.settings['brake_kd'], 0)
    assert any(e == 'BRAKE_START' for e, _ in io.log.events)
    # Holding saturates its estimate instead of withdrawing support.
    run.transition('HOLD', 0.1)
    t.settings['hold_kp'] = 100
    io.feedback[13] = h.Feedback(0.1, 3, t.zero_target - 0.04, 0, 0, 25)
    run.last = None
    runner.pump(force=True, spin=False)
    command = io.commands[13]
    estimate = command[2] * 0.04 + command[4]
    assert estimate == pytest.approx(t.settings['max_torque_nm'])
    assert run.hold_limited and t.settings['hold_kp'] == 100


def test_reversing_after_overshoot_does_not_reset_overall_return_timeout():
    _, targets = commissioned(index=13)
    t = targets[0]
    t.settings['brake_position_error_rad'] = 10  # Isolate overshoot replanning from tracking-loss detection.
    run = h.MotorRun(t, 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    for sequence in range(1, 101):
        q = (t.zero_target + 0.2) * sequence / 100
        at = sequence / 100
        command = run.step(at, h.Feedback(at, sequence + 1, q, t.zero_target + 0.2, 0, 25))
        if run.return_replans:
            break
    assert command[1] < 0 and run.return_replans == 1
    with pytest.raises(h.HomingError, match="RETURN_TIMEOUT"):
        run.step(t.settings["max_return_time_s"] + 0.1,
                 h.Feedback(t.settings["max_return_time_s"] + 0.1, 3, q, 0, 0, 25))


@pytest.mark.parametrize('modes', [('position', 'position', 'position'), ('damping', 'position', 'damping')])
def test_final_modes_keep_each_completed_motor_streaming_its_selected_command(modes):
    motors, targets = commissioned()
    for target, mode in zip(targets, modes):
        target.settings.update(final_mode=mode, adaptive_torque_enabled=True)
    log = Log()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and runner.completed == [14, 13, 12]
        for run in runner.runs:
            command = io.commands[run.target.motor.index]
            if run.p['final_mode'] == 'position':
                assert run.phase == 'HOLD'
                assert command[:4] == (run.target.zero_target, 0, run.p['hold_kp'], run.p['hold_kd'])
                assert command[4] == run.adaptive_torque
            else:
                assert run.phase == 'DAMPING'
                assert command == (0, 0, 0, run.p['damping_kd'], 0)
        result = next(data for event, data in log.events if event == 'CALIBRATION_PASS')
        assert result['state'] == ('HOLD' if len(set(modes)) == 1 else 'MIXED')
        assert result['final_states'] == {r.target.motor.index: r.phase for r in runner.runs}
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())


def test_near_target_position_mode_inherits_pre_brake_compensation():
    _, targets = commissioned(index=13)
    target = targets[0]
    target.settings.update(final_mode='position', adaptive_torque_enabled=True)
    run = h.MotorRun(target, 0)
    run.brake_compensation_nm = 0.5
    run.begin_return(0, h.Feedback(0, 1, target.zero_target - 0.005, 0, 0, 25), resume=True)
    for seq in range(2, 9):
        at = (seq - 1) * 0.01
        command = run.step(at, h.Feedback(at, seq, target.zero_target - 0.005, 0, 0, 25))
    assert run.phase == 'HOLD'
    assert command[:4] == (target.zero_target, 0, target.settings['hold_kp'], target.settings['hold_kd'])
    assert command[4] >= 0.5  # Compensation remains continuous while settling.


def test_final_mode_yaml_override_and_invalid_values(tmp_path):
    from test_motor_auto_zero import PACKAGE
    document = h.read_yaml(PACKAGE / 'config/motor_zero_calibration.yaml')
    document['defaults'].update(final_mode='position', brake_kd=0.2)
    for name in document['modules']['left_arm']:
        for key in ('brake_kd', 'final_mode'):
            document['joints'][name].pop(key, None)
    document['joints']['left_elbow_roll_joint']['final_mode'] = 'damping'
    document['joints']['left_elbow_roll_joint']['brake_kd'] = 0.35
    path = tmp_path / 'final_modes.yaml'
    path.write_text(h.yaml.safe_dump(document))
    _, targets = h.load_plan(h.motor_config_path(), path, PACKAGE / 'config/l0_v5.urdf', module='left_arm')
    assert [t.settings['final_mode'] for t in targets] == ['damping', 'position', 'position']
    assert [t.settings['brake_kd'] for t in targets] == [0.35, 0.2, 0.2]
    for value in ['hold', None, True, 0, {}]:
        targets[1].settings['final_mode'] = value
        assert 'final_mode must be damping or position' in h.motion_errors(targets[1])


@pytest.mark.parametrize('mode', ['damping', 'position'])
def test_controllable_motor_returns_directly_without_manual_catch_selection(mode):
    motors, targets = commissioned(index=14)
    target = targets[0]
    target.settings.update(final_mode=mode)
    assert not h.motion_errors(target)
    log = Log()
    io = Drive(targets, log, disable_on_zero=True)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and runner.completed == [14]
        assert not any(e == 'BRAKE_START' for e, _ in log.events)
        assert runner.runs[0].brake_count == 0 and io.zero_calls == [14] * 3
        assert runner.runs[0].phase == ('HOLD' if mode == 'position' else 'DAMPING')
    finally:
        runner.shutdown()
    assert not io.enabled[14]


@pytest.mark.parametrize('key,value', [('brake_kd', None), ('brake_kd', 6),
    ('brake_max_torque_nm', 3.1), ('brake_speed_ratio', 1), ('brake_settle_s', 0),
    ('brake_velocity_rad_s', 0), ('max_brake_attempts', 0), ('max_brake_attempts', True)])
def test_brake_config_validation(key, value):
    _, targets = commissioned(index=13)
    targets[0].settings[key] = value
    assert h.motion_errors(targets[0])


def test_speed_sign_reversal_changes_braking_direction_without_locking():
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    run.begin_brake(0, 'TEST')
    for sequence, velocity in enumerate([3, 1, 0.1, -0.1, -1, -3], 2):
        at = sequence * 0.01
        command = run.step(at, h.Feedback(at, sequence, 0, velocity, 0, 25))
        assert run.phase == 'BRAKE'
        torque = -command[3] * velocity
        assert torque * velocity < 0
        assert abs(torque) <= run.p['brake_max_torque_nm'] + 1e-12
        assert command[2] == command[4] == 0


def test_auto_brake_resume_full_sequence_and_continuous_mit():
    motors, targets = commissioned(index=14)
    log = Log()
    class FallingDrive(Drive):
        injected = False
        def spin(self, timeout):
            super().spin(timeout)
            command = self.commands.get(14)
            if not self.injected and command and command[2] > 0:
                f = self.feedback[14]
                self.feedback[14] = h.Feedback(f.received_at, f.sequence, f.q, 6.0, f.tau, f.temperature, f.enabled)
                self.injected = True
    io = FallingDrive(targets, log, disable_on_zero=True)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and runner.completed == [14]
        assert sum(e == 'BRAKE_START' for e, _ in log.events) == 1
        assert sum(e == 'BRAKE_RESUME' for e, _ in log.events) == 1
        assert runner.runs[0].brake_count == 1
        assert runner.runs[0].phase == 'DAMPING'
        stamps = [at for at, _ in io.command_batches]
        assert max(b-a for a, b in zip(stamps, stamps[1:])) <= 1 / runner.rate + 1e-9
    finally:
        runner.shutdown()
    assert not io.enabled[14]


@pytest.mark.parametrize('mode', ['damping', 'position'])
@pytest.mark.parametrize('value,valid', [(0.0, True), (0.5, True), (-0.1, False),
    (float('nan'), False), (float('inf'), False), (True, False), (None, False)])
def test_hold_kp_is_nonnegative_in_all_final_modes(mode, value, valid):
    _, targets = commissioned(index=14)
    t = targets[0]
    t.settings.update(final_mode=mode, hold_kp=value)
    errors = [error for error in h.motion_errors(t) if 'hold_kp' in error]
    assert bool(errors) is not valid


@pytest.mark.parametrize('reason', ['RETURN_OVERSPEED', 'RETURN_TRACKING_ERROR',
    'RETURN_POSITION_OUTSIDE_CORRIDOR', 'COMMAND_TORQUE_BUDGET'])
def test_return_braking_conditions_take_priority_over_position_jump(reason):
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.p['max_feedback_step_rad'] = 0.02
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    at, sequence, q, velocity = 0.01, 2, 0.03, 6.0
    if reason == 'RETURN_TRACKING_ERROR':
        run.step(0.2, h.Feedback(0.2, 2, 0, 0, 0, 25))
        run.step(0.21, h.Feedback(0.21, 3, -0.019, 0, 0, 25))
        run.step(0.22, h.Feedback(0.22, 4, -0.038, 0, 0, 25))
        at, sequence, q, velocity = 0.34, 5, -0.06, 0
    elif reason == 'RETURN_POSITION_OUTSIDE_CORRIDOR':
        q, velocity = -0.06, 0
    elif reason == 'COMMAND_TORQUE_BUDGET':
        run.p['bck_kp'] = 100
        at, velocity = 0.1, 0.1
    command = run.step(at, h.Feedback(at, sequence, q, velocity, 0, 25))
    assert run.phase == 'BRAKE' and run.brake_reason == reason
    assert command[2] == command[4] == 0
    assert abs(command[3] * velocity) <= run.p['brake_max_torque_nm'] + 1e-12
    assert run.last.q == q  # The next frame compares against this braking sample.


def test_unexplained_return_jump_still_fails_and_records_trigger_frame():
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.p['max_feedback_step_rad'] = 0.02
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    with pytest.raises(h.HomingError, match='POSITION_JUMP') as caught:
        run.step(0.01, h.Feedback(0.01, 2, 0.03, 0, 0, 25))
    d = caught.value.diagnostics
    assert d['previous_q_rad'] == 0 and d['q_rad'] == 0.03
    assert d['position_delta_rad'] == 0.03 and d['feedback_interval_s'] == 0.01
    assert d['sequence'] == 2 and d['previous_sequence'] == 1
    assert run.phase == 'MOVE' and run.brake_count == 0


@pytest.mark.parametrize('phase', ['SEEK', 'BRAKE', 'HOLD', 'RECOVER', 'DAMPING'])
def test_other_phases_keep_position_jump_as_a_hard_guard(phase):
    _, targets = commissioned(index=13)
    run = h.MotorRun(targets[0], 0)
    run.p['max_feedback_step_rad'] = 0.02
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    run.transition(phase, 0)
    with pytest.raises(h.HomingError, match='POSITION_JUMP'):
        run.step(0.01, h.Feedback(0.01, 2, 0.03, 3, 0, 25))


def test_brake_priority_keeps_invalid_stale_and_measured_torque_guards_first():
    _, targets = commissioned(index=13)
    for now, feedback, reason in [
        (0.01, h.Feedback(0.01, 2, float('nan'), 3, 0, 25), 'INVALID_FEEDBACK'),
        (0.3, h.Feedback(0.01, 2, 0.03, 3, 0, 25), 'FEEDBACK_TIMEOUT'),
        (0.01, h.Feedback(0.01, 2, 0.03, 3, 3, 25), 'TORQUE_LIMIT')]:
        run = h.MotorRun(targets[0], 0)
        run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
        with pytest.raises(h.HomingError, match=reason):
            run.step(now, feedback)
        assert run.brake_count == 0


def test_jump_with_overspeed_publishes_only_brake_and_logs_both_positions():
    motors, targets = commissioned(index=13)
    targets[0].settings['max_feedback_step_rad'] = 0.02
    io = Drive(targets, Log())
    runner = h.Runner(io, motors, targets, io.log, clock=io.clock)
    run = h.MotorRun(targets[0], 0)
    run.begin_return(0, h.Feedback(0, 1, 0, 0, 0, 25))
    runner.runs = [run]
    io.now = runner.last_pump = 0.01
    io.feedback[13] = h.Feedback(0.01, 2, 0.03, 6, 0, 25)
    runner.pump(force=True, spin=False)
    assert io.commands[13][2] == io.commands[13][4] == 0
    event = next(d for e, d in io.log.events if e == 'BRAKE_START')
    assert event['q_rad'] == 0.03 and event['previous_q_rad'] == 0
    assert event['position_delta_rad'] == 0.03 and event['feedback_interval_s'] == 0.01


def test_hold_error_keeps_enabled_motor_in_recovery_and_logs_transition():
    from copy import deepcopy
    motors, targets = commissioned(index=14)
    t = targets[0]
    t.settings.update(final_mode='position', hold_kp=0, adaptive_torque_enabled=True,
                      bck_kp=5, kd=0.3, bck_kd=0.3, hold_kd=0.3, hold_error_rad=0.17453292519943295,
                      arrival_tolerance_rad=0.01, arrival_settle_s=0.05)
    original = deepcopy(t.settings)
    io = Drive(targets, Log())
    runner = h.Runner(io, motors, targets, io.log, clock=io.clock)
    run = h.MotorRun(t, 0)
    run.transition('HOLD', 0)
    runner.runs = [run]
    io.now = runner.last_pump = 0.01
    io.feedback[14] = h.Feedback(0.01, 1, t.zero_target - 0.18, -0.5, 0.19, 25)
    runner.pump(force=True, spin=False)
    assert run.phase == 'RECOVER'
    assert io.commands[14][2:4] == (5, 0.3)
    assert any(e == 'HOLD_RECOVER_START' for e, _ in io.log.events)
    for step in range(1, 37):
        at = 0.01 + step * 0.01
        q = t.zero_target - 0.18 * (1 - step / 36)
        io.now = runner.last_pump = at
        io.feedback[14] = h.Feedback(at, step + 1, q, 0.5 if step < 36 else 0, 0, 25)
        runner.pump(force=True, spin=False)
    for seq in range(38, 46):
        at = 0.37 + (seq - 37) * 0.01
        io.now = runner.last_pump = at
        io.feedback[14] = h.Feedback(at, seq, t.zero_target, 0, 0, 25)
        runner.pump(force=True, spin=False)
    assert run.phase == 'HOLD' and io.commands[14][2] == 0
    assert any(e == 'HOLD_RECOVERED' for e, _ in io.log.events)
    assert not any(kind == 'disable' for kind, _, _ in io.calls)
    assert t.settings == original


@pytest.mark.parametrize('error,velocity,ff', [(1, -2, 0.5), (-1, 2, -0.5),
    (0, -20, 0.5), (0, 20, -0.5)])
def test_hold_output_uses_allowed_torque_without_altering_configuration(error, velocity, ff):
    from copy import deepcopy
    _, targets = commissioned(index=14)
    t = targets[0]
    run = h.MotorRun(t, 0)
    original = deepcopy(t.settings)
    command = (t.zero_target, 0, 5, 0.3, ff)
    f = h.Feedback(0.01, 1, t.zero_target - error, velocity, 0, 25)
    limited = run.limit_hold_torque(command, f)
    q, dq, kp, kd, applied_ff = limited
    estimate = kp * (q - f.q) + kd * (dq - f.dq) + applied_ff
    assert abs(estimate) == pytest.approx(t.settings['max_torque_nm'])
    assert applied_ff == ff and kp <= command[2] and kd <= command[3]
    assert run.hold_limited and run.hold_applied_torque_nm == pytest.approx(estimate)
    assert t.settings == original


def test_compensation_handoff_over_limit_keeps_return_gains_until_supported():
    _, targets = commissioned(index=13)
    t = targets[0]
    t.settings.update(final_mode='position', bck_kp=5, kd=0.3, bck_kd=0.3, hold_kp=0, hold_kd=0.3,
                      adaptive_torque_enabled=True, adaptive_torque_limit_nm=0.1,
                      arrival_tolerance_rad=0.15, arrival_settle_s=0.05)
    run = h.MotorRun(t, 0)
    run.transition('MOVE', 0)
    run.adaptive_torque = 0.09
    run.last = h.Feedback(0.01, 1, t.zero_target - 0.1, 0, 0, 25)
    run.transition('HOLD', 0.01)
    assert run.phase == 'RECOVER' and run.adaptive_torque == 0.09
    for seq, at in [(2, 0.02), (3, 0.08)]:
        command = run.step(at, h.Feedback(at, seq, t.zero_target - 0.1, 0, 0, 25))
    assert run.phase == 'RECOVER' and command[2] == 5
    assert run.hold_transfer_limited and abs(command[4]) <= 0.1
    assert t.settings['hold_kp'] == 0


@pytest.mark.parametrize('load', [-0.6, 0.6])
def test_zero_hold_kp_recovers_under_opposite_loads_without_dropping_support(load):
    _, targets = commissioned(index=13)
    t = targets[0]
    t.settings.update(final_mode='position', bck_kp=5, kd=0.2, bck_kd=0.2, hold_kp=0, hold_kd=0.2,
                      adaptive_torque_enabled=True, hold_error_rad=0.17453292519943295,
                      arrival_tolerance_rad=0.17453292519943295, arrival_settle_s=0.3)
    run = h.MotorRun(t, 0)
    run.transition('HOLD', 0)
    q, velocity, torque, recovered = t.zero_target, 0.0, 0.0, False
    dt = 0.002
    for sequence in range(1, 15001):
        now = sequence * dt
        command = run.step(now, h.Feedback(now, sequence, q, velocity, torque, 25))
        desired, dq, kp, kd, ff = command
        recovered |= run.phase == 'RECOVER'
        torque = kp * (desired - q) + kd * (dq - velocity) + ff
        assert abs(torque) <= t.settings['max_torque_nm'] + 1e-9
        assert abs(run.adaptive_torque) <= t.settings['adaptive_torque_limit_nm']
        velocity += (torque + load - 0.5 * velocity) / 0.05 * dt
        q += velocity * dt
    assert recovered and run.phase == 'HOLD'
    assert abs(q - t.zero_target) < t.settings['arrival_tolerance_rad']
    # kp=0 does not guarantee asymptotic position stability; support stays bounded.
    assert abs(run.adaptive_torque) <= t.settings['adaptive_torque_limit_nm']
    assert run.adaptive_torque == pytest.approx(-load, abs=0.1)


@pytest.mark.parametrize('adaptive', [False, True])
def test_hold_feedforward_in_recovery_is_fixed_only_when_adaptive_is_disabled(adaptive):
    _, targets = commissioned(index=13)
    t = targets[0]
    t.settings.update(hold_kp=0, adaptive_torque_enabled=adaptive, hold_torque_ff_nm=0.4,
                      move_torque_ff_nm=-0.2, bck_kp=5, kd=0.3, bck_kd=0.3)
    run = h.MotorRun(t, 0)
    run.transition('HOLD', 0)
    run.adaptive_torque = 0.1
    command = run.step(0.01, h.Feedback(0.01, 1, t.zero_target - 0.06, 0, 0, 25))
    assert run.phase == 'RECOVER'
    assert command[4] == pytest.approx(0.1 if adaptive else 0.4)


@pytest.mark.parametrize('phase', ['HOLD', 'RECOVER'])
def test_support_does_not_override_measured_torque_or_feedback_failures(phase):
    _, targets = commissioned(index=13)
    t = targets[0]
    run = h.MotorRun(t, 0)
    run.transition(phase, 0)
    with pytest.raises(h.HomingError, match='TORQUE_LIMIT'):
        run.step(0.01, h.Feedback(0.01, 1, t.zero_target - 0.2, 0, 2.1, 25))
    with pytest.raises(h.HomingError, match='FEEDBACK_TIMEOUT'):
        run.step(0.3, h.Feedback(0.01, 2, t.zero_target, 0, 0, 25))


def test_completed_motor_keeps_streaming_during_recovery_without_disabling():
    motors, targets = commissioned(index=14)
    t = targets[0]
    t.settings.update(final_mode='position', hold_kp=0, adaptive_torque_enabled=True)
    log = Log()
    class DisturbedDrive(Drive):
        injected = False
        def spin(self, timeout):
            super().spin(timeout)
            if self.completed_at is not None and not self.injected:
                self.positions[14] = t.zero_target - 0.06
                self.feedback[14] = h.Feedback(self.now, self.seq, self.positions[14], -0.2, 0.01, 25, self.enabled[14])
                self.injected = True
    io = DisturbedDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done and runner.completed == [14]
        assert any(e == 'HOLD_RECOVER_START' for e, _ in log.events)
        assert any(e == 'HOLD_RECOVERED' for e, _ in log.events)
        assert runner.runs[0].phase == 'HOLD'
        assert not any(kind == 'disable' for kind, _, _ in io.calls)
        stamps = [at for at, _ in io.command_batches]
        assert max(b-a for a,b in zip(stamps, stamps[1:])) <= 1 / runner.rate + 1e-9
    finally:
        runner.shutdown()
    assert not io.enabled[14]
