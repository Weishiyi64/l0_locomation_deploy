# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Offline module-only knee damping; no ROS graph or hardware is used."""
import math
from copy import deepcopy

import pytest

from test_motor_auto_zero import Drive, Log, PACKAGE, commissioned, h


@pytest.mark.parametrize('module,knee_index,hip_index', [('left_leg', 3, 0), ('right_leg', 9, 6)])
@pytest.mark.parametrize('final_mode', ['position', 'damping'])
def test_knee_damps_without_prebend_then_restores_original_hold(module, knee_index, hip_index, final_mode):
    motors, targets = commissioned(module)
    knee = next(t for t in targets if t.motor.index == knee_index)
    hip = next(t for t in targets if t.motor.index == hip_index)
    knee.settings.update(final_mode=final_mode, hip_pitch_assist_seek_kd=0.15,
                         hip_pitch_assist_return_kd=0.45)
    hip.settings['final_mode'] = final_mode
    original = deepcopy(knee.settings)
    log, samples = Log(), []

    class ObservingDrive(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            kr = next((r for r in runner.runs if r.target.motor.index == knee_index), None)
            # The fixture can move under pure damping; simulate fresh position drift.
            if kr and kr.phase == 'DAMPING' and any(e == 'KNEE_ASSIST_DAMPING' for e, _ in log.events):
                q = max(knee.zero_target - 0.15, self.positions[knee_index] - 0.001)
                self.positions[knee_index] = q
                self.feedback[knee_index] = h.Feedback(self.now, self.seq, q, -0.1, 0.03, 25, self.enabled[knee_index])

        def command(self, commands):
            indices = [m.index for m, _ in commands]
            assert len(indices) == len(set(indices))
            assert {i for i, enabled in self.enabled.items() if enabled} <= set(indices)
            super().command(commands)
            if any(e == 'KNEE_ASSIST_DAMPING' for e, _ in log.events):
                kr = next(r for r in runner.runs if r.target.motor.index == knee_index)
                hr = next(r for r in runner.runs if r.target.motor.index == hip_index)
                stage = [d['stage'] for e, d in log.events if e == 'KNEE_ASSIST_DAMPING'][-1]
                samples.append((stage, kr.phase, hr.phase, self.commands[knee_index]))

    io = ObservingDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock,
                      knee_assist=h.knee_assist_plan(targets, module))
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done
        assert not any(e == 'KNEE_ASSIST_START' for e, _ in log.events)
        for stage, kd in [('seek', 0.15), ('return', 0.45)]:
            damping = [cmd for st, kp, _, cmd in samples if st == stage and kp == 'DAMPING']
            assert damping and all(cmd == (0, 0, 0, kd, 0) for cmd in damping)
        during_seek = [kp for _, kp, hp, _ in samples if hp in {'SEEK', 'STOP', 'ZERO_PREPARE', 'ZERO_PENDING'}]
        assert during_seek and set(during_seek) == {'DAMPING'}
        during_return = [kp for _, kp, hp, _ in samples if hp in {'MOVE', 'BRAKE'}]
        assert during_return and set(during_return) == {'DAMPING'}
        for idx in (knee_index, hip_index):
            run = next(r for r in runner.runs if r.target.motor.index == idx)
            assert run.phase == 'HOLD'
        kr = next(r for r in runner.runs if r.target.motor.index == knee_index)
        assert kr.position_target == knee.zero_target
        assert kr.p is knee.settings and knee.settings == original
        restore = next(d for e, d in log.events if e == 'KNEE_ASSIST_RESTORE')
        assert restore['target_rad'] == knee.zero_target
        assert abs(restore['actual_rad'] - knee.zero_target) > 0.1
        assert any(kp == 'MOVE' and hp == 'HOLD' for _, kp, hp, _ in samples)
        assert io.zero_calls.count(knee_index) == 3
        if module == 'right_leg':
            assert 7 not in runner.owned
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())


@pytest.mark.parametrize('module', ['left_leg', 'right_leg'])
def test_yaml_switch_and_single_motor_selection(module, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('No ROS IO allowed')
    monkeypatch.setattr(h, 'make_ros_io', forbidden)
    doc = h.read_yaml(PACKAGE / 'config/motor_zero_calibration.yaml')
    config = tmp_path / 'profile.yaml'
    for enabled in (False, True):
        doc['hip_pitch_assist_enabled'] = enabled
        config.write_text(h.yaml.safe_dump(doc))
        args = h.parse_args(['--module', module, '--calibration-config', str(config)])
        assert (h.prepare_run(args)[4] is not None) == enabled
    _, targets = commissioned(index=0 if module == 'left_leg' else 6)
    assert h.knee_assist_plan(targets, None) is None


def test_damping_gain_validation_and_order():
    _, targets = commissioned('left_leg')
    knee = next(t for t in targets if t.motor.index == 3)
    for key in ('hip_pitch_assist_seek_kd', 'hip_pitch_assist_return_kd'):
        previous = knee.settings[key]
        for bad in (None, True, '1', -0.1, math.nan, math.inf, knee.limits['kd_max'] + 1):
            knee.settings[key] = bad
            with pytest.raises(ValueError, match=key):
                h.knee_assist_plan(targets, 'left_leg')
        knee.settings[key] = previous
    knee.settings.update(hip_pitch_assist_seek_kd=0.5, hip_pitch_assist_return_kd=0.2)
    with pytest.raises(ValueError, match='must not be below'):
        h.knee_assist_plan(targets, 'left_leg')
    with pytest.raises(ValueError, match='knee must be calibrated before'):
        h.knee_assist_plan(list(reversed(targets)), 'left_leg')


def test_feedback_loss_during_damping_stops_sequence_and_disables_owned_motors():
    motors, targets = commissioned('left_leg')
    log = Log()

    class LostDrive(Drive):
        def spin(self, timeout):
            if any(e == 'KNEE_ASSIST_DAMPING' for e, _ in log.events):
                self.dropped.add(3)
            super().spin(timeout)

    io = LostDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock,
                      knee_assist=h.knee_assist_plan(targets, 'left_leg'))
    try:
        with pytest.raises(h.HomingError, match='FEEDBACK_TIMEOUT'):
            runner.calibrate(write_zero=True)
        assert runner.current.target.motor.index == 3
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())
