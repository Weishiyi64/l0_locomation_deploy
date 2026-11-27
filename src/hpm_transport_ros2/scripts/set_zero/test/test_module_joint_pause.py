# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Module pacing must keep commands and shutdown guards alive."""
import math
import pytest
from test_motor_auto_zero import Drive, Log, PACKAGE, commissioned, h


@pytest.mark.parametrize('count,pause', [(2, 0.5), (2, 0.0), (1, 0.5)])
def test_pause_between_joints_only_and_keeps_streaming(count, pause):
    motors, targets = commissioned('left_arm')
    targets = targets[:count]
    stamped = []

    class TimedLog(Log):
        def write(self, event, **data):
            super().write(event, **data)
            stamped.append((io.now, event, data))

    log = TimedLog()
    io = Drive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.module_joint_pause_s = pause
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.all_done
        starts = [(t, d) for t, e, d in stamped if e == 'JOINT_PAUSE_START']
        ends = [(t, d) for t, e, d in stamped if e == 'JOINT_PAUSE_END']
        assert len(starts) == len(ends) == (count - 1 if pause else 0)
        for (start, event), (end, _) in zip(starts, ends):
            assert pause <= end - start <= pause + 2 / runner.rate
            next_start = next(t for t, e, d in stamped if e == 'MOTOR_START' and d['index'] == event['next_index'])
            assert next_start >= end
            batches = [(t, c) for t, c in io.command_batches if start <= t <= end]
            assert len(batches) >= pause * runner.rate / 2
            assert all(event['index'] in [m.index for m, _ in cmds] for _, cmds in batches)
            assert all(b[0] - a[0] <= 2 / runner.rate for a, b in zip(batches, batches[1:]))
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())


@pytest.mark.parametrize('fault', ['feedback', 'interrupt'])
def test_pause_still_checks_feedback_and_handles_operator_stop(fault):
    motors, targets = commissioned('left_arm')
    targets = targets[:2]
    first, second = (t.motor.index for t in targets)
    log = Log()

    class FailingDrive(Drive):
        def spin(self, timeout):
            if any(e == 'JOINT_PAUSE_START' for e, _ in log.events):
                if fault == 'interrupt' and not getattr(self, 'interrupted', False):
                    self.interrupted = True
                    raise KeyboardInterrupt
                if fault == 'feedback':
                    self.dropped.add(first)
            super().spin(timeout)

    io = FailingDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    runner.module_joint_pause_s = 0.5
    try:
        with pytest.raises(KeyboardInterrupt if fault == 'interrupt' else h.HomingError):
            runner.calibrate(write_zero=True)
        assert second not in runner.owned and not runner.all_done
    finally:
        runner.shutdown()
    assert not any(io.enabled.values())


def test_yaml_pause_validation_before_ros(tmp_path, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError('No ROS IO allowed')
    monkeypatch.setattr(h, 'make_ros_io', forbidden)
    doc = h.read_yaml(PACKAGE / 'config/motor_zero_calibration.yaml')
    file = tmp_path / 'config.yaml'
    args = h.parse_args(['--module', 'left_arm', '--calibration-config', str(file)])
    for value in (0, 0.5, 1.0):
        doc['module_joint_pause_s'] = value
        file.write_text(h.yaml.safe_dump(doc))
        assert h.prepare_run(args)[3] == value
    for value in (-1, True, None, '0.5', math.nan, math.inf):
        doc['module_joint_pause_s'] = value
        file.write_text(h.yaml.safe_dump(doc))
        with pytest.raises(ValueError, match='module_joint_pause_s'):
            h.prepare_run(args)
    doc['module_joint_pause_s'] = 0.5
    file.write_text(h.yaml.safe_dump(doc))
    assert h.main(['--motor-index', '14', '--calibration-config', str(file), '--dry-run']) == 0
    assert 'MODULE_PLAN' not in capsys.readouterr().out
