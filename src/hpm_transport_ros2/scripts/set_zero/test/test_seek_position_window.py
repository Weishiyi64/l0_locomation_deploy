# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Contact detection uses position windows, not individual noisy MIT velocities."""
import math

import pytest

from test_motor_auto_zero import Drive, Log, commissioned, h


def test_ls_noisy_standstill_completes_seek_stop_and_zero_prepare_without_command_gap():
    motors, targets = commissioned(index=11)
    log = Log()
    class NoisyStopDrive(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            for index, f in list(self.feedback.items()):
                if index not in self.zeroed and abs(f.q - self.stop_positions[index]) < 1e-8:
                    noise = 0.17 if f.sequence % 2 else -0.14
                    self.feedback[index] = h.Feedback(f.received_at, f.sequence, f.q, noise, f.tau, f.temperature, f.enabled)
    io = NoisyStopDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.completed == [11] and io.zero_calls == [11]
        assert any(event == 'SEEK_STATUS' for event, _ in log.events)
        contacts = [data for event, data in log.events if event == 'CONTACT_DETECTED']
        assert len(contacts) == 1 and contacts[0]['position_stable'] and contacts[0]['torque_contact']
        assert contacts[0]['reported_velocity_peak_rad_s'] > targets[0].settings['stall_velocity_rad_s']
        stamps = [at for at, _ in io.command_batches]
        assert max(b-a for a,b in zip(stamps, stamps[1:])) <= 1 / runner.rate + 1e-9
    finally:
        runner.shutdown()
    assert not io.enabled[11]


def test_stationary_positions_without_contact_torque_do_not_write_zero():
    _, targets = commissioned(index=11)
    run = h.MotorRun(targets[0], 0)
    run.p['max_seek_time_s'] = 0.6
    for i in range(1, 61):
        at = i * 0.01
        run.step(at, h.Feedback(at, i, 0, 0.15, 0, 25))
        assert run.phase == 'SEEK'
    with pytest.raises(h.HomingError, match='SEEK_TIME_OR_TRAVEL_LIMIT') as error:
        run.step(0.62, h.Feedback(0.62, 61, 0, 0.15, 0, 25))
    assert error.value.diagnostics['seek_window']['position_stable']
    assert not error.value.diagnostics['seek_window']['torque_contact']


@pytest.mark.parametrize('motion', ['sliding', 'oscillation'])
def test_position_motion_blocks_contact_even_when_reported_velocity_is_zero(motion):
    _, targets = commissioned(index=11)
    run = h.MotorRun(targets[0], 0)
    for i in range(1, 101):
        at = i * 0.01
        q = -0.04 * at if motion == 'sliding' else 0.003 * math.sin(20 * math.pi * at)
        run.step(at, h.Feedback(at, i, q, 0, 0.2, 25))
        assert run.phase == 'SEEK'
    assert not run.seek_diagnostics['position_stable']


def test_repeated_zero_prepare_reads_do_not_erase_or_advance_position_window():
    _, targets = commissioned(index=11)
    run = h.MotorRun(targets[0], 0)
    run.transition('ZERO_PREPARE', 0)
    for i in range(1, 7):
        f = h.Feedback(i * 0.01, i, 0, 0.15, 0.2, 25)
        result = run.motion_settled(f, 0.05, 0.02, position_limit=0.001)
        length = len(run.window)
        assert not run.motion_settled(f, 0.05, 0.02, position_limit=0.001)
        assert len(run.window) == length
        assert result is (i == 6)
