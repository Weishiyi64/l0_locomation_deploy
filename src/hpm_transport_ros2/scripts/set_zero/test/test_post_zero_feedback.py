"""Replay the write-zero handoff with simulated feedback only."""
from dataclasses import replace

import pytest

from test_motor_auto_zero import Drive, Log, commissioned, h


@pytest.mark.parametrize("index", [8, 9, 11])  # MR, RS, LS
@pytest.mark.parametrize("fault", ["recovers", "disabled", "old_position", "missing", "unknown"])
def test_return_requires_fresh_stable_zero_and_enable(index, fault):
    motors, targets = commissioned(index=index)
    log = Log()

    class Replay(Drive):
        gate_start = None
        disabled_at = None
        first_enabled_after_retry = None

        def spin(self, timeout):
            super().spin(timeout)
            if self.gate_start is None or self.log.events[-1][0] in ("RETURN_START", "MOTOR_PASS"):
                return
            if runner.current.zero_confirmed:
                return
            f = self.feedback[index]
            elapsed = self.now - self.gate_start
            if elapsed < 0.038:
                # The MR initially replied enabled before the delayed reset.
                return
            if self.disabled_at is None:
                self.disabled_at = self.now
                self.enabled[index] = False
            if fault == "missing":
                self.dropped.add(index)
            elif fault == "unknown":
                self.feedback[index] = replace(f, enabled=None)
            elif fault == "old_position":
                self.feedback[index] = replace(f, q=0.3)
            elif fault == "disabled" or not self.enabled[index]:
                self.feedback[index] = replace(f, enabled=False)
            elif self.first_enabled_after_retry is None:
                self.first_enabled_after_retry = self.now

    io = Replay(targets, log)

    class Runner(h.Runner):
        def confirm_return_ready(self, run):
            io.gate_start = self.clock()
            return super().confirm_return_ready(run)

    runner = Runner(io, motors, targets, log, clock=io.clock)
    try:
        if fault == "recovers":
            runner.calibrate(write_zero=True, keep_running=False)
            assert runner.completed == [index]
            ready = next(data for event, data in log.events if event == "RETURN_FEEDBACK_READY")
            assert ready["enabled"] and ready["zero_verified"] and 0.1 - 1e-9 <= ready["stable_s"] < 0.15
            assert any(kind == "enable" and at > io.disabled_at for kind, _, at in io.calls)
            first_move = next(t for t, commands in io.command_batches
                              if t > io.gate_start and commands[0][1][2] > 0)
            assert 0.1 - 1e-9 <= first_move - io.first_enabled_after_retry < 0.15
        else:
            reason = {"missing": "FEEDBACK_TIMEOUT", "unknown": "ENABLE_FEEDBACK_UNAVAILABLE"}.get(
                fault, "RETURN_READY_TIMEOUT")
            with pytest.raises(h.HomingError, match=reason):
                runner.calibrate(write_zero=True, keep_running=False)
            assert not runner.completed and runner.current.zero_unknown
            assert not any(e == "RETURN_START" for e, _ in log.events)
            pending = [command for t, batch in io.command_batches for _, command in batch if t > io.gate_start]
            assert pending and all(cmd[2] == 0 for cmd in pending)
            if targets[0].motor.protocol != "LS":
                assert all(cmd == (0, 0, 0, 0, 0) for cmd in pending)
    finally:
        runner.shutdown()
    assert not io.enabled[index]


def test_return_cannot_pass_if_disabled_when_manually_moved_to_target():
    motors, targets = commissioned(index=8)
    log = Log()

    class Replay(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            if self.commands.get(8, (0, 0, 0))[2] > 0:
                self.feedback[8] = replace(self.feedback[8], q=targets[0].zero_target, enabled=False)

    io = Replay(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(h.HomingError, match="ENABLE_FEEDBACK_LOST"):
            runner.calibrate(write_zero=True, keep_running=False)
        assert not runner.completed and not any(e == "MOTOR_PASS" for e, _ in log.events)
    finally:
        runner.shutdown()


@pytest.mark.parametrize("index", [8, 9, 11])
@pytest.mark.parametrize("offset,passes", [(0.0192645, True), (-0.0192645, True), (0.021, False),
                                            pytest.param(None, False, id="moving-inside-band")])
def test_static_post_zero_offset(index, offset, passes):
    motors, targets = commissioned(index=index)
    log = Log()

    class OffsetDrive(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            if index in self.zeroed and not runner.current.zero_confirmed:
                sample_q = offset if offset is not None else (0.014 if self.seq % 2 else 0.019)
                self.feedback[index] = replace(self.feedback[index], q=sample_q)

    io = OffsetDrive(targets, log)
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        if passes:
            runner.calibrate(write_zero=True, keep_running=False)
            assert runner.completed == [index]
        else:
            targets[0].settings.update(zero_max_attempts=1, ls_zero_total_timeout_s=0.3)
            with pytest.raises(h.HomingError, match="RETURN_READY_TIMEOUT|LS_ZERO_TOTAL_TIMEOUT"):
                runner.calibrate(write_zero=True, keep_running=False)
            assert not any(e == "RETURN_START" for e, _ in log.events)
    finally:
        runner.shutdown()


@pytest.mark.parametrize("index", [0, 6, 12, 15])
@pytest.mark.parametrize("speed,braking", [(7.07, True), (0.0, False)])
def test_pitch_first_post_write_command_can_brake(index, speed, braking):
    motors, targets = commissioned(index=index)
    log = Log()
    handoff = []

    class Runner(h.Runner):
        def confirm_return_ready(self, run):
            handoff.append(self.clock())
            return super().confirm_return_ready(run)

    class Replay(Drive):
        def spin(self, timeout):
            super().spin(timeout)
            if handoff and not runner.current.return_feedback_ready:
                self.feedback[index] = replace(self.feedback[index], q=0.10, dq=speed)

        def command(self, commands):
            super().command(commands)
            if handoff and runner.current.return_feedback_ready:
                assert runner.current.phase == ("BRAKE" if braking else "MOVE")
                assert self.now - handoff[0] < 0.1
                if braking:
                    q, dq, kp, kd, ff = self.commands[index]
                    assert (q, dq, kp, ff) == (0, 0, 0, 0) and kd > 0
                    assert kd * abs(speed) <= targets[0].settings["brake_max_torque_nm"] + 1e-9
                raise KeyboardInterrupt

    io = Replay(targets, log)
    runner = Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True, keep_running=False)
        ready = next(data for event, data in log.events if event == "RETURN_FEEDBACK_READY")
        assert ready["handoff"] == "TWO_FRESH_FRAMES" and ready["enable_verified"]
        assert not ready["zero_verified"] and ready["feedback_interval_s"] > 0
    finally:
        handoff.clear()
        runner.shutdown()
