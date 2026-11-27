# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Repeated enables during dispatch/braking, each immediately followed by MIT."""
import pytest

from test_motor_auto_zero import Drive, Log, commissioned, h


@pytest.mark.parametrize("rate", [100.0, 500.0])
def test_repeated_enable_without_spin_and_continuous_mit(rate):
    motors, targets = commissioned(index=14)
    targets[0].settings["rate_hz"] = rate
    log = Log()
    class RecordingDrive(Drive):
        def __init__(self):
            super().__init__(targets, log, disable_on_zero=True)
            self.events = []

        def spin(self, timeout):
            self.events.append(("spin", self.now, None))
            super().spin(timeout)

        def enable(self, motors, enabled):
            self.events.append(("enable" if enabled else "disable", self.now, None))
            super().enable(motors, enabled)

        def command(self, commands):
            self.events.append(("command", self.now, list(commands)))
            super().command(commands)
    io = RecordingDrive()
    runner = h.Runner(io, motors, targets, log, clock=io.clock)
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert runner.completed == [14]
        assert len([c for c in io.calls if c[0] == "enable"]) > 9
        acks = [at for kind, _, at in io.calls if kind == "zero_ack"]
        enabled = [at for kind, _, at in io.calls if kind == "enable"][1:]
        assert len(acks) == 3 and all(at in enabled for at in acks)
        for i, (event, at, _) in enumerate(io.events):
            if event == "enable" and at in enabled:
                following, stamp, commands = io.events[i + 1]
                assert following == "command" and stamp == at  # No spin/sleep between them.
                command = commands[0][1]
                assert command[:3] == (0, 0, 0)
                assert command[3] == 0
                assert command[4] == 0
        stamps = [t for t, _ in io.command_batches]
        assert max(b - a for a, b in zip(stamps, stamps[1:])) <= 1 / rate + 1e-9
        assert not any(e == "disable" for e, _, _ in io.calls)
    finally:
        runner.shutdown()
    assert not io.enabled[14]
