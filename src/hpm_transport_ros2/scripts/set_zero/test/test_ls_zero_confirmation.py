"""LS ACK/coordinate retries with a fake drive; never touches a ROS graph."""
from types import SimpleNamespace
import pytest
from test_motor_auto_zero import Drive, Future, Log, commissioned, h


class LSDrive(Drive):
    def __init__(self, targets, log, outcomes):
        super().__init__(targets, log)
        self.outcomes = outcomes
        self.ls_pending = None

    def zero(self, motor):
        if motor.protocol != "LS":
            return super().zero(motor)
        p = self.targets[motor.index].settings
        assert self.commands[motor.index][1:] == (
            p["seek_direction"] * p["seek_speed_rad_s"], 0, p["kd"], p["seek_torque_ff_nm"])
        self.zero_calls.append(motor.index)
        self.calls.append(("zero_request", motor.index, self.now))
        outcome = self.outcomes[min(len(self.zero_calls) - 1, len(self.outcomes) - 1)]
        result = 71 if outcome == "timeout" else 72 if outcome == "device_error" else 73 if outcome == "host_reject" else 0
        future = Future(SimpleNamespace(motor_types=[motor.index], results=[result],
                                        messages=[outcome], TIMEOUT=71, DEVICE_FAILED=72, REJECTED=73), False)
        self.ls_pending = (motor.index, self.now + 0.04, future, outcome)
        return future

    def spin(self, timeout):
        super().spin(timeout)
        if self.ls_pending and self.now >= self.ls_pending[1]:
            index, _, future, outcome = self.ls_pending
            if outcome == "service_lost":
                return
            if outcome == "ok":
                self.zeroed.add(index)
                self.stop_positions[index] -= self.positions[index]
                self.positions[index] = 0.0
            future.ready = True
            if future.result().results == [0]:
                self.calls.append(("ls_zero_ack", index, self.now))
            self.ls_pending = None


def ls_case(outcomes):
    motors, targets = commissioned(index=18)
    targets[0].settings.update(direction_confirmed=True, ls_zero_attempt_timeout_s=0.15,
                               ls_zero_total_timeout_s=0.5, zero_max_attempts=3)
    log = Log()
    drive = LSDrive(targets, log, outcomes)
    return h.Runner(drive, motors, targets, log, clock=drive.clock), drive, log


@pytest.mark.parametrize("outcomes, attempts", [(["ok"], 1), (["timeout", "ok"], 2),
                                               (["old_position", "ok"], 2),
                                               (["timeout"] * 4 + ["ok"], 5)])
def test_ls_requires_ack_and_fresh_zero_position(outcomes, attempts):
    runner, drive, log = ls_case(outcomes)
    runner.targets[0].settings["zero_max_attempts"] = max(3, attempts)
    assert not h.motion_errors(runner.targets[0])
    with pytest.raises(KeyboardInterrupt):
        runner.calibrate(write_zero=True)
    assert runner.all_done and runner.completed == [18]
    assert len(drive.zero_calls) == attempts
    expected_order = ["enable"]
    for outcome in outcomes:
        expected_order.append("zero_request")
        if outcome in {"ok", "old_position"}:
            expected_order.extend(("ls_zero_ack", "enable"))
    assert [kind for kind, _, _ in drive.calls if kind in expected_order] == expected_order
    confirmed = [data for event, data in log.events if event == "ZERO_CONFIRMED"]
    assert len(confirmed) == 1 and abs(confirmed[0]["q_rad"]) <= 0.01
    assert next(data for event, data in log.events if event == "MOTOR_PASS")["set_zero"] == "CONFIRMED"
    assert not runner.runs[0].zero_unknown
    # Control continues at the configured rate throughout every confirmation wait.
    times = [t for t, _ in drive.command_batches]
    assert max(b - a for a, b in zip(times, times[1:])) <= 0.011


@pytest.mark.parametrize("outcome, reason, attempts", [
    ("timeout", "LS_ZERO_NOT_CONFIRMED", 3), ("old_position", "LS_ZERO_NOT_CONFIRMED", 3),
    ("device_error", "LS_ZERO_FAILED", 1), ("service_lost", "LS_ZERO_SERVICE_TIMEOUT", 1)])
def test_ls_failure_never_returns_or_advances(outcome, reason, attempts):
    runner, drive, log = ls_case([outcome])
    try:
        with pytest.raises(h.HomingError, match=reason):
            runner.calibrate(write_zero=True)
        assert len(drive.zero_calls) == attempts
        assert runner.current.zero_unknown and not runner.completed
        assert not any(e in {"RETURN_START", "MOTOR_PASS", "CALIBRATION_PASS"} for e, _ in log.events)
    finally:
        runner.shutdown()
    assert not drive.enabled[18]


def test_ls_total_deadline_bounds_retries():
    runner, drive, log = ls_case(["old_position"])
    runner.targets[0].settings["ls_zero_total_timeout_s"] = 0.2
    with pytest.raises(h.HomingError, match="LS_ZERO_TOTAL_TIMEOUT"):
        runner.calibrate(write_zero=True)
    start = next(data for e, data in log.events if e == "ZERO_REQUEST")
    assert start["attempt"] == 1 and len(drive.zero_calls) == 2
    first_write = next(t for kind, _, t in drive.calls if kind == "zero_request")
    assert drive.clock() - first_write <= 0.211
    assert not runner.completed


@pytest.mark.parametrize("value", [0, -1, float("nan"), True, None])
def test_ls_timeouts_validated(value):
    runner, _, _ = ls_case(["ok"])
    runner.targets[0].settings["ls_zero_total_timeout_s"] = value
    assert any("ls_zero_total_timeout_s" in e for e in h.motion_errors(runner.targets[0]))


@pytest.mark.parametrize("unlimited", [False, True])
def test_ls_feedback_interruption_still_aborts_without_retry(unlimited):
    runner, drive, log = ls_case(["old_position"])
    runner.targets[0].settings["ls_zero_retry_until_success"] = unlimited
    original_zero = drive.zero
    def drop_after_write(motor):
        future = original_zero(motor)
        drive.dropped.add(motor.index)
        return future
    drive.zero = drop_after_write
    runner.targets[0].settings["ls_zero_attempt_timeout_s"] = 1.0
    with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
        runner.calibrate(write_zero=True)
    assert len(drive.zero_calls) == 1 and not runner.completed


@pytest.mark.parametrize("failure", ["timeout", "old_position", "device_error", "host_reject"])
def test_ls_retry_until_success_can_exceed_count_within_total_timeout(failure):
    runner, drive, log = ls_case([failure] * 6 + ["ok"])
    runner.targets[0].settings.update(ls_zero_retry_until_success=True, zero_max_attempts=1,
                                       ls_zero_total_timeout_s=2.0)
    with pytest.raises(KeyboardInterrupt):
        runner.calibrate(write_zero=True)
    assert runner.all_done and len(drive.zero_calls) == 7
    assert runner.current is None or not runner.current.zero_unknown
    assert all(d["will_retry"] for e, d in log.events if e == "ZERO_RETRY")
    times = [t for t, _ in drive.command_batches]
    assert max(b-a for a,b in zip(times,times[1:])) <= 0.011


def test_ls_unlimited_wait_can_be_stopped_and_disables_motor():
    runner, drive, log = ls_case(["timeout"])
    runner.targets[0].settings.update(ls_zero_retry_until_success=True, zero_max_attempts=1,
                                       ls_zero_total_timeout_s=2.0)
    original_spin = drive.spin
    def operator_stop(timeout):
        original_spin(timeout)
        if len(drive.zero_calls) >= 5:
            raise KeyboardInterrupt
    drive.spin = operator_stop
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.calibrate(write_zero=True)
        assert len(drive.zero_calls) >= 5 and not runner.completed
        assert runner.current.zero_unknown
        assert not any(e == "RETURN_START" for e, _ in log.events)
    finally:
        drive.spin = original_spin
        runner.shutdown()
    assert not drive.enabled[18]


@pytest.mark.parametrize("outcome", ["timeout", "old_position", "service_lost"])
def test_ls_total_timeout_applies_to_retry_until_success_and_keeps_control_streaming(outcome):
    runner, drive, log = ls_case([outcome])
    runner.targets[0].settings.update(ls_zero_retry_until_success=True, zero_max_attempts=1,
                                       ls_zero_attempt_timeout_s=1.0, ls_zero_total_timeout_s=0.45)
    try:
        with pytest.raises(h.HomingError, match="LS_ZERO_TOTAL_TIMEOUT"):
            runner.calibrate(write_zero=True)
        started=next(at for kind,_,at in drive.calls if kind=="zero_request")
        assert 0.45 <= drive.clock()-started <= 0.461
        assert not any(e in {"RETURN_START","MOTOR_PASS","CALIBRATION_PASS"} for e,_ in log.events)
        if outcome=="timeout":
            assert len(drive.zero_calls)>1
        if outcome in {"timeout", "service_lost"}:
            assert sum(kind == "enable" for kind, _, _ in drive.calls) == 1
        stamps=[at for at,_ in drive.command_batches]
        assert max(b-a for a,b in zip(stamps,stamps[1:])) <= 0.011
    finally:
        runner.shutdown()
    assert not drive.enabled[18]


@pytest.mark.parametrize("version", ["old", "new", "missing"])
def test_launch_firmware_selects_old_enable_burst_or_new_status_check(version):
    from dataclasses import replace

    runner, drive, log = ls_case(["ok"])
    index = 18
    runner.targets[0].settings["ls_zero_attempt_timeout_s"] = 0.4
    values = drive.normal_mode().result().values
    values[-1] = SimpleNamespace(type=4 if version != "missing" else 0, string_value=version)
    drive.normal_mode = lambda: Future(SimpleNamespace(values=values))
    original_spin = drive.spin

    def no_enable_bit(timeout):
        original_spin(timeout)
        if index in drive.feedback:
            drive.feedback[index] = replace(drive.feedback[index], enabled=None)

    drive.spin = no_enable_bit
    try:
        if version == "missing":
            with pytest.raises(h.HomingError, match="ls_firmware_version"):
                runner.preflight(True)
            assert not drive.calls
            return
        runner.preflight(True)
        if version == "new":
            with pytest.raises(h.HomingError, match="ENABLE_FEEDBACK_UNAVAILABLE"):
                runner.calibrate(write_zero=True, keep_running=False)
            assert not drive.zero_calls and not runner.completed
            return
        runner.calibrate(write_zero=True, keep_running=False)
        ack = next(t for event, _, t in drive.calls if event == "ls_zero_ack")
        enables = [t for event, _, t in drive.calls if event == "enable" and t >= ack]
        assert len(enables) == 3 and min(b - a for a, b in zip(enables, enables[1:])) >= 0.05 - 1e-9
        assert runner.completed == [index]
        ready = next(data for event, data in log.events if event == "RETURN_FEEDBACK_READY")
        assert ready["enabled"] is None and not ready["enable_verified"] and 0.1 - 1e-9 <= ready["stable_s"] < 0.15
    finally:
        runner.shutdown()
