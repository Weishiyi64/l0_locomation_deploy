"""Offline module motion-test and return checks."""
import math
import pytest
from test_motor_auto_zero import Drive, Log, commissioned, h


class TimedLog(Log):
    def write(self, event, **data):
        super().write(event, _test_time=self.clock(), **data)


class TestDrive(Drive):
    __test__ = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.returning = set()

    def spin(self, timeout):
        # The legacy seek fake clamps both velocity signs to a single stop.
        # Already-zeroed test joints need an ideal bidirectional velocity plant.
        # SETZERO changes the encoder origin. The stop still resists the LS
        # preload until the controller actually hands off to position return.
        self.returning.update(i for i in self.zeroed if self.commands[i][2] > 0)
        stops = {i: self.stop_positions[i] for i in self.returning}
        for index in self.returning:
            velocity = self.commands.get(index, (0, 0, 0, 0, 0))[1]
            self.stop_positions[index] = -math.inf if velocity < 0 else math.inf
        try:
            super().spin(timeout)
        finally:
            self.stop_positions.update(stops)


def module_runner(index=None):
    motors, targets = commissioned(index=index)
    for target in targets:
        target.settings["zero_max_attempts"] = 3  # Explicit fake calibration baseline.
    log = TimedLog()
    drive = TestDrive(targets, log)
    log.clock = drive.clock
    drive.stop_after_complete = False
    runner = h.Runner(drive, motors, targets, log, clock=drive.clock)
    runner.module = "left_arm" if index is None else "left_leg"
    return runner, drive, log


@pytest.mark.parametrize("index", [None, 3])
def test_action_runs_sequential_sines_then_returns_every_motor_to_zero(index):
    runner, drive, log = module_runner(index)
    original_return = runner.return_after_test
    returned = []
    def return_only_after_test(run, support):
        assert log.events[-1][0] == "MOTION_TEST_RETURN"
        assert run.position_target == run.target.zero_target
        returned.append(run.target.motor.index)
        original_return(run, support)
    runner.return_after_test = return_only_after_test
    # Exercise a signed seek FF; test FF must follow velocity instead of its sign.
    for target in runner.targets:
        target.settings["seek_torque_ff_nm"] = target.settings["seek_direction"] * 0.02
        target.settings["motion_test_torque_ff_nm"] = 0.02
    runner.calibrate(write_zero=True, keep_running=False)
    runner.run_motion_test()
    starts = [d for e, d in log.events if e == "MOTION_TEST_START"]
    passes = [d for e, d in log.events if e == "MOTION_TEST_MOTOR_PASS"]
    returns = [d for e, d in log.events if e == "MOTION_TEST_RETURN"]
    assert not any(e == "MOTION_TEST_PREPARE" for e, _ in log.events)
    assert returned == [t.motor.index for t in runner.targets]
    assert all(d["preposition"] is False for d in starts)
    for previous, following in zip(passes, starts[1:]):
        assert previous["_test_time"] <= following["_test_time"]
    for start, returning in zip(starts, returns):
        assert returning["completed_cycles"] == start["cycles"] == 3
        assert returning["elapsed_s"] < start["timeout_s"]
    assert [d["index"] for d in starts] == [t.motor.index for t in runner.targets]
    assert len(passes) == len(starts) and runner.all_done and not runner.motion_test_running
    assert all(d["completed_cycles"] == 3 for e, d in log.events if e == "MOTION_TEST_RETURN")
    for target in runner.targets:
        assert [d["completed_cycles"] for e,d in log.events
                if e == "MOTION_TEST_CYCLE" and d["index"] == target.motor.index] == [1,2,3]
    assert all(d["ki_frozen"] for d in starts)
    for run, start, result in zip(runner.runs, starts, passes):
        i = run.target.motor.index
        assert abs(result["actual_rad"] - run.target.zero_target) <= run.p["arrival_tolerance_rad"]
        assert start["peak_speed_rad_s"] <= run.p["motion_test_speed_rad_s"] + 1e-12
        assert start["cycles"] == 3 and start["kp"] == 0 and start["kd"] == run.p["kd"]
        commands = [(t, cmd) for t, batch in drive.command_batches for m, cmd in batch
                    if m.index == i and cmd[2] == 0 and abs(cmd[1]) > 1e-7
                    and start["low_rad"] - 1e-9 <= cmd[0] <= start["high_rad"] + 1e-9]
        # Both directions, matching FF sign, and no speed above the seek cap.
        assert any(cmd[1] > 0 for _, cmd in commands) and any(cmd[1] < 0 for _, cmd in commands)
        assert all(abs(cmd[1]) <= run.p["motion_test_speed_rad_s"] + 1e-12 for _, cmd in commands)
        assert all(cmd[4] * cmd[1] >= 0 for _, cmd in commands)
        assert max(abs(cmd[4]) for _, cmd in commands) <= abs(run.p["seek_torque_ff_nm"]) + 1e-12
        stamps = [t for t, batch in drive.command_batches if any(m.index == i for m, _ in batch)]
        assert max(b-a for a,b in zip(stamps, stamps[1:])) <= 0.011
    if index == 3:
        assert starts[0]["single_sided"] and starts[0]["high_rad"] <= 1e-12
    # Only calibration sends zero/enable; tests reuse already enabled motors.
    assert len(drive.zero_calls) == 3 * len(runner.targets)
    ready = next(d["_test_time"] for e, d in log.events if e == "CALIBRATION_PASS")
    assert all(at <= ready for kind, _, at in drive.calls if kind in {"zero_request", "enable"})


def test_test_fault_uses_normal_exit_cleanup():
    runner, drive, log = module_runner(3)
    original_spin = drive.spin
    def fault(timeout):
        original_spin(timeout)
        if runner.current and runner.current.phase == "MOTION_TEST":
            drive.dropped.add(3)
    drive.spin = fault
    try:
        runner.calibrate(write_zero=True, keep_running=False)
        with pytest.raises(h.HomingError, match="FEEDBACK_TIMEOUT"):
            runner.run_motion_test()
        assert runner.motion_test_running
        assert not any(e == "MOTION_TEST_PASS" for e, _ in log.events)
    finally:
        runner.shutdown()
    assert not any(drive.enabled.values())


@pytest.mark.parametrize("speed", [0.02, 0.1, 0.3])
def test_plan_starts_and_ends_at_rest_and_limits_speed(speed):
    _, targets = commissioned(index=14)
    targets[0].settings["motion_test_speed_rad_s"] = speed
    p = h.motion_test_plan(targets[0])
    assert p["peak_speed_rad_s"] <= speed + 1e-12
    assert math.isclose(p["center_rad"] + p["cosine_rad"], p["start_rad"])
    assert abs(math.sin(p["omega_rad_s"] * p["nominal_period_s"])) < 1e-12
    assert p["cycles"] == 3
    assert p["actual_span_deg"] == pytest.approx(30)


@pytest.mark.parametrize("value", [0, -1, True, float("nan")])
def test_amplitude_config_rejects_invalid_values(value):
    _, targets = commissioned(index=14)
    targets[0].settings["motion_test_amplitude_deg"] = value
    with pytest.raises(ValueError, match="motion_test_amplitude_deg"):
        h.motion_test_plan(targets[0])


def test_optional_ff_and_test_torque_budget():
    _, targets = commissioned(index=14)
    p = targets[0].settings
    p.update(seek_torque_ff_nm=-1.95, motion_test_torque_ff_nm=1.95, max_torque_nm=1.95, motion_test_use_seek_feedforward=True)
    with pytest.raises(ValueError, match="torque budget"):
        h.motion_test_plan(targets[0])
    p["motion_test_use_seek_feedforward"] = False
    assert h.motion_test_plan(targets[0])["torque_ff_peak_nm"] == 0.0


@pytest.mark.parametrize("key,value", [("motion_test_amplitude_deg",0), ("motion_test_amplitude_deg",float("nan")),
    ("motion_test_position_margin_deg",-1), ("motion_test_use_seek_feedforward",1)])
def test_invalid_motion_test_profile_is_rejected(key, value):
    _, targets = commissioned(index=14)
    targets[0].settings[key] = value
    with pytest.raises(ValueError):
        h.motion_test_plan(targets[0])


def test_test_position_guard_and_original_ki_freeze_remain_active():
    _, targets = commissioned(index=3)
    run = h.MotorRun(targets[0],0)
    run.motion_test = h.motion_test_plan(targets[0])
    run.adaptive_ki_frozen = True
    run.transition("MOTION_TEST",0)
    with pytest.raises(h.HomingError,match="MOTION_TEST_POSITION_LIMIT"):
        run.step(0.01,h.Feedback(0.01,1,0.1,0,0,25))
    assert run.adaptive_ki_frozen


@pytest.mark.parametrize("bias,tolerance", [(0.020018344985289815, 2), (-0.01812958134527659, 2),
                                           (0.5632486343383789 - 0.43633231299858233, 8)])
def test_static_test_return_error_can_finish_with_frozen_ki(bias, tolerance):
    runner,drive,log=module_runner(14)
    target=runner.targets[0]
    target.settings.update(arrival_tolerance_rad=math.radians(10), adaptive_torque_enabled=True,
                           motion_test_arrival_tolerance_deg=tolerance)
    original=target.settings.copy()
    original_spin=drive.spin
    def biased_plant(timeout):
        original_spin(timeout)
        run=runner.current
        if run and run.phase in {"MOVE", "HOLD", "RECOVER"} and runner.all_done:
            if abs(run.return_reference - run.position_target) < 1e-9:
                q=run.position_target+bias
                # Reach the biased endpoint continuously; do not inject a sensor jump.
                previous=drive.feedback[14].q
                q=previous+max(-0.005,min(0.005,q-previous))
                drive.positions[14]=q
                drive.feedback[14]=h.Feedback(drive.clock(),drive.seq,q,0,0.1,25,drive.enabled[14])
    drive.spin=biased_plant
    runner.calibrate(write_zero=True, keep_running=False)
    runner.run_motion_test()
    start=next(d for e,d in log.events if e=="MOTION_TEST_START")
    assert start["arrival_tolerance_deg"] == pytest.approx(tolerance)
    assert start["ki_frozen"]
    assert any(e=="MOTION_TEST_PASS" for e,_ in log.events)
    assert runner.runs[0].adaptive_ki_frozen
    assert target.settings == original and runner.runs[0].p is target.settings
    assert not any(e=="BRAKE_START" for e,_ in log.events)


def test_motion_test_tolerance_respects_joint_accuracy_without_widening_position_guard():
    _,targets=commissioned(index=14)
    target=targets[0]
    target.settings.update(arrival_tolerance_rad=math.radians(10),motion_test_arrival_tolerance_deg=3,
                           motion_test_position_margin_deg=2)
    assert h.motion_test_plan(target)["arrival_tolerance_rad"] == pytest.approx(math.radians(3))
    assert h.motion_test_plan(target)["margin_rad"] == pytest.approx(math.radians(2))
    target.settings["arrival_tolerance_rad"]=math.radians(0.5)
    assert h.motion_test_plan(target)["arrival_tolerance_rad"] == pytest.approx(math.radians(0.5))
    target.settings["motion_test_arrival_tolerance_deg"]=0
    with pytest.raises(ValueError,match="motion_test_arrival_tolerance_deg"):
        h.motion_test_plan(target)


def test_overspeed_log_with_zero_test_ff_commands_deceleration_and_keeps_protection():
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(seek_speed_rad_s=0.3,kd=0.3,seek_torque_ff_nm=-0.5,
                           motion_test_torque_source="fixed",motion_test_torque_ff_nm=0.0,
                           motion_test_speed_rad_s=0.5)
    target.settings.pop("motion_test_speed_limit_rad_s", None)  # Exercise the unchanged default.
    target.settings.pop("motion_test_emergency_speed_rad_s", None)
    run=h.MotorRun(target,0)
    run.motion_test=h.motion_test_plan(target)
    run.transition("MOTION_TEST",0)
    q=0.3336000442504883
    f=h.Feedback(0.32,1,q,0.8774356842041016,0.4249076843261719,25)
    command=run.step(0.32,f)
    assert command[2]==0 and command[3]==0.3 and command[4]==0
    assert command[3]*(command[1]-f.dq)+command[4]<0
    assert target.settings["seek_torque_ff_nm"] == -0.5
    hot=h.MotorRun(target,0)
    hot.motion_test=h.motion_test_plan(target)
    hot.transition("MOTION_TEST",0)
    hot.step(0.32,h.Feedback(0.32,1,0.25,1.1,f.tau,25))
    with pytest.raises(h.HomingError,match="MOTION_TEST_OVERSPEED"):
        hot.step(0.38,h.Feedback(0.38,2,0.25+0.06*1.1,1.1,f.tau,25))


@pytest.mark.parametrize("value", [-0.1,True,None,float("nan")])
def test_motion_test_ff_rejects_invalid_peak_magnitude(value):
    _,targets=commissioned(index=20)
    targets[0].settings["motion_test_torque_ff_nm"]=value
    with pytest.raises(ValueError,match="motion_test_torque_ff_nm"):
        h.motion_test_plan(targets[0])


def test_motion_return_seek_assistance_reverses_with_goal_and_withdraws_at_speed_cap():
    _,targets=commissioned(index=19)
    target=targets[0]
    target.settings.update(kd=0.2,bck_kp=5,bck_kd=0.2,seek_torque_ff_nm=-0.5,
                           arrival_tolerance_rad=math.radians(2),return_speed_rad_s=0.5,
                           motion_test_torque_source="seek",adaptive_torque_enabled=True)
    run=h.MotorRun(target,0)
    run.motion_test=h.motion_test_plan(target)
    run.position_target=math.radians(15)
    run.transition("MOVE",0)
    run.motion_test_return_active=True
    run.adaptive_ki_frozen=True
    run.adaptive_torque=0.050187185774658874
    q=0.3011751174926758
    assist=[]
    for n in range(1,61):
        f=h.Feedback(n/100,n,q,0,0.1,25)
        command=run.position_command(run.position_target,0,5,0.2,f)
        command=run.motion_test_return_command(command,f)
        assist.append(run.motion_test_torque_nm)
        assert abs(5*(run.position_target-q)+command[4]) <= run.p["max_torque_nm"]
    assert assist[-1] < -0.1
    assert max(abs(b-a) for a,b in zip(assist,assist[1:])) <= 0.002501
    assert run.adaptive_torque == 0.050187185774658874 and run.adaptive_ki_frozen
    f=h.Feedback(0.61,61,run.position_target-0.05,0,0.1,25)
    run.motion_test_return_command((run.position_target,0,5,0.2,run.adaptive_torque),f)
    assert run.motion_test_torque_nm > 0  # Overshot: assistance must reverse.
    start_q = f.q
    for n in range(62,113):
        f=h.Feedback(n/100,n,start_q+0.05*(n-61)/100,0.05,0.1,25)
        run.motion_test_return_command((run.position_target,0,5,0.2,run.adaptive_torque),f)
    assert run.motion_test_torque_nm == 0  # Confirmed encoder motion withdraws assistance.
    f=h.Feedback(1.13,113,run.position_target+math.radians(1),0,0.1,25)
    run.motion_test_return_command((run.position_target,0,5,0.2,run.adaptive_torque),f)
    assert run.motion_test_torque_nm == 0


@pytest.mark.parametrize("source,enabled,peak", [("seek",True,0.5),("fixed",True,0.2),("seek",False,0.0)])
def test_sine_feedforward_has_constant_magnitude_and_only_changes_sign(source,enabled,peak):
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(seek_torque_ff_nm=-0.5,motion_test_torque_source=source,
                           motion_test_torque_ff_nm=0.2,motion_test_use_seek_feedforward=enabled,
                           motion_test_speed_rad_s=0.7)
    plan=h.motion_test_plan(target)
    for angle in [0,0.1,math.pi/2,math.pi,math.pi+0.1,1.5*math.pi,2*math.pi]:
        for feedback_velocity in [-0.6,0,0.6]:
            run=h.MotorRun(target,0)
            run.motion_test=plan
            run.transition("MOTION_TEST",0)
            now=angle/plan["omega_rad_s"]
            q=plan["center_rad"]+plan["cosine_rad"]*math.cos(angle)
            command=run.step(now,h.Feedback(now,1,q,feedback_velocity,0,25))
            assert abs(command[4]) == peak
            expected_direction=command[1] if abs(command[1])>1e-9 else -plan["cosine_rad"]*math.cos(angle)
            assert command[4]*expected_direction >= 0
            assert run.motion_test_torque_nm == command[4]
    # The fixed feedforward is never silently shrunk to fit a torque budget.
    target.settings.update(motion_test_use_seek_feedforward=True,motion_test_torque_source="seek",kd=0.3)
    run=h.MotorRun(target,0);run.motion_test=h.motion_test_plan(target);run.transition("MOTION_TEST",0)
    run.p={**run.p,"max_torque_nm":0.55}
    now=math.pi/2/run.motion_test["omega_rad_s"]
    with pytest.raises(h.HomingError,match="MOTION_TEST_COMMAND_TORQUE_BUDGET"):
        run.step(now,h.Feedback(now,1,run.motion_test["center_rad"],-0.6,0,25))


def test_thirty_degree_span_clips_asymmetric_neck_and_reversed_knee():
    for index, home, seek, lower, upper, expected in [
        (19, -20, -1, -20, 10, (5, 30)),
        (20, -17, -1, -17, 17, (2, 32)),
        (18, -90, -1, -90, 90, (75, 105)),
        (3, 0, 1, 0, 113, (-15, 0)),
        (3, 0, -1, 0, 113, (0, 15)),
    ]:
        _, targets = commissioned(index=index)
        target = targets[0]
        target.lower, target.upper = math.radians(lower), math.radians(upper)
        target.settings.update(home_angle_deg=home, seek_direction=seek,
                               motion_test_amplitude_deg=15, motion_test_speed_rad_s=0.5)
        plan = h.motion_test_plan(target)
        assert math.degrees(plan["low_rad"]) == pytest.approx(expected[0])
        assert math.degrees(plan["high_rad"]) == pytest.approx(expected[1])
        assert plan["actual_span_deg"] <= 30.000001
        assert plan["peak_speed_rad_s"] <= 0.5
        assert plan["joint_low_rad"] <= plan["low_rad"] < plan["high_rad"] <= plan["joint_high_rad"]


def test_stationary_position_with_noisy_velocity_builds_bounded_test_assistance():
    _, targets = commissioned(index=19)
    target = targets[0]
    target.settings.update(arrival_tolerance_rad=math.radians(2), seek_torque_ff_nm=-0.5,
                           motion_test_torque_source="seek", motion_test_torque_rate_nm_s=0.25,
                           return_speed_rad_s=0.5, arrival_settle_s=0.3)
    run = h.MotorRun(target, 0)
    run.motion_test = h.motion_test_plan(target)
    run.transition("MOVE", 0)
    run.motion_test_return_active = True
    run.adaptive_ki_frozen = True
    run.position_target = math.radians(15)
    noisy = [-0.03890037536621094, -0.021612167358398438, 0.004322052001953125,
             -0.012966156005859375, -0.030256271362304688]
    command = (run.position_target, 0, 5, 0.2, 0.002731692689924986)
    for n in range(1,201):
        f = h.Feedback(n/100,n,0.3007936477661133,noisy[n%len(noisy)],-0.6593,25)
        out = run.motion_test_return_command(command,f)
        assert abs(5*(out[0]-f.q)+0.2*(out[1]-f.dq)+out[4]) < run.p["max_torque_nm"]
    assert run.motion_test_velocity_rad_s == 0
    assert -0.5 <= run.motion_test_torque_nm < -0.35
    previous = run.motion_test_torque_nm
    run.motion_test_return_command(command,f)
    assert run.motion_test_torque_nm == previous  # Duplicate feedback cannot grow torque.
    run.motion_test_return_command(command,h.Feedback(2.01,201,f.q,-0.5,0,25))
    assert run.motion_test_torque_nm == 0  # Immediate high-speed withdrawal remains.
    run.transition("BRAKE",2.02)
    assert not run.motion_test_velocity_window and run.motion_test_torque_nm == 0


def test_sine_speed_override_allows_logged_peak_but_keeps_confirmation_and_emergency():
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(motion_test_speed_rad_s=0.7,motion_test_speed_limit_rad_s=1.6,
                           motion_test_emergency_speed_rad_s=2.5,
                           brake_speed_ratio=1.5,brake_speed_margin_rad_s=0.2,
                           brake_emergency_speed_ratio=2.0,brake_overspeed_confirm_s=0.05)
    plan=h.motion_test_plan(target)
    assert plan["speed_limit_rad_s"]==1.6
    assert plan["emergency_speed_rad_s"]==pytest.approx(2.5)
    run=h.MotorRun(target,0);run.motion_test=plan;run.transition("MOTION_TEST",0)
    for n in range(1,11):
        run.step(n/100,h.Feedback(n/100,n,plan["center_rad"],1.4739189147949219,0.4,25))
    run.step(.11,h.Feedback(.11,11,plan["center_rad"],1.61,0.4,25))
    with pytest.raises(h.HomingError,match="MOTION_TEST_OVERSPEED") as caught:
        run.step(.17,h.Feedback(.17,12,plan["center_rad"],1.61,0.4,25))
    assert caught.value.diagnostics["speed_limit_rad_s"]==1.6
    hot=h.MotorRun(target,0);hot.motion_test=plan;hot.transition("MOTION_TEST",0)
    with pytest.raises(h.HomingError,match="MOTION_TEST_OVERSPEED"):
        hot.step(.01,h.Feedback(.01,1,plan["center_rad"],2.51,0.4,25))
    # This override never changes the existing MOVE/return braking calculation.
    move=h.MotorRun(target,0);move.transition("MOVE",0);move.move_start=target.zero_target
    move.return_reference=move.position_target;move.return_reference_at=0
    emergency=(move.p["return_speed_rad_s"]*1.5+0.2)*2
    move.step(.01,h.Feedback(.01,1,move.position_target,emergency+.1,0.4,25))
    assert move.phase=="BRAKE"
    for bad in [True,0.0,0.6,2.6,float("nan"),float("inf")]:
        target.settings["motion_test_speed_limit_rad_s"]=bad
        with pytest.raises(ValueError,match="motion_test_speed_limit_rad_s"):
            h.motion_test_plan(target)


def test_common_sine_thresholds_apply_to_all_joints_independent_of_return_speed():
    for index in range(21):
        _,targets=commissioned(index=index)
        target=targets[0]
        target.settings.update(motion_test_speed_limit_rad_s=1.6,motion_test_emergency_speed_rad_s=2.5)
        for speed in [0.3,0.7]:
            target.settings["motion_test_speed_rad_s"]=speed
            plan=h.motion_test_plan(target)
            assert plan["speed_limit_rad_s"]==1.6 and plan["emergency_speed_rad_s"]==2.5
    for bad in [True,0.0,-1,float("nan"),float("inf"),1.5]:
        target.settings["motion_test_emergency_speed_rad_s"]=bad
        with pytest.raises(ValueError,match="motion_test_(emergency_speed|speed_limit)_rad_s"):
            h.motion_test_plan(target)


@pytest.mark.parametrize("side", [-1,1])
def test_actual_test_edge_reverses_fixed_torque_and_latches_until_retreat(side):
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(seek_torque_ff_nm=-0.5,motion_test_torque_source="seek",
                           motion_test_speed_rad_s=0.7,motion_test_speed_limit_rad_s=2.4,
                           motion_test_emergency_speed_rad_s=3.0,motion_test_reverse_margin_deg=2)
    plan=h.motion_test_plan(target)
    run=h.MotorRun(target,0);run.motion_test=plan;run.transition("MOTION_TEST",0)
    angle=math.pi/2 if side*plan["cosine_rad"]<0 else 3*math.pi/2
    now=angle/plan["omega_rad_s"]
    boundary=plan["reverse_high_rad"] if side>0 else plan["reverse_low_rad"]
    # Real position is at the end well before the time-based sine turns.
    command=run.step(now,h.Feedback(now,1,boundary+side*.001,side*1.65,0.5,25))
    assert side*command[1]<0 and command[4]==-side*0.5
    assert abs(command[1])<=0.7 and run.motion_test_reverse_count==1
    for n in range(2,12):
        t=now+n*.002
        f=h.Feedback(t,n,boundary+side*(.001 if n%2 else -.001),side*.1,0.5,25)
        command=run.step(t,f)
        assert side*command[1]<0 and command[4]==-side*0.5
        assert run.motion_test_reverse_count==1
    t=now+.03
    command=run.step(t,h.Feedback(t,12,boundary-side*.006,0,0.5,25))
    assert run.motion_test_reverse_side==0 and side*command[1]<0
    assert command[4]==-side*0.5 and run.phase_at==0  # Fault timeout was not restarted.


def test_edge_reversal_does_not_hide_hard_faults_and_margin_is_validated():
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(motion_test_speed_limit_rad_s=2.4,motion_test_emergency_speed_rad_s=3.0)
    plan=h.motion_test_plan(target)
    for q,dq,reason in [(plan["joint_high_rad"]+.01,0,"MOTION_TEST_POSITION_LIMIT"),
                         (plan["reverse_high_rad"],3.1,"MOTION_TEST_OVERSPEED")]:
        run=h.MotorRun(target,0);run.motion_test=plan;run.transition("MOTION_TEST",0)
        with pytest.raises(h.HomingError,match=reason):
            run.step(.01,h.Feedback(.01,1,q,dq,.1,25))
        assert run.motion_test_reverse_count==0
    for bad in [-1,True,float("nan"),float("inf")]:
        target.settings["motion_test_reverse_margin_deg"]=bad
        with pytest.raises(ValueError,match="motion_test_reverse_margin_deg"):
            h.motion_test_plan(target)
    target.settings.update(motion_test_reverse_margin_deg=100,motion_test_amplitude_deg=1)
    narrow=h.motion_test_plan(target)
    assert narrow["reverse_margin_deg"]<=narrow["actual_span_deg"]/4+1e-9


def test_neck_roll_recorded_motion_reverses_before_the_seventeen_degree_stop():
    _,targets=commissioned(index=20)
    target=targets[0]
    target.settings.update(home_angle_deg=-17,seek_direction=-1,seek_torque_ff_nm=-0.5,
                           motion_test_torque_source="seek",motion_test_speed_rad_s=0.7,
                           motion_test_amplitude_deg=15,motion_test_reverse_margin_deg=2,
                           motion_test_speed_limit_rad_s=2.4,motion_test_emergency_speed_rad_s=3.0)
    run=h.MotorRun(target,0);plan=h.motion_test_plan(target);run.motion_test=plan;run.transition("MOTION_TEST",0)
    now=math.acos((0.11745702881449069-plan["center_rad"])/plan["cosine_rad"])/plan["omega_rad_s"]
    outward=run.step(now,h.Feedback(now,9289,0.522430419921875,1.6295242309570312,.5128,25))
    assert outward[1]>0 and outward[4]==.5
    now+=20.97454917699997-20.972182592999957
    inward=run.step(now,h.Feedback(now,9290,0.5262451171875,1.6208782196044922,.5128,25))
    assert inward[1]<0 and inward[4]==-.5
    assert math.degrees(run.last.q-target.zero_target)<15
    assert run.motion_test_reverse_count==1




def test_stalled_motion_test_times_out_without_counting_cycles_or_starting_next_motor():
    runner, drive, log = module_runner()
    for target in runner.targets:
        target.settings["motion_test_timeout_s"] = 0.2
    original_spin = drive.spin
    frozen_q = None
    def stalled(timeout):
        nonlocal frozen_q
        original_spin(timeout)
        run = runner.current
        if run and run.phase == "MOTION_TEST":
            index = run.target.motor.index
            if frozen_q is None:
                frozen_q = drive.positions[index]
            drive.positions[index] = frozen_q
            drive.feedback[index] = h.Feedback(drive.clock(), drive.seq, frozen_q, 0, 0.1, 25, drive.enabled[index])
    drive.spin = stalled
    try:
        runner.calibrate(write_zero=True, keep_running=False)
        with pytest.raises(h.HomingError, match="MOTION_TEST_TIMEOUT"):
            runner.run_motion_test()
        assert len([1 for e,_ in log.events if e == "MOTION_TEST_START"]) == 1
        assert not any(e in {"MOTION_TEST_RETURN", "MOTION_TEST_CYCLE", "MOTION_TEST_PASS"} for e,_ in log.events)
    finally:
        runner.shutdown()
    assert not any(drive.enabled.values())


def test_cycle_count_and_fault_timeout_configuration():
    _, targets = commissioned(index=14)
    target = targets[0]
    target.settings["motion_test_cycles"] = 2
    target.settings["motion_test_timeout_s"] = 40
    assert h.motion_test_plan(target)["cycles"] == 2
    assert h.motion_test_plan(target)["timeout_s"] == 40
    for bad in [True, 0, -1, 3.0, None]:
        target.settings["motion_test_cycles"] = bad
        with pytest.raises(ValueError, match="motion_test_cycles"):
            h.motion_test_plan(target)
    target.settings["motion_test_cycles"] = 3
    for bad in [True, 0, -1, None, float("inf"), float("nan")]:
        target.settings["motion_test_timeout_s"] = bad
        with pytest.raises(ValueError, match="motion_test_timeout_s"):
            h.motion_test_plan(target)



@pytest.mark.parametrize("first_direction", [-1, 1])
def test_cycles_count_command_direction_changes_without_reaching_boundaries(first_direction):
    _, targets = commissioned(index=20)
    run = h.MotorRun(targets[0], 0)
    plan = h.motion_test_plan(targets[0])
    run.motion_test = plan
    run.transition("MOTION_TEST", 0)
    run.motion_test_phase_offset = 0 if first_direction > 0 else math.pi
    period = plan["nominal_period_s"]
    for seq in range(1, math.ceil(3 * period * 100) + 1):
        now = seq / 100
        f = h.Feedback(now, seq, plan["center_rad"], 0, 0.1, 25)
        run.step(now, f)
        count = run.motion_test_half_cycles
        run.step(now, f)
        assert run.motion_test_half_cycles == count
    assert run.motion_test_half_cycles == 6
    assert run.motion_test_reverse_count == 0
    run.transition("MOTION_TEST", now)
    assert run.motion_test_half_cycles == 0 and run.motion_test_last_direction == 0


def test_repeated_lower_edge_counts_clock_and_boundary_reversals_together():
    _, targets = commissioned(index=1)
    run = h.MotorRun(targets[0], 0)
    plan = h.motion_test_plan(targets[0])
    run.motion_test = plan
    run.transition("MOTION_TEST", 0)
    edge = plan["reverse_low_rad"]
    # Reproduce repeated lower-edge visits; never reach the upper boundary.
    q = edge + 0.01
    for seq in range(1, 2001):
        now = seq / 100
        q = edge - 0.001 if run.motion_test_last_direction < 0 else edge + 0.01
        previous = run.motion_test_half_cycles
        run.step(now, h.Feedback(now, seq, q, 0, 0.1, 25))
        assert run.motion_test_half_cycles - previous <= 1
        if run.motion_test_half_cycles >= 6:
            break
    assert run.motion_test_half_cycles == 6
    assert run.motion_test_reversal["side"] == "lower"
    assert now < plan["timeout_s"]

