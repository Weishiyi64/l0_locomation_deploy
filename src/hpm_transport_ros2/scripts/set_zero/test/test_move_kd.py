"""Offline checks for independently tuning return damping; no motor I/O."""
import math

import pytest
import yaml

from test_motor_auto_zero import PACKAGE, commissioned, h


@pytest.mark.parametrize("override", [0.0, 0.2, 0.3])
def test_bck_kd_changes_move_and_recover_but_preserves_seek_and_hold(override):
    _, targets = commissioned(index=14)
    target = targets[0]
    p = target.settings
    p.update(kd=0.3, bck_kp=2.0, hold_kp=1.0, adaptive_torque_enabled=False)
    p["bck_kd"] = override
    expected = override
    run = h.MotorRun(target, 0.0)
    f = h.Feedback(0.001, 1, 0.0, 0.0, 0.0, 25.0)
    assert run.step(0.001, f)[2:4] == (0.0, 0.3)
    run.contact_q = f.q
    for phase in ("STOP", "ZERO_REQUEST", "ZERO_PENDING"):
        run.phase = phase
        assert run.step(0.002, f)[2:4] == (0.0, 0.3)
    run.begin_return(0.01, f)
    assert run.step(0.011, h.Feedback(0.011, 2, 0.0, 0.0, 0.0, 25.0))[2:4] == (2.0, expected)
    run.phase, run.last = "RECOVER", None
    assert run.step(0.02, h.Feedback(0.02, 3, target.zero_target + 0.2, 0.0, 0.0, 25.0))[2:4] == (2.0, expected)
    run.phase, run.last = "HOLD", None
    assert run.step(0.03, h.Feedback(0.03, 4, target.zero_target, 0.0, 0.0, 25.0))[2:4] == (1.0, p["hold_kd"])


def test_bck_kd_is_used_by_endpoint_and_hold_compensation_handoffs():
    _, targets = commissioned(index=14)
    target = targets[0]
    p = target.settings
    p.update(kd=0.3, bck_kd=0.2, return_speed_rad_s=0.3, adaptive_torque_enabled=True,
             adaptive_ki=1.0, adaptive_torque_limit_nm=1.0, adaptive_torque_rate_nm_s=0.25,
             arrival_tolerance_rad=0.01, hold_error_rad=0.05)
    run = h.MotorRun(target, 0.0)
    q = target.zero_target + 0.021
    run.begin_return(0.0, h.Feedback(0.0, 1, q, 0.02, 0.0, 25.0))
    f = h.Feedback(0.1, 2, q, 0.02, 0.0, 25.0)
    command = run.step(0.1, f)
    assert command[1] == 0.0
    assert command[3] == 0.2
    assert run.endpoint_handoff_nm == pytest.approx(-0.2 * 0.3)
    before = command[2] * (command[0] - f.q) - command[3] * f.dq + command[4]
    run.transition("HOLD", 0.11)
    assert run.phase == "HOLD"
    after = p["hold_kp"] * (target.zero_target - f.q) - p["hold_kd"] * f.dq + run.adaptive_torque
    assert after == pytest.approx(before)


@pytest.mark.parametrize("value", [None, True, "0.2", -0.1, math.nan, math.inf, 5.1])
def test_invalid_bck_kd_cannot_pass_motion_preflight(value):
    _, targets = commissioned(index=14)
    target = targets[0]
    target.settings["bck_kd"] = value
    assert any("bck_kd" in error or "gains exceed" in error for error in h.motion_errors(target))


def test_bck_kd_checks_combined_overspeed_gain_and_stays_per_joint(tmp_path):
    _, targets = commissioned(index=14)
    target = targets[0]
    target.settings.update(kd=4.9, bck_kd=0.2, return_overspeed_kd=0.2)
    assert not any("gains exceed" in error for error in h.motion_errors(target))
    target.settings.update(bck_kd=4.9)
    assert "gains exceed protocol encoding range" in h.motion_errors(target)
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    document["defaults"]["bck_kd"] = 0.2
    path = tmp_path / "shared.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="per joint"):
        h.load_plan(h.motor_config_path(), path, h.data_path("l0_v5.urdf"), motor_index=14)


@pytest.mark.parametrize("key,value", [
    ("bck_kp", 0), ("bck_kp", -1), ("bck_kp", True), ("bck_kp", math.nan),
    ("bck_kp", None), ("bck_kp", 999999), ("kp", 1), ("move_kd", 0.3),
])
def test_invalid_or_obsolete_return_gains_are_rejected(key, value):
    _, targets = commissioned(index=14)
    targets[0].settings[key] = value
    assert h.motion_errors(targets[0])


@pytest.mark.parametrize("key", ["bck_kp", "bck_kd"])
def test_return_gains_are_required_and_cannot_be_shared(key, tmp_path):
    _, targets = commissioned(index=14)
    del targets[0].settings[key]
    assert any(key in error for error in h.motion_errors(targets[0]))
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    document["defaults"][key] = 0.2
    path = tmp_path / "shared.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="per joint"):
        h.load_plan(h.motor_config_path(), path, h.data_path("l0_v5.urdf"), motor_index=14)
