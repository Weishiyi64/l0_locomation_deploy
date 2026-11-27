# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Each joint has its own torque guard; defaults remain a fallback."""
import pytest

from test_motor_auto_zero import PACKAGE, h


def test_all_joint_torque_limits_are_explicit_and_within_their_protocol_range():
    path = PACKAGE / 'config/motor_zero_calibration.yaml'
    config = h.read_yaml(path)
    assert config['defaults']['max_torque_nm'] == 3.0
    for module in config['modules']:
        _, targets = h.load_plan(h.motor_config_path(), path, PACKAGE / 'config/l0_v5.urdf', module=module)
        for target in targets:
            configured = config['joints'][target.motor.name]['max_torque_nm']
            assert target.settings['max_torque_nm'] == configured
            assert 0 < configured <= min(-target.limits['t_min'], target.limits['t_max'])


def test_joint_limit_overrides_common_value_and_missing_joint_limit_falls_back(tmp_path):
    config = h.read_yaml(PACKAGE / 'config/motor_zero_calibration.yaml')
    config['defaults']['max_torque_nm'] = 0.8
    config['joints']['right_knee_joint']['max_torque_nm'] = 12.0
    path = tmp_path / 'torque.yaml'
    def resolved(overrides=None):
        path.write_text(h.yaml.safe_dump(config))
        return h.load_plan(h.motor_config_path(), path, PACKAGE / 'config/l0_v5.urdf',
                           motor_index=9, overrides=overrides)[1][0]
    assert resolved().settings['max_torque_nm'] == 12.0
    assert resolved({'max_torque_nm': 2.0}).settings['max_torque_nm'] == 2.0
    del config['joints']['right_knee_joint']['max_torque_nm']
    assert resolved().settings['max_torque_nm'] == pytest.approx(0.8)


def test_knee_guard_and_hold_cap_use_joint_limit_instead_of_common_three_nm():
    _, targets = h.load_plan(h.motor_config_path(), PACKAGE / 'config/motor_zero_calibration.yaml',
                             PACKAGE / 'config/l0_v5.urdf', motor_index=9)
    target = targets[0]
    target.settings['max_torque_nm'] = 36.0  # Test fixture, independent of field tuning.
    run = h.MotorRun(target, 0)
    limit = target.settings['max_torque_nm']
    assert limit == 36.0
    run.step(0.01, h.Feedback(0.01, 1, 0, 0, 4, 25))  # Above common 3; below this joint's cap.
    f = h.Feedback(0.02, 2, target.zero_target - 0.2, 0, 0, 25)
    q, dq, kp, kd, ff = run.limit_hold_torque((target.zero_target, 0, 300, 0, 0), f)
    assert kp * (q - f.q) + kd * (dq - f.dq) + ff == pytest.approx(limit)
    with pytest.raises(h.HomingError, match='TORQUE_LIMIT'):
        run.step(0.03, h.Feedback(0.03, 3, 0, 0, limit + 0.1, 25))
