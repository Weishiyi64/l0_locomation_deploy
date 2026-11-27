# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
import math

import pytest
import yaml

from test_motor_auto_zero import PACKAGE, h


@pytest.mark.parametrize("deg,rad", h.DEGREE_TOLERANCES.items())
def test_degree_tolerance_precedence_and_validation(tmp_path, deg, rad):
    document = h.read_yaml(PACKAGE / "config/motor_zero_calibration.yaml")
    profile = document["joints"]["left_shoulder_roll_joint"]
    path = tmp_path / "degrees.yaml"
    def load(overrides=None):
        path.write_text(yaml.safe_dump(document))
        return h.load_plan(h.motor_config_path(), path, PACKAGE / "config/l0_v5.urdf",
                           motor_index=13, overrides=overrides)[1][0].settings[rad]
    default = 0.1 if deg == "motion_settle_position_deg" else 10
    document["defaults"].pop(rad, None)
    document["defaults"][deg] = default
    profile.pop(deg, None)
    profile.pop(rad, None)
    assert load() == pytest.approx(math.radians(default))
    profile[deg] = 5
    assert load() == pytest.approx(math.radians(5))
    assert load({deg: 2}) == pytest.approx(math.radians(2))
    assert load({rad: 0.03}) == pytest.approx(0.03)
    profile.pop(deg)
    profile[rad] = 0.04
    assert load() == pytest.approx(0.04)  # Legacy joint override beats degree defaults.
    profile.pop(rad)
    document["defaults"].pop(deg)
    document["defaults"][rad] = 0.05
    assert load() == pytest.approx(0.05)
    profile[deg] = 3
    assert load() == pytest.approx(math.radians(3))
    for invalid in (None, True, "10", 0, -1, math.nan, math.inf):
        profile[deg] = invalid
        with pytest.raises(ValueError, match=deg):
            load()
    profile[deg], profile[rad] = 10, math.radians(10)
    with pytest.raises(ValueError, match="only one"):
        load()
    profile.pop(rad)
    with pytest.raises(ValueError, match="only one"):
        load({deg: 10, rad: 0.1})
    args = h.parse_args(["--motor-index", "13", "--" + deg.replace("_", "-"), "7", "--dry-run"])
    assert load({deg: getattr(args, deg)}) == pytest.approx(math.radians(7))
