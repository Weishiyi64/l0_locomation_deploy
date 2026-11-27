# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
from glob import glob
from pathlib import Path
from xml.etree import ElementTree

from setuptools import setup

setup(
    name="set_zero",
    version=ElementTree.parse(Path(__file__).with_name("package.xml")).findtext("version"),
    packages=["set_zero"],
    # Remote clocks can lag copied build/install files. Refresh this small package
    # at both stages so ordinary colcon builds cannot silently keep stale files.
    options={"build_py": {"force": True}, "install_lib": {"force": True},
             "install_data": {"force": True}},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/set_zero"]),
        ("share/set_zero", ["package.xml", "README.md"]),
        ("share/set_zero/launch", glob("launch/*.launch.py")),
        ("share/set_zero/config", glob("config/*")),
        ("lib/set_zero", ["set_zero/version.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="HPM project maintainers",
    maintainer_email="maintainers@example.com",
    description="Sequential fixture-based motor homing",
    license="BSD-3-Clause",
    entry_points={"console_scripts": ["motor_auto_zero = set_zero.motor_auto_zero:main",
                                      "set_zero_server = set_zero.action_server:main"]},
)
