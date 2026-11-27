# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Offline install regression: future-dated outputs must not hide source edits."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def test_install_refreshes_older_source_files(tmp_path):
    source = tmp_path / "source"
    shutil.copytree(Path(__file__).resolve().parents[1], source,
                    ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache", "*.egg-info", "build"))
    build = tmp_path / "build"
    install = tmp_path / "install"
    command = [sys.executable, "setup.py", "build_py", "--build-lib", str(build),
               "install_lib", "--build-dir", str(build), "--install-dir", str(install / "lib"),
               "--skip-build", "--no-compile", "install_data", "--install-dir", str(install)]
    def run():
        result = subprocess.run(command, cwd=source, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
    run()
    pairs = [
        ("set_zero/motor_auto_zero.py", "lib/set_zero/motor_auto_zero.py"),
        ("config/motor_zero_calibration.yaml", "share/set_zero/config/motor_zero_calibration.yaml"),
        ("launch/motor_auto_zero.launch.py", "share/set_zero/launch/motor_auto_zero.launch.py"),
    ]
    now = time.time()
    for directory in (build, install):
        for path in directory.rglob("*"):
            if path.is_file():
                os.utime(path, (now + 86400, now + 86400))
    for relative, _ in pairs:
        path = source / relative
        path.write_bytes(path.read_bytes() + b"\n# offline install refresh regression\n")
        os.utime(path, (now - 86400, now - 86400))
    run()
    for relative, destination in pairs:
        assert (install / destination).read_bytes() == (source / relative).read_bytes(), relative
    assert (build / pairs[0][0]).read_bytes() == (source / pairs[0][0]).read_bytes()
