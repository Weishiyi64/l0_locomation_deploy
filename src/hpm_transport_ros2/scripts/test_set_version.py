# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

"""Run with python3 scripts/test_set_version.py from ros2_ws."""

from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

from set_version import CMAKE_PACKAGES, WORKSPACE


def main():
    manifests = [Path("src") / name / "package.xml" for name in CMAKE_PACKAGES]
    manifests.append(Path("scripts/set_zero/package.xml"))
    marker = Path("scripts/set_zero/set_zero/version.py")
    metadata = [*manifests, marker, Path("VERSION")]
    with TemporaryDirectory() as temp:
        root = Path(temp)
        for path in [*metadata, Path("cmake/version.py.in"), Path("scripts/set_version.py")]:
            (root / path).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(WORKSPACE / path, root / path)

        command = [sys.executable, str(root / "scripts/set_version.py")]

        def run(argument, expected_status=0):
            result = subprocess.run([*command, argument], capture_output=True, text=True)
            assert result.returncode == expected_status, result.stdout + result.stderr

        run("--check")
        for version in ("1.2.3-dev.2", "1.2.3-rc.1", "1.2.3"):
            run(version)
            run("--check")
            for path in manifests:
                assert ElementTree.parse(root / path).findtext("version") == "1.2.3", path

        before = [(root / path).read_bytes() for path in metadata]
        for invalid_version in ("1.2.3-dev.0", "1.2.3-rc.01", "v1.2.3"):
            run(invalid_version, 2)
        assert before == [(root / path).read_bytes() for path in metadata]

        (root / marker).write_text('APP_VERSION = "1.2.3-dev.2"\n', encoding="utf-8")
        run("--check", 2)
        run("1.2.3")
        run("--check")


if __name__ == "__main__":
    main()
