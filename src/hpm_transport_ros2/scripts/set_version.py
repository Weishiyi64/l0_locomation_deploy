#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Set the ROS 2 release version or check that all source metadata matches it."""

import argparse
from pathlib import Path
import re
from xml.etree import ElementTree


WORKSPACE = Path(__file__).resolve().parents[1]
CMAKE_PACKAGES = (
    "hpm_battery", "hpm_display", "hpm_imu", "hpm_motor", "hpm_system",
    "hpm_tactile", "hpm_transport", "hpm_ultrasonic",
)
VERSION_PATTERN = re.compile(r"((?:0|[1-9][0-9]*)(?:\.(?:0|[1-9][0-9]*)){2})(?:-(?:dev|rc)\.[1-9][0-9]*)?")


def version_updates(version):
    match = VERSION_PATTERN.fullmatch(version)
    if not match:
        raise ValueError("Expected MAJOR.MINOR.PATCH, MAJOR.MINOR.PATCH-dev.N or MAJOR.MINOR.PATCH-rc.N (N starts at 1)")
    base_version = match[1]
    python_package = WORKSPACE / "scripts/set_zero"
    manifests = [WORKSPACE / "src" / name / "package.xml" for name in CMAKE_PACKAGES]
    manifests.append(python_package / "package.xml")
    updates = {}
    for manifest in manifests:
        source = manifest.read_text(encoding="utf-8")
        if ElementTree.fromstring(source).findtext("version") == base_version:
            continue
        updated, count = re.subn(r"(<version>)[^<]+(</version>)", rf"\g<1>{base_version}\g<2>", source)
        if count != 1 or ElementTree.fromstring(updated).findtext("version") != base_version:
            raise ValueError(f"Expected a single package version in {manifest}")
        updates[manifest] = updated

    template = (WORKSPACE / "cmake/version.py.in").read_text(encoding="utf-8")
    python_marker = template.replace("@PROJECT_NAME@", "set_zero").replace("@APP_VERSION@", version)
    for path, contents in ((python_package / "set_zero/version.py", python_marker),
                           (WORKSPACE / "VERSION", version + "\n")):
        if path.read_text(encoding="utf-8") != contents:
            updates[path] = contents
    return updates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("version", nargs="?", metavar="VERSION")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()

    try:
        version = (WORKSPACE / "VERSION").read_text(encoding="utf-8").strip() if args.check else args.version
        updates = version_updates(version)
        if args.check and updates:
            paths = ", ".join(str(path.relative_to(WORKSPACE)) for path in updates)
            parser.error(f"Files out of sync with VERSION: {paths}")
        for path, contents in updates.items():
            path.write_text(contents, encoding="utf-8")
    except (OSError, ValueError, ElementTree.ParseError) as exc:
        parser.error(str(exc))
    print(version)


if __name__ == "__main__":
    main()
