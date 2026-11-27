"""Check two real reporting windows against a loopback HPM, without hardware."""

import os
import re
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from luvbot_hardware_interfaces.msg import MotorCommand, MotorTarget
from test_motor_can_log import launching, until, wire


def check(folder):
    config = yaml.safe_load((Path(get_package_share_directory("hpm_motor")) /
                             "config/motors.yaml").read_text())
    config["motors"] = {name: dict(index=index, motor_type="LS", bus_index=index, motor_id=5)
                        for index, name in enumerate(("left", "right", "idle"))}
    (folder / "motors.yaml").write_text(yaml.safe_dump(config))
    node = rclpy.create_node("motor_rates_offline_test")
    pub = node.create_publisher(MotorCommand, "/actuator/motor/command", 10)
    try:
        with launching(folder, os.environ.copy(), "launch_display:=false",
                       "enable_tactile_uart:=false", "motor_log_enabled:=false") as peer:
            until(lambda: pub.get_subscription_count() == 1)
            for indices in ((0, 1, 98), (1,)):
                message = MotorCommand(control_mode=MotorCommand.CONTROL_MODE_MIT)
                for index in indices:
                    target = MotorTarget()
                    target.route.motor_type = index
                    message.commands.append(target)
                pub.publish(message)
            assert [peer.recv(30)[13] for _ in range(3)] == [0, 1, 1]
            state = bytes.fromhex("20 00 00 00 00 00 00 00")
            left, right = (wire(bus, 5, state) for bus in (0, 1))
            peer.snapshot({0: left, 1: right})
            peer.snapshot({0: left})
            peer.send_frame(wire(1, 5, bytes.fromhex("A7 24 0E 60")))

            deadline = time.monotonic() + 26
            while time.monotonic() < deadline:
                log = (folder / "launch_output").read_text()
                windows = log.split("Motor rates (window=")[1:]
                if len(windows) >= 2 and re.search(r"RX= *0\.00 Hz\n[^\n]*-{6,}", windows[1]):
                    break
                time.sleep(0.1)
            else:
                raise AssertionError(log)
            for window, counts in zip(windows, (((1, 2), (2, 1), (0, 0)), ((0, 0),) * 3)):
                seconds = float(window.split(" s):")[0])
                assert 9.5 <= seconds <= 15, window
                lines = [line for line in window.splitlines() if "  index=" in line]
                assert len({line.index("TX=") for line in lines}) == 1, window
                assert len({line.index("RX=") for line in lines}) == 1, window
                rows = re.findall(r"index=(\d+) +(\w+) +TX= *([\d.]+) Hz  RX= *([\d.]+) Hz", window)
                assert len(rows) == 3, window
                for index, name, tx, rx in rows:
                    assert name == ("left", "right", "idle")[int(index)], window
                    for rate, count in zip((tx, rx), counts[int(index)]):
                        assert abs(float(rate) - count / seconds) < 0.006, window
            borders = re.findall(r"(-{6,})$", log, re.MULTILINE)
            assert len(borders) == 4 and len(set(borders)) == 1, log
    finally:
        node.destroy_node()


if __name__ == "__main__":
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="179")
    rclpy.init(args=[])
    try:
        with tempfile.TemporaryDirectory() as directory:
            check(Path(directory))
    finally:
        rclpy.shutdown()
