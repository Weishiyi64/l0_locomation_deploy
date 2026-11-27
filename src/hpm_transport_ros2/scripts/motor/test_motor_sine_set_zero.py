#!/usr/bin/env python3
"""ROS regression check without hardware; source install/setup.bash before running."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "164")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorState, MotorStateArray
from luvbot_hardware_interfaces.srv import MotorSetZero


def main() -> None:
    rclpy.init()
    node = Node("motor_sine_set_zero_test")
    events = []
    scenario = {"result": 0, "delay": 0.0, "confirmed": False}

    def set_zero(request, response):
        assert list(request.motor_types) == [0]
        events.append("zero_request")
        time.sleep(scenario["delay"])
        response.motor_types = request.motor_types
        response.results = [scenario["result"]]
        response.messages = ["simulated result"]
        scenario["confirmed"] = scenario["result"] == 0
        events.append("zero_response")
        return response

    def enable(message):
        events.append("enable" if message.enabled[0] else "disable")

    def command(message):
        target = message.commands[0]
        events.append("motion" if target.kp > 0 else "probe")
        if target.kp > 0:
            assert scenario["confirmed"], "motion before zero confirmation"
            assert abs(target.q) <= 0.101, "motion used position from before zero"

    publisher = node.create_publisher(MotorStateArray, "/actuator/motor/state", qos_profile_sensor_data)

    def feedback():
        state = MotorState()
        state.route.motor_type = 0
        state.route.bus_index = 0
        state.route.motor_id = 1
        state.valid = True
        state.q = 0.0 if scenario["confirmed"] else 1.0
        message = MotorStateArray()
        message.states = [state]
        publisher.publish(message)

    node.create_service(MotorSetZero, "/actuator/motor/set_zero", set_zero,
                        callback_group=ReentrantCallbackGroup())
    node.create_subscription(MotorEnableCommand, "/actuator/motor/enable", enable, 10)
    node.create_subscription(MotorCommand, "/actuator/motor/command", command, 10)
    node.create_timer(0.01, feedback)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "motors.yaml"
            config.write_text(
                "motors:\n  test_motor:\n    motor_type: MR\n    model: PA35-10-8\n    index: 0\n"
                "    bus_index: 0\n    motor_id: 1\n", encoding="utf-8")
            script = Path(__file__).with_name("motor_sine_test.py")
            for result, delay, timeout in [(0, 0.1, 2.0), (1, 0.0, 2.0), (4, 0.0, 2.0),
                                           (0, 0.2, 0.05)]:
                events.clear()
                scenario.update(result=result, delay=delay, confirmed=False)
                completed = subprocess.run(
                    [sys.executable, str(script), "--config", str(config), "--frequency", "10",
                     "--set-zero-timeout", str(timeout)], capture_output=True, text=True, timeout=15)
                time.sleep(0.3)
                success = result == 0 and delay < timeout
                assert completed.returncode == (0 if success else 1), completed.stdout + completed.stderr
                assert "zero_request" in events, events
                if success:
                    assert events.index("zero_response") < events.index("enable") < events.index("motion"), events
                else:
                    assert "enable" not in events and "motion" not in events and "probe" not in events, events
                assert "disable" in events, events
        print("set_zero ordering, rejection, device timeout and client timeout OK")
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
