#!/usr/bin/env python3
"""Set each configured motor's current position to zero, then test a small MIT sine wave."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "src/hpm_motor/config/motors.yaml"


@dataclass(frozen=True)
class Motor:
    name: str
    index: int
    model: str | None
    bus_index: int
    motor_id: int


def load_motors(path: Path) -> list[Motor]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    entries = document.get("motors") if isinstance(document, dict) else None
    if not isinstance(entries, dict) or not entries:
        raise ValueError("config must contain a nonempty motors map")

    motors = []
    indices = set()
    routes = set()
    for name, entry in entries.items():
        if not isinstance(name, str) or not name or not isinstance(entry, dict):
            raise ValueError(f"invalid motor entry: {name!r}")
        motor_type = entry["motor_type"]
        if motor_type not in {"DM", "RS", "ENC", "MR", "LS"}:
            raise ValueError(f"{name}: unsupported motor_type")
        model = entry.get("model")
        if motor_type == "MR" and model not in {"PA43-10-40", "PA43-10-20.25", "PA35-10-8"}:
            raise ValueError(f"{name}: MR model must be PA43-10-40, PA43-10-20.25 or PA35-10-8")
        index = int(entry["index"])
        bus_index = int(entry["bus_index"])
        motor_id = int(str(entry["motor_id"]), 0)
        if not 0 <= index <= 255 or not 0 <= bus_index <= 7 or not 0 <= motor_id <= 0xFFFFFFFF:
            raise ValueError(f"{name}: index, bus_index or motor_id out of range")
        route = (bus_index, motor_id)
        if index in indices or route in routes:
            raise ValueError(f"{name}: duplicate index or bus_index/motor_id")
        indices.add(index)
        routes.add(route)
        motors.append(Motor(name, index, model, bus_index, motor_id))
    return sorted(motors, key=lambda motor: motor.index)


def sine(q0: float, amplitude: float, frequency: float, elapsed: float) -> tuple[float, float]:
    phase = 2.0 * math.pi * frequency * elapsed
    return q0 + amplitude * math.sin(phase), amplitude * 2.0 * math.pi * frequency * math.cos(phase)


def run(motors: list[Motor], amplitude: float, frequency: float, rate_hz: float,
        kp: float, kd: float, feedback_timeout: float, set_zero_timeout: float = 5.0) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorStateArray, MotorTarget
    from luvbot_hardware_interfaces.srv import MotorSetZero

    class SineTest(Node):
        def __init__(self) -> None:
            super().__init__("motor_sine_test")
            self.command_pub = self.create_publisher(MotorCommand, "/actuator/motor/command", 10)
            self.enable_pub = self.create_publisher(MotorEnableCommand, "/actuator/motor/enable", 10)
            self.state_sub = self.create_subscription(
                MotorStateArray, "/actuator/motor/state", self.on_state, qos_profile_sensor_data)
            self.set_zero_client = self.create_client(MotorSetZero, "/actuator/motor/set_zero")
            self.sequence = 0
            self.current = None
            self.q0 = None
            self.latest = None
            self.latest_at = None
            self.target_q = None
            self.target_dq = None
            self.feedback_count = 0
            self.max_error = 0.0
            self.failure = None

        def next_sequence(self) -> int:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            return self.sequence

        def publish_enable(self, motor: Motor, enabled: bool) -> None:
            message = MotorEnableCommand()
            message.stamp = self.get_clock().now().to_msg()
            message.sequence_id = self.next_sequence()
            message.motor_types = [motor.index]
            message.enabled = [enabled]
            self.enable_pub.publish(message)

        def publish_mit(self, motor: Motor, q: float, dq: float, gain_p: float, gain_d: float) -> None:
            target = MotorTarget()
            target.route.motor_type = motor.index
            target.q = q
            target.dq = dq
            target.tau = 0.0
            target.kp = gain_p
            target.kd = gain_d
            message = MotorCommand()
            message.stamp = self.get_clock().now().to_msg()
            message.sequence_id = self.next_sequence()
            message.control_mode = MotorCommand.CONTROL_MODE_MIT
            message.commands = [target]
            self.target_q = q if gain_p else None
            self.target_dq = dq if gain_p else None
            self.command_pub.publish(message)

        def on_state(self, message: MotorStateArray) -> None:
            motor = self.current
            if motor is None:
                return
            for state in message.states:
                route = state.route
                if (route.motor_type, route.bus_index, route.motor_id) != (
                    motor.index, motor.bus_index, motor.motor_id):
                    continue
                if not state.valid or state.error_code != 0 or not all(
                    math.isfinite(value) for value in (state.q, state.dq)):
                    self.failure = f"{motor.name}: invalid feedback or motor error {state.error_code}"
                    return
                self.latest = state
                self.latest_at = time.monotonic()
                self.feedback_count += 1
                if self.q0 is None:
                    self.q0 = state.q
                if self.target_q is not None:
                    self.max_error = max(self.max_error, abs(state.q - self.target_q))
                return

        def check_feedback(self, timeout: float | None = None) -> None:
            if self.failure:
                raise RuntimeError(self.failure)
            if timeout is not None and (
                self.latest_at is None or time.monotonic() - self.latest_at > timeout):
                raise RuntimeError(f"{self.current.name}: feedback timeout")

        def report(self) -> None:
            state = self.latest
            if state is None:
                return
            temperatures = " ".join(
                f"{name}={value:.1f}" for name, value in (
                    ("mos_C", state.mos_temperature), ("rotor_C", state.rotor_temperature))
                if math.isfinite(value))
            target = self.target_q if self.target_q is not None else state.q
            target_dq = self.target_dq if self.target_dq is not None else 0.0
            self.get_logger().info(
                f"{self.current.name}: target_q={target:.4f} q={state.q:.4f} "
                f"error={state.q - target:+.4f} target_dq={target_dq:.4f} dq={state.dq:.4f} "
                f"error_code={state.error_code} {temperatures}")

        def pace(self, deadline: float) -> None:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=deadline - time.monotonic())
                self.check_feedback()

        def wait_topics(self) -> None:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if (self.count_subscribers("/actuator/motor/command") > 0
                    and self.count_subscribers("/actuator/motor/enable") > 0
                    and self.count_publishers("/actuator/motor/state") > 0):
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            raise RuntimeError("motor topics did not connect within 5 seconds")

        def disable(self, motor: Motor) -> None:
            for _ in range(3):
                self.publish_enable(motor, False)
                deadline = time.monotonic() + 0.05
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=deadline - time.monotonic())

        def set_zero(self, motor: Motor) -> None:
            if not self.set_zero_client.wait_for_service(timeout_sec=set_zero_timeout):
                raise RuntimeError(f"{motor.name}: set_zero service unavailable")
            request = MotorSetZero.Request()
            request.motor_types = [motor.index]
            self.get_logger().info(f"{motor.name}: setting current position to zero")
            future = self.set_zero_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=set_zero_timeout)
            if not future.done():
                future.cancel()
                raise RuntimeError(f"{motor.name}: set_zero response timeout; result unknown")
            try:
                response = future.result()
            except Exception as exc:
                raise RuntimeError(f"{motor.name}: set_zero service failed: {exc}") from exc
            if (response is None or list(response.motor_types) != [motor.index]
                or len(response.results) != 1 or len(response.messages) != 1):
                raise RuntimeError(f"{motor.name}: invalid set_zero response")
            if response.results[0] != MotorSetZero.Response.CONFIRMED:
                raise RuntimeError(
                    f"{motor.name}: set_zero failed (result={response.results[0]}): "
                    f"{response.messages[0]}")
            self.get_logger().info(f"{motor.name}: set_zero confirmed: {response.messages[0]}")

        def test_motor(self, motor: Motor) -> None:
            self.current = None
            self.q0 = self.latest = self.latest_at = self.target_q = self.target_dq = self.failure = None
            self.feedback_count = 0
            self.max_error = 0.0
            self.get_logger().info(
                f"testing {motor.name}: index={motor.index} model={motor.model or '-'} "
                f"bus={motor.bus_index} id=0x{motor.motor_id:X}")
            try:
                self.set_zero(motor)
                self.current = motor
                self.publish_enable(motor, True)
                deadline = time.monotonic() + 2.0
                while self.q0 is None and time.monotonic() < deadline:
                    self.publish_mit(motor, 0.0, 0.0, 0.0, 0.0)
                    self.pace(time.monotonic() + 1.0 / rate_hz)
                if self.q0 is None:
                    raise RuntimeError(f"{motor.name}: no valid feedback within 2 seconds")

                self.get_logger().info(f"{motor.name}: initial q={self.q0:.4f}")
                started = time.monotonic()
                duration = 2.0 / frequency
                next_report = started
                while time.monotonic() - started < duration:
                    self.check_feedback(feedback_timeout)
                    elapsed = time.monotonic() - started
                    q, dq = sine(self.q0, amplitude, frequency, elapsed)
                    self.publish_mit(motor, q, dq, kp, kd)
                    if time.monotonic() >= next_report:
                        self.report()
                        next_report += 0.5
                    self.pace(time.monotonic() + 1.0 / rate_hz)

                deadline = time.monotonic() + 0.2
                while time.monotonic() < deadline:
                    self.check_feedback(feedback_timeout)
                    self.publish_mit(motor, self.q0, 0.0, kp, kd)
                    self.pace(time.monotonic() + 1.0 / rate_hz)
            finally:
                self.disable(motor)
                self.get_logger().info(
                    f"{motor.name}: disabled, feedback={self.feedback_count}, "
                    f"max_abs_q_error={self.max_error:.4f}")
                self.current = None

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SineTest()
    try:
        node.wait_topics()
        for motor in motors:
            node.test_motor(motor)
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(f"test stopped: {exc or 'Ctrl+C'}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--amplitude", type=float, default=0.1, help="radians from initial position")
    parser.add_argument("--frequency", type=float, default=0.25, help="cycles per second")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--kp", type=float, default=4.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--feedback-timeout", type=float, default=2.0,
                        help="maximum gap between feedback messages while moving, in seconds")
    parser.add_argument("--set-zero-timeout", type=float, default=5.0,
                        help="timeout for set_zero service discovery and response, in seconds")
    parser.add_argument("--self-test", action="store_true", help="check config and waveform without ROS")
    args = parser.parse_args()
    if not math.isfinite(args.feedback_timeout) or args.feedback_timeout <= 0:
        parser.error("--feedback-timeout must be finite and positive")
    if not math.isfinite(args.set_zero_timeout) or args.set_zero_timeout <= 0:
        parser.error("--set-zero-timeout must be finite and positive")
    if (not all(math.isfinite(value) for value in (
        args.amplitude, args.frequency, args.rate_hz, args.kp, args.kd))
        or args.amplitude <= 0 or args.frequency <= 0 or args.rate_hz <= 0
        or args.kp < 0 or args.kd < 0):
        parser.error("amplitude, frequency and rate-hz must be positive; kp and kd must be nonnegative")
    try:
        motors = load_motors(args.config)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.error(f"invalid config: {exc}")
    if args.self_test:
        q0, amplitude, frequency = 1.0, args.amplitude, args.frequency
        assert math.isclose(sine(q0, amplitude, frequency, 0.0)[0], q0)
        assert math.isclose(sine(q0, amplitude, frequency, 0.25 / frequency)[0], q0 + amplitude)
        assert math.isclose(sine(q0, amplitude, frequency, 0.75 / frequency)[0], q0 - amplitude)
        assert math.isclose(sine(q0, amplitude, frequency, 2.0 / frequency)[0], q0)
        print("config and sine OK:",
              [(m.name, m.index, m.model, m.bus_index, hex(m.motor_id)) for m in motors])
        return 0
    return run(motors, args.amplitude, args.frequency, args.rate_hz, args.kp, args.kd,
               args.feedback_timeout, args.set_zero_timeout)


if __name__ == "__main__":
    raise SystemExit(main())
