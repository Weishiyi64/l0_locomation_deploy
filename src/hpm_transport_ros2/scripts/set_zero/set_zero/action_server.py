"""Persistent ROS endpoints for fixture calibration. One thread owns all motor IO."""
import queue
import re
import signal
import threading
import time
from contextlib import ExitStack
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from luvbot_hardware_interfaces.action import ModuleOperation as Operation

from . import motor_auto_zero as homing
from .version import APP_VERSION


def failure(code, message, phase="", index=-1, path=""):
    return Operation.Result(error_code=code, message=str(message), failed_phase=phase,
                            failed_index=index, log_path=str(path))


class StopOperation(Exception):
    pass


class TopicStop(StopOperation):
    def __init__(self):
        super().__init__("Stop requested on /set_zero/stop")


# A queue item wakes an idle worker; the latch also interrupts a running goal.
_RELEASE = object()


class OperationServer(Node):
    def __init__(self, io_factory=homing.make_ros_io):
        super().__init__("set_zero_server")
        self.io_factory = io_factory
        self.declare_parameter("config", str(homing.motor_config_path()))
        self.declare_parameter("calibration_config", str(homing.data_path("motor_zero_calibration.yaml")))
        self.declare_parameter("urdf", str(homing.data_path("l0_v5.urdf")))
        self.declare_parameter("log_dir", "set_zero_logs")
        self.declare_parameter("system_node", "/hpm_system")
        self.declare_parameter("service_timeout", 10.0)
        self.requests = queue.Queue()
        self.stopping = threading.Event()
        self.guard = threading.Lock()
        self.active = None
        self.cancel_requested = False
        self.finishing = False
        self.release_pending = False
        self.releasing = False
        self.topic_interrupted = False
        self.create_subscription(Empty, "/set_zero/stop", self.request_release,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.VOLATILE))
        self.runner = None
        self.resources = ExitStack()
        self.hold_fault = None
        self.servers = []
        for op in ("calibrate", "motion_test"):
            async def execute(goal, operation=op):
                return await self.execute(operation, goal)
            self.servers.append(ActionServer(
                self, Operation, "/set_zero/" + op, execute, cancel_callback=self.cancel,
                handle_accepted_callback=self.accepted,
                callback_group=ReentrantCallbackGroup()))
        self.worker = threading.Thread(target=self.control_loop, name="motor-control")
        self.worker.start()

    def prepare(self, module):
        args = homing.parse_args(["--module", module])
        for key in ("config", "calibration_config", "urdf", "log_dir"):
            setattr(args, key, Path(self.get_parameter(key).value))
        args.service_timeout = self.get_parameter("service_timeout").value
        args.system_node = self.get_parameter("system_node").value
        if not 0 < args.service_timeout < float("inf"):
            raise ValueError("service_timeout must be finite and positive")
        _, motors, selected, pause, knee, issues = homing.prepare_run(args)
        if not args.write_zero:
            raise ValueError("Module calibration requires write_zero=true")
        for joint, errors in issues.items():
            if errors:
                raise ValueError(f"{joint}: {', '.join(errors)}")
        return args, motors, selected, pause, knee

    def accepted(self, goal):
        # Reserve at acceptance so a following stop also covers a goal whose
        # execute callback has not run yet. Busy goals still get a useful result.
        with self.guard:
            if not (self.active or self.stopping.is_set() or self.release_pending or self.releasing):
                self.active = goal
                self.cancel_requested = self.finishing = self.topic_interrupted = False
        goal.execute()

    def request_release(self, _message):
        with self.guard:
            if self.release_pending or (self.runner is None and self.active is None):
                return
            self.release_pending = True
            if self.active is not None and not self.finishing:
                self.topic_interrupted = True
            self.requests.put(_RELEASE)

    async def execute(self, operation, goal):
        module = goal.request.module
        if goal != self.active:
            goal.abort()
            return failure("BUSY", "Operation or stop/disable cleanup in progress")
        try:
            if module not in homing.MODULES:
                raise ValueError(f"Unknown module: {module}")
            # Validation runs on the action executor; the control thread keeps pumping HOLD.
            plan = self.prepare(module) if operation == "calibrate" else None
        except Exception as exc:
            with self.guard:
                self.active = None
            goal.abort()
            return failure("CONFIG_ERROR", exc, "CONFIG")
        done = Future(executor=self.executor)
        self.requests.put((operation, goal, plan, done))
        result = await done
        try:
            if result.success:
                goal.succeed()
            elif goal.is_cancel_requested and result.error_code == "CANCELED":
                goal.canceled()
            else:
                goal.abort()
            return result
        finally:
            with self.guard:
                self.active = None

    def cancel(self, goal):
        with self.guard:
            if goal != self.active or self.finishing:
                return CancelResponse.REJECT
            self.cancel_requested = True
            return CancelResponse.ACCEPT

    def release(self):
        with self.guard:
            self.releasing = True
            self.runner = None
        try:
            self.resources.close()
        finally:
            with self.guard:
                self.releasing = False

    def start(self, plan):
        args, motors, targets, pause, knee = plan
        self.resources.enter_context(homing.acquire_motion_lock())
        log = homing.Logger(args.log_dir)
        self.resources.callback(log.close)
        log.write("RUN_START", module=args.module, source="action",
                  resolved_settings={t.motor.name: t.settings for t in targets})
        io = self.io_factory(targets, True, args.system_node, [], shared_context=True)
        self.resources.callback(io.close)
        runner = homing.Runner(io, motors, targets, log, args.service_timeout)

        def stop():
            if not runner.shutdown():
                raise homing.HomingError("SHUTDOWN_ERROR: stop/disable requests failed")

        self.resources.callback(stop)
        runner.module, runner.module_joint_pause_s, runner.knee_assist = args.module, pause, knee
        self.runner = runner
        return runner

    def location(self):
        run = self.runner
        if run is None:
            return "PREPARE", -1, ""
        motor = run.current.target.motor if run.current else (None if run.all_done else run.preflight_motor)
        phase = run.current.phase if run.current else ("HOLD" if run.all_done else "PREFLIGHT")
        return phase, motor.index if motor else -1, motor.name if motor else ""

    def operate(self, operation, goal, plan):
        if self.topic_interrupted:
            raise TopicStop()
        if operation == "motion_test":
            if self.runner is None:
                return self.hold_fault or failure("NOT_READY", "Calibrate the module first")
            if self.runner.module != goal.request.module:
                return failure("MODULE_MISMATCH", f"Currently holding {self.runner.module}")
        else:
            previous = self.runner
            try:
                self.release()
            except Exception as exc:
                self.hold_fault = failure("CLEANUP_FAILED", exc, "SHUTDOWN", path=(
                    previous.log.path.resolve() if previous else ""))
                return self.hold_fault
            self.hold_fault = None
            self.start(plan)

        run = self.runner
        started = time.monotonic()
        previous, next_at = None, started

        def checkpoint():
            nonlocal previous, next_at
            if self.topic_interrupted:
                raise TopicStop()
            if self.stopping.is_set() or self.cancel_requested:
                raise StopOperation()
            phase, index, joint = self.location()
            count = run.test_completed if operation == "motion_test" else len(run.completed)
            key = (phase, index, count)
            now = time.monotonic()
            if key != previous or now >= next_at:
                goal.publish_feedback(Operation.Feedback(
                    module=run.module, joint_name=joint, motor_index=index, phase=phase,
                    completed=count, total=len(run.targets), elapsed_s=now-started))
                previous, next_at = key, now + 1

        run.checkpoint = checkpoint
        try:
            if operation == "calibrate":
                checkpoint()
                run.preflight(True)
                run.calibrate(write_zero=True, keep_running=False)
            else:
                run.run_motion_test()
            checkpoint()
        finally:
            run.checkpoint = None
        return Operation.Result(success=True, message="CALIBRATION_PASS" if operation == "calibrate"
                                else "MOTION_TEST_PASS", log_path=str(run.log.path.resolve()))

    def fault(self, exc, canceled=False):
        phase, index, _ = self.location()
        path = str(self.runner.log.path.resolve()) if self.runner else ""
        message = str(exc) or ("Server stopping" if self.stopping.is_set() else "Operation canceled")
        token = message.rsplit(": ", 1)[-1]
        if isinstance(exc, TopicStop):
            code = "STOP_REQUESTED"
        elif canceled:
            code = "CANCELED" if self.cancel_requested else "SERVER_STOPPED"
        elif re.fullmatch("[A-Z][A-Z0-9_]+", token):
            code = token
        elif token.lower().endswith("timeout"):
            code = token.upper().replace(" ", "_")
        else:
            code = "HOMING_FAILED"
        try:
            if self.runner:
                self.runner.log.write("OPERATOR_STOP" if canceled else "RUN_FAIL",
                                      phase=phase, failed_index=index, reason=message,
                                      zero_unknown=bool(self.runner.current and self.runner.current.zero_unknown))
        finally:
            self.release()
        return failure(code, message, phase, index, path)

    def clean_fault(self, exc):
        phase, index, _ = self.location()
        path = str(self.runner.log.path.resolve()) if self.runner else ""
        try:
            return self.fault(exc, isinstance(exc, StopOperation))
        except Exception as cleanup:
            return failure("CLEANUP_FAILED", f"{exc}; cleanup: {cleanup}", phase, index, path)

    def control_loop(self):
        try:
            while True:
                try:
                    job = self.requests.get(block=self.runner is None)
                except queue.Empty:
                    try:
                        self.runner.pump()
                    except Exception as exc:
                        self.hold_fault = self.clean_fault(exc)
                        self.get_logger().error(self.hold_fault.message)
                    continue
                if job is _RELEASE:
                    if self.runner is not None:
                        result = self.clean_fault(TopicStop())
                        if result.error_code == "CLEANUP_FAILED":
                            self.hold_fault = result
                            self.get_logger().error(result.message)
                    with self.guard:
                        self.release_pending = False
                    continue
                if job is None:
                    return
                operation, goal, plan, done = job
                try:
                    result = self.operate(operation, goal, plan)
                except Exception as exc:
                    result = self.clean_fault(exc)
                # Close the race between the final checkpoint and accepting cancellation.
                with self.guard:
                    late_stop = self.topic_interrupted and self.runner is not None
                    late_cancel = self.cancel_requested and result.success
                    self.finishing = True
                if late_stop:
                    result = self.clean_fault(TopicStop())
                elif late_cancel:
                    result = self.clean_fault(StopOperation())
                if result.error_code == "CLEANUP_FAILED":
                    self.hold_fault = result
                done.set_result(result)
                self.executor.wake()
        finally:
            self.release()

    def stop(self):
        self.stopping.set()
        self.requests.put(None)
        self.worker.join()


def main():
    print(f"[VERSION] set_zero_server version={APP_VERSION}", flush=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    node = OperationServer()
    executor.add_node(node)
    exiting = threading.Event()

    def interrupt(signum, frame):
        exiting.set()
        executor.wake()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    try:
        while not exiting.is_set():
            executor.spin_once(timeout_sec=0.2)
    finally:
        node.stop()
        # Let the executor deliver the result of any interrupted operation before
        # destroying its action server. The worker has already finished cleanup.
        while node.active is not None:
            executor.spin_once(timeout_sec=0.05)
        executor.spin_once(timeout_sec=0.0)
        for server in node.servers:
            server.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
