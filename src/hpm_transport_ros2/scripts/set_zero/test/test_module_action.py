"""Real ROS Action clients, isolated ROS domain, simulated motor transport only."""
import os
import threading
import time

import pytest
from test_module_motion_test import TestDrive
from test_motor_auto_zero import commissioned, Future, h, id_reply, Log

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("luvbot_hardware_interfaces.action")
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Empty
from set_zero.action_server import Operation, OperationServer


class Plant(TestDrive):
    def __init__(self, targets, offline):
        super().__init__(targets, Log(), offline=offline)
        self.stop_after_complete = False
        self.closed = False

    def query(self, motor):
        ids = [t.motor.motor_id for t in self.targets.values()
               if t.motor.protocol == motor.protocol and t.motor.bus == motor.bus]
        return Future(id_reply(motor, ids if motor.protocol == "LS" else None,
                               result=4 if self.offline else 0))

    def close(self):
        self.closed = True

    def spin(self, timeout):
        super().spin(timeout)
        time.sleep(0.0001)


def test_module_action_lifecycle(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 80))
    plants = []
    offline = False
    force_timeout = False

    def factory(targets, *args, **kwargs):
        plant = Plant(targets, offline)
        plants.append(plant)
        return plant

    pause_at = None
    paused, resume = threading.Event(), threading.Event()

    class PausingRunner(h.Runner):
        def __init__(self, io, *args):
            super().__init__(io, *args, clock=io.clock)

        def pause(self, phase):
            if pause_at == phase and not paused.is_set():
                paused.set()
                assert resume.wait(6)

        def pump(self, *args, **kwargs):
            if self.current:
                self.pause(self.current.phase)
            return super().pump(*args, **kwargs)

        def preflight(self, motion):
            self.pause("PREFLIGHT")
            super().preflight(motion)


    monkeypatch.setattr(h, "Runner", PausingRunner)
    idle_stop_seen = threading.Event()
    request_release = OperationServer.request_release
    def observe_stop(server, message):
        request_release(server, message)
        if server.runner is None and server.active is None:
            idle_stop_seen.set()
    monkeypatch.setattr(OperationServer, "request_release", observe_stop)
    rclpy.init()
    server = OperationServer(io_factory=factory)
    server.set_parameters([Parameter("log_dir", value=str(tmp_path))])
    operate = server.operate

    def pause_before_result(*args):
        outcome = operate(*args)
        if outcome.success:
            server.runner.pause("BEFORE_RESULT")
        return outcome

    monkeypatch.setattr(server, "operate", pause_before_result)
    prepare = server.prepare

    def simulated_plan(module):
        args, _, _, pause, knee = prepare(module)
        motors, targets = commissioned(module)
        for t in targets:
            t.settings["motion_test_torque_ff_nm"] = 0.02
            if force_timeout:
                t.settings["max_return_time_s"] = 0.001
        return args, motors, targets, pause, knee

    monkeypatch.setattr(server, "prepare", simulated_plan)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = rclpy.create_node("fixture_operator")
    client_executor = SingleThreadedExecutor()
    client_executor.add_node(client)
    cal = ActionClient(client, Operation, "/set_zero/calibrate")
    test = ActionClient(client, Operation, "/set_zero/motion_test")
    stop_pub = client.create_publisher(Empty, "/set_zero/stop", 1)
    feedback = []

    def resolve(future):
        client_executor.spin_until_future_complete(future, timeout_sec=20)
        assert future.done()
        return future.result()

    def send(action, module):
        assert action.wait_for_server(timeout_sec=2)
        return resolve(action.send_goal_async(Operation.Goal(module=module),
                                             feedback_callback=lambda f: feedback.append(f.feedback)))

    def result(goal):
        return resolve(goal.get_result_async())

    try:
        assert result(send(test, "left_leg")).result.error_code == "NOT_READY"
        goal = send(cal, "left_leg")
        assert result(send(cal, "left_leg")).result.error_code == "BUSY"
        assert result(goal).result.success
        assert any(f.total == 6 and f.completed == 6 for f in feedback)
        plant = plants[-1]
        before = len(plant.command_batches)
        assert result(send(cal, "typo")).result.error_code == "CONFIG_ERROR"
        assert result(send(test, "neck")).result.error_code == "MODULE_MISMATCH"
        assert len(plant.command_batches) > before and not plant.closed
        for _ in range(2):
            response = result(send(test, "left_leg")).result
            assert response.success, response
        goal = send(cal, "right_leg")
        assert result(goal).result.success
        assert plant.closed and not any(plant.enabled.values())
        goal = send(test, "right_leg")
        assert resolve(goal.cancel_goal_async()).goals_canceling
        canceled = result(goal)
        assert canceled.status == GoalStatus.STATUS_CANCELED and canceled.result.error_code == "CANCELED"
        assert plants[-1].closed and not any(plants[-1].enabled.values())
        offline = True
        failed = result(send(cal, "left_leg")).result
        assert failed.failed_phase == "PREFLIGHT" and failed.failed_index == 5
        assert "ID check failed" in failed.message
        offline = False
        force_timeout = True
        timed_out = result(send(cal, "right_arm")).result
        assert timed_out.error_code == "RETURN_TIMEOUT" and timed_out.failed_index == 17, timed_out
        force_timeout = False
        goal = send(cal, "left_leg")
        assert resolve(goal.cancel_goal_async()).goals_canceling
        assert result(goal).status == GoalStatus.STATUS_CANCELED
        assert result(send(cal, "right_arm")).result.success
        plants[-1].competitor = True
        deadline = time.monotonic() + 3
        while not server.hold_fault and time.monotonic() < deadline:
            time.sleep(0.01)
        assert "publisher" in result(send(test, "right_arm")).result.message
        assert result(send(cal, "left_arm")).result.success

        def until(predicate):
            deadline = time.monotonic() + 4
            while not predicate():
                assert time.monotonic() < deadline
                time.sleep(0.005)

        def release_hold():
            plant = plants[-1]
            stop_pub.publish(Empty())
            until(lambda: plant.closed and not server.release_pending)
            assert not any(plant.enabled.values())
            assert result(send(test, "left_arm")).result.error_code == "NOT_READY"

        until(lambda: stop_pub.get_subscription_count() == 1)
        release_hold()  # Successful calibration is holding, with no active goal.
        for phase in ("PREFLIGHT", "SEEK", "ZERO_PENDING", "MOVE", "BEFORE_RESULT", "MOTION_TEST"):
            if phase == "MOTION_TEST":
                assert result(send(cal, "left_arm")).result.success
            resume.clear()
            paused.clear()
            pause_at = phase
            goal = send(test if phase == "MOTION_TEST" else cal, "left_arm")
            assert paused.wait(6), phase
            plant = plants[-1]
            before = len(plant.calls)
            commands_before = len(plant.command_batches)
            stop_pub.publish(Empty())
            until(lambda: server.release_pending)
            stop_pub.publish(Empty())
            assert result(send(cal, "right_arm")).result.error_code == "BUSY"
            resume.set()
            stopped = result(goal)
            assert stopped.status == GoalStatus.STATUS_ABORTED
            assert stopped.result.error_code == "STOP_REQUESTED", stopped
            assert stopped.result.failed_phase == ("HOLD" if phase == "BEFORE_RESULT" else phase)
            assert not any(kind in ("enable", "zero_request") for kind, _, _ in plant.calls[before:])
            cleanup_batches = plant.command_batches[commands_before:]
            assert len(cleanup_batches) == (1 if plant.enabled else 0)
            assert all(cmd == (0.0,) * 5 for _, batch in cleanup_batches for _, cmd in batch)
            assert plant.closed and not any(plant.enabled.values())
            until(lambda: not server.release_pending)
            pause_at = None
        assert result(send(cal, "left_arm")).result.success
        assert result(send(test, "left_arm")).result.success
        release_hold()  # Successful motion test is also releasable.
        idle_stop_seen.clear()
        stop_pub.publish(Empty())
        assert idle_stop_seen.wait(2)
        assert result(send(cal, "right_arm")).result.success
        plant = plants[-1]
        enable = plant.enable
        def disable_error(motors, enabled):
            enable(motors, enabled)
            if not enabled:
                raise RuntimeError("injected disable error")
        plant.enable = disable_error
        stop_pub.publish(Empty())
        until(lambda: plant.closed and not server.release_pending)
        failed = result(send(test, "right_arm")).result
        assert failed.error_code == "CLEANUP_FAILED"
    finally:
        resume.set()
        server.stop()
        executor.shutdown()
        thread.join(timeout=2)
        for action in server.servers:
            action.destroy()
        server.destroy_node()
        cal.destroy()
        test.destroy()
        client.destroy_node()
        client_executor.shutdown()
        rclpy.shutdown()
    assert plants[-1].closed and not any(plants[-1].enabled.values())
