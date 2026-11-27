import runpy
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def test_realtime():
    package = Path(__file__).resolve().parents[1]
    launch = runpy.run_path(str(package / "launch/hpm_system.launch.py"))
    context = LaunchContext()
    for action in launch["generate_launch_description"]().entities:
        if isinstance(action, DeclareLaunchArgument) and action.name in {
                "set_realtime_priority", "thread_scheduling_method", "thread_priority"}:
            action.execute(context)
    prefix = launch["_SchedulingPrefix"]("set_realtime_priority")
    assert prefix.perform(context) == "chrt --fifo 99"
    context.launch_configurations.update(thread_scheduling_method="SCHED_RR", thread_priority="50")
    assert prefix.perform(context) == "chrt --rr 50"
    for invalid in ("0", "100", "bad"):
        context.launch_configurations["thread_priority"] = invalid
        try:
            prefix.perform(context)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid priority: {invalid}")
    context.launch_configurations["set_realtime_priority"] = "false"
    assert prefix.perform(context) == ""


if __name__ == "__main__":
    test_realtime()
