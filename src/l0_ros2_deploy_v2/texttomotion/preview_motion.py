#!/usr/bin/env python3
"""Show an exported text trajectory beside the live L0 v5 MuJoCo window."""

import argparse
import json
from pathlib import Path
from queue import Empty, Queue
import subprocess
import threading
import time


ROOT = Path(__file__).resolve().parent.parent
MJCF = ROOT / "legged_description/l0_v5_description/mjcf/l0_v5.xml"
JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint",
)


def main() -> None:
    parser = argparse.ArgumentParser(description="并排查看文本动作的 MuJoCo 参考轨迹")
    parser.add_argument("--input", type=Path, help="可选的初始 JSON；--ros-sync 会自动接收后续发送的完整轨迹")
    parser.add_argument("--ros-sync", action="store_true",
                        help="常驻窗口；接收 /text_motion/json 并等待同一 ROS_DOMAIN_ID 的 started 后播放")
    parser.add_argument("--headless", action="store_true", help="逐帧运动学校验，不打开窗口")
    args = parser.parse_args()
    if args.input is None and (args.headless or not args.ros_sync):
        parser.error("离线预览和 --headless 需要 --input；常驻 --ros-sync 可不指定文件")

    import mujoco
    import numpy as np

    def decode_trajectory(raw: str):
        payload = json.loads(raw)
        if payload.get("format") != "gmr_upper_body_joint_trajectory_v1" or \
                payload.get("robot") != "l0_v5" or payload.get("angle_unit") != "rad" or \
                payload.get("joint_names") != list(JOINTS):
            raise ValueError("不是 L0 v5 的 9 关节文本轨迹")
        fps = float(payload["fps"])
        frames = payload["frames"]
        if not np.isfinite(fps) or fps <= 0 or fps > 200 or not isinstance(frames, list) or \
                len(frames) != payload["frame_count"] or not 2 <= len(frames) <= 5000:
            raise ValueError("FPS 或帧数无效")
        if any(frame.get("frame_index") != index for index, frame in enumerate(frames)):
            raise ValueError("帧序号不连续")
        positions = np.asarray([frame["joint_positions"] for frame in frames], dtype=np.float64)
        if positions.shape != (len(frames), len(JOINTS)) or not np.isfinite(positions).all():
            raise ValueError("关节轨迹形状或数值无效")
        return payload, fps, positions

    payload = fps = positions = None
    if args.input is not None:
        payload, fps, positions = decode_trajectory(args.input.read_text(encoding="utf-8"))

    print(f"参考模型：{MJCF.resolve()}；参考轨迹为发送给控制器的限位裁剪后 JSON", flush=True)
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    if model.nkey < 1:
        raise ValueError("L0 v5 MJCF 缺少默认站姿 keyframe")
    mujoco.mj_resetDataKeyframe(model, data, 0)
    addresses = []
    for name in JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"MJCF 缺少关节 {name}")
        addresses.append(int(model.jnt_qposadr[joint_id]))

    def frame(index: int) -> None:
        if positions is None:
            return
        data.qpos[addresses] = positions[index]
        mujoco.mj_forward(model, data)
        if not np.isfinite(data.xpos).all():
            raise ValueError(f"MuJoCo 第 {index} 帧运动学无效")

    if args.headless:
        for index in range(len(positions)):
            frame(index)
        print(f"参考动作校验通过：{len(positions)} 帧 / {len(positions) / fps:.2f} 秒")
        return

    watcher = None
    events = Queue()
    if args.ros_sync:
        watcher = subprocess.Popen(
            ["/usr/bin/python3", str(Path(__file__).with_name("text_motion_status_watch.py"))],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
        )

        def read_status() -> None:
            for line in watcher.stdout:
                if line.startswith("TRAJECTORY\t"):
                    try:
                        events.put(("TRAJECTORY", json.loads(line.split("\t", 1)[1])))
                    except json.JSONDecodeError:
                        events.put(("ERROR", "ROS 轨迹传输数据无法解码"))
                else:
                    events.put((line.strip(), None))
            events.put(("WATCHER_EXITED", None))

        threading.Thread(target=read_status, daemon=True).start()

    try:
        import mujoco.viewer
        frame(0)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("常驻参考窗口已打开；请把它放在实时 MuJoCo 窗口旁边", flush=True)
            if args.ros_sync:
                print("等待 /text_motion/json 和 started；后续动作会自动切换参考轨迹", flush=True)
            start = time.monotonic()
            playing = not args.ros_sync and positions is not None
            pending_start = False
            fresh_trajectory = not args.ros_sync and positions is not None
            last_index = -1
            while viewer.is_running():
                try:
                    while True:
                        event, value = events.get_nowait()
                        if event == "TRAJECTORY":
                            try:
                                payload, fps, positions = decode_trajectory(value)
                            except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                                print(f"参考轨迹被拒绝：{exc}", flush=True)
                                continue
                            playing = False
                            fresh_trajectory = True
                            last_index = 0
                            with viewer.lock():
                                frame(0)
                            print(f"收到参考动作：{payload.get('text', '')}；{len(positions)} 帧", flush=True)
                            if pending_start:
                                start = time.monotonic()
                                playing = True
                                pending_start = False
                                fresh_trajectory = False
                                last_index = -1
                                print("参考动作开始", flush=True)
                        if event == "STARTED":
                            if positions is None or not fresh_trajectory:
                                pending_start = True
                            else:
                                start = time.monotonic()
                                playing = True
                                fresh_trajectory = False
                                last_index = -1
                                print("参考动作开始", flush=True)
                        elif event == "COMPLETED":
                            pending_start = False
                            fresh_trajectory = False
                            if playing and positions is not None:
                                playing = False
                                with viewer.lock():
                                    frame(len(positions) - 1)
                                print("参考动作完成；等待下条文本轨迹", flush=True)
                        elif event == "READY":
                            print("ROS 轨迹和状态监听已启动", flush=True)
                        elif event == "ERROR":
                            print(value, flush=True)
                        elif event == "WATCHER_EXITED":
                            raise RuntimeError("ROS 状态监听进程已退出；检查 ROS 环境和 ROS_DOMAIN_ID")
                except Empty:
                    pass
                if playing and positions is not None:
                    elapsed_frames = int((time.monotonic() - start) * fps)
                    index = min(elapsed_frames, len(positions) - 1) if args.ros_sync else elapsed_frames % len(positions)
                    if index != last_index:
                        with viewer.lock():
                            frame(index)
                        last_index = index
                viewer.sync()
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if watcher is not None:
            watcher.terminate()
            try:
                watcher.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                watcher.kill()
                watcher.communicate()


if __name__ == "__main__":
    main()
