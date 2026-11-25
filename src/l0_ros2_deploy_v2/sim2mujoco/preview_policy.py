#!/usr/bin/env python3
"""Preview a deployed WBC reference_motion.onnx on the L0 v5 MuJoCo model.

This is kinematic reference playback, not the RKNN WBC closed-loop controller.
"""

import argparse
from pathlib import Path
import time


ROOT = Path(__file__).resolve().parent.parent
POLICIES = ROOT / "legged_deploy/policy/l0_v5"
MJCF = ROOT / "legged_description/l0_v5_description/mjcf/l0_v5.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description="在 MuJoCo 中预览 L0 v5 部署动作的参考轨迹")
    parser.add_argument("action", help="legged_deploy/policy/l0_v5 下的动作目录名")
    parser.add_argument("--headless", action="store_true", help="只校验完整参考轨迹，不打开窗口")
    args = parser.parse_args()
    if args.action in ("", ".", "..") or Path(args.action).name != args.action:
        parser.error("action 必须是单个动作目录名")
    directory = POLICIES / args.action
    if not (directory / "rl_config.yaml").is_file():
        directory = POLICIES / "wbc" / args.action
    directory = directory.resolve()
    if directory.parent not in (POLICIES.resolve(), (POLICIES / "wbc").resolve()):
        parser.error("action 必须位于 l0_v5 动作目录中")

    import mujoco
    import numpy as np
    import onnxruntime as ort
    import yaml

    config = yaml.safe_load((directory / "rl_config.yaml").read_text(encoding="utf-8"))
    robot = config["RobotCfg"]
    motion = config["WbcCfg"]
    names = robot["joint_order_mj"]
    count = int(motion["reference_num_frames"])
    dt = float(motion["control_dt"])
    if len(names) != 21 or count < 2 or count > 100000 or dt <= 0:
        raise ValueError("WBC 关节数、帧数或帧间隔无效")
    session = ort.InferenceSession(str(directory / "reference_motion.onnx"),
                                   providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    if len(inputs) != 1 or inputs[0].name != "time_step":
        raise ValueError("参考模型需要 time_step 输入")
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
    if any(index < 0 for index in joint_ids):
        raise ValueError("参考轨迹关节名与 L0 v5 MJCF 不一致")
    qpos_addresses = [int(model.jnt_qposadr[index]) for index in joint_ids]
    outputs = ["joint_pos", "body_pos_w", "body_quat_w"]
    if not set(outputs).issubset(item.name for item in session.get_outputs()):
        raise ValueError("参考模型缺少关节或底座轨迹输出")

    def frame(index: int) -> None:
        joints, body_pos, body_quat = session.run(
            outputs, {"time_step": np.asarray([[index]], dtype=np.float32)})
        joints = np.asarray(joints).reshape(-1)
        root_index = int(motion["reference_root_body_index"])
        position = np.asarray(body_pos)[0, root_index]
        quaternion = np.asarray(body_quat)[0, root_index]
        if joints.size != 21 or not np.isfinite(joints).all() or not np.isfinite(position).all() or \
                not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 0.5:
            raise ValueError(f"第 {index} 帧的参考模型输出无效")
        data.qpos[:3] = position
        data.qpos[3:7] = quaternion / np.linalg.norm(quaternion)
        for address, value in zip(qpos_addresses, joints):
            data.qpos[address] = value
        mujoco.mj_forward(model, data)
        if not np.isfinite(data.xpos).all():
            raise ValueError(f"第 {index} 帧 MuJoCo 正向运动学无效")

    if args.headless:
        for index in range(count):
            frame(index)
        print(f"{args.action}: {count} 帧 / {count * dt:.2f} 秒；L0 v5 MuJoCo 逐帧校验通过")
        return

    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print(f"预览 {args.action}: {count} 帧；关闭窗口或 Ctrl+C 退出")
        start = time.monotonic()
        while viewer.is_running():
            index = int((time.monotonic() - start) / dt) % count
            with viewer.lock():
                frame(index)
            viewer.sync()
            time.sleep(max(0.0, dt * 0.5))


if __name__ == "__main__":
    main()
