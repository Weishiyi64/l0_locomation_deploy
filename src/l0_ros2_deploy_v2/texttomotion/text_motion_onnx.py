#!/usr/bin/env python3
"""Run the L0 v5 text-to-motion ONNX model and write a deployment JSON."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time


EXPECTED_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_elbow_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_elbow_roll_joint",
    "neck_yaw_joint",
    "neck_pitch_joint",
    "neck_roll_joint",
]

# L0 v5 MJCF/URDF hard position limits, in the model's joint order.
JOINT_LIMITS = [
    (-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571),
    (-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571),
    (-1.571, 1.571), (-0.34906585, 0.17453293), (-0.297, 0.297),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="L0 v5 文本动作离线生成器")
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parent / "run_fp16.onnx",
                        help="文本动作 ONNX 模型；默认使用保留原始模型的 FP16 参数副本 run_fp16.onnx")
    parser.add_argument("--text", default="", help="中文动作描述")
    parser.add_argument("--start-pose", type=str,
                        help="9 维初始姿态 JSON 数组或文件；默认使用模型内置 A-pose")
    parser.add_argument("--end-pose", type=str,
                        help="9 维回正目标；提供 --start-pose 时默认与初始姿态相同")
    parser.add_argument("--output", type=Path, help="输出 JSON")
    parser.add_argument("--export-fp16", type=Path, help="只从 --model 导出 FP16 参数副本，不生成或发送动作")
    parser.add_argument("--custom-ops", type=Path, help="libluvbot_onnx_text.so.1 路径")
    parser.add_argument("--speed", type=float, help="覆盖模型默认速度倍率")
    parser.add_argument("--overwrite", action="store_true", help="兼容参数；输出始终原子式更新")
    parser.add_argument("--skip-mujoco-check", action="store_true", help="跳过 L0 v5 MJCF 运动学校验")
    return parser.parse_args()


def resolve_custom_ops(model: Path, explicit: Path | None) -> Path:
    candidates = [
        explicit,
        model.resolve().parent / "libluvbot_onnx_text.so.1",
        Path(__file__).resolve().parent / "libluvbot_onnx_text.so.1",
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError("找不到 libluvbot_onnx_text.so.1；请放在模型或脚本旁，或传 --custom-ops")


def load_robot_pose(raw: str) -> list[float]:
    path = Path(raw)
    data = json.loads(path.read_text(encoding="utf-8") if path.is_file() else raw)
    if isinstance(data, dict):
        if data.get("robot") != "l0_v5" or data.get("angle_unit") != "rad" or data.get("joint_names") != EXPECTED_JOINTS:
            raise ValueError("初始姿态文件的机器人、单位或关节顺序与 L0 v5 不一致")
        data = data.get("joint_positions")
    if not isinstance(data, list) or len(data) != len(EXPECTED_JOINTS):
        raise ValueError("姿态必须包含按文本模型顺序排列的 9 个关节角")
    try:
        values = [float(value) for value in data]
    except (TypeError, ValueError) as exc:
        raise ValueError("姿态必须包含 9 个有限浮点数") from exc
    for name, value, (lower, upper) in zip(EXPECTED_JOINTS, values, JOINT_LIMITS):
        if not math.isfinite(value) or value < lower - 1e-6 or value > upper + 1e-6:
            raise ValueError(f"姿态中 {name} 超出 L0 v5 关节限位")
    return values


def main() -> None:
    args = parse_args()
    if not args.model.is_file():
        raise FileNotFoundError(f"找不到模型：{args.model}")
    if args.export_fp16 is not None:
        if args.export_fp16.resolve() == args.model.resolve():
            raise ValueError("FP16 导出必须写入新路径，禁止覆盖原始模型")
        import onnx
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from convert_onnx_to_rknn_rk3588_wbc_policy import fp16_parameter_storage
        model = onnx.load(str(args.model))
        count = fp16_parameter_storage(model, convert_constants=False, preserve_initializers=("head_limits",))
        metadata = {item.key: item.value for item in model.metadata_props}
        metadata.update(deploy_parameter_storage="float16", deploy_application_dtype="float32",
                        deploy_source_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                        deploy_source_name=args.model.name)
        onnx.helper.set_model_props(model, metadata)
        onnx.checker.check_model(model)
        args.export_fp16.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.export_fp16.with_suffix(".onnx.tmp")
        onnx.save(model, str(temporary))
        temporary.replace(args.export_fp16)
        print(f"FP16 参数副本已导出：{args.export_fp16}；转换 {count} 个张量；原始模型未修改；不是 RKNN")
        return
    if args.output is None:
        raise ValueError("生成动作需要 --output")
    if not args.text.strip():
        raise ValueError("--text 不能为空")
    if args.speed is not None and (not math.isfinite(args.speed) or args.speed <= 0.0):
        raise ValueError("--speed 必须是大于 0 的有限数")

    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError("缺少 numpy/onnxruntime；请使用项目 run_offline.sh") from exc

    custom_ops = resolve_custom_ops(args.model, args.custom_ops)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    options.log_severity_level = 3
    options.register_custom_ops_library(str(custom_ops))
    session = ort.InferenceSession(str(args.model.resolve()), options, providers=["CPUExecutionProvider"])

    metadata = session.get_modelmeta().custom_metadata_map
    source_name = metadata.get("deploy_source_name")
    if source_name:
        source = args.model.resolve().parent / source_name
        if source.is_file() and hashlib.sha256(source.read_bytes()).hexdigest() != metadata.get("deploy_source_sha256"):
            raise ValueError("原始模型已更新，请重新 --export-fp16，禁止使用过期的 FP16 副本")
    try:
        unified = json.loads(metadata["unified_motion"])
        model_metadata = unified["metadata"]
        joint_names = model_metadata["joint_names"]
        fps = float(model_metadata["fps"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("模型缺少有效的 unified_motion/joint_names/fps 元数据") from exc
    if joint_names != EXPECTED_JOINTS:
        raise ValueError(f"模型关节顺序与 L0 v5 不一致：{joint_names}")
    if not math.isfinite(fps) or fps <= 0.0:
        raise ValueError(f"模型 FPS 无效：{fps}")

    feeds = {"text": np.asarray([args.text], dtype=object)}
    robot_start_pose = load_robot_pose(args.start_pose) if args.start_pose is not None else None
    robot_end_pose = load_robot_pose(args.end_pose) if args.end_pose is not None else robot_start_pose

    def to_model_pose(robot_pose: list[float]) -> np.ndarray:
        # The model uses viewer-left/right; deployment JSON uses robot anatomical
        # left/right. Apply the inverse of the output arm mirror to both poses.
        pose = np.asarray(robot_pose, dtype=np.float32)
        model_pose = pose.copy()
        mirror_sign = np.asarray([1.0, -1.0, -1.0], dtype=np.float32)
        model_pose[:3] = pose[3:6] * mirror_sign
        model_pose[3:6] = pose[:3] * mirror_sign
        return model_pose

    if robot_start_pose is not None:
        feeds["init_pose"] = to_model_pose(robot_start_pose)
    if robot_end_pose is not None:
        feeds["end_pose"] = to_model_pose(robot_end_pose)
    model_inputs = {item.name for item in session.get_inputs()}
    model_inputs.update(item.name for item in session.get_overridable_initializers())
    if not set(feeds).issubset(model_inputs):
        raise ValueError(f"模型缺少输入：{sorted(set(feeds) - model_inputs)}")
    if args.speed is not None:
        feeds["speed"] = np.asarray([args.speed], dtype=np.float32)
    output_names = [item.name for item in session.get_outputs()]
    required = {"motion", "frame_count", "success"}
    if not required.issubset(output_names):
        raise ValueError(f"模型输出缺少 {sorted(required - set(output_names))}")

    started = time.perf_counter()
    values = session.run(output_names, feeds)
    elapsed = time.perf_counter() - started
    outputs = dict(zip(output_names, values))
    motion = np.asarray(outputs["motion"], dtype=np.float32)
    frame_count = int(np.asarray(outputs["frame_count"]).reshape(-1)[0])
    success = bool(np.asarray(outputs["success"]).reshape(-1)[0])
    if not success:
        raise RuntimeError("模型报告无法解析该文本动作")
    if motion.ndim != 2 or motion.shape[1] != len(EXPECTED_JOINTS):
        raise ValueError(f"动作张量应为 [frames, 9]，实际为 {motion.shape}")
    if frame_count != motion.shape[0] or frame_count < 2 or frame_count > 5000:
        raise ValueError(f"模型帧数无效：声明 {frame_count}，张量 {motion.shape[0]}")
    if not np.isfinite(motion).all():
        raise ValueError("动作包含 NaN 或 Inf")

    # The supplied ONNX model interprets 左/右 from the viewer's perspective:
    # single-arm probes show that "左手" moves its right-arm channels and vice
    # versa. Mirror the two arms into the robot's anatomical joint convention.
    # Shoulder pitch axes match; shoulder/elbow roll axes reverse under y mirror.
    source_motion = motion.copy()
    motion[:, :3] = source_motion[:, 3:6] * np.asarray([1.0, -1.0, -1.0], dtype=np.float32)
    motion[:, 3:6] = source_motion[:, :3] * np.asarray([1.0, -1.0, -1.0], dtype=np.float32)

    lower = np.asarray([limit[0] for limit in JOINT_LIMITS], dtype=np.float32)
    upper = np.asarray([limit[1] for limit in JOINT_LIMITS], dtype=np.float32)
    clipped = np.clip(motion, lower, upper)
    clipped_values = int(np.count_nonzero(clipped != motion))
    clipping_by_joint = {
        name: {"clipped_values": int(np.count_nonzero(clipped[:, j] != motion[:, j])),
               "clipped_values_over_1e_6_rad": int(np.count_nonzero(np.abs(clipped[:, j] - motion[:, j]) > 1e-6)),
               "max_clip_rad": float(np.max(np.abs(clipped[:, j] - motion[:, j]))),
               "raw_min_rad": float(motion[:, j].min()), "raw_max_rad": float(motion[:, j].max()),
               "sent_min_rad": float(clipped[:, j].min()), "sent_max_rad": float(clipped[:, j].max())}
        for j, name in enumerate(joint_names)
    }
    motion = clipped
    if robot_start_pose is not None and np.max(np.abs(motion[0] - robot_start_pose)) > 0.005:
        raise ValueError("模型首帧未到达 --start-pose")
    if robot_end_pose is not None and np.max(np.abs(motion[-1] - robot_end_pose)) > 0.005:
        raise ValueError("模型末帧未回到 --end-pose；检查新模型的回正参数")

    if not args.skip_mujoco_check:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("缺少 mujoco，无法执行 sim2mujoco 校验；可显式传 --skip-mujoco-check") from exc
        mjcf = (Path(__file__).resolve().parent.parent /
                "legged_description/l0_v5_description/mjcf/l0_v5.xml")
        if not mjcf.is_file():
            raise FileNotFoundError(f"找不到 L0 v5 MJCF：{mjcf}")
        mj_model = mujoco.MjModel.from_xml_path(str(mjcf))
        mj_data = mujoco.MjData(mj_model)
        for row in motion:
            for name, value in zip(EXPECTED_JOINTS, row):
                joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id < 0:
                    raise ValueError(f"MJCF 缺少文本动作关节：{name}")
                mj_data.qpos[mj_model.jnt_qposadr[joint_id]] = float(value)
            mujoco.mj_forward(mj_model, mj_data)
            if not np.isfinite(mj_data.xpos).all():
                raise ValueError("MuJoCo 正向运动学产生 NaN/Inf")

    diagnostics = {}
    for name in (
        "material_ids", "action_frame_count_before_speed", "part_transition_frames",
        "return_home_start_frame", "token_ids",
    ):
        if name in outputs:
            diagnostics[name] = np.asarray(outputs[name]).reshape(-1).tolist()
    result = {
        "format": "gmr_upper_body_joint_trajectory_v1",
        "robot": "l0_v5",
        "text": args.text,
        "fps": fps,
        "angle_unit": "rad",
        "joint_names": joint_names,
        "frame_count": frame_count,
        "frames": [
            {
                "frame_index": index,
                "time": index / fps,
                "joint_positions": frame.tolist(),
            }
            for index, frame in enumerate(motion)
        ],
        "model_diagnostics": diagnostics,
        "safety_adjustments": {"joint_limit_clipped_values": clipped_values, "per_joint": clipping_by_joint},
        "coordinate_adjustments": {"arm_side_mirror": True},
        "start_pose": {"source": "provided" if robot_start_pose is not None else "model_default",
                       "joint_positions": robot_start_pose},
        "end_pose": {"source": "provided" if robot_end_pose is not None else "model_default",
                     "joint_positions": robot_end_pose},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = args.output.with_suffix(args.output.suffix + ".tmp")
    with temporary_output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary_output.replace(args.output)

    print(f"文本动作生成成功：{args.output.resolve()}")
    print(f"{frame_count} 帧 / {frame_count / fps:.2f} 秒 / {fps:g} FPS；ONNX 推理 {elapsed:.3f} 秒")
    print("初始/回正姿态：" + ("已传入当前机器人姿态" if robot_start_pose is not None else "模型内置 A-pose"))
    if not args.skip_mujoco_check:
        print(f"MuJoCo 校验通过：L0 v5 全部 {frame_count} 帧运动学有效；限位裁剪 {clipped_values} 个值")
    for name, detail in clipping_by_joint.items():
        if detail["clipped_values"]:
            print(f"  限位裁剪 {name}: {detail}")
    print("关节：" + ", ".join(joint_names))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"文本动作生成失败：{exc}", file=sys.stderr)
        sys.exit(1)
