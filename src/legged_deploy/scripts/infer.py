#!/usr/bin/env python3
"""Generate one complete text-motion JSON and publish it atomically to WBC."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def _default_model() -> Path:
    candidates = (
        Path(__file__).resolve().with_name('text_to_motion_fp16.onnx'),
        Path.cwd() / 'text_to_motion_fp16.onnx',
        Path.home() / '下载' / 'text_to_motion_fp16.onnx',
        Path.home() / 'Downloads' / 'text_to_motion_fp16.onnx',
    )
    return next((path for path in candidates if path.is_file()), candidates[0])


def _publish_complete_json(payload: str, json_path: Path, topic: str, timeout: float) -> None:
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except (ImportError, ModuleNotFoundError):
        # Conda commonly provides ONNX Runtime with Python 3.11 while ROS 2
        # Humble provides rclpy for system Python 3.10.  Keep the user-facing
        # command atomic by delegating only the publish step to system Python.
        helper = Path(__file__).resolve().with_name('text_motion_publish.py')
        command = [
            '/usr/bin/python3', str(helper), '--input', str(json_path),
            '--topic', topic, '--subscriber-timeout', str(timeout),
        ]
        try:
            subprocess.run(command, check=True)
            return
        except (OSError, subprocess.CalledProcessError) as exc:
            raise RuntimeError(f'ROS 2 JSON 发布辅助进程失败：{exc}') from exc

    rclpy.init(args=None)
    node = rclpy.create_node('text_motion_infer')
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(String, topic, qos)
    try:
        deadline = time.monotonic() + timeout
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f'{timeout:g} 秒内未发现 WBC 订阅者：{topic}')
        message = String()
        # A single ROS message is the transaction boundary: WBC cannot see partial frames.
        message.data = payload
        publisher.publish(message)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description='输入文本，输出完整 JSON 并异步提交给 WBC。')
    parser.add_argument('text', help='动作描述，使用引号包裹')
    parser.add_argument('-o', '--output', type=Path, default=Path('motion.json'))
    parser.add_argument('--model', type=Path, default=_default_model())
    parser.add_argument('--device', choices=['auto', 'cpu', 'cuda'], default='auto')
    parser.add_argument(
        '--current-pose',
        type=float,
        nargs=9,
        metavar=('LSP', 'LSR', 'LER', 'RSP', 'RSR', 'RER', 'NY', 'NP', 'NR'),
        help='模型起始上半身姿态（9个关节，弧度）；默认使用模型元数据中的 a_pose',
    )
    parser.add_argument(
        '--speed',
        type=float,
        help='动作速度倍率；默认使用模型元数据中的 speed_default',
    )
    parser.add_argument('--overwrite', action='store_true', help='允许覆盖已有输出文件')
    parser.add_argument('--topic', default='/text_motion/json', help='完整 JSON 发布话题')
    parser.add_argument('--subscriber-timeout', type=float, default=10.0)
    parser.add_argument('--no-publish', action='store_true', help='只保存 JSON，不发给 WBC')
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f'找不到模型：{args.model}')
    if args.output.exists() and not args.overwrite:
        parser.error(f'输出已存在：{args.output}，请换路径或使用 --overwrite')
    if args.subscriber_timeout <= 0:
        parser.error('--subscriber-timeout 必须大于 0')
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as exc:
        parser.error(f'缺少运行依赖：{exc}。CPU 安装：python -m pip install numpy onnxruntime。')

    available = ort.get_available_providers()
    cuda = (
        args.device == 'cuda'
        or (args.device == 'auto' and 'CUDAExecutionProvider' in available)
    )
    if cuda and 'CUDAExecutionProvider' not in available:
        parser.error('未找到 CUDAExecutionProvider；请安装 GPU 运行库或使用 --device cpu。')
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    options.log_severity_level = 3
    providers = (
        ['CUDAExecutionProvider', 'CPUExecutionProvider']
        if cuda else ['CPUExecutionProvider']
    )
    session = ort.InferenceSession(str(args.model), options, providers=providers)
    if cuda and 'CUDAExecutionProvider' not in session.get_providers():
        raise RuntimeError('CUDA 初始化失败；请检查运行库，或显式使用 --device cpu。')

    metadata = session.get_modelmeta().custom_metadata_map
    names = json.loads(metadata['joint_names'])
    if len(names) != 9 or len(set(names)) != len(names):
        raise ValueError(f'模型 joint_names 必须是互不重复的 9 个关节，实际为 {names}')
    inputs = {item.name: item for item in session.get_inputs()}
    if 'text' not in inputs:
        raise ValueError('模型缺少 text 输入')
    feeds = {'text': np.array([args.text], dtype=object)}

    current_pose = None
    if 'current_pose' in inputs:
        if args.current_pose is not None:
            current_pose = np.asarray(args.current_pose, dtype=np.float16)
        else:
            try:
                current_pose = np.asarray(json.loads(metadata['a_pose']), dtype=np.float16)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(
                    '模型需要 current_pose，但没有可用的 a_pose 元数据；请传入 --current-pose 后跟9个弧度值'
                ) from exc
        if current_pose.shape != (len(names),) or not np.isfinite(current_pose).all():
            raise ValueError(f'current_pose 必须包含 {len(names)} 个有限关节角')
        feeds['current_pose'] = current_pose.reshape(1, len(names))
    elif args.current_pose is not None:
        raise ValueError('当前模型不接受 --current-pose')

    overridable_inputs = {item.name: item for item in session.get_overridable_initializers()}
    speed_supported = 'speed' in inputs or 'speed' in overridable_inputs
    speed = None
    if speed_supported:
        try:
            speed = args.speed if args.speed is not None else float(metadata.get('speed_default', '1.0'))
        except ValueError as exc:
            raise ValueError('模型 speed_default 不是有效数字') from exc
        if not np.isfinite(speed) or speed <= 0.0:
            raise ValueError('--speed 必须是大于0的有限数值')
        feeds['speed'] = np.asarray(speed, dtype=np.float32)
    elif args.speed is not None:
        raise ValueError('当前模型不接受 --speed')

    start = time.perf_counter()
    motion, lengths = session.run(['motion', 'segment_lengths'], feeds)
    elapsed = time.perf_counter() - start
    if motion.ndim != 2 or motion.shape[1] != len(names) or not np.isfinite(motion).all():
        raise ValueError('模型返回的动作数据无效')
    motion = motion.astype(np.float32)
    if not np.isfinite(motion).all():
        raise ValueError('模型返回的动作数据无效')
    fps = float(metadata['fps'])
    result = {
        'format': 'gmr_upper_body_joint_trajectory_v1',
        'robot': 'l0_v3',
        'text': args.text,
        'fps': fps,
        'angle_unit': metadata['angle_unit'],
        'joint_names': names,
        'frame_count': len(motion),
        'segment_lengths': lengths.tolist(),
        **({'current_pose': current_pose.astype(np.float32).tolist()} if current_pose is not None else {}),
        **({'speed': float(speed)} if speed is not None else {}),
        'frames': [
            {'frame_index': index, 'time': index / fps, 'joint_positions': frame.tolist()}
            for index, frame in enumerate(motion)
        ],
    }
    payload = json.dumps(result, ensure_ascii=False, separators=(',', ':'))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    mode = 'w' if args.overwrite else 'x'
    with args.output.open(mode, encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

    print(f'已保存完整 JSON：{args.output.resolve()}')
    print(f'{len(motion)} 帧 / {len(motion) / fps:.2f} 秒 / {fps:g} FPS；推理耗时 {elapsed:.3f} 秒')
    if not args.no_publish:
        _publish_complete_json(payload, args.output, args.topic, args.subscriber_timeout)
        print(f'已一次性提交全部 {len(motion)} 帧到 {args.topic}')
    print('执行后端：' + ', '.join(session.get_providers()))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'推理/提交失败：{exc}', file=sys.stderr)
        sys.exit(1)
