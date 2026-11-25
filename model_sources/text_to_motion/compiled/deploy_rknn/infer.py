"""Run complete text/manual-token motion generation with an RK3588 package."""
import argparse
import json
from pathlib import Path

from .pipeline import DEFAULT_MODEL, RknnMotionPipeline
from .mirror import mirror_output


def boolean(value):
    if value.lower() not in ("true", "false"):
        raise argparse.ArgumentTypeError("需要 True 或 False")
    return value.lower() == "true"


def parse_pose(value):
    if value is None:
        return None
    raw = value.strip()
    if not raw.startswith("[") and "," not in raw:
        raw = Path(raw).read_text(encoding="utf-8").strip()
    return json.loads(raw) if raw.startswith("[") else [float(item) for item in raw.split(",")]


def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    result.add_argument("--mode", choices=("text", "web", "random"), default="text")
    result.add_argument("--text", default="")
    for name in ("head-token", "left-hand-token", "right-hand-token"):
        result.add_argument("--" + name, type=int, default=0)
    result.add_argument("--start-pose", help="九维弧度 JSON 数组、逗号分隔数字或数组文件；默认内置 A-pose")
    result.add_argument("--end-pose", help="同 start-pose 格式；默认 A-pose，仅作用于有动作部位")
    result.add_argument("--return", dest="return_home", type=boolean, default=True)
    result.add_argument("--return-frames", type=int, choices=(-1, 30, 60, 90), default=-1)
    result.add_argument("--speed", type=float, default=1.2)
    result.add_argument("--seed", type=int)
    result.add_argument('--mirror-head', type=boolean, nargs='?', const=True, default=False, help='输出头部左右镜像，默认 false')
    result.add_argument('--mirror-hands', type=boolean, nargs='?', const=True, default=True, help='输出双手左右镜像，默认 true；用 false 关闭')
    result.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="完整部署包 manifest.json，不是单个 .rknn")
    result.add_argument("--backend", choices=("lite2", "onnx-reference"), default="lite2",
                        help="默认 RK3588 NPU；onnx-reference 仅用于主机 CPU 对照，需 prepared manifest")
    result.add_argument("--device", choices=("npu", "cpu"), help="可省略；lite2 对应 npu，onnx-reference 对应 cpu，不自动降级")
    result.add_argument("--output", type=Path)
    return result


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    device = "npu" if args.backend == "lite2" else "cpu"
    if args.device is not None and args.device != device:
        p.error(f"--backend {args.backend} requires --device {device}; no automatic backend fallback")
    with RknnMotionPipeline(args.model, args.backend) as pipeline:
        output = pipeline.generate(mode=args.mode, text=args.text,
            token_ids=[args.head_token, args.left_hand_token, args.right_hand_token],
            init_pose=parse_pose(args.start_pose), end_pose=parse_pose(args.end_pose),
            return_home=args.return_home, return_frames=args.return_frames, speed=args.speed, seed=args.seed)
        output = mirror_output(output, {'head': args.mirror_head, 'hands': args.mirror_hands})
        output = {key: value.tolist() if hasattr(value, "tolist") else value for key, value in output.items()}
        output.update(input_text=args.text, mode=args.mode, fps=50, joint_names=pipeline.metadata["joint_names"],
                      return_home=args.return_home, return_frames=args.return_frames, speed=args.speed,
                      device=device)
        encoded = json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)


if __name__ == "__main__":
    main()
