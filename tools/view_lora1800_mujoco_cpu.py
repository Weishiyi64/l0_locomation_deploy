#!/usr/bin/env python3
"""Export a delivered LoRA 1800 adapter, then open live native MuJoCo."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


CODE = Path('/home/luvbotic/l0_mimic-lite')
ROOT = Path('/home/luvbotic/checkpoints/wholebody_11motion_1030')
DELIVERY = ROOT / 'bushu_8motion_1030/xuxun300_8motion'
EXPERIMENT = CODE / 'projects/mimic-lite/experiments/20261030_lora8_resume500_randomupper'
SCRIPTS = CODE / 'projects/mimic-lite/scripts'
ACTIONS = {
    'L0_angry_9': 'angry',
    'L0_angry_new_2': 'angry',
    'L0_excited_7': 'excited',
    'L0_excited_8': 'excited',
    'L0_fear_defend': 'fearful',
    'L0_satisfied_10': 'satisfied',
    'L0_strange_1': 'curious',
    'L0_strange_new_1_A': 'curious',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=ACTIONS)
    parser.add_argument('--duration', type=float, default=0,
                        help='Simulation seconds; 0 keeps the final reference until the window closes.')
    parser.add_argument('--prepare-only', action='store_true', help='Export without opening the viewer.')
    args = parser.parse_args()
    if args.duration < 0:
        parser.error('--duration must be non-negative')

    base = ROOT / 'base/checkpoint_10000.pt'
    adapter = DELIVERY / args.action / 'lorackpt_1800.pt'
    reference = DELIVERY / args.action / 'reference_motion.onnx'
    original = EXPERIMENT / 'motions' / ACTIONS[args.action] / args.action / 'original'
    for path in (base, adapter, reference, original / (args.action + '.npz')):
        if not path.is_file():
            raise FileNotFoundError(path)

    env = os.environ.copy()
    env.update(HF_HUB_OFFLINE='1', HF_HUB_DISABLE_TELEMETRY='1',
               MIMIC_LITE_LOCAL_ASSETS_ONLY='1', OMP_NUM_THREADS='1',
               OPENBLAS_NUM_THREADS='1', ANY4HDMI_CACHE_BUILD_BATCH_SIZE='256',
               PYTHONPATH=f'{CODE / "projects/mimic-lite"}:{CODE}',
               PYTHONUNBUFFERED='1')
    identity = {str(p): digest(p) for p in (base, adapter, reference)}
    token = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:16]
    cache = Path(tempfile.gettempdir()) / f'l0_lora1800_view_{os.getuid()}' / args.action / token
    cache.mkdir(parents=True, exist_ok=True)
    policy = cache / 'policy.onnx'
    record = cache / 'export.json'
    import fcntl
    with (cache / 'export.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        valid_cache = False
        if policy.is_file() and record.is_file():
            saved = json.loads(record.read_text())
            valid_cache = saved['inputs'] == identity and saved['policy_sha256'] == digest(policy)
        if not valid_cache:
            import torch
            from omegaconf import OmegaConf
            sys.path[:0] = [str(SCRIPTS), str(CODE / 'projects/mimic-lite'), str(CODE)]
            from lora_checkpoint import merge
            state = torch.load(adapter, map_location='cpu', weights_only=False)
            if state['num_updates'] != 1800:
                raise ValueError('Expected the delivered 1800-update LoRA adapter')
            merged = cache / 'checkpoint_1800.pt'
            merge(base, adapter, merged)
            cfg = OmegaConf.create(OmegaConf.to_container(state['cfg'], resolve=True))
            OmegaConf.set_struct(cfg, False)
            cfg.checkpoint_path = str(merged)
            cfg.checkpoint_load_mode = 'eval'
            cfg.algo.lora_enabled = False
            cfg.device = 'cpu'
            cfg.backend = 'mjlab'
            cfg.headless = True
            cfg.task.num_envs = 1
            cfg.task.name = f'L0Preview1800_{args.action}_{token}'
            cfg.task.command.motion_cfgs = {'original': {
                'path': str(original), 'weight': 1., 'full_motion': True,
                'filenames': [args.action + '.npz'],
            }}
            cfg.wandb.mode = 'disabled'
            cfg.wandb.id = None
            cfg.export_policy = True
            cfg.export_only = True
            cfg.export_name_suffix = token
            cfg.hydra = {'run': {'dir': str(cache / 'export_log')}}
            OmegaConf.save(cfg, cache / 'view_export.yaml')
            export_env = dict(env, MUJOCO_GL='glfw', CUDA_VISIBLE_DEVICES='')
            print(f'首次导出 {args.action}: Base + LoRA 1800，缓存 {cache}', flush=True)
            subprocess.run([sys.executable, str(SCRIPTS / 'play.py'),
                            '--config-path', str(cache), '--config-name', 'view_export'],
                           cwd=CODE, env=export_env, check=True)
            exports = list((SCRIPTS / 'exports' / cfg.task.name).glob(f'policy-*-1800-{token}.onnx'))
            if len(exports) != 1:
                raise RuntimeError(f'Expected one canonical export, found {exports}')
            import onnx
            onnx.checker.check_model(onnx.load(exports[0]))
            shutil.copy2(exports[0], policy)
            shutil.copy2(exports[0].with_suffix('.yaml'), cache / 'policy.yaml')
            record.write_text(json.dumps({'inputs': identity, 'policy_sha256': digest(policy),
                                          'canonical_export': str(exports[0])}, indent=2))
    if args.prepare_only:
        print(policy)
        return
    print(f'实时 MuJoCo：{args.action} / LoRA 1800 / 原动作末端 +1s。关闭窗口退出。', flush=True)
    viewer_env = dict(env, MUJOCO_GL='glfw')
    subprocess.run([sys.executable, str(SCRIPTS / 'deploy/run_l0_v2_mimic_lite_onnx_mujoco.py'),
                    '--policy', str(policy), '--motion', str(reference),
                    '--model', str(CODE / 'l0_v5_description/mjcf/l0.xml'),
                    '--provider', 'cpu', '--onnx-threads', '1', '--no-randomization',
                    '--tn-motor', '--seed', '42', '--realtime', '--duration', str(args.duration)],
                   cwd=CODE, env=viewer_env, check=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
