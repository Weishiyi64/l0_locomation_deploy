#!/usr/bin/env python3
"""Compile/install a fixed-actor bundle, without ROS launch or NPU inference.

--reuse-bundle reuses reference RKNNs only with identical ONNX provenance.
The ordinary two-input policy converter is used, without a LoRA manifest.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import yaml


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def asset(root, name, exists=True):
    path = (root / name).resolve(strict=exists)
    if Path(name).is_absolute() or not path.is_relative_to(root.resolve()) or (exists and not path.is_file()):
        raise ValueError('Invalid bundle asset: ' + name)
    return path


def compiled(path, source_hash):
    record = Path(str(path) + '.json')
    if not path.is_file() or not record.is_file():
        return False
    data = json.loads(record.read_text())
    return (data.get('format') == 'wbc_rknn_conversion_v1' and data.get('source_sha256') == source_hash
            and data.get('rknn_sha256') == sha(path) and data.get('precision') == 'float16'
            and data.get('do_quantization') is False and data.get('target_platform') == 'rk3588')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--reuse-bundle', action='append', type=Path, default=[])
    parser.add_argument('--install-to', type=Path, help='New policy directory; never replace existing deployments')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    root = args.bundle.resolve(strict=True)
    graph = json.loads((root / 'base_manifest.json').read_text())
    if graph.get('format') != 'wbc_base_graph_v1' or graph.get('kind') != 'policy':
        raise ValueError('Expected wbc_base_graph_v1 policy bundle')
    if args.install_to and (args.install_to.exists() or args.install_to.is_symlink()):
        raise ValueError('Install destination already exists')
    if args.install_to and args.install_to.resolve().is_relative_to(root):
        raise ValueError('Install destination must be outside source bundle')
    cache = {}
    for directory in args.reuse_bundle:
        source = json.loads((directory / 'base_manifest.json').read_text())
        for item in source['actions']:
            reference = asset(directory, item['reference_rknn'], exists=False)
            fingerprint = item['reference_onnx_sha256']
            if compiled(reference, fingerprint):
                cache[fingerprint] = reference
    actions = graph['actions']
    ids = [item['action_id'] for item in actions]
    if not ids or len(set(ids)) != len(ids):
        raise ValueError('Empty/duplicate action catalog')
    jobs = [(graph['model']['onnx'], graph['model']['rknn'], graph['model']['sha256'], 'policy')]
    first_robot = None
    for item in actions:
        config = asset(root, item['config'])
        if sha(config) != item['config_sha256']:
            raise ValueError('Config hash mismatch')
        data = yaml.safe_load(config.read_text())
        if data['WbcCfg']['motion_id'] != item['action_id']:
            raise ValueError('Action/config identity mismatch')
        if first_robot is not None and data['RobotCfg'] != first_robot:
            raise ValueError('All actions must use identical fixed-actor RobotCfg')
        first_robot = data['RobotCfg']
        jobs.append((item['reference_onnx'], item['reference_rknn'], item['reference_onnx_sha256'], 'reference'))
    scripts = Path(__file__).resolve().parent
    counts = {'existing': 0, 'reused': 0, 'compiled': 0}
    for source_name, target_name, fingerprint, kind in jobs:
        source = asset(root, source_name)
        target = asset(root, target_name, exists=False)
        if sha(source) != fingerprint:
            raise ValueError('Source hash mismatch: ' + source_name)
        if compiled(target, fingerprint):
            counts['existing'] += 1
            continue
        if kind == 'reference' and fingerprint in cache:
            counts['reused'] += 1
            if not args.dry_run:
                shutil.copy2(cache[fingerprint], target)
                shutil.copy2(Path(str(cache[fingerprint]) + '.json'), Path(str(target) + '.json'))
            continue
        counts['compiled'] += 1
        command = [sys.executable, '-B', str(scripts / ('convert_onnx_to_rknn_rk3588_wbc_' + kind + '.py')),
                   str(source), '--output', str(target), '--build-only']
        print('compile', kind, source_name, flush=True)
        if args.dry_run:
            continue
        log = root / 'conversion_logs' / (target.parent.name + '_' + target.stem + '.log')
        log.parent.mkdir(exist_ok=True)
        # Toolkit debug ONNXs stay outside the source tree.
        with tempfile.TemporaryDirectory(prefix='wbc_base_compile_') as work, log.open('w') as stream:
            result = subprocess.run(command, cwd=work, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode or not compiled(target, fingerprint):
            raise RuntimeError('RKNN conversion failed; see ' + str(log))
    print(json.dumps(counts), flush=True)
    if args.dry_run:
        return
    files = {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob('*'))
             if p.is_file() and p.name != 'INSTALLATION.json'}
    record = {'format': 'wbc_base_installation_v1', 'mode': 'base', 'files': files,
              'actions': ids, 'runtime_validation': 'not_run'}
    (root / 'INSTALLATION.json').write_text(json.dumps(record, indent=2) + '\n')
    if args.install_to:
        shutil.copytree(root, args.install_to)
        print('installed', args.install_to, flush=True)


if __name__ == '__main__':
    main()
