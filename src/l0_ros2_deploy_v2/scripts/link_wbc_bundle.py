#!/usr/bin/env python3
"""Expose one workspace-owned WBC asset directory in the source and installed policy trees."""
import argparse
from pathlib import Path
import json
import re

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='Directory name under WORKSPACE/assets/wbc')
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.name):
        parser.error('Invalid policy package name')
    ws = args.workspace.resolve(strict=True)
    asset_root = (ws / 'assets/wbc').resolve(strict=True)
    target = (asset_root / args.name).resolve(strict=True)
    if target.parent != asset_root:
        parser.error('Bundle must be a child directory of assets/wbc')
    catalog = json.loads((target / 'base_manifest.json').read_text())
    if catalog.get('format') not in ('wbc_base_graph_v1', 'wbc_lora_graph_v1') or not catalog.get('actions'):
        parser.error('Expected a WBC policy and action catalog')
    paths = [ws / 'src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5' / args.name]
    installed = ws / 'install/legged_deploy/share/legged_deploy/policy/l0_v5'
    if installed.is_dir():
        paths.append(installed / args.name)
    for path in paths:
        if path.is_symlink() and path.resolve() == target:
            continue
        if path.exists() or path.is_symlink():
            parser.error(f'Policy name already in use: {path}')
    for path in paths:
        print(f'{path} -> {target}')
        if not args.dry_run and not path.is_symlink():
            try:
                path.symlink_to(ws / 'assets/wbc' / args.name, target_is_directory=True)
            except FileExistsError:
                if not path.is_symlink() or path.resolve() != target:
                    raise

if __name__ == '__main__':
    main()
