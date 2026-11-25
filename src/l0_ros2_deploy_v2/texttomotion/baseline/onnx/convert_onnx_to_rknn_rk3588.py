#!/usr/bin/env python3
"""Build the text-to-motion RK3588 package with Toolkit2 2.3.2.

Board compilation never initializes a runtime. --build-only is accepted for
compatibility; --dry-run checks admission without importing the SDK or writing.
The input is a two-model manifest, not a WBC policy/reference ONNX.
"""
import argparse
import importlib.metadata
import json
import platform
from pathlib import Path
import shutil
import sys
import tempfile

HERE = Path(__file__).resolve().parent


def environment_versions():
    versions = {'python': platform.python_version(), 'machine': platform.machine()}
    for name in ('rknn-toolkit2', 'onnx', 'numpy', 'onnxruntime', 'protobuf',
                 'onnxoptimizer', 'torch', 'scipy', 'opencv-python', 'psutil',
                 'ruamel.yaml', 'tqdm', 'fast-histogram'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    default_manifest = HERE/'manifest.json' if (HERE/'manifest.json').is_file() else HERE.parent/'onnx/manifest.json'
    parser.add_argument('--manifest', type=Path, default=default_manifest)
    parser.add_argument('-o', '--output', type=Path, default=HERE / 'compiled', help='New deployment directory')
    parser.add_argument('--build-only', action='store_true', default=True, help='Default behavior; never initializes an inference runtime')
    parser.add_argument('--dry-run', '--check-only', dest='dry_run', action='store_true', help='Validate and print plan without SDK import or writes')
    args = parser.parse_args(argv)
    manifest, output = args.manifest.resolve(), args.output.resolve()
    if not (manifest.parent / 'deploy_rknn/convert.py').is_file():
        raise FileNotFoundError('Missing bundled compiler; export with runs/deploy_onnx_rknn.sh first')
    sys.path.insert(0, str(manifest.parent))
    from deploy_rknn.convert import convert, work_root, sha256
    plan = convert(manifest, output, check_only=True)
    plan.update(toolkit2_version='2.3.2', target_platform='rk3588', quantized=False,
                output=str(output), verification='pending board inference verification')
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0
    if output.exists():
        raise FileExistsError(f'Choose a new output directory: {output}')
    temporary = work_root()
    temporary.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='validated_release_', dir=temporary))
    candidate = staging / 'compiled'
    convert(manifest, candidate)
    record = {'toolkit2_version': '2.3.2', 'target_platform': 'rk3588',
              'do_quantization': False, 'source_manifest_sha256': sha256(manifest),
              'build_only': args.build_only, 'board_verified': False,
              'verification': 'pending',
              'compiled_manifest_sha256': sha256(candidate / 'manifest.json'),
              'environment': environment_versions()}
    (candidate / 'conversion_record.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(candidate), str(output))
    print(f'Exported: {output}; verification={record["verification"]}; board verification pending')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(f'ERROR: {error}', file=sys.stderr)
        sys.exit(1)
