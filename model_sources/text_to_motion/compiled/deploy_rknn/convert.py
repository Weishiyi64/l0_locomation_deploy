"""Compile prepared static neural ONNXs with RKNN-Toolkit2, never Toolkit3."""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_TOOLKIT_VERSION = '2.3.2'


def work_root():
    return Path(os.environ.get('RKNN_WORK_DIR', str(ROOT / 'tmp/deploy_rknn'))).resolve()


def require_toolkit_version():
    """Pin the compiler ABI independently of the caller's selected Python."""
    version = importlib.metadata.version('rknn-toolkit2')
    if version != REQUIRED_TOOLKIT_VERSION:
        raise RuntimeError(f'This deployment requires rknn-toolkit2=={REQUIRED_TOOLKIT_VERSION}; '
                           f'found {version}. Install it in the isolated RKNN_PYTHON environment.')
    return version


@contextmanager
def working_directory(path):
    """SDK-only cwd isolation; also importable on board Python 3.9/3.10."""
    previous = Path.cwd()
    try:
        os.chdir(path)
        yield
    finally:
        os.chdir(previous)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_file(root, record):
    relative = Path(record['file'])
    path = (root / relative).resolve()
    if relative.is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError(f'Package path must stay inside its directory: {relative}')
    if not path.is_file() or sha256(path) != record['sha256']:
        raise ValueError(f'Missing or changed package file: {path}')
    return path


def inspect_model(path):
    """Conservative admission checks, NOT a complete vendor operator checker."""
    import onnx
    model = onnx.load(path, load_external_data=False)
    onnx.checker.check_model(model)
    problems = []
    for value in list(model.graph.input) + list(model.graph.output):
        tensor = value.type.tensor_type
        if tensor.elem_type not in (onnx.TensorProto.FLOAT, onnx.TensorProto.FLOAT16,
                                     onnx.TensorProto.INT32, onnx.TensorProto.INT64, onnx.TensorProto.BOOL):
            problems.append(f'{value.name}: nonnumeric tensor')
        if not tensor.HasField('shape') or any(not d.HasField('dim_value') or d.dim_value <= 0 for d in tensor.shape.dim):
            problems.append(f'{value.name}: dynamic/unknown shape')
    def walk(graph):
        for tensor in graph.initializer:
            if tensor.data_location == onnx.TensorProto.EXTERNAL:
                problems.append('external tensor data is not accepted')
        for node in graph.node:
            if node.domain not in ('', 'ai.onnx'):
                problems.append(f'custom operator: {node.domain}::{node.op_type}')
            if node.op_type in ('If', 'Loop', 'Scan', 'NonZero', 'RandomUniformLike', 'RandomNormalLike'):
                problems.append(f'dynamic/control operator: {node.op_type}')
            for attr in node.attribute:
                if attr.type == onnx.AttributeProto.GRAPH:
                    walk(attr.g)
    walk(model.graph)
    if model.functions:
        problems.append('local functions must be inlined')
    if problems:
        raise ValueError('Not an admitted static neural model: ' + '; '.join(sorted(set(problems))))
    return {'inputs': [v.name for v in model.graph.input], 'outputs': [v.name for v in model.graph.output],
            'operators': sorted({n.op_type for n in model.graph.node}),
            'note': 'Static preflight only; actual Toolkit2 compilation and board validation are still required.'}


def check_return(code, operation):
    if code != 0:
        raise RuntimeError(f'RKNN {operation} failed (return code {code})')


def compile_one(source, destination, factory):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    debug_root = work_root()
    debug_root.mkdir(parents=True, exist_ok=True)
    debug = Path(tempfile.mkdtemp(prefix=f'compiler_{destination.stem}_', dir=debug_root))
    # Toolkit2 writes check*.onnx to cwd even when no debug paths are requested.
    # Keep these diagnostics in tmp, not the caller's project or deployment.
    with working_directory(debug):
        rknn = factory(verbose=True)
        try:
            check_return(rknn.config(target_platform='rk3588'), 'config')
            check_return(rknn.load_onnx(model=str(source)), 'load_onnx')
            # Do not apply image normalization or fabricate an INT8 calibration set.
            check_return(rknn.build(do_quantization=False), 'build')
            check_return(rknn.export_rknn(str(destination)), 'export_rknn')
            if not destination.is_file() or destination.stat().st_size == 0:
                raise RuntimeError(f'Compiler reported success but produced no model: {destination}')
        finally:
            rknn.release()


def convert(manifest_path, output, check_only=False):
    manifest_path, output = Path(manifest_path).resolve(), Path(output).absolute()
    root = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('format') != 'rknn_static_neural_v1':
        raise ValueError('Use deploy_rknn.prepare first; the complete text-to-motion ONNX cannot be directly converted.')
    models = manifest['models']
    if set(models) != {'qwen_tokenizer', 'inbetweener'}:
        raise ValueError('Expected exactly qwen_tokenizer and inbetweener models')
    reports = {name: inspect_model(checked_file(root, record)) for name, record in models.items()}
    auxiliary = [manifest['vocabulary'], *manifest['validation_cases'], *manifest.get('pipeline_cases', [])]
    if 'motion_assets' in manifest:
        if 'motion_metadata' not in manifest:
            raise ValueError('motion_assets requires matching motion_metadata')
        auxiliary.append(manifest['motion_assets'])
    for record in auxiliary:
        checked_file(root, record)
    if check_only:
        return {'status': 'static_preflight_passed_not_compiled', 'models': reports}
    if output.exists():
        raise FileExistsError(f'Choose a new output directory: {output}')
    try:
        version = require_toolkit_version()
        from rknn.api import RKNN
    except (ImportError, importlib.metadata.PackageNotFoundError) as exc:
        raise RuntimeError('RKNN-Toolkit2 is required in an isolated conversion environment; '
                           'install the official RK3588-compatible Toolkit2 wheel, not RKNN3. '
                           'No .rknn files have been produced.') from exc
    temp_root = work_root()
    temp_root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='build_', dir=temp_root))
    compiled = json.loads(json.dumps(manifest))
    compiled.update(format='rknn_neural_package_v1', target_platform='rk3588',
                    toolkit2_version=version, quantized=False, status='compiled_board_unverified',
                    complete_text_to_motion_pipeline='motion_assets' in manifest,
                    preparation_manifest_sha256=sha256(manifest_path))
    compiled['validation'].update(rknn_compiled=True, board_verified=False)
    # Preserve all portable vocabulary and validation artifacts. ONNX files are
    # not required by the board verifier and are intentionally not copied.
    for record in auxiliary:
        file = checked_file(root, record)
        target = staging / record['file']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, target)
    for name, record in models.items():
        target = staging / f'{name}.rknn'
        compile_one(checked_file(root, record), target, RKNN)
        compiled['models'][name].update(file=target.name, sha256=sha256(target),
                                       source_onnx_sha256=record['sha256'])
    # Copy the lightweight host package so the deployment directory is portable.
    # Keep compiler/export modules out; convert.py is included for shared hash
    # validation helpers only and imports SDK/ONNX lazily.
    if 'motion_assets' in manifest:
        host = staging / 'deploy_rknn'
        host.mkdir()
        compiled['host_files'] = []
        for name in ('__init__.py', 'convert.py', 'tokenizer.py', 'motion.py', 'pipeline.py', 'infer.py', 'verify.py', 'support.py', 'mirror.py'):
            target = host / name
            source = Path(__file__).parent / name
            if not source.is_file():
                source = ROOT/'runtime/inference/portable'/name
            shutil.copy2(source, target)
            if name == 'verify.py':
                target.write_text(target.read_text().replace('from runtime.inference.portable.pipeline', 'from .pipeline'))
            compiled['host_files'].append({'file': str(target.relative_to(staging)), 'sha256': sha256(target)})
    (staging / 'manifest.json').write_text(json.dumps(compiled, ensure_ascii=False, indent=2))
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(staging), str(output))
    return {'status': compiled['status'], 'output': str(output), 'models': list(models)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'rknn/prepared/manifest.json')
    parser.add_argument('--output', type=Path, default=ROOT/'rknn/compiled')
    parser.add_argument('--check-only', action='store_true', help='No RKNN SDK required; does NOT establish compiler support')
    args = parser.parse_args()
    print(json.dumps(convert(args.manifest, args.output, args.check_only), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
