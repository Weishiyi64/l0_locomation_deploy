"""Run prepared numerical parity fixtures on a real RK3588 using RKNNLite2."""
import argparse
import json
from pathlib import Path
import numpy as np
from .convert import ROOT, checked_file, check_return, sha256


def compare_output(name, actual, expected, atol=0.005, rtol=0.02):
    actual, expected = np.asarray(actual), np.asarray(expected)
    if actual.shape != expected.shape:
        raise ValueError(f'{name}: shape {actual.shape} != {expected.shape}')
    if not np.isfinite(actual).all():
        raise ValueError(f'{name}: nonfinite values')
    if name == 'output_frame_count' or expected.dtype.kind in 'biu':
        np.testing.assert_array_equal(actual, expected)
    elif name.endswith('_logits'):
        # Logits span hundreds while an irrelevant class may be close to zero.
        # Pointwise relative error at zero is not a useful FP16 acceptance rule.
        # Require exactly the same decision AND <= 0.2% tensor-scale error.
        np.testing.assert_array_equal(actual.argmax(-1), expected.argmax(-1))
        scale = max(1.0, float(np.max(np.abs(expected))))
        normalized_error = float(np.max(np.abs(actual.astype(np.float64) - expected))) / scale
        if normalized_error > .002:
            raise ValueError(f'{name}: normalized logit error {normalized_error} exceeds 0.002')
    else:
        np.testing.assert_allclose(actual, expected, atol=atol, rtol=rtol)
        if name == 'qwen_embedding':
            cosine = np.sum(actual * expected) / max(1e-12, np.linalg.norm(actual) * np.linalg.norm(expected))
            if cosine < .999:
                raise ValueError(f'Embedding cosine {cosine} is below 0.999')
    return float(np.max(np.abs(actual.astype(np.float64)-expected)))


def verify(manifest_path, report_path):
    manifest_path, report_path = Path(manifest_path).resolve(), Path(report_path)
    if report_path.exists():
        raise FileExistsError(report_path)
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('format') != 'rknn_neural_package_v1' or manifest.get('target_platform') != 'rk3588':
        raise ValueError('Expected a compiled RK3588 neural package')
    compatible = Path('/proc/device-tree/compatible')
    if not compatible.exists() or b'rk3588' not in compatible.read_bytes():
        raise RuntimeError('This verification must run on an RK3588 board; no CPU/simulator fallback is used.')
    from rknnlite.api import RKNNLite
    cases = manifest.get('validation_cases', [])
    if not cases or {case['model'] for case in cases} != set(manifest['models']):
        raise ValueError('Validation cases must cover every model')
    report = {'status': 'running', 'manifest_sha256': sha256(manifest_path), 'cases': [],
              'board_verified': False, 'complete_text_to_motion_pipeline': False}
    sessions = {}
    try:
        for name, record in manifest['models'].items():
            session = RKNNLite()
            sessions[name] = session
            check_return(session.load_rknn(str(checked_file(manifest_path.parent, record))), 'load_rknn')
            check_return(session.init_runtime(core_mask=RKNNLite.NPU_CORE_0), 'init_runtime')
        for case in cases:
            model = manifest['models'][case['model']]
            with np.load(checked_file(manifest_path.parent, case), allow_pickle=False) as data:
                inputs = [np.ascontiguousarray(data['input__'+v['name']]) for v in model['inputs']]
                actual = sessions[case['model']].inference(inputs=inputs, data_format=['nchw']*len(inputs))
                if actual is None or len(actual) != len(model['outputs']):
                    raise RuntimeError('Unexpected RKNN output count')
                errors = {v['name']: compare_output(v['name'], got, data['output__'+v['name']])
                          for v, got in zip(model['outputs'], actual)}
            report['cases'].append({'model': case['model'], 'file': case['file'], 'max_abs_errors': errors})
        report['status'] = 'board_numerical_parity_passed'
        # Release the fixture sessions before loading the complete pipeline.
        while sessions:
            _, session = sessions.popitem()
            session.release()
        if manifest.get('pipeline_cases'):
            from .pipeline import RknnMotionPipeline
            report['pipeline_cases'] = []
            with RknnMotionPipeline(manifest_path, backend='lite2') as pipeline:
                for case in manifest['pipeline_cases']:
                    actual = pipeline.generate(**case['request'])
                    with np.load(checked_file(manifest_path.parent, case), allow_pickle=False) as expected:
                        errors = {key: compare_output(key, actual[key], expected[key]) for key in expected.files}
                    report['pipeline_cases'].append({'file': case['file'], 'max_abs_errors': errors})
            report['complete_text_to_motion_pipeline'] = True
        report['board_verified'] = True
    except Exception as exc:
        report.update(status='failed', board_verified=False, error=str(exc))
        raise
    finally:
        release_errors = []
        for session in sessions.values():
            try:
                session.release()
            except Exception as exc:
                release_errors.append(str(exc))
        if release_errors:
            report.update(status='failed', board_verified=False, release_errors=release_errors)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if report['status'] != 'board_numerical_parity_passed':
        raise RuntimeError(f'RKNN cleanup failed; see {report_path}')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'rknn/compiled/manifest.json')
    parser.add_argument('--report', type=Path, default=ROOT/'rknn/board_validation.json')
    args = parser.parse_args()
    print(json.dumps(verify(args.manifest, args.report), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
