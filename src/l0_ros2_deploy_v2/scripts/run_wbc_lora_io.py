#!/usr/bin/env python3
"""Fixed-input WBC numerical probe; board inference uses the deployed C++ loader.

prepare freezes selected rows of replay_models_10ms.py's audited replay.
local repeats its FP16-asset ONNX baseline. board invokes rl_controller's
--wbc-io-probe mode, which shares WbcLoraAssets/RknnModel with the live worker.
No Python RKNN implementation or motor/ROS publishing is involved.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from wbc_lora_assets import (ID_PATTERN, asset_path, check_graph, graph_feeds,
                            load_adapter, load_graph_manifest, read_json, sha256)


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def new_directory(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path


def prepare(args):
    replay = Path(args.replay).resolve()
    metadata = read_json(replay.with_suffix('.json'))
    segments_path = replay.parent / 'segments.json'
    if sha256(segments_path) != metadata['input_files_sha256']['segments.json']:
        raise ValueError('Segments changed since the baseline replay')
    segments = json.loads(segments_path.read_text())
    graph = load_graph_manifest(args.manifest)
    if (graph['model']['sha256'] != metadata['base_onnx_sha256'] or
            graph['base_checkpoint_sha256'] != metadata['base_checkpoint_sha256']):
        raise ValueError('Model identity differs from the baseline replay')
    catalog = {x['action_id']: x for x in graph['actions']}
    root = Path(args.manifest).resolve().parent
    data = np.load(replay, allow_pickle=False)
    indices, cases, assets, scales = [], [], {}, []
    for block, action_id in enumerate(args.actions):
        if not ID_PATTERN.fullmatch(action_id):
            raise ValueError(f'Invalid action ID: {action_id}')
        si, segment = next((i, s) for i, s in enumerate(segments) if s['action_id'] == action_id)
        start, end = segment['start_index'], segment['end_index']
        # Keep preparation plus samples after the approximate initial history.
        if args.samples < 2 or args.samples - 1 > end - start - 25:
            raise ValueError('Need 2..(segment length - 24) samples per action')
        chosen = np.r_[start, np.linspace(start + 25, end - 1, args.samples - 1, dtype=int)]
        config_path = asset_path(root, catalog[action_id]['config'])
        if sha256(config_path) != metadata['action_assets'][action_id]['config_sha256']:
            raise ValueError(f'Config changed: {action_id}')
        cfg = yaml.safe_load(config_path.read_text())['RobotCfg']
        if list(cfg['joint_order_isaac']) != data['joint_names'].tolist():
            raise ValueError('Joint order differs from baseline outputs')
        assets[action_id] = metadata['action_assets'][action_id]
        for index in chosen:
            cutoff = int(data['command_stamp'][index]) - 10_000_000
            if int(data['observation_stamp'][index]) > cutoff or int(data['segment_id'][index]) != si:
                raise ValueError('Replay row violates the 10 ms cutoff or action identity')
            cases.append(dict(case_index=len(cases), action_id=action_id, block=block,
                              replay_index=int(index), segment=si,
                              command_stamp_ns=int(data['command_stamp'][index]),
                              observation_stamp_ns=int(data['observation_stamp'][index]),
                              observation_cutoff_ns=cutoff,
                              reference_frame=int(data['reference_frame'][index])))
            indices.append(index)
            scales.append(cfg['action_scale'])
    if len(cases) > 8192:
        raise ValueError('At most 8192 cases are supported')
    arrays = {key: data[key][indices] for key in ('policy', 'command_short', 'action_fp16', 'action_fp32')}
    arrays['action_scale'] = np.asarray(scales, dtype=np.float64)
    arrays['joint_names'] = data['joint_names']
    if not all(np.isfinite(v).all() for k, v in arrays.items() if k != 'joint_names'):
        raise ValueError('Non-finite input or baseline')
    output = new_directory(args.output_dir)
    for key in ('policy', 'command_short'):
        np.asarray(arrays[key], dtype='<f4').tofile(output / (key + '.f32'))
    np.savez_compressed(output / 'baseline.npz', **arrays)
    request = dict(format='wbc_policy_io_cases_v1', cases=cases,
                   input_rule=metadata['input_rule'],
                   scope='Frozen inputs for policy-only numerical comparison; not live input-builder validation.',
                   fp16_definition=metadata['model_fp16'], fp32_definition=metadata['model_fp32'],
                   reconstruction_caveats=metadata['caveats'],
                   base_onnx_sha256=metadata['base_onnx_sha256'],
                   base_checkpoint_sha256=metadata['base_checkpoint_sha256'], action_assets=assets,
                   source_replay_sha256=sha256(replay), source_metadata_sha256=sha256(replay.with_suffix('.json')),
                   files_sha256={name: sha256(output / name) for name in
                                 ('policy.f32', 'command_short.f32', 'baseline.npz')})
    save_json(output / 'cases.json', request)
    print(f'Prepared {len(cases)} cases in {output}', flush=True)


def load_cases(directory):
    directory = Path(directory).resolve()
    request = read_json(directory / 'cases.json')
    if request['format'] != 'wbc_policy_io_cases_v1' or not 0 < len(request['cases']) <= 8192:
        raise ValueError('Invalid case package')
    for name in ('policy.f32', 'command_short.f32', 'baseline.npz'):
        if sha256(directory / name) != request['files_sha256'][name]:
            raise ValueError(f'Case checksum mismatch: {name}')
    with np.load(directory / 'baseline.npz', allow_pickle=False) as data:
        arrays = dict(data)
    count = len(request['cases'])
    for key, width in (('policy', 399), ('command_short', 240), ('action_fp16', 21),
                       ('action_fp32', 21), ('action_scale', 21)):
        if arrays[key].shape != (count, width) or not np.isfinite(arrays[key]).all():
            raise ValueError(f'Invalid tensor: {key}')
    if arrays['joint_names'].shape != (21,):
        raise ValueError('Invalid joint names')
    for key in ('policy', 'command_short'):
        binary = np.fromfile(directory / (key + '.f32'), dtype='<f4')
        if not np.array_equal(binary, arrays[key].ravel()):
            raise ValueError(f'Binary input differs from baseline: {key}')
    for i, item in enumerate(request['cases']):
        if item['case_index'] != i or not ID_PATTERN.fullmatch(item['action_id']):
            raise ValueError('Invalid case index/action ID')
    return request, arrays


def validate_assets(manifest, request):
    manifest = Path(manifest).resolve()
    graph = load_graph_manifest(manifest)
    if (graph['model']['sha256'] != request['base_onnx_sha256'] or
            graph['base_checkpoint_sha256'] != request['base_checkpoint_sha256']):
        raise ValueError('Requested baseline and deployed model differ')
    catalog = {x['action_id']: x for x in graph['actions']}
    for action_id in {c['action_id'] for c in request['cases']}:
        item = catalog[action_id]
        adapter = asset_path(manifest.parent, item['adapter_manifest'])
        load_adapter(adapter, graph, action_id)  # Reuse full tensor/hash validation.
        expected = request['action_assets'][action_id]
        if read_json(adapter)['weights']['sha256'] != expected['weights_fp16_sha256']:
            raise ValueError(f'Adapter differs from baseline: {action_id}')
        if sha256(asset_path(manifest.parent, item['config'])) != expected['config_sha256']:
            raise ValueError(f'Action scale/config differs from baseline: {action_id}')
    return graph, catalog


def error_stats(error):
    error = np.asarray(error, dtype=np.float64)
    return dict(rmse=float(np.sqrt(np.mean(error ** 2))), max_abs=float(np.max(abs(error))),
                p99_abs=float(np.percentile(abs(error), 99)))


def compare(output, request, arrays, actions):
    if actions.shape != arrays['action_fp16'].shape or not np.isfinite(actions).all():
        raise ValueError('Incomplete/non-finite inference output; retain raw logs for diagnosis')
    residual16 = actions.astype(np.float64) - arrays['action_fp16']
    residual32 = actions.astype(np.float64) - arrays['action_fp32']
    scaled16 = residual16 * arrays['action_scale']
    scaled32 = residual32 * arrays['action_scale']
    np.savez_compressed(output / 'outputs.npz', action=actions, action_fp16=arrays['action_fp16'],
                        action_fp32=arrays['action_fp32'], residual_fp16=residual16,
                        residual_fp32=residual32, residual_fp16_scaled_rad=scaled16,
                        residual_fp32_scaled_rad=scaled32, joint_names=arrays['joint_names'])
    report = dict(samples=len(actions), output='Raw policy action before clipping/scaling',
                  scaled_residual='(output - baseline) * action_scale, radians before clipping',
                  vs_local_fp16=dict(raw_action=error_stats(residual16), scaled_rad=error_stats(scaled16)),
                  vs_original_fp32=dict(raw_action=error_stats(residual32), scaled_rad=error_stats(scaled32)))
    report['per_action'] = {}
    for action_id in dict.fromkeys(c['action_id'] for c in request['cases']):
        mask = np.array([c['action_id'] == action_id for c in request['cases']])
        report['per_action'][action_id] = dict(samples=int(mask.sum()),
            vs_fp16_scaled_rad=error_stats(scaled16[mask]), vs_fp32_scaled_rad=error_stats(scaled32[mask]))
    repeats, first = [], {}
    for i, c in enumerate(request['cases']):
        key = (c['action_id'], c['replay_index'])
        if key in first:
            j = first[key]
            if not all(np.array_equal(arrays[k][i], arrays[k][j]) for k in ('policy', 'command_short')):
                raise ValueError('Repeated case has different input')
            repeats.append(actions[i].astype(np.float64) - actions[j])
        else:
            first[key] = i
    report['repeated_identical_inputs'] = dict(pairs=len(repeats),
        raw_action=error_stats(repeats) if repeats else None)
    with (output / 'per_joint.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case_index', 'action_id', 'replay_index', 'joint', 'output', 'local_fp16',
                         'original_fp32', 'residual_fp16', 'residual_fp32',
                         'residual_fp16_scaled_rad', 'residual_fp32_scaled_rad'])
        for i, c in enumerate(request['cases']):
            for j, name in enumerate(arrays['joint_names']):
                writer.writerow([i, c['action_id'], c['replay_index'], name, actions[i, j],
                                 arrays['action_fp16'][i, j], arrays['action_fp32'][i, j],
                                 residual16[i, j], residual32[i, j], scaled16[i, j], scaled32[i, j]])
    return report


def run_local(args, output, request, arrays, graph, catalog):
    import onnxruntime as ort
    root = Path(args.manifest).resolve().parent
    model = asset_path(root, graph['model']['onnx'])
    check_graph(model, args.manifest)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
    actions, active, parameters = [], None, None
    with (output / 'actions.f32').open('wb') as binary, (output / 'frames.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case_index', 'action_id', 'inference_ms'])
        for i, c in enumerate(request['cases']):
            if active != c['action_id']:
                active = c['action_id']
                parameters = load_adapter(asset_path(root, catalog[active]['adapter_manifest']), graph, active)
            feeds = graph_feeds(graph, {k: arrays[k][i] for k in ('policy', 'command_short')}, parameters)
            started = time.perf_counter()
            action = np.asarray(session.run(['action'], feeds)[0], dtype='<f4').reshape(21)
            elapsed_ms = (time.perf_counter() - started) * 1000
            if not np.isfinite(action).all():
                raise ValueError(f'Non-finite local output at case {i}')
            binary.write(action.tobytes())
            binary.flush()
            writer.writerow([i, active, elapsed_ms])
            stream.flush()
            actions.append(action)
    return np.asarray(actions), dict(onnxruntime=ort.__version__, providers=session.get_providers(),
        arithmetic='CPU FP32 computation with FP16 stored weights; not native RKNN FP16')


def run_board(args, output, request, arrays, graph, catalog):
    executable = Path(args.executable).resolve(strict=True)
    command = [str(executable), '--wbc-io-probe', str(Path(args.manifest).resolve()),
               str(Path(args.cases).resolve()), str(output / 'resident')]
    model_root = Path(args.manifest).resolve().parent
    model_path = asset_path(model_root, graph['model']['rknn'])
    invocation = dict(command=command, executable_sha256=sha256(executable),
                      model_sha256=sha256(model_path),
                      conversion_record_sha256=sha256(Path(str(model_path) + '.json')))
    save_json(output / 'invocation.json', invocation)
    # The regular rl_controller executable retains its installed RPATH/runtime.
    # No LD_LIBRARY_PATH override and no alternative Python RKNN loader.
    with (output / 'runtime.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            returncode = process.wait(timeout=args.timeout_seconds)
        except BaseException:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            raise
    if returncode:
        raise RuntimeError(f'rl_controller returned {returncode}; see runtime.log and resident/runtime.yaml')
    runtime = yaml.safe_load((output / 'resident/runtime.yaml').read_text())
    count = len(request['cases'])
    switches = 1 + sum(a['action_id'] != b['action_id'] for a, b in zip(request['cases'], request['cases'][1:]))
    if (runtime['status'] != 'completed' or runtime['completed_samples'] != count or
            runtime['context_initializations'] != 1 or runtime['runs'] != count or
            runtime['parameter_prepares'] != switches or runtime['commits'] != switches or runtime['rollbacks'] != 0):
        raise ValueError('Incomplete run or unexpected resident context/adapter counters')
    with (output / 'resident/frames.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != count or any(int(row['case_index']) != i or row['action_id'] != request['cases'][i]['action_id']
                                 for i, row in enumerate(rows)):
        raise ValueError('Frame log does not match requested cases')
    raw = np.fromfile(output / 'resident/actions.f32', dtype='<f4')
    if raw.size != count * 21:
        raise ValueError('Output byte count mismatch')
    return raw.reshape(count, 21), runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    prepare_parser = sub.add_parser('prepare', help='Freeze the audited 10 ms replay inputs and FP16/FP32 baselines')
    prepare_parser.add_argument('--replay', required=True)
    prepare_parser.add_argument('--actions', nargs='+', default=['L0_peace_new_02_A', 'L0_angry_7', 'L0_strange_1', 'L0_peace_new_02_A'])
    prepare_parser.add_argument('--samples', type=int, default=33)
    for mode in ('local', 'board'):
        p = sub.add_parser(mode, help='Run fixed inputs using ' + ('ONNX CPU baseline' if mode == 'local' else 'the existing C++ RKNN loader'))
        p.add_argument('--cases', required=True)
        if mode == 'board':
            p.add_argument('--executable', required=True, help='Installed RKNN rl_controller with --wbc-io-probe')
            p.add_argument('--timeout-seconds', type=float, default=300)
    for p in sub.choices.values():
        p.add_argument('--manifest', required=True, help='base_manifest.json')
        p.add_argument('--output-dir', required=True, help='New directory; existing results are never overwritten')
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare(args)
        return 0
    output = new_directory(args.output_dir)
    report = dict(format='wbc_policy_io_result_v1', mode=args.mode, status='initializing',
                  python=sys.executable, python_version=platform.python_version(), numpy=np.__version__,
                  script_sha256=sha256(Path(__file__)), asset_helper_sha256=sha256(Path(__file__).with_name('wbc_lora_assets.py')))
    save_json(output / 'report.json', report)
    try:
        if args.mode == 'board' and (not np.isfinite(args.timeout_seconds) or args.timeout_seconds <= 0):
            raise ValueError('Timeout must be finite and positive')
        request, arrays = load_cases(args.cases)
        graph, catalog = validate_assets(args.manifest, request)
        report.update(cases_sha256=sha256(Path(args.cases) / 'cases.json'),
                      manifest_sha256=sha256(args.manifest), input_files_sha256=request['files_sha256'],
                      numerical_acceptance='Metrics only; no engineering tolerance has been specified.')
        runner = run_local if args.mode == 'local' else run_board
        actions, runtime = runner(args, output, request, arrays, graph, catalog)
        report.update(runtime=runtime, comparison=compare(output, request, arrays, actions), status='completed')
    except (Exception, KeyboardInterrupt) as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        print(report['error'], file=sys.stderr)
    save_json(output / 'report.json', report)
    print(json.dumps(dict(status=report['status'], output_dir=str(output), comparison=report.get('comparison')), indent=2))
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
