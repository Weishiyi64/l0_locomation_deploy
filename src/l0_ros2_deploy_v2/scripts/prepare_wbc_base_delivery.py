#!/usr/bin/env python3
"""Build five independent fixed-actor source bundles with the same complete motion catalog.

Runs on the export host. RKNN compilation/install is a separate build_wbc_base.py step.
The four catalog checkpoints did not supply numerical PD/action scaling; these bundles
explicitly retain the existing deployment actuation contract, checked against SoftSM.
"""
import argparse
import copy
import io
import json
from pathlib import Path
import re
import shutil
import zipfile

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import torch
import yaml

from export_wbc_base_actor import export_actor, sha
from convert_onnx_to_rknn_rk3588_wbc_reference import check_reference_model, REFERENCE_OUTPUTS


def extract(archive, destination):
    destination.mkdir()
    with zipfile.ZipFile(archive) as source:
        for member in source.infolist():
            path = (destination / member.filename).resolve()
            if not path.is_relative_to(destination.resolve()):
                raise ValueError('Archive member escapes destination')
            if member.is_dir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(source.read(member))


def export_reference(source, output, joint_names):
    with np.load(source, allow_pickle=False) as motion:
        source_names = motion['joint_names'].astype(str).tolist()
        bodies = motion['body_names'].astype(str).tolist()
        if len(set(source_names)) != 21 or set(source_names) != set(joint_names):
            raise ValueError('Reference joint names disagree with deployment: ' + str(source))
        if len(set(bodies)) != 23 or 'base_link' not in bodies or float(motion['fps'].reshape(-1)[0]) != 50:
            raise ValueError('Reference requires 23 named bodies and 50 Hz')
        order = [source_names.index(name) for name in joint_names]
        arrays = {name: np.asarray(motion[name], dtype=np.float32).copy() for name in REFERENCE_OUTPUTS}
    for name in ('joint_pos', 'joint_vel'):
        arrays[name] = arrays[name][:, order]
    count = len(arrays['joint_pos'])
    for name, shape in REFERENCE_OUTPUTS.items():
        if arrays[name].shape != (count, *shape[1:]) or not np.isfinite(arrays[name]).all():
            raise ValueError('Invalid reference table: ' + name)
    if not 0 < count <= 100000 or not np.allclose(np.linalg.norm(arrays['body_quat_w'], axis=-1), 1, atol=1e-3):
        raise ValueError('Invalid reference frame count/quaternions')
    constants = [numpy_helper.from_array(np.array([-1], dtype=np.int64), 'index_shape'),
                 numpy_helper.from_array(np.array(0, dtype=np.int64), 'min_frame'),
                 numpy_helper.from_array(np.array(count - 1, dtype=np.int64), 'max_frame')]
    nodes = [helper.make_node('Cast', ['time_step'], ['integer_step'], to=TensorProto.INT64),
             helper.make_node('Reshape', ['integer_step', 'index_shape'], ['flat_step']),
             helper.make_node('Clip', ['flat_step', 'min_frame', 'max_frame'], ['frame'])]
    for name in REFERENCE_OUTPUTS:
        constants.append(numpy_helper.from_array(arrays[name], name + '_table'))
        nodes.append(helper.make_node('Gather', [name + '_table', 'frame'], [name], axis=0))
    graph = helper.make_graph(nodes, 'wbc_reference',
        [helper.make_tensor_value_info('time_step', TensorProto.FLOAT, [1, 1])],
        [helper.make_tensor_value_info(k, TensorProto.FLOAT, v) for k, v in REFERENCE_OUTPUTS.items()], constants)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)], ir_version=8)
    helper.set_model_props(model, {'fps': '50', 'num_frames': str(count), 'joint_names': ','.join(joint_names),
        'body_names': ','.join(bodies), 'quaternion_order': 'wxyz',
        'time_step_semantics': 'zero_based_frame_index_clamped', 'source_npz_sha256': sha(source)})
    onnx.save(model, str(output))
    check_reference_model(output)
    # Check actual lookup values and both clamped boundaries against reordered source data.
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(output), sess_options=options, providers=['CPUExecutionProvider'])
    for frame in (-8, 0, count // 2, count - 1, count + 4):
        actual = session.run(list(REFERENCE_OUTPUTS), {'time_step': np.array([[frame]], dtype=np.float32)})
        for name, value in zip(REFERENCE_OUTPUTS, actual):
            np.testing.assert_array_equal(value, arrays[name][[min(max(frame, 0), count - 1)]])
    return count, bodies.index('base_link')


def inside(root, relative):
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('Asset escapes source bundle: ' + relative)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--soft-zip', type=Path, required=True)
    parser.add_argument('--actors-zip', type=Path, required=True)
    parser.add_argument('--old-bundle', type=Path, required=True, help='86-action LoRA bundle; only references/configs reused')
    parser.add_argument('--output', type=Path, required=True, help='New directory; never overwrite existing delivery')
    args = parser.parse_args()
    torch.set_num_threads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = args.output / 'source_inputs'
    inputs.mkdir()
    extract(args.soft_zip, inputs / 'soft')
    extract(args.actors_zip, inputs / 'actors')
    soft = next((inputs / 'soft').iterdir())
    contract = json.loads((soft / 'models/deploy_config.json').read_text())
    old = json.loads((args.old_bundle / 'base_manifest.json').read_text())
    template = yaml.safe_load(inside(args.old_bundle, old['actions'][0]['config']).read_text())
    joints = template['RobotCfg']['joint_order_mj']
    if joints != contract['policy_joint_names']:
        raise ValueError('SoftSM and existing deployment have different joint order')
    for key, expected in [('action_scale', contract['action_scale']),
                          ('default_joint_angles', [contract['default_joint_pos'][n] for n in joints]),
                          ('kps', [contract['joint_kp'][n] for n in joints]),
                          ('kds', [contract['joint_kd'][n] for n in joints])]:
        np.testing.assert_array_equal(np.asarray(template['RobotCfg'][key], np.float32), np.asarray(expected, np.float32))
    template['RobotCfg']['policy_name'] = 'fixed_actor'
    common = args.output / 'common_references'
    common.mkdir()
    actions = []
    for item in old['actions']:
        action_id = item['action_id']
        cfg = yaml.safe_load(inside(args.old_bundle, item['config']).read_text())
        for key in ('joint_order_mj', 'action_scale', 'default_joint_angles', 'kps', 'kds'):
            if cfg['RobotCfg'][key] != template['RobotCfg'][key]:
                raise ValueError('Existing action actuation mismatch: ' + action_id)
        cfg['RobotCfg'] = copy.deepcopy(template['RobotCfg'])
        directory = common / 'actions' / action_id
        directory.mkdir(parents=True)
        reference = inside(args.old_bundle, item['reference_onnx'])
        if sha(reference) != item['reference_onnx_sha256']:
            raise ValueError('Existing reference hash mismatch: ' + action_id)
        _, _, meta = check_reference_model(reference)
        if meta['joint_names'].split(',') != joints:
            raise ValueError('Existing reference joint order mismatch')
        shutil.copy2(reference, directory / 'reference_motion.onnx')
        (directory / 'rl_config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        # Retain existing compiled references only if provenance matches this ONNX.
        rknn = args.old_bundle / item['reference_rknn']
        record = rknn.with_suffix('.rknn.json')
        if rknn.is_file() and record.is_file():
            data = json.loads(record.read_text())
            if (data.get('source_sha256') == sha(reference) and data.get('rknn_sha256') == sha(rknn)
                    and data.get('precision') == 'float16' and data.get('do_quantization') is False):
                shutil.copy2(rknn, directory / 'reference_motion.rknn')
                shutil.copy2(record, directory / 'reference_motion.rknn.json')
        actions.append(action_id)
    for source in sorted((soft / 'motions').glob('*.npz')):
        action_id = re.sub(r'^\d+_', '', source.stem)
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', action_id) or action_id in actions:
            raise ValueError('Invalid/duplicate action ID: ' + action_id)
        directory = common / 'actions' / action_id
        directory.mkdir(parents=True)
        count, root = export_reference(source, directory / 'reference_motion.onnx', joints)
        shutil.copy2(source, directory / 'source_motion.npz')
        cfg = copy.deepcopy(template)
        cfg['WbcCfg'].update(motion_id=action_id, reference_num_frames=count, reference_root_body_index=root)
        (directory / 'rl_config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        actions.append(action_id)
    entries = []
    for action_id in actions:
        prefix = 'actions/' + action_id + '/'
        entries.append({'action_id': action_id, 'config': prefix + 'rl_config.yaml',
            'config_sha256': sha(common / prefix / 'rl_config.yaml'),
            'reference_onnx': prefix + 'reference_motion.onnx',
            'reference_onnx_sha256': sha(common / prefix / 'reference_motion.onnx'),
            'reference_rknn': prefix + 'reference_motion.rknn'})
    candidates = [('soft500w_80k', soft / 'models/actor500w_80000.pt', soft / 'models',
                   soft / 'models/actor500w_80000.torchscript.pt')]
    for name in ('100w_s0', '300w_s0', '500w_s0', 'supermimic_s0'):
        directory = inputs / 'actors' / name
        candidates.append((name, next(directory.glob('actor_iter*.pt')), directory, None))
    summary = []
    for name, checkpoint, metadata, original in candidates:
        bundle = args.output / ('wbc_base_' + name)
        shutil.copytree(common, bundle)
        model_dir = bundle / 'model'
        model_dir.mkdir()
        shutil.copy2(checkpoint, model_dir / 'actor.pt')
        for source in metadata.glob('*.json'):
            shutil.copy2(source, model_dir / source.name)
        if original:
            shutil.copy2(original, model_dir / original.name)
        report, catalog = export_actor(checkpoint, model_dir / 'actor.onnx', torchscript=original)
        (model_dir / 'actor_architecture_catalog.json').write_text(json.dumps(catalog, indent=2) + '\n')
        manifest = {'format': 'wbc_base_graph_v1', 'kind': 'policy', 'target_precision': 'float16',
            'model': {'onnx': 'model/actor.onnx', 'rknn': 'model/actor.rknn', 'sha256': sha(model_dir / 'actor.onnx')},
            'checkpoint_sha256': sha(checkpoint), 'actions': entries,
            'actuation_source': 'existing deployment RobotCfg, numerically checked against SoftSM deploy_config.json',
            'catalog_checkpoint_actuation_note': 'Four catalog PTs omit numeric PD/scale; existing deployment values retained',
            'input_contract': {'policy': [1, 399], 'command_short': [1, 240], 'action': [1, 21]},
            'runtime_validation': 'not_run'}
        (bundle / 'base_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        summary.append({'bundle': bundle.name, 'actions': len(entries), **report})
        print(bundle.name, 'actions=', len(entries), 'parameters=', report['parameter_count'], flush=True)
    (args.output / 'EXPORT_SUMMARY.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
