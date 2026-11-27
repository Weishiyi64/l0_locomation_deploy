#!/usr/bin/env python3
"""Convert the dense recurrent planner; run on Linux with RKNN-Toolkit2."""
import argparse
import copy
import json
from pathlib import Path
import tempfile


def partition(source, directory):
    import numpy as np
    import onnx
    from onnx import helper as h, numpy_helper as nh
    model = onnx.load(str(source))
    metadata = {p.key: p.value for p in model.metadata_props}
    c = json.loads(metadata['l0_dense_path_contract'])
    if (c['schema'] != 'l0_dense_path_planner' or c['version'] != 2 or
            c['history'] != {'length': 5, 'order': 'oldest_to_newest', 'reset': 'repeat_first_frame'} or
            c['path']['points'] != 8 or c['path']['spacing_m'] != .4 or
            c['path']['sampling_mode'] != 'fixed_step' or c['depth']['height'] != 40 or
            c['depth']['width'] != 64 or not c['recurrent'] or c['planner_hz'] != 5 or
            c['previous_input'] != 'issued_velocity' or c['action_mapping'] != 'residual' or
            c['path']['frame'] != 'base_yaw' or c['depth']['units'] != 'normalized_metric_depth' or
            c['depth']['metric_range_m'] != [.2, 5.] or
            c['observation_order'] != ['state_history', 'latest_path_xy_body_m', 'normalized_metric_depth_40x64']):
        raise ValueError('Unsupported dense recurrent planner contract')
    # Retain only the learned graph, excluding command validation and mapping.
    needed = {'actions', 'next_hidden_state'}
    selected = []
    for node in reversed(model.graph.node):
        if needed.intersection(node.output):
            selected.append(copy.deepcopy(node))
            needed.update(node.input)
    selected.reverse()
    if 'command_state' in needed:
        raise ValueError('Learned graph unexpectedly depends on command_state')
    initial = {i.name: nh.to_array(i) for i in model.graph.initializer}
    weights = [copy.deepcopy(i) for i in model.graph.initializer if i.name in needed]
    nodes = []

    def const(name, value):
        name = '__dense_' + name
        weights.append(nh.from_array(np.asarray(value), name))
        return name

    def op(kind, inputs, name, **attrs):
        name = '__dense_' + name
        nodes.append(h.make_node(kind, inputs, [name], name=name, **attrs))
        return name

    gru_count = 0
    for node in selected:
        if node.op_type != 'GRU':
            nodes.append(node)
            continue
        gru_count += 1
        attrs = {a.name: h.get_attribute_value(a) for a in node.attribute}
        if attrs != {'hidden_size': 128, 'linear_before_reset': 1} or node.input[4]:
            raise ValueError('Expected single forward GRU step, linear_before_reset=1')
        w, r, b = (initial[node.input[i]][0] for i in (1, 2, 3))
        x = op('Reshape', [node.input[0], const('x_shape', np.array([1, w.shape[1]], np.int64))], 'x')
        prev = op('Reshape', [node.input[5], const('h_shape', np.array([1, 128], np.int64))], 'h')
        gates = []
        for index, gate in enumerate(('z', 'r', 'n')):
            wx = op('MatMul', [x, const(gate+'w', w[index*128:(index+1)*128].T.copy())], gate+'wx')
            wx = op('Add', [wx, const(gate+'wb', b[index*128:(index+1)*128].copy())], gate+'wx_b')
            rh = op('MatMul', [prev, const(gate+'r', r[index*128:(index+1)*128].T.copy())], gate+'rh')
            rh = op('Add', [rh, const(gate+'rb', b[(3+index)*128:(4+index)*128].copy())], gate+'rh_b')
            if gate == 'n':
                rh = op('Mul', [gates[1], rh], 'reset_candidate')
            gates.append(op('Tanh' if gate == 'n' else 'Sigmoid',
                            [op('Add', [wx, rh], gate+'sum')], gate+'gate'))
        inverse = op('Sub', [const('one', np.array(1., np.float32)), gates[0]], 'one_minus_z')
        nxt = op('Add', [op('Mul', [inverse, gates[2]], 'candidate'),
                         op('Mul', [gates[0], prev], 'retained')], 'next')
        nodes.append(h.make_node('Reshape', [nxt, const('yh_shape', np.array([1, 1, 128], np.int64))], [node.output[1]]))
        nodes.append(h.make_node('Reshape', [nxt, const('y_shape', np.array([1, 1, 1, 128], np.int64))], [node.output[0]]))
    if gru_count != 1:
        raise ValueError('Expected exactly one GRU')
    inputs = [('observations', [1, 2631]), ('hidden_state', [1, 1, 128])]
    outputs = [('actions', [1, 2]), ('next_hidden_state', [1, 1, 128])]
    graph = h.make_graph(nodes, 'dense_planner_single_step',
        [h.make_tensor_value_info(n, onnx.TensorProto.FLOAT, s) for n, s in inputs],
        [h.make_tensor_value_info(n, onnx.TensorProto.FLOAT, s) for n, s in outputs], weights)
    result = h.make_model(graph, opset_imports=copy.deepcopy(model.opset_import))
    result.ir_version = model.ir_version
    onnx.checker.check_model(result)
    path = Path(directory) / 'dense_single_step.onnx'
    onnx.save(result, str(path))
    return path, inputs, outputs, metadata


def heading_speed(error, speed):
    import numpy as np
    fade = np.clip((1.-abs(error))/(1.-np.pi/12), 0., 1.)
    magnitude = speed-(speed-min(speed, .3))*np.expm1(3.*fade)/np.expm1(3.)
    return 0. if abs(error) <= np.pi/12 else float(np.copysign(magnitude, error))


def verify(rknn, source, extracted, args):
    import numpy as np
    import onnxruntime as ort
    reference = ort.InferenceSession(str(source), providers=['CPUExecutionProvider'])
    lowered = ort.InferenceSession(str(extracted), providers=['CPUExecutionProvider'])
    if rknn.init_runtime() != 0:
        raise RuntimeError('RKNN simulator initialization failed')
    rng = np.random.default_rng(0)
    hidden = np.zeros((1, 1, 128), np.float32)
    actual_hidden = hidden.copy()
    previous = np.zeros(2, np.float32)
    history = None
    worst = 0.
    invariants = {k: False for k in ('nav_vy_zero', 'face_vx_vy_zero', 'hold_all_zero')}
    for step in range(args.samples):
        # Include persistent sequences, task resets and normal Hold transitions.
        if step % 50 == 0:
            hidden.fill(0); actual_hidden.fill(0); previous.fill(0); history = None
        mode = (step // 10) % 3
        task = np.zeros(5, np.float32)
        if mode == 0:
            task[:3] = [1., .2, .35]
        elif mode == 1:
            task[3:] = [.4, 1.57]
        yaw = heading_speed(float(task[3]), float(task[4])) if mode == 1 else 0.
        frame = np.r_[rng.uniform(-.2, .2, 3), [0., 0., -1.], previous, task[2], task[3], yaw].astype(np.float32)
        history = np.tile(frame, (5, 1)) if history is None else np.vstack([history[1:], frame])
        path = np.array([[.4*i, .1] for i in range(1, 9)], np.float32) if mode == 0 else np.zeros((8, 2), np.float32)
        depth = rng.uniform(.04, 1., 2560).astype(np.float32)
        depth[rng.random(2560) < .2] = 0.
        obs = np.r_[history.ravel(), path.ravel(), depth].reshape(1, 2631).astype(np.float32)
        command = np.r_[frame[:8], task].reshape(1, 13).astype(np.float32)
        velocity, actions, next_hidden = reference.run(['velocity_command', 'actions', 'next_hidden_state'],
            {'observations': obs, 'command_state': command, 'hidden_state': hidden})
        exact = lowered.run(['actions', 'next_hidden_state'], {'observations': obs, 'hidden_state': hidden})
        if any(np.max(np.abs(a-b)) > 2e-5 for a, b in zip(exact, [actions, next_hidden])):
            raise RuntimeError('GRU lowering differs from original ONNX')
        actual = rknn.inference(inputs=[obs, actual_hidden])
        if len(actual) != 2:
            raise RuntimeError('RKNN output count mismatch')
        actual_actions = np.asarray(actual[0], np.float32).reshape(1, 2)
        actual_hidden = np.asarray(actual[1], np.float32).reshape(1, 1, 128)
        candidate = np.zeros((1, 3), np.float32)
        if mode == 0:
            candidate[0, 0] = np.clip(task[2]+actual_actions[0, 0], -1., 1.2)
        if mode != 2:
            candidate[0, 2] = np.clip(yaw+actual_actions[0, 1], -1.57, 1.57)
        velocity = np.clip(velocity, [-1., 0., -1.57], [1.2, 0., 1.57])
        for expected, value in ((actions, actual_actions), (next_hidden, actual_hidden), (velocity, candidate)):
            if not np.isfinite(value).all():
                raise RuntimeError('Non-finite RKNN output')
            error = float(np.max(np.abs(expected-value)))
            worst = max(worst, error)
            if error > args.max_abs_error:
                raise RuntimeError(f'Continuous sequence error {error} exceeds tolerance')
            denominator = float(np.linalg.norm(expected)*np.linalg.norm(value))
            if denominator > 1e-10 and float(np.sum(expected*value)/denominator) < args.cosine_threshold:
                raise RuntimeError('Continuous sequence cosine check failed')
        keys = ('nav_vy_zero', 'face_vx_vy_zero', 'hold_all_zero')
        zero_indices = ([1], [0, 1], [0, 1, 2])[mode]
        if any(abs(float(v[0, i])) > 1e-6 for v in (velocity, candidate) for i in zero_indices):
            raise RuntimeError('Velocity interlock mismatch')
        invariants[keys[mode]] = True
        hidden = next_hidden
        previous = candidate[0, [0, 2]].copy()
    return {'performed': True, 'samples': args.samples, 'maximum_absolute_error': worst,
            'max_abs_error': args.max_abs_error, 'cosine_threshold': args.cosine_threshold,
            'invariants': invariants, 'continuous_sequence': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('onnx', type=Path)
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--max-abs-error', type=float, default=.05)
    parser.add_argument('--cosine-threshold', type=float, default=.999)
    args = parser.parse_args()
    if args.samples < 200 or not 0 < args.max_abs_error <= .05 or not .999 <= args.cosine_threshold <= 1.:
        parser.error('Require >=200 steps, error <=0.05, cosine >=0.999')
    source = args.onnx.resolve()
    from rknn.api import RKNN
    rknn = RKNN(verbose=False)
    try:
        with tempfile.TemporaryDirectory(prefix='dense_planner_') as directory:
            graph, inputs, outputs, _metadata = partition(source, directory)
            if rknn.config(target_platform='rk3588') != 0:
                raise RuntimeError('RKNN config failed')
            if rknn.load_onnx(model=str(graph), inputs=[n for n, _ in inputs],
                              input_size_list=[s for _, s in inputs], outputs=[n for n, _ in outputs]) != 0:
                raise RuntimeError('RKNN load failed')
            if rknn.build(do_quantization=False) != 0: raise RuntimeError('RKNN build failed')
            verification = verify(rknn, source, graph, args)
            artifact = Path(directory)/'planner.rknn'
            if rknn.export_rknn(str(artifact)) != 0: raise RuntimeError('RKNN export failed')
            destination = source.with_suffix('.rknn')
            # Only publish a usable artifact after sequence verification passes.
            destination.write_bytes(artifact.read_bytes())
            print(f'Verified model: {destination}')
            print(json.dumps(verification, indent=2))
    finally:
        rknn.release()


if __name__ == '__main__':
    main()
