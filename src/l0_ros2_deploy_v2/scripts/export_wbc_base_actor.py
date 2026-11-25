#!/usr/bin/env python3
"""Export the delivered SoftSM / SuperMimic catalog actors, without training dependencies.

The model consumes physical policy[1,399], command_short[1,240] and returns
raw action[1,21] in deployment joint order. Normalization stays inside ONNX.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def activation(name):
    return {'mish': nn.Mish, 'silu': nn.SiLU, 'tanh': nn.Tanh, 'identity': nn.Identity}[name]()


def mlp(spec, eps):
    layers = []
    dims = [spec['input_dim'], *spec['hidden_dims'], spec['output_dim']]
    if dims != spec['linear_dims']:
        raise ValueError('Catalog linear_dims disagree with hidden dimensions')
    for i, (din, dout) in enumerate(zip(dims[:-1], dims[1:])):
        layers.append(nn.Linear(din, dout))
        if i < len(dims) - 2:
            if spec['layer_norm']:
                layers.append(nn.LayerNorm(dout, eps=eps))
            layers.append(activation(spec['activation']))
    if spec['output_activation'] != 'identity':
        layers.append(activation(spec['output_activation']))
    return nn.Sequential(*layers)


class FrozenNormalization(nn.Module):
    def __init__(self, variance_floor):
        super().__init__()
        self.variance_floor = variance_floor
        for key, width in [('policy', 399), ('command_short', 240)]:
            for suffix in ('sum', 'ssq', 'divisor'):
                self.register_buffer(f'{key}_{suffix}', torch.zeros(width))
            self.register_buffer(f'{key}_count', torch.zeros(()))

    def normalize(self, value, key):
        count = getattr(self, key + '_count')
        mean = getattr(self, key + '_sum') / count
        variance = getattr(self, key + '_ssq') / count - mean.square()
        return (value - mean) / variance.clamp_min(self.variance_floor).sqrt()


class CatalogActor(nn.Module):
    def __init__(self, catalog, preset, profile):
        super().__init__()
        if profile != 'mimic_lite_vecnorm':
            raise ValueError('Only the delivered per-feature mean/std profile is supported')
        spec = catalog['presets'][preset]
        self.encoder_inputs = spec['encoder']['inputs']
        self.decoder_inputs = spec['decoder']['inputs']
        widths = {'policy': 399, 'command_short': 240, 'latent': spec['encoder']['output_dim']}
        for key, names in [('encoder', self.encoder_inputs), ('decoder', self.decoder_inputs)]:
            if sum(widths[n] for n in names) != spec[key]['input_dim']:
                raise ValueError('Catalog input wiring width mismatch: ' + key)
        if 'latent' in self.encoder_inputs or spec['decoder']['output_dim'] != 21:
            raise ValueError('Unsupported catalog wiring/output width')
        eps = catalog['architecture_conventions']['layer_norm_eps']
        self.encoder = mlp(spec['encoder'], eps)
        self.decoder = mlp(spec['decoder'], eps)
        self.normalization = FrozenNormalization(catalog['normalization_profiles'][profile]['variance_floor'])
        # Included for strict checkpoint loading; forward is already in native deployment order.
        self.register_buffer('native_from_sim', torch.zeros(21, dtype=torch.long))
        self.register_buffer('sim_from_native', torch.zeros(21, dtype=torch.long))

    def forward(self, policy, command_short):
        values = {'policy': self.normalization.normalize(policy, 'policy'),
                  'command_short': self.normalization.normalize(command_short, 'command_short')}
        values['latent'] = self.encoder(torch.cat([values[k] for k in self.encoder_inputs], dim=-1))
        return self.decoder(torch.cat([values[k] for k in self.decoder_inputs], dim=-1))


def load_actor(checkpoint, catalog_path=None):
    payload = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if payload['format'] == 'softsm_l0_v5_actor500w_deploy_v1':
        config, state = payload['policy_config'], payload['actor_state_dict']
        catalog = payload['architecture_catalog']
    elif payload['format'] == 'supermimic_actor_only_v1':
        config = payload['config']['policy']
        if any(not k.startswith('catalog_actor.') for k in payload['policy']):
            raise ValueError('Unexpected weights outside catalog_actor')
        state = {k.removeprefix('catalog_actor.'): v for k, v in payload['policy'].items()}
        catalog = json.loads((catalog_path or checkpoint.parent / 'actor_architecture_catalog.json').read_text())
    else:
        raise ValueError('Unsupported actor checkpoint format')
    if config.get('action_bound', 0) != 0:
        raise ValueError('This export contract requires unbounded raw actor outputs')
    model = CatalogActor(catalog, config['architecture_preset'], config['normalization_profile'])
    model.load_state_dict(state, strict=True)
    for key, value in model.state_dict().items():
        if not torch.isfinite(value).all():
            raise ValueError('Non-finite checkpoint tensor: ' + key)
    for key in ('policy', 'command_short'):
        if getattr(model.normalization, key + '_count').item() <= 0:
            raise ValueError('Normalization statistics are uninitialized: ' + key)
    for mapping in (model.native_from_sim, model.sim_from_native):
        if sorted(mapping.tolist()) != list(range(21)):
            raise ValueError('Invalid joint permutation in checkpoint')
    if not torch.equal(model.native_from_sim[model.sim_from_native], torch.arange(21)):
        raise ValueError('Checkpoint joint permutations are not inverses')
    return model.eval(), payload, catalog


def export_actor(checkpoint, output, catalog_path=None, torchscript=None):
    model, payload, catalog = load_actor(checkpoint, catalog_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    inputs = (torch.zeros(1, 399), torch.zeros(1, 240))
    torch.onnx.export(model, inputs, str(output), input_names=['policy', 'command_short'],
                      output_names=['action'], opset_version=17, dynamo=False,
                      do_constant_folding=True)
    graph = onnx.load(str(output))
    onnx.checker.check_model(graph)
    from convert_onnx_to_rknn_rk3588_wbc_policy import check_model
    check_model(output)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(output), sess_options=options, providers=['CPUExecutionProvider'])
    original = torch.jit.load(str(torchscript), map_location='cpu').eval() if torchscript else None
    rng = np.random.default_rng(20261104)
    max_error, source_error = 0.0, 0.0
    with torch.inference_mode():
        for i in range(24):
            feed = {k: rng.normal(0, 0 if i == 0 else 0.5, (1, n)).astype(np.float32)
                    for k, n in [('policy', 399), ('command_short', 240)]}
            tensors = [torch.from_numpy(feed[k]) for k in ('policy', 'command_short')]
            expected = model(*tensors).numpy()
            actual = session.run(['action'], feed)[0]
            if not np.isfinite(expected).all() or not np.isfinite(actual).all():
                raise ValueError('Actor produced non-finite output')
            np.testing.assert_allclose(actual, expected, rtol=2e-4, atol=2e-4)
            max_error = max(max_error, float(np.abs(actual - expected).max()))
            if original is not None:
                source = original(*tensors).numpy()
                np.testing.assert_allclose(expected, source, rtol=1e-5, atol=1e-5)
                source_error = max(source_error, float(np.abs(source - expected).max()))
    report = {'format': 'wbc_base_actor_export_v1', 'checkpoint_sha256': sha(checkpoint),
              'onnx_sha256': sha(output), 'checkpoint_format': payload['format'],
              'inputs': {'policy': [1, 399], 'command_short': [1, 240]}, 'output': {'action': [1, 21]},
              'normalization_inside_model': True, 'strict_state_load': True,
              'parameter_count': sum(p.numel() for p in model.parameters()),
              'comparison_samples': 24, 'onnx_vs_reconstructed_pt_max_abs': max_error,
              'delivered_torchscript_max_abs': source_error if original else None,
              'rknn_runtime_validation': 'not_run'}
    output.with_suffix('.export.json').write_text(json.dumps(report, indent=2) + '\n')
    return report, catalog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoint', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--torchscript', type=Path, help='Optional delivered TorchScript for independent parity check')
    args = parser.parse_args()
    torch.set_num_threads(1)
    report, _ = export_actor(args.checkpoint, args.output, args.catalog, args.torchscript)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
