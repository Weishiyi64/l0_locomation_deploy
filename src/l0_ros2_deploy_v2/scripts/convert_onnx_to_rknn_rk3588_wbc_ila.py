#!/usr/bin/env python3
"""Build the optional SONIC ILA correction network for RK3588."""
import argparse
import os
from pathlib import Path
import tempfile

from wbc_lora_assets import conversion_record, rknn_build_config


def main():
    import onnx
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('onnx', type=Path)
    parser.add_argument('-o', '--output', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--build-only', action='store_true', default=True)
    args = parser.parse_args()
    source = args.onnx.resolve(strict=True)
    target = args.output.absolute()
    model = onnx.load(source)
    onnx.checker.check_model(model)
    for tensors, name, dims in ((model.graph.input, 'raw_features', [1, 639]),
                                (model.graph.output, 'arm_window', [1, 30])):
        if (len(tensors) != 1 or tensors[0].name != name or
                tensors[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT or
                [d.dim_value for d in tensors[0].type.tensor_type.shape.dim] != dims):
            raise ValueError(f'Expected float32 {name}{dims}')
    if args.dry_run:
        return
    from rknn.api import RKNN
    with tempfile.TemporaryDirectory(dir=target.parent) as tmp:
        work = Path(tmp)
        previous = Path.cwd()
        rknn = RKNN()
        try:
            os.chdir(work)
            for func, params in ((rknn.config, rknn_build_config()),
                                 (rknn.load_onnx, {'model': str(source)}),
                                 (rknn.build, {'do_quantization': False}),
                                 (rknn.export_rknn, {'export_path': str(work / 'ila.rknn')})):
                if func(**params) != 0:
                    raise RuntimeError(f'{func.__name__} failed')
        finally:
            os.chdir(previous)
            rknn.release()
        (work / 'ila.rknn').replace(target)
    args.target = None
    args.max_abs_error = 0.05
    conversion_record(source, target, args, None)


if __name__ == '__main__':
    main()
