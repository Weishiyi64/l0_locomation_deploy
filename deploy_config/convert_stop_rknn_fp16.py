import argparse
from pathlib import Path
from rknn.api import RKNN

p = argparse.ArgumentParser()
p.add_argument("source", type=Path)
p.add_argument("destination", type=Path)
args = p.parse_args()
rknn = RKNN()
try:
    steps = (
        ("config", lambda: rknn.config(target_platform="rk3588", float_dtype="float16",
                                        disable_rules=["fuse_exnorm_add"])),
        ("load", lambda: rknn.load_onnx(model=str(args.source))),
        ("build", lambda: rknn.build(do_quantization=False)),
        ("export", lambda: rknn.export_rknn(str(args.destination))),
    )
    for stage, run in steps:
        result = run()
        if result != 0:
            raise RuntimeError(f"{stage} returned {result}")
finally:
    rknn.release()
