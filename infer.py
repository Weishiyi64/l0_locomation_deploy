#!/usr/bin/env python3
"""Text-motion entry point."""

import importlib.util
import os
from pathlib import Path
import runpy
import sys


# ROS 2 Humble normally supplies /usr/bin/python3, while ONNX Runtime for this
# model is installed in the project machine's unitree_rl_mjlab environment.
# Transparently re-exec there so the documented `python3 infer.py ...` command
# works; publishing still falls back to the ROS system Python when necessary.
_MODEL_PYTHON = Path('/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python')
if importlib.util.find_spec('onnxruntime') is None and _MODEL_PYTHON.is_file():
    if Path(sys.executable).resolve() != _MODEL_PYTHON.resolve():
        os.execv(
            str(_MODEL_PYTHON),
            [str(_MODEL_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
        )


runpy.run_path(
    str(Path(__file__).resolve().parent / 'src' / 'legged_deploy' / 'scripts' / 'infer.py'),
    run_name='__main__',
)
