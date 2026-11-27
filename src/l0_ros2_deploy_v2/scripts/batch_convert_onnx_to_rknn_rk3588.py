#!/usr/bin/env python3
"""Batch conversion uses the versioned per-file converters in the repo root."""
from pathlib import Path
import runpy
import sys

if __name__ == '__main__':
    repo = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo))
    runpy.run_path(str(repo / Path(__file__).name), run_name='__main__')
