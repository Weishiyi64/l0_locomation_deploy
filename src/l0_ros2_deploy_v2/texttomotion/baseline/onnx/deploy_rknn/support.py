"""Dependency-free package integrity helpers; no compiler/runtime imports."""
import hashlib
from pathlib import Path

ROOT = (Path(__file__).resolve().parents[3] if Path(__file__).parent.name == 'portable'
        else Path(__file__).resolve().parents[1])

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def checked_file(root, record):
    root = Path(root).resolve()
    relative = Path(record['file'])
    path = (root / relative).resolve()
    if relative.is_absolute() or not path.is_relative_to(root):
        raise ValueError(f'Package path must stay inside its directory: {relative}')
    if not path.is_file() or sha256(path) != record['sha256']:
        raise ValueError(f'Missing or changed package file: {path}')
    return path

def check_return(code, operation):
    if code != 0:
        raise RuntimeError(f'RKNN {operation} failed (return code {code})')
