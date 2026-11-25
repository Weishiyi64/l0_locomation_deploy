"""Portable text-to-motion host pipeline for RK3588.

The normal backend is RKNNLite2 on a real RK3588.  The explicitly selected
``onnx-reference`` backend uses local ONNX Runtime (CPU by default, explicit CUDA supported); it
does not assert NPU execution or numerical parity on a board.  Neither path
loads training checkpoints or the original tokenizer shared library. Explicit CUDA
loads PyTorch only to initialize compatible CUDA libraries, never for inference.
"""
from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np

from .support import ROOT, checked_file, check_return
from .motion import MotionComposer
from .tokenizer import fixed_tokenize


DEFAULT_MODEL = ROOT / "manifest.json" if (ROOT / "manifest.json").is_file() else ROOT / "rknn/compiled/manifest.json"
PARTS = ("head", "left_hand", "right_hand")


def random_token_ids(metadata, seed=None):
    """Web random-button policy: uniform 1–3 parts, then uniform valid tokens."""
    rng = np.random.default_rng(seed)
    count = int(rng.integers(1, 4))
    selected = set(rng.permutation(3)[:count].tolist())
    result = [0, 0, 0]
    for index, key in enumerate(PARTS):
        if index not in selected:
            continue
        excluded = set(metadata['source'].get('unavailable_tokens', {}).get(key, []))
        choices = [t['id'] for t in metadata['vocabulary']['parts'][key]['tokens']
                   if t['fields'] is not None and t['id'] not in excluded]
        if not choices:
            raise ValueError(f'{key}: no available action tokens')
        result[index] = int(rng.choice(choices))
    return result


def _validated_tensors(values, specs, *, outputs=False):
    """Check the declared static ABI before/after each neural invocation."""
    if set(values) != {spec["name"] for spec in specs}:
        raise ValueError("Neural input/output names differ from the package manifest")
    result = {}
    for spec in specs:
        name, dtype = spec["name"], np.dtype(spec["dtype"])
        value = np.asarray(values[name])
        if list(value.shape) != spec["shape"]:
            raise ValueError(f"{name}: shape {value.shape} != {spec['shape']}")
        if value.dtype.kind not in "fibu" or not np.isfinite(value).all():
            raise ValueError(f"{name}: expected finite numeric values")
        if not outputs and value.dtype != dtype:
            raise ValueError(f"{name}: dtype {value.dtype} != {dtype}")
        # RKNN can expose an integer output tensor as float.  Never truncate an
        # invalid prediction silently, and reject overflow before conversion.
        if outputs and dtype.kind in "iu":
            bounds = np.iinfo(dtype)
            if np.any(value != np.round(value)) or np.any(value < bounds.min) or np.any(value > bounds.max):
                raise ValueError(f"{name}: invalid integer output")
        converted = np.ascontiguousarray(value, dtype=dtype)
        if not np.isfinite(converted).all():
            raise ValueError(f"{name}: conversion overflow")
        result[name] = converted
    return result


class _OnnxSession:
    def __init__(self, path, record, device='cpu'):
        if device not in ('cpu', 'cuda'):
            raise ValueError('ONNX device must be cpu or cuda')
        if device == 'cuda':
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA不可用，拒绝静默回退 CPU')
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.log_severity_level = 3
        providers = [('CUDAExecutionProvider', {'use_tf32': 0, 'arena_extend_strategy': 'kSameAsRequested',
                                               'gpu_mem_limit': 384*1024*1024})] if device == 'cuda' else ['CPUExecutionProvider']
        if device == 'cuda' and 'CUDAExecutionProvider' not in ort.get_available_providers():
            raise RuntimeError('CUDAExecutionProvider 不可用')
        self.session = ort.InferenceSession(str(path), options, providers=providers)
        if device == 'cuda' and 'CUDAExecutionProvider' not in self.session.get_providers():
            raise RuntimeError('CUDA provider 初始化失败，拒绝静默回退 CPU')
        self.names = [spec["name"] for spec in record["outputs"]]
        if [value.name for value in self.session.get_inputs()] != [spec["name"] for spec in record["inputs"]]:
            raise ValueError("ONNX input names/order differ from the package manifest")
        if [value.name for value in self.session.get_outputs()] != self.names:
            raise ValueError("ONNX output names/order differ from the package manifest")

    def run(self, inputs):
        return dict(zip(self.names, self.session.run(self.names, inputs)))

    def close(self):
        self.session = None


class _LiteSession:
    def __init__(self, path, record):
        from rknnlite.api import RKNNLite
        self.session = RKNNLite()
        self.inputs = [spec["name"] for spec in record["inputs"]]
        self.outputs = [spec["name"] for spec in record["outputs"]]
        try:
            check_return(self.session.load_rknn(str(path)), "load_rknn")
            check_return(self.session.init_runtime(core_mask=RKNNLite.NPU_CORE_0), "init_runtime")
        except Exception:
            self.session.release()
            self.session = None
            raise

    def run(self, inputs):
        result = self.session.inference(inputs=[inputs[name] for name in self.inputs],
                                        data_format=["nchw"] * len(self.inputs))
        if result is None or len(result) != len(self.outputs):
            raise RuntimeError("Unexpected RKNN output count")
        return dict(zip(self.outputs, result))

    def close(self):
        if self.session is not None:
            session, self.session = self.session, None
            session.release()


def _open_session(backend, path, record, device='cpu'):
    return _LiteSession(path, record) if backend == 'lite2' else _OnnxSession(path, record, device)


class RknnMotionPipeline:
    """One reusable pair of neural sessions plus portable NumPy composition."""

    def __init__(self, model=DEFAULT_MODEL, backend="lite2", device='cpu'):
        self.sessions = {}
        if backend not in ("lite2", "onnx-reference"):
            raise ValueError("backend must be lite2 or onnx-reference")
        self.backend = backend
        path = Path(model).resolve()
        self.info = json.loads(path.read_text(encoding="utf-8"))
        expected = "rknn_neural_package_v1" if backend == "lite2" else "rknn_static_neural_v1"
        if self.info.get("format") != expected or self.info.get("target_platform") != "rk3588":
            raise ValueError(f"{backend} requires a {expected} RK3588 package")
        if "motion_assets" not in self.info or "motion_metadata" not in self.info:
            raise ValueError("This old package only contains neural models; rerun prepare/convert for the complete pipeline")
        if self.info.get("fps") != 50:
            raise ValueError("The complete pipeline requires 50 FPS assets")
        self.models = self.info["models"]
        if set(self.models) != {"qwen_tokenizer", "inbetweener"}:
            raise ValueError("Expected exactly qwen_tokenizer and inbetweener models")
        model_paths = {name: checked_file(path.parent, record) for name, record in self.models.items()}
        vocabulary = json.loads(checked_file(path.parent, self.info["vocabulary"]).read_text(encoding="utf-8"))
        self.metadata = self.info["motion_metadata"]
        if vocabulary["vocabulary"] != self.metadata["vocabulary"] or vocabulary["token_order"] != list(PARTS):
            raise ValueError("Character/class vocabulary and motion metadata do not match")
        self.char_vocab = vocabulary["char_vocab"]
        self.max_length = vocabulary["max_length"]
        if type(self.max_length) is not int or self.max_length < 1:
            raise ValueError("Invalid tokenizer max_length")
        if not {"<pad>", "<cls>", "<unk>"} <= set(self.char_vocab) or any(
                type(value) is not int or value < 0 for value in self.char_vocab.values()):
            raise ValueError("Invalid tokenizer vocabulary")
        if vocabulary.get("text_preprocessing") != "identity":
            raise ValueError("Unsupported text preprocessing policy")
        for key in PARTS:
            ids = [token["id"] for token in self.metadata["vocabulary"]["parts"][key]["tokens"]]
            if ids != list(range(len(ids))):
                raise ValueError(f"{key}: logits require contiguous token IDs starting at zero")
        with np.load(checked_file(path.parent, self.info["motion_assets"]), allow_pickle=False) as assets:
            atoms = np.asarray(assets["atom_motion"], np.float32)
            lengths = np.asarray(assets["atom_lengths"])
            self.a_pose = np.asarray(assets["a_pose"], np.float32).copy()
        if not np.array_equal(self.a_pose, np.asarray(self.info["a_pose"], np.float32)):
            raise ValueError("Asset and manifest A-pose do not match")
        self.composer = MotionComposer(self.metadata, atoms, lengths, self.a_pose)
        if backend == "lite2":
            compatible = Path("/proc/device-tree/compatible")
            if not compatible.exists() or b"rk3588" not in compatible.read_bytes():
                raise RuntimeError("lite2 requires a real RK3588 board; no CPU/simulator fallback is used. "
                                   "Use --backend onnx-reference with a prepared manifest for CPU validation.")
        try:
            for name, record in self.models.items():
                self.sessions[name] = _open_session(backend, model_paths[name], record, device)
        except Exception:
            self.close()
            raise

    @property
    def providers(self):
        if self.backend == 'lite2':
            return ['RKNNLite2']
        return self.sessions['qwen_tokenizer'].session.get_providers()

    def close(self):
        errors = []
        for session in self.sessions.values():
            try:
                session.close()
            except Exception as exc:
                errors.append(exc)
        self.sessions.clear()
        if errors:
            raise RuntimeError("Failed to release one or more neural sessions") from errors[0]

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()

    def _run(self, name, inputs):
        if name not in self.sessions:
            raise RuntimeError("The pipeline is closed")
        record = self.models[name]
        inputs = _validated_tensors(inputs, record["inputs"])
        return _validated_tensors(self.sessions[name].run(inputs), record["outputs"], outputs=True)

    def _transition(self, first, last, frames=-1):
        if type(frames) is not int or frames not in (-1, 30, 60, 90):
            raise ValueError("Transition frame count must be -1, 30, 60 or 90")
        for value in (first, last):
            if np.asarray(value).shape != (9,) or not np.isfinite(value).all():
                raise ValueError("Transition endpoints must be finite nine-dimensional poses")
        result = self._run("inbetweener", {
            "initial_pose": np.asarray(first, np.float32).reshape(1, 9),
            "target_pose": np.asarray(last, np.float32).reshape(1, 9),
            "frame_count": np.asarray([frames], np.int32),
        })
        count = int(result["output_frame_count"][0])
        config = self.info["inbetweener"]
        if not 1 <= count <= config["padded_frames"]:
            raise ValueError("Inbetweener predicted an invalid frame count")
        if frames == -1 and not config["auto_min_frames"] <= count <= config["auto_max_frames"]:
            raise ValueError("Inbetweener automatic length is outside its trained range")
        if frames != -1 and count != frames:
            raise ValueError("Inbetweener did not honor the fixed frame count")
        motion = np.asarray(result["motion"][0, :count], np.float32).copy()
        if motion.shape != (count, 9):
            raise ValueError("Invalid inbetweener trajectory dimensions")
        return motion

    def generate(self, *, mode="text", text="", token_ids=None, init_pose=None, end_pose=None,
                 return_home=True, return_frames=-1, speed=1.2, seed=None,
                 material_selector=None, shoulder_fraction=None, randomize=True):
        if mode not in ("text", "web", "random"):
            raise ValueError("mode must be text, web or random")
        start = time.perf_counter()
        if mode == "text":
            result = self._run("qwen_tokenizer", fixed_tokenize(text, self.char_vocab, self.max_length))
            token_ids = []
            for key in PARTS:
                logits = result[f"{key}_logits"]
                expected = len(self.metadata["vocabulary"]["parts"][key]["tokens"])
                if logits.shape != (1, expected):
                    raise ValueError(f"{key}: classifier shape does not match the class vocabulary")
                token_ids.append(int(logits.argmax(-1)[0]))
        elif mode == 'random':
            token_ids = random_token_ids(self.metadata, seed)
        elif token_ids is None:
            token_ids = [0, 0, 0]
        output = self.composer.generate(token_ids, init_pose=init_pose, end_pose=end_pose,
            return_home=return_home, return_frames=return_frames, speed=speed, seed=seed,
            material_selector=material_selector, shoulder_fraction=shoulder_fraction,
            randomize=randomize, transition=self._transition)
        if not bool(np.asarray(output["success"]).all()):
            raise ValueError("Invalid request or unsupported material category")
        motion = np.asarray(output["motion"])
        if motion.ndim != 2 or motion.shape[1] != 9 or len(motion) < 1 or not np.isfinite(motion).all():
            raise ValueError("The composed trajectory must be finite [frames, 9]")
        output["motion"] = np.asarray(motion, np.float32)
        output["inference_ms"] = (time.perf_counter() - start) * 1000
        output["backend"] = self.backend
        return output
