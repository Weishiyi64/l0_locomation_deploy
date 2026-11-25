"""NumPy host orchestration for the frozen text-to-motion deployment contract.

Only the endpoint decoder is delegated to a numerical neural backend. This
module intentionally has no Torch, ORT, RKNN or project-runtime dependency so
the board can copy ``deploy_rknn`` together with its prepared/compiled bundle.
Rules mirror runtime.motion.graph and the frozen angle_processing ONNXs.
"""
from __future__ import annotations

import numpy as np

PARTS = ("head", "left_hand", "right_hand")
COLS = ((6, 7, 8), (3, 4, 5), (0, 1, 2))
LIMITS = np.asarray([
    ((-1.571, 1.571), (-0.34906585, 0.17453293), (-0.297, 0.297)),
    ((-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571)),
    ((-2.618, 2.618), (-1.571, 1.571), (-1.571, 1.571)),
], np.float32)
FULL_LIMITS = np.concatenate([LIMITS[2], LIMITS[1], LIMITS[0]])
HAND_RANDOM = frozenset(("放后面", "前伸", "指向右侧", "指向左侧", "上下摆动", "前后摆动", "斜下举", "张开", "抬起"))
HAND_ONE = frozenset(("放后面", "前伸", "指向右侧", "指向左侧", "碰头", "抱胸", "斜下举", "张开", "抬起", "抱肚子"))
HAND_TWO = frozenset(("拍肚子", "拍胸", "挠头", "摸肚子", "弯曲回收", "上下摆动", "前后摆动", "向下环绕"))
HEAD_OFFSET = frozenset(("点头", "向左倾斜", "向右倾斜", "左右倾斜", "摇头"))
HEAD_ONE = frozenset(("抬头", "低头", "向左转头", "向右转头", "向左低头", "向右低头", "向左抬头", "向右抬头", "后仰", "前倾"))
HEAD_TWO = frozenset(("画弧", "画圈", "左右转头"))


def _pose(value, label):
    value = np.asarray(value, np.float32)
    if value.shape != (9,) or not np.isfinite(value).all():
        raise ValueError(f"{label} needs 9 finite radians")
    return value


def _clip(value, limits):
    return np.maximum(np.minimum(value, limits[:, 1]), limits[:, 0])


def augment_shoulders(source, fraction):
    """Frozen angle_augment: feasible +/-10 degrees, 0.02-radian margin."""
    first = source[0, :2]
    delta = source[:, :2] - first
    maximum = np.float32(np.deg2rad(10))
    limits = LIMITS[1, :2]
    lower = np.maximum(limits[:, 0] + np.float32(.02) - delta.min(0), first - maximum)
    upper = np.minimum(limits[:, 1] - np.float32(.02) - delta.max(0), first + maximum)
    if not np.isfinite(source).all() or np.any(lower > upper):
        raise ValueError("No feasible shoulder offset for the selected material")
    offset = lower + fraction * (upper - lower) - first
    result = source.copy()
    result[:, :2] += offset
    return result


def offset_head(source, first):
    # The original operator accumulates per-frame FP32 deltas, then clamps.
    # Do not substitute a relative subtraction: rounding can differ at seams.
    deltas = np.concatenate([np.zeros_like(source[:1]), np.diff(source, axis=0)])
    return _clip(first + np.cumsum(deltas, axis=0, dtype=np.float32), LIMITS[0])


def trim_nod(source, first):
    candidate = first + (source - source[:1])
    valid = np.isfinite(candidate).all(1) & ((candidate >= LIMITS[0, :, 0]) & (candidate <= LIMITS[0, :, 1])).all(1)
    if not valid[0]:
        raise ValueError("点头初始姿态超出头部限位")
    return source[valid]


def mirror_shake(source, first):
    relative = source - source[:1]
    candidate = first + relative
    lo, hi = LIMITS[0, :, 0], LIMITS[0, :, 1]
    if not np.any((candidate < lo) | (candidate > hi)):
        return source
    candidate = first - relative
    valid = ((candidate >= lo) & (candidate <= hi)).all(1)
    if not valid[0]:
        raise ValueError("摇头初始姿态超出头部限位")
    kept = [candidate[0]]
    for i in range(1, len(candidate)):
        if valid[i]:
            kept.append(candidate[i])
        elif valid[i - 1]:
            start = candidate[i - 1].astype(np.float64)
            delta = candidate[i].astype(np.float64) - start
            fractions = np.full(3, np.inf)
            positive, negative = delta > 0, delta < 0
            fractions[positive] = (hi[positive] - start[positive]) / delta[positive]
            fractions[negative] = (lo[negative] - start[negative]) / delta[negative]
            boundary = np.clip(start + min(1., float(fractions.min())) * delta, lo, hi).astype(np.float32)
            if np.any(np.abs(boundary - candidate[i - 1]) > np.float32(1.e-7)):
                kept.append(boundary)
    return np.asarray(kept, np.float32)


def _transition(callback, first, last, frames):
    value = np.asarray(callback(first, last, frames), np.float32)
    if value.ndim != 2 or value.shape[1] != 9 or not 1 <= len(value) <= 90 or not np.isfinite(value).all():
        raise ValueError("Inbetweener returned invalid motion")
    if frames != -1 and len(value) != frames:
        raise ValueError("Inbetweener returned an incorrect fixed frame count")
    return value


def bridge_part(source, first, part, crop, transition, input_precision='float32'):
    """FP16 deployment boundary plus frozen inbetween1/2 adapters.

    The deployed ONNX casts both inputs to FP16 *before* choosing the nearest
    frame. Preserve that input rounding: otherwise near ties can select a very
    different frame and change a whole gesture, not merely its numeric error.
    Host reductions use FP32 (as the reference CPU ORT executes them), rather
    than accumulating squared distances in NumPy FP16. The original first
    pose is retained for the singleton playback prepend in the outer graph.
    """
    playback_first = first
    if input_precision == 'float16':
        source = source.astype(np.float16).astype(np.float32)
        first = first.astype(np.float16).astype(np.float32)
    elif input_precision != 'float32':
        raise ValueError("Unsupported bridge input precision")
    cut = int(np.argmin(np.sum((source[:(len(source) + 1) // 2] - first) ** 2, axis=1, dtype=np.float32))) if crop else 0
    target = source[cut] if crop else source[-1]
    columns = (6, 7, 8) if part == 0 else (3, 4, 5)
    sign = np.asarray([1., -1., -1.] if part == 2 else [1., 1., 1.], np.float32)
    first9, last9 = np.zeros(9, np.float32), np.zeros(9, np.float32)
    first9[list(columns)], last9[list(columns)] = first * sign, target * sign
    bridge = _transition(transition, first9, last9, -1)[:, columns] * sign
    count = len(bridge)
    motion = np.concatenate([bridge, source[cut + 1:]], axis=0) if crop else bridge
    motion = _clip(motion, LIMITS[part])
    # A one-frame model prediction contains the target only. Preserve that
    # prediction and explicitly prepend the authoritative start pose.
    if count == 1 and np.any(motion[0] != playback_first):
        motion = np.concatenate([playback_first[None], motion])
        count += 1
    return motion, count


def resample_speed(motion, speed, action_count, return_home):
    n = len(motion)
    speed = np.float32(speed)
    count = max(1, int(np.ceil(np.float32(n - 1) / speed)) + 1)
    grid = np.arange(count, dtype=np.float32) / np.float32(max(count - 1, 1)) * np.float32(n - 1)
    left = np.floor(grid).astype(np.int64)
    right = np.minimum(left + 1, n - 1)
    alpha = grid - left.astype(np.float32)
    result = motion[left] * (1 - alpha[:, None]) + motion[right] * alpha[:, None]
    seam = int(np.ceil(np.float32(action_count - 1) * np.float32(count - 1) / np.float32(max(1, n - 1)))) if return_home else -1
    return result, seam


def finish_timing(motion, speed, action_count, return_home, active):
    """Speed-adjust endpoints independently, then add a fixed 25-frame hold."""
    hold_frames = 25 if return_home and active else 0
    if hold_frames:
        action, _ = resample_speed(motion[:action_count], speed, action_count, False)
        home, _ = resample_speed(motion[action_count - 1:], speed, 1, False)
        action_after_speed = len(action)
        motion = np.concatenate([action, np.repeat(action[-1:], hold_frames, axis=0), home[1:]])
        seam = len(action) - 1 + hold_frames
    else:
        motion, seam = resample_speed(motion, speed, action_count, return_home)
        action_after_speed = seam + 1 if return_home else len(motion)
    return motion, seam, action_after_speed, hold_frames


class MotionComposer:
    """Compose materials using a supplied endpoint predictor, with no I/O."""
    def __init__(self, metadata, atoms, lengths, a_pose):
        self.metadata = metadata
        # Older bundles describe the original FP32 frozen operators. Prepared
        # bundles from FP16 deployments explicitly record their ingress cast.
        self.bridge_precision = metadata.get('host_bridge_input_precision', 'float32')
        if self.bridge_precision not in ('float16', 'float32'):
            raise ValueError("Unsupported host_bridge_input_precision")
        self.atoms = np.asarray(atoms, np.float32)
        self.lengths = np.asarray(lengths)
        self.a_pose = _pose(np.asarray(a_pose).reshape(-1), "a_pose")
        if self.atoms.ndim != 3 or self.atoms.shape[-1] != 3 or not np.isfinite(self.atoms).all():
            raise ValueError("Invalid atom_motion table")
        if self.lengths.shape != (len(self.atoms),) or self.lengths.dtype.kind not in 'iu' or np.any(self.lengths < 1) or np.any(self.lengths > self.atoms.shape[1]):
            raise ValueError("Invalid atom_lengths table")
        self.tokens = []
        self.candidates = [{0: [0]} for _ in PARTS]
        for p, key in enumerate(PARTS):
            tokens = metadata['vocabulary']['parts'][key]['tokens']
            by_id = {t['id']: t for t in tokens}
            if len(by_id) != len(tokens) or 0 not in by_id or by_id[0]['fields'] is not None:
                raise ValueError("Invalid material vocabulary")
            self.tokens.append(by_id)
            for t in tokens:
                if t['id'] == 0:
                    continue
                action = t['fields']['动作']
                allowed = HEAD_OFFSET | HEAD_ONE | HEAD_TWO if p == 0 else HAND_ONE | HAND_TWO
                if action not in allowed:
                    raise ValueError("Unclassified action requires an explicit route: " + action)
        seen = {0}
        for material in metadata['source']['materials'][1:]:
            p = PARTS.index(material['part'])
            token, identifier = material['token_id'], material['id']
            if token not in self.tokens[p] or token == 0 or type(identifier) is not int or identifier in seen or not 0 < identifier < len(self.atoms):
                raise ValueError("Invalid material mapping")
            if material.get('fields') != self.tokens[p][token]['fields']:
                raise ValueError("Material label and vocabulary disagree")
            seen.add(identifier)
            self.candidates[p].setdefault(token, []).append(identifier)

    def generate(self, token_ids, *, transition, init_pose=None, end_pose=None,
                 return_home=True, return_frames=-1, speed=1.2, material_selector=None,
                 shoulder_fraction=None, randomize=True, seed=None):
        if len(token_ids) != 3 or any(not isinstance(t, (int, np.integer)) or isinstance(t, (bool, np.bool_)) for t in token_ids):
            raise ValueError("Need three integer tokens in head/left/right order")
        tokens = np.asarray(token_ids, np.int64)
        for p, token in enumerate(tokens):
            if int(token) not in self.tokens[p] or not self.candidates[p].get(int(token)):
                raise ValueError("Invalid or unavailable action token")
        if type(return_home) is not bool or type(randomize) is not bool:
            raise ValueError("return_home/randomize must be booleans")
        if type(return_frames) is not int or return_frames not in (-1, 30, 60, 90):
            raise ValueError("return_frames must be -1, 30, 60 or 90")
        if not np.isfinite(speed) or speed <= 0 or not np.isfinite(np.float32(speed)) or np.float32(speed) <= 0:
            raise ValueError("speed must be finite and positive in float32")
        initial = self.a_pose.copy() if init_pose is None else _pose(init_pose, "init_pose")
        target = self.a_pose.copy() if end_pose is None else _pose(end_pose, "end_pose")
        if return_home and np.any((target < FULL_LIMITS[:, 0]) | (target > FULL_LIMITS[:, 1])):
            raise ValueError("end_pose exceeds joint limits")
        rng = np.random.default_rng(seed)
        if material_selector is None:
            selector = rng.integers(0, 2**31 - 1, 3, dtype=np.int64)
        else:
            selector = np.asarray(material_selector)
        if selector.shape != (3,) or selector.dtype.kind not in 'iu' or np.any(selector < 0):
            raise ValueError("material_selector needs three nonnegative integers")
        if seed is not None:
            if shoulder_fraction is None:
                shoulder_fraction = rng.random((3, 2), dtype=np.float32)
            randomize = False
        fraction = np.full((3, 2), .5, np.float32) if shoulder_fraction is None else np.asarray(shoulder_fraction, np.float32)
        if fraction.shape != (3, 2) or not np.isfinite(fraction).all() or np.any((fraction < 0) | (fraction > 1)):
            raise ValueError("shoulder_fraction needs 3x2 finite values in [0,1]")
        parts, material_ids, transitions = [], [], []
        for p, token in enumerate(tokens):
            candidate_ids = self.candidates[p][int(token)]
            material = candidate_ids[int(selector[p]) % len(candidate_ids)]
            first = initial[list(COLS[p])]
            source = self.atoms[material, :int(self.lengths[material])].copy()
            frames = 1
            if token == 0:
                part = first[None]
            else:
                action = self.tokens[p][int(token)]['fields']['动作']
                if p > 0 and action in HAND_RANDOM:
                    sample = rng.random(2, dtype=np.float32) if randomize else fraction[p]
                    source = augment_shoulders(source, sample)
                if p == 0 and action in HEAD_OFFSET:
                    if action == '摇头':
                        source = mirror_shake(source, first)
                    if action == '点头':
                        source = trim_nod(source, first)
                    part = offset_head(source, first)
                else:
                    crop = action in (HEAD_TWO if p == 0 else HAND_TWO)
                    part, frames = bridge_part(source, first, p, crop, transition, self.bridge_precision)
            parts.append(part)
            material_ids.append(material)
            transitions.append(frames)
        action_count = max(map(len, parts))
        index = np.arange(action_count)
        motion = np.concatenate([parts[p][np.minimum(index, len(parts[p]) - 1)] for p in (2, 1, 0)], axis=1)
        if return_home:
            home = _clip(_transition(transition, motion[-1].copy(), target.copy(), return_frames), FULL_LIMITS)
            if len(home) == 1:
                home = np.concatenate([motion[-1:], home])
            active = np.repeat(tokens[[2, 1, 0]] != 0, 3)
            home = np.where(active[None], home, motion[-1:])
            motion = np.concatenate([motion, home[1:]])
        before_speed = len(motion)
        motion, seam, action_after_speed, hold_frames = finish_timing(
            motion, speed, action_count, return_home, bool(np.any(tokens != 0)))
        if not np.isfinite(motion).all():
            raise ValueError("Composed motion contains nonfinite values")
        return {
            'motion': np.ascontiguousarray(motion, dtype=np.float32),
            'frame_count': np.asarray([len(motion)], np.int64),
            'token_ids': tokens,
            'material_ids': np.asarray(material_ids, np.int64),
            'success': np.asarray([True]),
            'action_frame_count_before_speed': np.asarray([action_count], np.int64),
            'action_frame_count': np.asarray([action_after_speed], np.int64),
            'return_home_hold_frames': np.asarray([hold_frames], np.int64),
            'part_transition_frames': np.asarray(transitions, np.int64),
            'return_home_start_frame': np.asarray([seam], np.int64),
            'frame_count_before_speed': np.asarray([before_speed], np.int64),
            'selector': selector.tolist(),
        }
