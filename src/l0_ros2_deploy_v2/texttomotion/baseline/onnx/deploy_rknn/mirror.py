"""Shared web/offline output-coordinate reflection; no model dependencies."""
import numpy as np

DEFAULT_MIRROR = {'head': False, 'hands': True}


def mirror_options(value):
    if not isinstance(value, dict) or set(value) != {'head', 'hands'}:
        raise ValueError('镜像设置需要 head 和 hands 两个布尔值')
    if any(type(value[key]) is not bool for key in ('head', 'hands')):
        raise ValueError('镜像设置必须为布尔值')
    return dict(value)


def mirror_pose(value, options):
    """Reflect [9] or [frames,9]; applying twice restores original coordinates."""
    options = mirror_options(options)
    source = np.asarray(value, dtype=np.float32)
    if source.ndim not in (1, 2) or source.shape[-1] != 9 or not np.isfinite(source).all():
        raise ValueError('镜像需要九维有限关节角度')
    result = source.copy()
    if options['hands']:
        signs = np.array([1, -1, -1], dtype=np.float32)
        result[..., :3] = source[..., 3:6] * signs
        result[..., 3:6] = source[..., :3] * signs
    if options['head']:
        result[..., 6] *= -1
        result[..., 8] *= -1
    return result


def mirror_output(result, options):
    """Copy output trajectory; retain token/material provenance in model space."""
    output = {**result, 'motion': mirror_pose(result['motion'], options),
              'mirror': mirror_options(options),
              'metadata_coordinate_space': 'original_model_output'}
    for key in ('initial_pose', 'init_pose', 'end_pose'):
        if key in result:
            output[key] = mirror_pose(result[key], options)
    return output
