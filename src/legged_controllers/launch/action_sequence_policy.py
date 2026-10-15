# Copyright 2026
"""Inspect action sequence YAML for dynamic imitation launch decisions.

Used by legged_controllers launch files. Mirrors path resolution used when loading
sequence YAML in ActionSequenceController (relative to `legged_controllers` share).
"""
from __future__ import annotations

import os
from typing import Any, Dict, Iterable, List, NamedTuple, Set, Tuple

try:
    import yaml
except ImportError as _e:  # pragma: no cover
    yaml = None  # type: ignore


def resolve_action_sequence_yaml_path(raw_path: str, package_share_legged_controllers: str) -> str:
    """Resolve sequence file path the same way the first ActionSequence load does: absolute or relative to share."""
    if not raw_path:
        return raw_path
    if os.path.isabs(raw_path):
        return os.path.normpath(raw_path)
    candidate = os.path.join(package_share_legged_controllers, raw_path)
    return os.path.normpath(candidate)


def _resolve_package_path(raw_path: str, package_share_legged_controllers: str) -> str:
    if not raw_path:
        return raw_path
    if os.path.isabs(raw_path):
        return os.path.normpath(raw_path)
    return os.path.normpath(os.path.join(package_share_legged_controllers, raw_path))


VALID_PHASE_MODES = {'rl_with_csv', 'rl_imitation'}


class ActionSequenceManifest(NamedTuple):
    """Validated action sequence manifest for launch-time dynamic worker setup."""

    sequence_paths: List[str]
    imitation_policies: List[str]
    preload_imitation_policies: List[str]
    rl_imitation_sequences: List[str]
    skipped_imitation_policies: List[str]


def _load_yaml_file(path: str, description: str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError(f'PyYAML is required to parse {description}; install python3-yaml.')

    with open(path, 'r', encoding='utf-8') as f:
        root = yaml.safe_load(f) or {}
    if not isinstance(root, dict):
        raise ValueError(f'{description} {path} must contain a YAML map.')
    return root


def _resolve_csv_path(raw_path: str, sequence_dir: str, package_share_legged_controllers: str) -> str:
    if not raw_path:
        return raw_path
    if os.path.isabs(raw_path):
        return os.path.normpath(raw_path)

    relative_to_sequence = os.path.normpath(os.path.join(sequence_dir, raw_path))
    if os.path.isfile(relative_to_sequence):
        return relative_to_sequence

    relative_to_share = os.path.normpath(os.path.join(package_share_legged_controllers, raw_path))
    if os.path.isfile(relative_to_share):
        return relative_to_share

    return relative_to_sequence


def _parse_sequence_yaml(path: str, package_share_legged_controllers: str) -> Tuple[bool, str]:
    """Validate sequence YAML and return (has_rl_imitation_phase, imitation_policy)."""
    root = _load_yaml_file(path, 'action sequence')

    sequence = root.get('sequence') or {}
    if not isinstance(sequence, dict):
        raise ValueError(f'Action sequence {path} must contain sequence map.')
    phases = sequence.get('phases') or []
    if not isinstance(phases, list) or not phases:
        raise ValueError(f'Action sequence {path} must contain non-empty sequence.phases list.')

    has_rl_imitation = False
    sequence_dir = os.path.dirname(path)
    for index, phase in enumerate(phases):
        if not isinstance(phase, dict):
            raise ValueError(f'Action sequence {path} phase[{index}] must be a map.')
        mode = str(phase.get('mode', '')).strip()
        phase_name = str(phase.get('name') or f'phase_{index}')
        if mode not in VALID_PHASE_MODES:
            raise ValueError(
                f"Action sequence {path} phase '{phase_name}' has invalid mode {mode!r}; "
                f'expected one of {sorted(VALID_PHASE_MODES)}.'
            )

        if mode == 'rl_imitation':
            has_rl_imitation = True
            if phase.get('csv'):
                raise ValueError(f"Action sequence {path} phase '{phase_name}' mode rl_imitation must not set csv.")
            if phase.get('velocity_timeline'):
                raise ValueError(
                    f"Action sequence {path} phase '{phase_name}' mode rl_imitation does not support velocity_timeline."
                )
            continue

        raw_csv = str(phase.get('csv') or '').strip()
        if not raw_csv:
            raise ValueError(f"Action sequence {path} phase '{phase_name}' mode {mode} requires csv.")
        resolved_csv = _resolve_csv_path(raw_csv, sequence_dir, package_share_legged_controllers)
        if not os.path.isfile(resolved_csv):
            raise ValueError(
                f"Action sequence {path} phase '{phase_name}' csv is missing: "
                f'{raw_csv!r} (resolved: {resolved_csv!r})'
            )
    raw_imitation = sequence.get('imitation_policy')
    if raw_imitation is None:
        raw_imitation = sequence.get('imitation_policy_name')
    imitation_policy = ''
    if raw_imitation is not None:
        imitation_policy = str(raw_imitation).strip()

    return has_rl_imitation, imitation_policy


def _model_asset_exists(policy_dir: str, model_name: str) -> bool:
    return any(os.path.isfile(os.path.join(policy_dir, model_name + ext)) for ext in ('.onnx', '.trt'))


def _validate_imitation_policy_assets(policy_name: str, robot_type: str, policy_root_path: str) -> None:
    policy_dir = os.path.normpath(os.path.join(policy_root_path, robot_type, policy_name))
    robot_cfg_path = os.path.join(policy_dir, 'rl_config.yaml')
    if not os.path.isfile(robot_cfg_path):
        raise ValueError(
            f"Imitation policy '{policy_name}' is missing rl_config.yaml for robot_type '{robot_type}': "
            f'{robot_cfg_path}'
        )

    root = _load_yaml_file(robot_cfg_path, 'imitation policy rl_config')
    rc = root.get('RobotCfg') or {}
    if not isinstance(rc, dict):
        raise ValueError(f"Imitation policy '{policy_name}' rl_config.yaml must contain RobotCfg map.")

    policy_type = str(rc.get('policy_type') or '').strip()
    if policy_type and policy_type != 'rl_imitation':
        raise ValueError(
            f"Imitation policy '{policy_name}' RobotCfg.policy_type must be 'rl_imitation', got {policy_type!r}."
        )

    policy_model = str(rc.get('policy_name') or '').strip()
    motion_model = str(rc.get('motion_name') or '').strip()
    if not policy_model:
        raise ValueError(f"Imitation policy '{policy_name}' RobotCfg.policy_name is missing.")
    if not motion_model:
        raise ValueError(f"Imitation policy '{policy_name}' RobotCfg.motion_name is missing.")
    if not _model_asset_exists(policy_dir, policy_model):
        raise ValueError(
            f"Imitation policy '{policy_name}' policy model asset is missing: "
            f'{os.path.join(policy_dir, policy_model)}.(onnx|trt)'
        )
    if not _model_asset_exists(policy_dir, motion_model):
        raise ValueError(
            f"Imitation policy '{policy_name}' motion model asset is missing: "
            f'{os.path.join(policy_dir, motion_model)}.(onnx|trt)'
        )


def _iter_keyboard_sequence_paths(config_path: str) -> Iterable[str]:
    if yaml is None:
        raise RuntimeError('PyYAML is required to parse keyboard action maps; install python3-yaml.')
    with open(config_path, 'r', encoding='utf-8') as f:
        root: Dict[str, Any] = yaml.safe_load(f) or {}

    cfg = root.get('keyboard_actions') or root
    actions = cfg.get('actions') or []
    if not isinstance(actions, list):
        raise ValueError(f'Keyboard action map {config_path} must contain keyboard_actions.actions list.')
    for action in actions:
        if not isinstance(action, dict):
            continue
        raw = action.get('sequence') or action.get('sequence_path') or ''
        raw = str(raw).strip()
        if raw:
            yield raw


def _iter_emotion_sequence_paths(config_path: str) -> Iterable[str]:
    if yaml is None:
        raise RuntimeError('PyYAML is required to parse emotion sequence maps; install python3-yaml.')
    with open(config_path, 'r', encoding='utf-8') as f:
        root: Dict[str, Any] = yaml.safe_load(f) or {}

    cfg = root.get('emotion_sequences') or root
    if not isinstance(cfg, dict):
        raise ValueError(f'Emotion sequence map {config_path} must contain emotion_sequences map.')
    for value in cfg.values():
        if isinstance(value, list):
            for raw in value:
                raw = str(raw).strip()
                if raw:
                    yield raw
        elif isinstance(value, str):
            raw = value.strip()
            if raw:
                yield raw


def _normalize_combo_name(value: str) -> str:
    return ''.join(ch for ch in str(value).strip().lower() if ch not in {'_', '-', ' '})


def _button_index(raw_name: str) -> int:
    name = _normalize_combo_name(raw_name)
    mapping = {
        'a': 0,
        'b': 1,
        'x': 3,
        'y': 4,
        'lb': 6,
        'l1': 6,
        'rb': 7,
        'r1': 7,
        'lt': 8,
        'ltbutton': 8,
        'l2': 8,
        'l2button': 8,
        'rt': 9,
        'rtbutton': 9,
        'r2': 9,
        'r2button': 9,
        'back': 10,
        'select': 10,
        'start': 11,
        'ls': 13,
        'lsbutton': 13,
        'l3': 13,
        'rs': 14,
        'rsbutton': 14,
        'r3': 14,
    }
    return mapping.get(name, -1)


def _dpad_axis(raw_name: str) -> int:
    name = _normalize_combo_name(raw_name)
    if name in {'dpadlr', 'dpadx', 'dirlr', 'hatx'}:
        return 6
    if name in {'dpadud', 'dpady', 'dirud', 'haty'}:
        return 7
    return -1


def _axis_sign(raw_direction: str, axis_index: int) -> int:
    direction = _normalize_combo_name(raw_direction)
    if direction in {'positive', 'pos', '+'}:
        return 1
    if direction in {'negative', 'neg', '-'}:
        return -1
    if direction == 'up' and axis_index == 7:
        return 1
    if direction == 'down' and axis_index == 7:
        return -1
    if direction == 'left' and axis_index == 6:
        return 1
    if direction == 'right' and axis_index == 6:
        return -1
    return 0


def _dpad_axis_for_direction(raw_direction: str) -> str:
    direction = _normalize_combo_name(raw_direction)
    if direction in {'up', 'down'}:
        return 'DPad_ud'
    if direction in {'left', 'right'}:
        return 'DPad_lr'
    return ''


def _as_string_list(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value.strip()]
    if isinstance(value, list):
        out: List[str] = []
        for item in value:
            if not isinstance(item, str):
                raise ValueError('combo button entries must be strings')
            out.append(item.strip())
        return out
    raise ValueError('combo buttons must be a string or list')


def _canonical_joystick_combo(combo: Any, action_name: str, kind: str = 'Joystick imitation action') -> str:
    if not isinstance(combo, dict):
        raise ValueError(f"{kind} '{action_name}' needs combo map.")

    buttons: List[int] = []
    axes: List[Tuple[int, int]] = []

    for button_name in _as_string_list(combo.get('buttons')):
        index = _button_index(button_name)
        if index < 0:
            raise ValueError(f"{kind} '{action_name}' has unknown button {button_name!r}.")
        buttons.append(index)

    if combo.get('dpad') is not None:
        direction = str(combo.get('dpad')).strip()
        axis_name = _dpad_axis_for_direction(direction)
        axis_index = _dpad_axis(axis_name)
        sign = _axis_sign(direction, axis_index)
        if axis_index < 0 or sign == 0:
            raise ValueError(f"{kind} '{action_name}' has invalid dpad direction {direction!r}.")
        axes.append((axis_index, sign))

    raw_axes = combo.get('axes') or []
    if not isinstance(raw_axes, list):
        raise ValueError(f"{kind} '{action_name}' combo.axes must be a list.")
    for axis in raw_axes:
        if not isinstance(axis, dict):
            raise ValueError(f"{kind} '{action_name}' combo.axes entries must be maps.")
        axis_name = str(axis.get('name') or axis.get('axis') or '').strip()
        axis_index = _dpad_axis(axis_name)
        if axis_index < 0:
            raise ValueError(f"{kind} '{action_name}' has unknown D-pad axis {axis_name!r}.")
        direction = str(axis.get('direction') or '').strip()
        sign = _axis_sign(direction, axis_index)
        if sign == 0:
            raise ValueError(f"{kind} '{action_name}' axis {axis_name!r} has invalid direction {direction!r}.")
        axes.append((axis_index, sign))

    if not buttons and not axes:
        raise ValueError(f"{kind} '{action_name}' combo has no requirements.")

    parts = [f'b{index}' for index in sorted(buttons)]
    parts.extend(f'a{index}{"+" if sign > 0 else "-"}' for index, sign in sorted(axes))
    return '|'.join(parts)


def _iter_joystick_imitation_policy_entries(config_path: str) -> Iterable[str]:
    root = _load_yaml_file(config_path, 'joystick imitation action map')
    cfg = root.get('joystick_imitation_actions') or root
    if not isinstance(cfg, dict):
        raise ValueError(f'Joystick imitation map {config_path} must contain a YAML map.')
    actions = cfg.get('actions') or []
    if not isinstance(actions, list):
        raise ValueError(f'Joystick imitation map {config_path} must contain joystick_imitation_actions.actions list.')

    seen_names: Set[str] = set()
    seen_combos: Set[str] = set()
    for index, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ValueError(f'Joystick imitation map {config_path} action[{index}] must be a map.')
        name = str(action.get('name') or '').strip()
        if not name:
            raise ValueError(f'Joystick imitation map {config_path} action[{index}] needs non-empty name.')
        if name in seen_names:
            raise ValueError(f"Joystick imitation map {config_path} has duplicate action name '{name}'.")
        seen_names.add(name)

        canonical_combo = _canonical_joystick_combo(action.get('combo'), name)
        if canonical_combo in seen_combos:
            raise ValueError(f"Joystick imitation map {config_path} has duplicate combo for action '{name}'.")
        seen_combos.add(canonical_combo)

        if not bool(action.get('enabled', False)):
            continue
        policy = str(action.get('policy') or '').strip()
        if not policy:
            raise ValueError(f"Joystick imitation action '{name}' is enabled but policy is empty.")
        yield policy


def _validate_joystick_sequence_yaml(
    raw_path: str,
    package_share_legged_controllers: str,
    action_name: str,
) -> None:
    resolved_sequence = resolve_action_sequence_yaml_path(raw_path, package_share_legged_controllers)
    if not resolved_sequence or not os.path.isfile(resolved_sequence):
        raise ValueError(
            f"Joystick sequence action '{action_name}' sequence is missing: "
            f'{raw_path!r} (resolved: {resolved_sequence!r})'
        )

    root = _load_yaml_file(resolved_sequence, 'joystick sequence action sequence')
    sequence = root.get('sequence') or {}
    if not isinstance(sequence, dict):
        raise ValueError(f"Joystick sequence action '{action_name}' sequence {resolved_sequence} must contain map.")
    phases = sequence.get('phases') or []
    if not isinstance(phases, list) or not phases:
        raise ValueError(
            f"Joystick sequence action '{action_name}' sequence {resolved_sequence} must contain "
            'non-empty sequence.phases list.'
        )

    sequence_dir = os.path.dirname(resolved_sequence)
    for index, phase in enumerate(phases):
        if not isinstance(phase, dict):
            raise ValueError(f"Joystick sequence action '{action_name}' phase[{index}] must be a map.")
        phase_name = str(phase.get('name') or f'phase_{index}')
        mode = str(phase.get('mode') or '').strip()
        if mode != 'rl_with_csv':
            raise ValueError(
                f"Joystick sequence action '{action_name}' phase '{phase_name}' must use mode 'rl_with_csv', "
                f'got {mode!r}.'
            )
        raw_csv = str(phase.get('csv') or '').strip()
        if not raw_csv:
            raise ValueError(
                f"Joystick sequence action '{action_name}' phase '{phase_name}' mode rl_with_csv requires csv."
            )
        resolved_csv = _resolve_csv_path(raw_csv, sequence_dir, package_share_legged_controllers)
        if not os.path.isfile(resolved_csv):
            raise ValueError(
                f"Joystick sequence action '{action_name}' phase '{phase_name}' csv is missing: "
                f'{raw_csv!r} (resolved: {resolved_csv!r})'
            )


def _iter_joystick_sequence_paths(config_path: str, package_share_legged_controllers: str) -> Iterable[str]:
    root = _load_yaml_file(config_path, 'joystick sequence action map')
    cfg = root.get('joystick_sequence_actions') or root
    if not isinstance(cfg, dict):
        raise ValueError(f'Joystick sequence map {config_path} must contain a YAML map.')
    actions = cfg.get('actions') or []
    if not isinstance(actions, list):
        raise ValueError(f'Joystick sequence map {config_path} must contain joystick_sequence_actions.actions list.')

    seen_names: Set[str] = set()
    seen_combos: Set[str] = set()
    for index, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ValueError(f'Joystick sequence map {config_path} action[{index}] must be a map.')
        name = str(action.get('name') or '').strip()
        if not name:
            raise ValueError(f'Joystick sequence map {config_path} action[{index}] needs non-empty name.')
        if name in seen_names:
            raise ValueError(f"Joystick sequence map {config_path} has duplicate action name '{name}'.")
        seen_names.add(name)

        canonical_combo = _canonical_joystick_combo(action.get('combo'), name, 'Joystick sequence action')
        if canonical_combo in seen_combos:
            raise ValueError(f"Joystick sequence map {config_path} has duplicate combo for action '{name}'.")
        seen_combos.add(canonical_combo)

        if not bool(action.get('enabled', False)):
            continue
        raw = str(action.get('sequence') or action.get('sequence_path') or '').strip()
        if not raw:
            raise ValueError(f"Joystick sequence action '{name}' is enabled but sequence is empty.")
        _validate_joystick_sequence_yaml(raw, package_share_legged_controllers, name)
        yield raw


def collect_dynamic_imitation_policies(
    *,
    keyboard_action_enabled: bool,
    keyboard_action_config_path: str,
    emotion_sequence_enabled: bool,
    emotion_sequence_map_path: str,
    package_share_legged_controllers: str,
    joystick_imitation_enabled: bool = False,
    joystick_imitation_config_path: str = '',
    joystick_sequence_enabled: bool = False,
    joystick_sequence_config_path: str = '',
) -> List[str]:
    """Return sorted unique imitation policies referenced by enabled keyboard/emotion sequence maps."""
    return collect_action_sequence_manifest(
        keyboard_action_enabled=keyboard_action_enabled,
        keyboard_action_config_path=keyboard_action_config_path,
        emotion_sequence_enabled=emotion_sequence_enabled,
        emotion_sequence_map_path=emotion_sequence_map_path,
        joystick_imitation_enabled=joystick_imitation_enabled,
        joystick_imitation_config_path=joystick_imitation_config_path,
        joystick_sequence_enabled=joystick_sequence_enabled,
        joystick_sequence_config_path=joystick_sequence_config_path,
        package_share_legged_controllers=package_share_legged_controllers,
    ).imitation_policies


def collect_action_sequence_manifest(
    *,
    keyboard_action_enabled: bool,
    keyboard_action_config_path: str,
    emotion_sequence_enabled: bool,
    emotion_sequence_map_path: str,
    package_share_legged_controllers: str,
    joystick_imitation_enabled: bool = False,
    joystick_imitation_config_path: str = '',
    joystick_sequence_enabled: bool = False,
    joystick_sequence_config_path: str = '',
    robot_type: str = '',
    policy_root_path: str = '',
) -> ActionSequenceManifest:
    """Validate enabled action maps and collect policies that should be preloaded for dynamic imitation."""
    policies: Set[str] = set()
    sequence_paths: List[str] = []
    rl_imitation_sequences: List[str] = []

    if keyboard_action_enabled:
        resolved_map = _resolve_package_path(keyboard_action_config_path, package_share_legged_controllers)
        if not resolved_map or not os.path.isfile(resolved_map):
            raise ValueError(f'keyboard_action_enabled but map is missing: {keyboard_action_config_path!r}')
        sequence_paths.extend(_iter_keyboard_sequence_paths(resolved_map))

    if emotion_sequence_enabled:
        resolved_map = _resolve_package_path(emotion_sequence_map_path, package_share_legged_controllers)
        if not resolved_map or not os.path.isfile(resolved_map):
            raise ValueError(f'emotion_sequence_enabled but map is missing: {emotion_sequence_map_path!r}')
        sequence_paths.extend(_iter_emotion_sequence_paths(resolved_map))

    if joystick_imitation_enabled:
        resolved_map = _resolve_package_path(joystick_imitation_config_path, package_share_legged_controllers)
        if not resolved_map or not os.path.isfile(resolved_map):
            raise ValueError(f'joystick_imitation_enabled but map is missing: {joystick_imitation_config_path!r}')
        for policy in _iter_joystick_imitation_policy_entries(resolved_map):
            policies.add(policy)

    if joystick_sequence_enabled:
        resolved_map = _resolve_package_path(joystick_sequence_config_path, package_share_legged_controllers)
        if not resolved_map or not os.path.isfile(resolved_map):
            raise ValueError(f'joystick_sequence_enabled but map is missing: {joystick_sequence_config_path!r}')
        sequence_paths.extend(_iter_joystick_sequence_paths(resolved_map, package_share_legged_controllers))

    for raw_sequence_path in sequence_paths:
        resolved_sequence = resolve_action_sequence_yaml_path(raw_sequence_path, package_share_legged_controllers)
        if not resolved_sequence or not os.path.isfile(resolved_sequence):
            raise ValueError(
                f'action sequence referenced by keyboard/emotion map is missing: '
                f'{raw_sequence_path!r} (resolved: {resolved_sequence!r})'
            )
        has_imitation, policy = _parse_sequence_yaml(resolved_sequence, package_share_legged_controllers)
        if has_imitation:
            if not policy:
                raise ValueError(
                    f'Action sequence {resolved_sequence} has rl_imitation phase(s) but '
                    'sequence.imitation_policy is missing.'
                )
            policies.add(policy)
            rl_imitation_sequences.append(resolved_sequence)

    sorted_policies = sorted(policies)
    skipped_policies: List[str] = []
    if sorted_policies and robot_type and policy_root_path:
        available_policies: List[str] = []
        for policy in sorted_policies:
            try:
                _validate_imitation_policy_assets(policy, robot_type, policy_root_path)
            except (RuntimeError, ValueError, OSError) as exc:
                skipped_policies.append(f"{policy}: {exc}")
                continue
            available_policies.append(policy)
        sorted_policies = available_policies

    return ActionSequenceManifest(
        sequence_paths=sorted(set(sequence_paths)),
        imitation_policies=sorted_policies,
        preload_imitation_policies=sorted_policies,
        rl_imitation_sequences=sorted(set(rl_imitation_sequences)),
        skipped_imitation_policies=skipped_policies,
    )
