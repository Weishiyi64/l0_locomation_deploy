# Copyright 2026
"""Load and validate imitation policy assets for launch."""
from __future__ import annotations

import os
from typing import Any, Dict, List, NamedTuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def _load_yaml_file(path: str, description: str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError(f'PyYAML is required to parse {description}; install python3-yaml.')

    with open(path, 'r', encoding='utf-8') as f:
        root = yaml.safe_load(f) or {}
    if not isinstance(root, dict):
        raise ValueError(f'{description} {path} must contain a YAML map.')
    return root


def _model_asset_exists(policy_dir: str, model_name: str) -> bool:
    return any(os.path.isfile(os.path.join(policy_dir, model_name + ext)) for ext in ('.onnx', '.rknn'))


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
            f'{os.path.join(policy_dir, policy_model)}.(onnx|rknn)'
        )
    if not _model_asset_exists(policy_dir, motion_model):
        raise ValueError(
            f"Imitation policy '{policy_name}' motion model asset is missing: "
            f'{os.path.join(policy_dir, motion_model)}.(onnx|rknn)'
        )


class ImitationAssets(NamedTuple):
    imitation_policies: List[str]
    preload_imitation_policies: List[str]
    skipped_imitation_policies: List[str]


def load_imitation_assets(
    *,
    imitation_policies: List[str],
    robot_type: str = '',
    policy_root_path: str = '',
) -> ImitationAssets:
    if not isinstance(imitation_policies, list) or any(
        not isinstance(policy, str) or not policy.strip() for policy in imitation_policies
    ):
        raise ValueError('imitation_policies must be a list of non-empty policy names.')
    policies = [policy.strip() for policy in imitation_policies]
    if any(policy in {'.', '..'} or '/' in policy or '\\' in policy for policy in policies):
        raise ValueError('imitation_policies must name directories under the robot policy root.')
    sorted_policies = sorted(set(policies))
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

    return ImitationAssets(
        imitation_policies=sorted_policies,
        preload_imitation_policies=sorted_policies,
        skipped_imitation_policies=skipped_policies,
    )
