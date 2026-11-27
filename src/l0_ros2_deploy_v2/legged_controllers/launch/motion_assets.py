"""Load and validate shared WBC/text motion assets for launch."""
import json
import math
from pathlib import Path

import yaml


def asset(root, relative):
    root = Path(root).resolve(strict=True)
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError('Expected a relative deployment asset path')
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('Deployment asset escapes its package: ' + relative)
    return path


def load_motion_assets(deployment):
    deployment = Path(deployment).resolve(strict=True)
    layout = json.loads(deployment.read_text())
    base = layout.get('format') == 'wbc_base_graph_v1'
    fixed_only = (base or layout.get('format') == 'wbc_lora_graph_v1') and layout.get('kind') == 'policy'
    if not fixed_only and layout.get('format') != 'wbc_motion_deployment_v1':
        raise ValueError('Expected the shared motion deployment manifest')
    model_manifest = deployment if fixed_only else asset(deployment.parent, layout['model_manifest'])
    graph = json.loads(model_manifest.read_text())
    actions = {}
    catalogs = [deployment] if fixed_only else [asset(deployment.parent, p) for p in layout['catalogs']]
    for path in catalogs:
        catalog = json.loads(path.read_text())
        graph_catalog = base or catalog.get('format') == 'wbc_lora_graph_v1'
        if not graph_catalog and catalog.get('format') != 'wbc_motion_catalog_v1':
            raise ValueError('Unsupported motion catalog')
        if graph_catalog and catalog.get('kind') != 'policy':
            raise ValueError('Fixed bundle must contain a policy graph')
        for item in catalog['actions']:
            key = item['action_id']
            if key in actions:
                raise ValueError('Duplicate motion ID: ' + key)
            config = asset(path.parent, item['config'])
            adapter = None if base else asset(path.parent, item['adapter_manifest'])
            actions[key] = dict(item, root=path.parent, config_path=config, adapter_path=adapter)
    if not actions:
        raise ValueError('Empty motion catalog')
    if fixed_only:
        names = yaml.safe_load(next(iter(actions.values()))['config_path'].read_text())['RobotCfg']['joint_order_mj']
        if len(names) != 21 or len(set(names)) != 21:
            raise ValueError('WBC requires 21 unique joint names')
        return dict(deployment=deployment, graph=graph, actions=actions, tracker=None,
                    joint_names=names, standing_path=None, standing_height=None)
    tracker = actions[layout['text_action']]
    if tracker['action_id'] != 'UpperBody_Enhanced' or tracker.get('json_only') is not True or tracker['parameter_source'] != 'lora':
        raise ValueError('Text tracker must be the JSON-only UpperBody_Enhanced LoRA')
    standing_path = asset(deployment.parent, layout['standing_config'])
    standing = yaml.safe_load(standing_path.read_text())
    height = standing['standing_root_height_m']
    if type(height) not in (int, float) or not 0.05 < height < 2:
        raise ValueError('Invalid configured standing height')
    names = yaml.safe_load(tracker['config_path'].read_text())['RobotCfg']['joint_order_mj']
    if len(names) != 21 or len(set(names)) != 21:
        raise ValueError('Text tracker needs 21 unique joint names')
    legs = standing['standing_leg_joint_angles']
    if not isinstance(legs, dict) or set(legs) != set(names[:12]) or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in legs.values()):
        raise ValueError('Text standing config requires 12 named finite leg angles')
    return dict(deployment=deployment, graph=graph, actions=actions, tracker=tracker,
                joint_names=names, standing_path=standing_path, standing_height=float(height))

