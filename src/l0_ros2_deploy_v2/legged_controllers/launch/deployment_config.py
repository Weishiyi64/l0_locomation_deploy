"""Optional deployment settings and preflight shared by direct and scripted sim2real.

The .env file contains literal key=value data, not executable shell code.
CLI launch arguments override this file; missing keys retain sim2real defaults.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
import time

import yaml


def workspace():
    for parent in Path(__file__).absolute().parents:
        if parent.name == 'install' and (parent / '.wbc_workspace_path').is_file():
            return Path((parent / '.wbc_workspace_path').read_text().strip()).resolve(strict=True)
        if parent.name in ('src', 'install'):
            return parent.parent
    raise RuntimeError('Cannot locate deployment workspace from launch file')


def read_settings(path):
    if not path.exists():
        return {}
    aliases = {'amp_policy': 'subpolicy', 'wbc_bundle': 'wbc_lora_bundle', 'wbc_actions': 'wbc_policies'}
    values = {}
    for number, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        match = re.fullmatch(r'(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=(.*)', line)
        if not match:
            raise ValueError(f'{path}:{number}: expected literal key=value')
        key, raw_value = match.groups()
        key = aliases.get(key, key)
        tokens = shlex.split(raw_value, comments=True, posix=True)
        if len(tokens) > 1 or key in values:
            raise ValueError(f'{path}:{number}: duplicate key or unquoted value: {key}')
        values[key] = tokens[0] if tokens else ''
    return values


def apply_settings(arguments):
    from launch.actions import DeclareLaunchArgument
    path = workspace() / 'deploy_config/wbc-production.env'
    settings = read_settings(path)
    unknown = set(settings) - {arg.name for arg in arguments}
    if unknown:
        raise ValueError(f'Unknown sim2real settings in {path}: {sorted(unknown)}')
    return [DeclareLaunchArgument(arg.name, default_value=settings[arg.name], description=arg.description)
            if arg.name in settings else arg for arg in arguments]


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def checked_asset(root, relative):
    path = (root / relative).resolve(strict=True)
    if Path(relative).is_absolute() or not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('Deployment asset escapes bundle: ' + relative)
    return path


def deployment_assets(config):
    from ament_index_python.packages import get_package_share_directory
    root = Path(get_package_share_directory('legged_deploy')) / 'policy'
    if not root.resolve().is_relative_to((workspace() / 'install').resolve()):
        raise ValueError('legged_deploy belongs to another workspace')
    for name in ('robot_type', 'subpolicy'):
        if not re.fullmatch(r'[A-Za-z0-9_-]+', config[name]):
            raise ValueError('Invalid deployment directory: ' + name)
    robot_root = root / config['robot_type']
    amp = robot_root / config['subpolicy']
    for name in ('policy.rknn', 'rl_config.yaml', 'joy_config.yaml'):
        if not (amp / name).is_file():
            raise ValueError('AMP asset missing: ' + str(amp / name))
    source_amp = workspace() / 'src/l0_ros2_deploy_v2/legged_deploy/policy' / config['robot_type'] / config['subpolicy']
    for name in ('rl_config.yaml', 'joy_config.yaml'):
        if (source_amp / name).is_file() and sha(source_amp / name) != sha(amp / name):
            raise ValueError('AMP source/install config mismatch: ' + name)
    text_enabled = config['txt2motion_enabled'].lower() == 'true'
    if config['wbc_enabled'].lower() != 'true' and not text_enabled:
        config['wbc_mode'] = 'disabled'
        return None, []
    name = config['wbc_lora_bundle']
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('Invalid WBC bundle name')
    bundle = robot_root / name
    graph = json.loads(checked_asset(bundle, 'base_manifest.json').read_text())
    mode = {'wbc_base_graph_v1': 'base', 'wbc_lora_graph_v1': 'lora'}[graph['format']]
    if config.get('wbc_mode', 'auto') not in ('auto', mode):
        raise ValueError('Selected mode does not match ' + name)
    config['wbc_mode'] = mode
    if text_enabled and (mode != 'lora' or not (bundle / 'deployment.json').is_file()):
        raise ValueError('Select txt2motion_enabled:=false for a fixed actor bundle')
    from motion_assets import load_motion_assets
    assets = load_motion_assets(bundle / ('deployment.json' if text_enabled else 'base_manifest.json'))
    catalog = {key: item for key, item in assets['actions'].items() if not item.get('json_only')}
    selected = yaml.safe_load(config['wbc_policies'])
    if selected == 'all' or selected == []:
        selected = list(catalog)
    if (not isinstance(selected, list) or not selected or
            any(not isinstance(a, str) or a not in catalog for a in selected) or
            len(selected) != len(set(selected))):
        raise ValueError('Invalid WBC action selection')
    checked_asset(bundle, graph['model']['rknn'])
    if mode == 'base':
        record = json.loads(checked_asset(bundle, 'INSTALLATION.json').read_text())
        for relative, expected in record['files'].items():
            if sha(checked_asset(bundle, relative)) != expected:
                raise ValueError('WBC asset hash mismatch: ' + relative)
    for action in selected:
        item = catalog[action]
        checked_asset(item['root'], item['reference_rknn'])
    return bundle, selected


def write_version_snapshot(repo, path):
    """Record diagnostic version metadata without making it a launch prerequisite."""
    from launch.logging import get_logger

    with path.open('w') as out:
        for command in (['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                        ['git', '-C', str(repo), 'status', '--short'], ['uname', '-a']):
            out.flush()
            try:
                subprocess.run(command, stdout=out, stderr=out, timeout=5, check=True)
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError) as exc:
                message = f'Version metadata unavailable for {shlex.join(command)}: {exc}'
                out.write('\nWARNING: ' + message + '\n')
                get_logger('sim2real').warning(message)
        out.write('\n'.join(f'{k}={os.environ.get(k, "unset")}' for k in
            ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')) + '\n')


def prepare_deployment(context):
    """Preflight happens before any hardware launch; also prepares console/bag snapshots."""
    import fcntl
    from launch.actions import SetEnvironmentVariable, LogInfo
    config = dict(context.launch_configurations)
    root = workspace() / 'runs/wbc_production'
    root.mkdir(parents=True, exist_ok=True)
    # Keep ownership on the launch context until its shutdown. Never inherited by children.
    lock = (root / '.sim2real.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError('A deployment already holds this workspace lock')
    try:
        for proc in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                executable = proc.read_bytes().split(b'\0', 1)[0].decode()
            except (OSError, UnicodeError):
                continue
            if Path(executable).name in ('rl_controller', 'topic_hardware_adapter'):
                raise RuntimeError('Deployment process is already running: ' + executable)
        bundle, selected = deployment_assets(config)
        if selected:
            config['wbc_policies'] = json.dumps(selected)
            context.launch_configurations['wbc_policies'] = config['wbc_policies']
        run = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'), dir=root))
        (run / 'ros').mkdir()
        (run / 'bags').mkdir()
        (run / 'launch_config.json').write_text(json.dumps(config, indent=2) + '\n')
        snapshot = {'robot_type': config['robot_type'], 'amp_policy': config['subpolicy'],
                    'wbc_mode': config['wbc_mode'], 'wbc_bundle': config['wbc_lora_bundle'],
                    'wbc_enabled': config['wbc_enabled'], 'wbc_actions': json.dumps(selected)}
        (run / 'environment.env').write_text(''.join(f'{k}={shlex.quote(v)}\n' for k, v in snapshot.items()))
        settings = workspace() / 'deploy_config/wbc-production.env'
        if settings.is_file():
            shutil.copy2(settings, run / 'deployment_settings.env')
        if bundle:
            for name in ('INSTALLATION.json', 'base_manifest.json'):
                shutil.copy2(bundle / name, run / name)
            (run / 'installed_bundle.txt').write_text(str(bundle) + '\n')
            record = json.loads((bundle / 'INSTALLATION.json').read_text())
            (run / 'models.sha256').write_text(''.join(f'{h}  {p}\n' for p, h in sorted(record.get('files', {}).items())))
        repo = workspace() / 'src/l0_ros2_deploy_v2'
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        worker = Path(get_package_prefix('legged_deploy')) / 'lib/legged_deploy/rl_controller'
        (run / 'worker.sha256').write_text(f'{sha(worker)}  {worker}\n')
        amp = Path(get_package_share_directory('legged_deploy')) / 'policy' / config['robot_type'] / config['subpolicy']
        (run / 'amp.sha256').write_text(''.join(f'{sha(amp / n)}  {amp / n}\n'
            for n in ('policy.rknn', 'rl_config.yaml', 'joy_config.yaml')))
        write_version_snapshot(repo, run / 'version.txt')
        pending = root / 'current_run_dir.pending'
        pending.write_text(str(run) + '\n')
        pending.replace(root / 'current_run_dir')
    except BaseException:
        lock.close()
        raise
    context._sim2real_deployment_lock = lock
    return [SetEnvironmentVariable('ROS_LOG_DIR', str(run / 'ros')),
            LogInfo(msg=f'[sim2real] mode={config["wbc_mode"]}, actions={len(selected)}, run={run}')]


def preview(overrides):
    """Resolve the actual launch declarations without executing launch actions."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    spec = importlib.util.spec_from_file_location('sim2real_preview', Path(__file__).with_name('sim2real.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for item in overrides:
        key, separator, value = item.partition(':=')
        if not separator:
            raise ValueError('Expected a launch argument name:=value')
        context.launch_configurations[key] = value
    declarations = [a for a in module.generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
    unknown = set(context.launch_configurations) - {a.name for a in declarations}
    if unknown:
        raise ValueError('Unknown launch arguments: ' + str(sorted(unknown)))
    for arg in declarations:
        if arg.name not in context.launch_configurations:
            context.launch_configurations[arg.name] = perform_substitutions(context, arg.default_value)
    config = dict(context.launch_configurations)
    _, actions = deployment_assets(config)
    if actions:
        config['wbc_policies'] = json.dumps(actions)
    print(json.dumps(config, indent=2))


if __name__ == '__main__':
    import sys
    preview(sys.argv[1:])
