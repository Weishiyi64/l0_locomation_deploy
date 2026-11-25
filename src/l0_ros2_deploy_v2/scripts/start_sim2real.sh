#!/usr/bin/env bash
# Formal sim2real entry only. HPM remains manually managed.
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo '用法：bash scripts/start_sim2real.sh [--workspace 工作空间] [--dry-run]'
  echo '读取 deploy_config/wbc-production.env，新建运行记录并启动 sim2real。'
  echo '不启动 HPM、录包或动作控制台。--dry-run 仅检查配置、打印命令。'
  exit 0
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/wbc_runtime_common.bash"
wbc_options "$@"
wbc_environment "$wbc_workspace/deploy_config/wbc-production.env"
cmd=(ros2 launch legged_controllers sim2real.launch.py
  "robot_type:=$robot_type" "subpolicy:=$amp_policy"
  wbc_enabled:=true wbc_mode:=lora hpm_mode:=real bag_enabled:=false
  "wbc_lora_bundle:=$wbc_bundle" "wbc_policies:=$wbc_actions")
wbc_print_command "${cmd[@]}"
$wbc_dry_run && exit 0
mkdir -p -- "$wbc_run_root"
exec 9>"$wbc_run_root/.sim2real.lock"
flock -n 9 || wbc_die '此工作空间的启动脚本已在运行'
wbc_run_dir=$(/usr/bin/python3 -B - "$wbc_workspace" "$wbc_prefix" "$wbc_installed" "$robot_type" "$amp_policy" <<'PY'
import datetime, hashlib, json, os, shutil, subprocess, sys, tempfile
from pathlib import Path
ws, prefix, bundle = map(Path, sys.argv[1:4])
robot, amp_name = sys.argv[4:]
repo = ws / 'src/l0_ros2_deploy_v2'
for proc in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        executable = proc.read_bytes().split(b'\0', 1)[0].decode()
    except (OSError, UnicodeError):
        continue
    if Path(executable).name in ('rl_controller', 'topic_hardware_adapter'):
        raise SystemExit('已有部署节点运行，请先按原流程结束该部署：' + executable)
amp = repo / 'legged_deploy/policy' / robot / amp_name
installed_amp = prefix / 'share/legged_deploy/policy' / robot / amp_name
def sha(path):
    with path.open('rb') as f:
        h = hashlib.sha256()
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
        return h.hexdigest()
for name in ('rl_config.yaml', 'joy_config.yaml'):
    if sha(amp / name) != sha(installed_amp / name):
        raise SystemExit('AMP 源码/安装配置不一致：' + name)
worker = prefix / 'lib/legged_deploy/rl_controller'
worker_hash = sha(worker)
amp_hashes = {str(p): sha(p) for p in (amp / 'policy.rknn', installed_amp / 'rl_config.yaml', installed_amp / 'joy_config.yaml')}
files = json.loads((bundle / 'INSTALLATION.json').read_text())['files']
for relative, expected in files.items():
    p = (bundle / relative).resolve()
    if not p.is_relative_to(bundle.resolve()) or sha(p) != expected:
        raise SystemExit('运行资产校验失败：' + relative)
root = ws / 'runs/wbc_production'
run = Path(tempfile.mkdtemp(prefix=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_'), dir=root))
(run / 'ros').mkdir()
(run / 'bags').mkdir()
shutil.copy2(ws / 'deploy_config/wbc-production.env', run / 'environment.env')
for name in ('INSTALLATION.json', 'base_manifest.json'):
    shutil.copy2(bundle / name, run / name)
preprocessing = ws / 'model_sources' / bundle.name / 'PREPROCESSING.json'
if preprocessing.is_file():
    shutil.copy2(preprocessing, run / preprocessing.name)
(run / 'installed_bundle.txt').write_text(str(bundle) + '\n')
(run / 'models.sha256').write_text(''.join(f'{h}  {p}\n' for p, h in sorted(files.items())))
(run / 'worker.sha256').write_text(f'{worker_hash}  {worker}\n')
(run / 'amp.sha256').write_text(''.join(f'{h}  {p}\n' for p, h in amp_hashes.items()))
with (run / 'version.txt').open('w') as out:
    for args in (['git', '-C', str(repo), 'rev-parse', 'HEAD'], ['git', '-C', str(repo), 'status', '--short'], ['uname', '-a']):
        subprocess.run(args, stdout=out, stderr=out, check=True)
    out.write('\n'.join(f'{k}={os.environ.get(k, "unset")}' for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')) + '\n')
with (run / 'source_changes.patch').open('w') as out:
    subprocess.run(['git', '-C', str(repo), 'diff', 'HEAD', '--binary'], stdout=out, check=True)
pending = root / 'current_run_dir.pending'
pending.write_text(str(run) + '\n')
pending.replace(root / 'current_run_dir')
print(run)
PY
) || wbc_die '运行记录初始化失败，未启动节点'
export ROS_LOG_DIR="$wbc_run_dir/ros"
wbc_print_command "${cmd[@]}" > "$wbc_run_dir/start_command.txt"
printf '运行记录：%s\nHPM 请按原流程手动维护；本终端只启动 sim2real。\n' "$wbc_run_dir"
if wbc_logged_command "$wbc_run_dir/deploy.log" "${cmd[@]}"; then result=0; else result=$?; fi
printf '%s\n' "$result" > "$wbc_run_dir/launch.exitcode"
printf '部署入口已退出。完整启动结果请查看 deploy.log；外层返回 0 不保证子节点成功。\n'
exit "$result"
