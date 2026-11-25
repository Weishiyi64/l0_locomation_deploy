#!/usr/bin/env bash
# RK3588 deployment build entry. Does not launch ROS, simulation or hardware.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash scripts/build_wbc_lora_rk3588.sh BUNDLE [options]

Default: compile one shared FP16 base_lora.rknn and all action reference models.
LoRA/Base parameter packages stay as weights.bin; no per-action policy is built.

  --python PATH       Toolkit2 Python (default: /usr/bin/python3)
  --policy-only       Compile only the shared policy, keeping all 33 inputs
  --include-probes    Also compile the manifest's diagnostic probes
  --skip-existing     Reuse only models with matching conversion provenance
  --log-dir DIR       New directory for build/model logs (default: workspace log)
  --install-to DIR    After conversion, install into a new versioned policy directory
  --colcon            Then build ROS packages from the workspace root (source ROS first)
  --dry-run           Validate the bundle and print commands; no writes/build/install
  -h, --help          Show this help

Actual execution requires Linux/aarch64 RK3588 and rknn-toolkit2 2.3.2.
--build-only is always used: no Python NPU Runtime, no claim of runtime acceptance.
After successful colcon, source WORKSPACE/install/setup.bash in your own terminal.
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
need_value() { [[ $# -ge 2 && -n ${2-} ]] || die "$1 requires a value"; }
print_command() { printf '  '; printf '%q ' "$@"; printf '\n'; }

if [[ ${1-} == --help || ${1-} == -h ]]; then usage; exit 0; fi
[[ $# -ge 1 ]] || { usage >&2; exit 2; }
bundle_arg=$1
shift
python_bin=/usr/bin/python3
policy_only=false
include_probes=false
skip_existing=false
dry_run=false
do_colcon=false
install_to=
requested_log_dir=
while [[ $# -gt 0 ]]; do
  case "$1" in
    --python) need_value "$@"; python_bin=$2; shift 2 ;;
    --log-dir) need_value "$@"; requested_log_dir=$2; shift 2 ;;
    --install-to) need_value "$@"; install_to=$2; shift 2 ;;
    --policy-only) policy_only=true; shift ;;
    --include-probes) include_probes=true; shift ;;
    --skip-existing) skip_existing=true; shift ;;
    --dry-run) dry_run=true; shift ;;
    --colcon) do_colcon=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done
$policy_only && $include_probes && die "--policy-only cannot be combined with --include-probes"
if $policy_only && [[ -n $install_to ]]; then die "--install-to requires a full bundle build"; fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$(cd -- "$script_dir/.." && pwd -P)
workspace=$(cd -- "$repo/../.." && pwd -P)
[[ -d $bundle_arg ]] || die "bundle directory not found: $bundle_arg"
bundle=$(realpath -- "$bundle_arg")
[[ -f $bundle/base_manifest.json ]] || die "bundle is missing base_manifest.json"
command -v "$python_bin" >/dev/null || die "Python not found: $python_bin"
export PYTHONDONTWRITEBYTECODE=1
if [[ -n $install_to ]]; then
  install_to=$(realpath -m -- "$install_to")
  [[ ! -e $install_to && ! -L $install_to ]] || die "installation destination already exists: $install_to"
  [[ "$install_to/" != "$bundle/"* ]] || die "installation destination must be outside source bundle"
fi
if $do_colcon; then
  [[ -d $workspace/src/luvbot_ros_interfaces && -f $repo/legged_deploy/CMakeLists.txt ]] ||
    die "--colcon requires this script inside the deployment workspace src repository"
fi

conversion=("$python_bin" -B "$script_dir/batch_convert_onnx_to_rknn_rk3588.py" "$bundle"
            --manifest "$bundle/base_manifest.json" --build-only)
if $policy_only; then conversion+=(--policy-only); fi
if $include_probes; then conversion+=(--include-probes); fi
if $skip_existing; then conversion+=(--skip-existing); fi
colcon_command=(colcon build --packages-up-to legged_deploy legged_real legged_controllers
                --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN)
if $dry_run; then
  "${conversion[@]}" --dry-run
  if [[ -n $install_to ]]; then
    print_command "$python_bin" -B "$script_dir/prepare_wbc_deploy.py" "$bundle" "$install_to" --mode install-lora
  fi
  if $do_colcon; then
    printf 'Workspace for colcon: %s\n' "$workspace"
    print_command "${colcon_command[@]}"
  fi
  printf 'DRY-RUN: no RKNN/ROS build, files, installation or hardware execution.\n'
  exit 0
fi

[[ $(uname -s) == Linux && $(uname -m) == aarch64 ]] || die "actual builds must run on the RK3588 Linux/aarch64 board"
compatible_file=/proc/device-tree/compatible
if [[ ! -r $compatible_file ]]; then compatible_file=/sys/firmware/devicetree/base/compatible; fi
[[ -r $compatible_file ]] || die "cannot identify RK3588: device-tree compatible is unavailable"
board_compatible=$(tr '\0' '\n' < "$compatible_file")
[[ $board_compatible == *rockchip,rk3588* ]] || die "this device-tree does not identify an RK3588 board"
if $do_colcon; then
  command -v colcon >/dev/null || die "colcon not found; source the existing ROS environment first"
  [[ -n ${AMENT_PREFIX_PATH-} ]] || die "source the existing ROS underlay before --colcon"
fi
if [[ -n $requested_log_dir ]]; then
  run_dir=$(realpath -m -- "$requested_log_dir")
  [[ ! -e $run_dir && ! -L $run_dir ]] || die "log directory must be new: $run_dir"
  mkdir -p -- "$run_dir"
else
  log_root="$workspace/smoke_logs/wbc_lora_build"
  mkdir -p -- "$log_root"
  run_dir=$(mktemp -d "$log_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
fi
printf 'run_dir=%s\n' "$run_dir"
run_step() {
  local step=$1
  shift
  local step_rc=0
  printf '[%s] log=%s/%s.log\n' "$step" "$run_dir" "$step"
  if "$@" > "$run_dir/$step.log" 2>&1; then step_rc=0; else step_rc=$?; fi
  printf '%s\n' "$step_rc" > "$run_dir/$step.exitcode"
  if [[ $step_rc -ne 0 ]]; then
    tail -n 40 -- "$run_dir/$step.log" >&2
    printf '[%s] failed, exit=%s; subsequent steps skipped\n' "$step" "$step_rc" >&2
    return "$step_rc"
  fi
  printf '[%s] OK\n' "$step"
}
run_step environment "$python_bin" -B - <<'PY'
import importlib.metadata
import platform
import sys
print(platform.platform())
print('Python:', sys.executable, sys.version)
for package in ('rknn-toolkit2', 'numpy', 'onnx'):
    print(package, importlib.metadata.version(package))
if importlib.metadata.version('rknn-toolkit2').split('+')[0] != '2.3.2':
    raise SystemExit('Expected the validated conversion toolchain rknn-toolkit2 2.3.2')
from rknn.api import RKNN
import onnx
import numpy
print('Imports OK; no NPU Runtime initialized')
PY
if [[ -f $bundle/ASSETS.sha256 ]]; then
  run_step assets bash -c 'cd -- "$1" && sha256sum -c ASSETS.sha256' _ "$bundle"
fi
if [[ -f $bundle/SOURCE.sha256 ]]; then
  run_step source bash -c 'cd -- "$1" && sha256sum -c "$2"' _ "$repo" "$bundle/SOURCE.sha256"
fi
run_step convert "${conversion[@]}" --log-dir "$run_dir/models"
if [[ -n $install_to ]]; then
  run_step install "$python_bin" -B "$script_dir/prepare_wbc_deploy.py" "$bundle" "$install_to" --mode install-lora
fi
if $do_colcon; then
  cd -- "$workspace"
  run_step build "${colcon_command[@]}"
  printf 'Build succeeded. In your launch terminal run:\n'
  print_command source "$workspace/install/setup.bash"
fi
printf 'Completed requested build steps. Formal C++ numerical/action/timing acceptance remains pending.\n'
