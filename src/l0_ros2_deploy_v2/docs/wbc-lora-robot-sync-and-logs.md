# RK3588 WBC-LoRA 部署与使用手册

更新：2026-10-27。本文包含当前常驻参数版本的同步、转换、构建、启动、动作触发、故障恢复和日志回收步骤，可以独立使用。

最新已核查板端运行：`20261027_103346_3Rz7nA`。FP16/AFFINE 初始化误拒绝已解决，实际 Runtime 为 2.3.2、driver 为 0.9.8；两个 LoRA 动作共完成 7 次、2100 次推理（含 7 次准备首帧），WBC 超时为 0，采样 cycle 最大 14.153 ms。参数在动作期间保持驻留，AMP/WBC 交接完成。

**这批日志不能用于板端与本地的数值残差验收。** `wbc_sequence.log` 只有 0～2099 的序号，未保存逐帧 policy[399]、command_short[240]、裁剪前 action[21] 或 reference 输入/输出。四份转换记录均为 `verification_backend: not_run`、`verification: null`；`max_abs_error` 是配置阈值，不是测得的残差。不能从耗时、序号、模型哈希或静止反馈事后恢复上述数值。本地导出样本误差也不能代替这次板端运行的误差。

Base 动作尚未触发；运行中有一次 AMP 命令过期，退出时出现 disable 投递确认超时，仍需排查。当前结论限定为已测两个 LoRA 动作的加载、驻留切换与推理链路通过隔离运行检查，未宣布数值精度或真实闭环验收通过。

## 1. 环境与操作约定

本地主机只负责同步和日志回收。RKNN 转换、ROS 构建和运行均在 RK3588 开发板执行。本文使用板端 HPM 隔离反馈，保留正式控制器和 WBC worker；静止关节/IMU 反馈用于检查软件链路，不能判断真实机器人运动效果。

| 项目 | 本文使用值 |
| --- | --- |
| 本地主机工作空间 | `/home/luvbot/SC/l0_deploy_v2_ws` |
| 板端 SSH | `forlinx@192.168.5.2` |
| 板端工作空间 | `/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws` |
| 仓库相对路径 | `src/l0_ros2_deploy_v2` |
| 源资产相对路径 | `deliveries/wbc_lora_resident_20261026` |
| 安装策略目录名 | `wbc_lora_resident` |
| ROS underlay | `/opt/ros/humble/setup.bash` |
| Toolkit2 环境 | `/usr/bin/python3`，`rknn-toolkit2==2.3.2` |
| 目标平台 | Linux aarch64、RK3588、RKNN Runtime 2.3.2 |
| 隔离 ROS 环境 | `ROS_DOMAIN_ID=97`、`ROS_LOCALHOST_ONLY=1` |

板端须已有 ROS2 Humble、colcon、仓库构建依赖、匹配的 `src/luvbot_ros_interfaces`、机器人描述以及 AMP 策略 `legged_deploy/policy/l0_v5/l0_v5_amp_07090`。这是已有部署工作空间的更新流程，源码同步清单不包含操作系统、接口仓库和既有 AMP 模型。RKNN 头文件和库须与随包 `SOURCE.sha256` 一致，校验失败时先解决版本差异。

| 动作 ID | 参数类型 | 名义持续时间 |
| --- | --- | --- |
| `L0_Hugme` | LoRA，记作 A | 6.58 秒 |
| `L0_act_cute_1` | LoRA，记作 B | 5.74 秒 |
| `L0_act_cute_2` | 显式 Base | 7.42 秒 |

Base 包将 LoRA A/B 置零并恢复基础 bias、LayerNorm，搭配 `L0_act_cute_2` 的原 reference；实际动作效果尚未验收。WBC 共享模型为 `base_lora.onnx/rknn`，AMP 行走模型通过 `subpolicy` 单独选择。

**每个命令块都重新设置必要变量；每个 ROS 命令块都重新 source 环境。请复制完整代码块。** 外层 `( ... )` 是子 shell，失败只结束本步骤，变量不会保留给下一个块。本地主机的 build/install/log 不同步到板端。

每个板端新终端都先独立 SSH 登录；以下一行在本地主机执行，登录成功后再执行相应板端代码块：

```bash
ssh forlinx@192.168.5.2
```

## 2. 本地主机：同步源码与源资产

先查看板端已有修改并预览。清单覆盖 LoRA 工具、常驻参数源码、启动链和隔离节点，不使用 `--delete`。如会覆盖板端独有修改，先保留并合并这些修改。

```bash
(
  set -e
  local_ws=/home/luvbot/SC/l0_deploy_v2_ws
  local_repo="$local_ws/src/l0_ros2_deploy_v2"
  local_bundle="$local_ws/deliveries/wbc_lora_resident_20261026"
  robot_target=forlinx@192.168.5.2
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  ssh "$robot_target" "git -C '$robot_repo' status --short && df -h '$robot_ws'"
  rsync -rtvn --itemize-changes --files-from="$local_repo/docs/wbc-lora-sync-files.txt" \
    "$local_repo/" "$robot_target:$robot_repo/"
  rsync -rtvn --itemize-changes "$local_bundle/" "$robot_target:$robot_bundle/"
)
```

确认预览内容后执行实际同步。以下完整块也可在另一个本地主机终端直接运行，结束时校验板端源码与资产：

```bash
(
  set -e
  local_ws=/home/luvbot/SC/l0_deploy_v2_ws
  local_repo="$local_ws/src/l0_ros2_deploy_v2"
  local_bundle="$local_ws/deliveries/wbc_lora_resident_20261026"
  robot_target=forlinx@192.168.5.2
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  (cd "$local_bundle" && sha256sum -c ASSETS.sha256)
  (cd "$local_repo" && sha256sum -c "$local_bundle/SOURCE.sha256")
  ssh "$robot_target" "test -d '$robot_repo' && mkdir -p '$robot_bundle'"
  rsync -rtv --itemize-changes --files-from="$local_repo/docs/wbc-lora-sync-files.txt" \
    "$local_repo/" "$robot_target:$robot_repo/"
  rsync -rtv --itemize-changes "$local_bundle/" "$robot_target:$robot_bundle/"
  ssh "$robot_target" \
    "cd '$robot_bundle' && sha256sum -c ASSETS.sha256 && cd '$robot_repo' && sha256sum -c '$robot_bundle/SOURCE.sha256'"
)
```

`SOURCE.sha256` 路径相对仓库，`ASSETS.sha256` 路径相对源资产目录，两者均需全部通过。源码变更必须更新交付清单，不能仅用 Git HEAD 判断版本。板端转换会新增 RKNN 文件与转换记录，不改写清单内的 ONNX、权重等源资产。

## 3. 板端：转换、安装与 ROS 构建

首次部署推荐使用以下统一入口，无需再执行第 4 节的单模型或批量转换。先停止此前运行的部署进程。

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  bundle_name=wbc_lora_resident
  toolkit_python=/usr/bin/python3
  build_root="$robot_ws/smoke_logs/wbc_lora_build"
  build_dir="$build_root/$(date +%Y%m%d_%H%M%S_%N)"
  source /opt/ros/humble/setup.bash
  test -d "$robot_ws/src/luvbot_ros_interfaces"
  mkdir -p "$build_root"
  bash "$robot_repo/build_wbc_lora_rk3588.sh" "$robot_bundle" \
    --python "$toolkit_python" --log-dir "$build_dir" \
    --install-to "$robot_repo/legged_deploy/policy/l0_v5/$bundle_name" --colcon
  source "$robot_ws/install/setup.bash"
  deploy_prefix=$(ros2 pkg prefix legged_deploy)
  [[ "$deploy_prefix" == "$robot_ws/install" || "$deploy_prefix" == "$robot_ws/install/"* ]]
  ros2 pkg prefix legged_real
  ldd "$deploy_prefix/lib/legged_deploy/rl_controller" | tee "$build_dir/rl_controller.ldd.log"
  printf '%s\n' "$build_dir" > "$build_root/latest_successful_build_dir"
  printf '构建成功，日志：%s\n' "$build_dir"
)
```

统一脚本检查 Linux/aarch64、设备树 RK3588、Toolkit2 2.3.2 和随包哈希，依次执行：

1. 编译一份共享 policy 和三个动作 reference，共 4 个 RKNN。
2. 将完整资产安装到源码下的新策略目录，生成 `INSTALLATION.json`。
3. 从工作空间根目录运行 `colcon build --packages-up-to legged_deploy legged_real legged_controllers --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN`，构建依赖并安装 package share。

任一阶段失败即停止。阶段日志和 `.exitcode` 在打印的构建目录中；逐模型日志在 `models/`，汇总为 `models/conversion_summary.json`。`--build-only` 不启动 Python NPU Runtime，记录中的 `converted_pending_runtime` 表示仍待正式 C++ 运行验收。

**安装目的目录必须不存在。** 如果安装成功、colcon 失败，修复代码后使用下面的“仅重建代码”块，不重复安装。更新资产时用新的目录名，并同时修改构建和启动块中的 `bundle_name`。

只有代码或启动文件变更、已安装资产不变时，执行以下完整块：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  bundle_name=wbc_lora_resident
  build_root="$robot_ws/smoke_logs/wbc_lora_build"
  source /opt/ros/humble/setup.bash
  test -f "$robot_repo/legged_deploy/policy/l0_v5/$bundle_name/INSTALLATION.json"
  mkdir -p "$build_root"
  build_dir=$(mktemp -d "$build_root/code_$(date +%Y%m%d_%H%M%S)_XXXXXX")
  cd "$robot_ws"
  set +e
  colcon build --packages-up-to legged_deploy legged_real legged_controllers \
    --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN 2>&1 | tee "$build_dir/build.log"
  result=("${PIPESTATUS[@]}")
  set -e
  printf '%s\n' "${result[0]}" > "$build_dir/build.exitcode"
  test "${result[0]}" -eq 0
  test "${result[1]}" -eq 0
  source "$robot_ws/install/setup.bash"
  ros2 pkg prefix legged_deploy
  printf '%s\n' "$build_dir" > "$build_root/latest_successful_build_dir"
  printf '构建成功，日志：%s\n' "$build_dir"
)
```

## 4. 板端：单模型、批量编译的独立用法

这些入口用于单独转换或排查问题，不安装资产、不构建 ROS。LoRA 的动态加载是运行时切换参数值，输入形状固定；保留 2 个观测输入和 31 个参数输入。不会为每个动作合并编译一个 policy，`weights.bin` 无需转成 RKNN。转换配置固定为 RK3588、FP16、非量化。

仅编译共享 policy：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  toolkit_python=/usr/bin/python3
  "$toolkit_python" -B "$robot_repo/convert_onnx_to_rknn_rk3588_wbc_policy.py" \
    "$robot_bundle/base_lora.onnx" --manifest "$robot_bundle/base_manifest.json" --build-only
)
```

批量编译共享 policy 和所有动作 reference，并保存逐模型日志：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  robot_bundle="$robot_ws/deliveries/wbc_lora_resident_20261026"
  toolkit_python=/usr/bin/python3
  compile_dir="$robot_ws/smoke_logs/wbc_lora_compile/$(date +%Y%m%d_%H%M%S_%N)"
  "$toolkit_python" -B "$robot_repo/batch_convert_onnx_to_rknn_rk3588.py" "$robot_bundle" \
    --manifest "$robot_bundle/base_manifest.json" --build-only --log-dir "$compile_dir"
  printf '转换日志：%s\n' "$compile_dir"
)
```

产物位于各 ONNX 旁边：`base_lora.rknn` 与 `base_lora.rknn.json`，以及各 `actions/<动作>/reference_motion.rknn` 和对应 `.rknn.json`。

| 选项 | 作用 |
| --- | --- |
| `--dry-run` | 单模型、批量、统一入口均支持；校验并打印计划，不编译、不写文件 |
| `--policy-only` | 批量或统一入口只转换共享 policy，不能与 `--include-probes` 或完整安装合用 |
| `--include-probes` | 批量或统一入口追加 3 个诊断 probe，总计 7 个；probe 不参与 ROS 启动 |
| `--skip-existing` | 批量或统一入口按 manifest、哈希、转换配置、工具及 SDK 版本匹配后复用 |
| `--python /绝对路径/python3` | 统一入口选择已有 Toolkit2 2.3.2 的 Python |
| `--log-dir /新目录` | 批量或统一入口指定日志目录，该目录必须尚不存在 |

旧转换记录缺少当前要求的来源字段时会重新编译，仅存在 RKNN 文件不够。直接运行 Python 转换脚本时也应使用板端 Toolkit2 2.3.2 环境。

## 5. 板端终端 A：从工作空间直接启动（不依赖 delivery）

上次运行已经加载工作空间中的正式模型。源码策略位于 `src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/wbc_lora_resident`；colcon 安装后，launch 实际使用 `install/legged_deploy/share/legged_deploy/policy/l0_v5/wbc_lora_resident`，通过 `ros2 pkg prefix legged_deploy` 定位。仅切换当前工作目录不会改变 launch 的模型选择。

已经转换、安装并构建成功后，日常启动直接执行下面完整块，完全不读取 `deliveries`。保留共享 RKNN、`base_manifest.json`、各 adapter manifest/weights、动作配置、reference RKNN 和转换来源记录；只拿出一个 RKNN 文件不足以运行动态 LoRA。模型没有变化时不需要重复转换或安装；修改源码策略目录后，仍需 colcon 安装再启动。

只保留一套本次部署进程，不同时运行真实 HPM 或另一套 AMP/WBC。下面继续使用 HPM 隔离反馈。主启动块创建本次日志目录，把路径存入 `smoke_logs/wbc_lora_resident/current_run_dir`。其他终端重新读取该文件，无需手动复制 `run_dir`。

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
  bundle_name=wbc_lora_resident
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
  deploy_prefix=$(ros2 pkg prefix legged_deploy)
  [[ "$deploy_prefix" == "$robot_ws/install" || "$deploy_prefix" == "$robot_ws/install/"* ]]
  installed_bundle="$deploy_prefix/share/legged_deploy/policy/l0_v5/$bundle_name"
  test -f "$installed_bundle/INSTALLATION.json"
  test -f "$installed_bundle/base_lora.rknn"
  mkdir -p "$run_root"
  run_dir=$(mktemp -d "$run_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
  export ROS_LOG_DIR="$run_dir/ros"
  mkdir -p "$ROS_LOG_DIR" "$run_dir/conversion"
  printf '%s\n' "$run_dir" > "$run_root/current_run_dir"
  if [[ -f "$robot_ws/smoke_logs/wbc_lora_build/latest_successful_build_dir" ]]; then
    build_dir=$(cat "$robot_ws/smoke_logs/wbc_lora_build/latest_successful_build_dir")
    if [[ "$build_dir" == "$robot_ws/smoke_logs/wbc_lora_build/"* && -d "$build_dir" ]]; then
      printf '%s\n' "$build_dir" > "$run_dir/build_dir.txt"
    fi
  fi
  printf '%s\n' "$installed_bundle" > "$run_dir/installed_bundle.txt"
  (
    cd "$robot_repo"
    while IFS= read -r relative; do
      sha256sum "$relative"
    done < docs/wbc-lora-resident-sync-files.txt
    sha256sum legged_deploy/third_party/rknn_runtime/include/rknn_api.h \
      legged_deploy/third_party/rknn_runtime/lib/librknnrt.so
  ) > "$run_dir/SOURCE.sha256"
  (
    set -o pipefail
    cd "$installed_bundle"
    find . -type f -print0 | sort -z | xargs -0 -r sha256sum
  ) > "$run_dir/ASSETS.sha256"
  cp "$installed_bundle/INSTALLATION.json" "$run_dir/"
  cp "$installed_bundle/base_lora.rknn.json" "$run_dir/conversion/"
  for action in L0_Hugme L0_act_cute_1 L0_act_cute_2; do
    cp "$installed_bundle/actions/$action/reference_motion.rknn.json" \
      "$run_dir/conversion/${action}_reference.rknn.json"
  done
  sha256sum "$deploy_prefix/lib/legged_deploy/rl_controller" > "$run_dir/binary.sha256"
  ldd "$deploy_prefix/lib/legged_deploy/rl_controller" > "$run_dir/rl_controller.ldd.log"
  find "$installed_bundle" -type f -name '*.rknn' -exec sha256sum {} + > "$run_dir/models.sha256"
  printf 'run_dir=%s\n' "$run_dir"
  trap ':' INT
  set +e
  ros2 launch legged_controllers sim2real.launch.py \
    robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 \
    wbc_enabled:=true wbc_mode:=lora wbc_lora_bundle:="$bundle_name" \
    'wbc_policies:=[L0_Hugme, L0_act_cute_1, L0_act_cute_2]' \
    hpm_mode:=isolated isolated_ros_domain:=97 rviz:=false battery_enabled:=false \
    2>&1 | tee "$run_dir/launch.log"
  result=("${PIPESTATUS[@]}")
  printf '%s\n' "${result[0]}" > "$run_dir/launch.exitcode"
  printf '%s\n' "${result[1]}" > "$run_dir/launch_tee.exitcode"
  trap - INT
)
```

等待硬件桥就绪、AMP 与 WBC worker 启动。日志应给出共享 Base 路径、Runtime/driver、逻辑/native 属性及初始化信息。仅 AMP 的 `Controller started successfully` 不代表 WBC 已就绪；还必须确认 `/rl_controller_wbc` 存活和 `/control/policy/wbc/prepare` 服务存在。遇到不支持的布局、哈希错误或 worker 启动失败，先停止排查。

该块持续占用终端 A。每次重启都会创建新日志目录；已经运行的监听终端须停止并重新执行完整监听块，才能切换到新的路径。路径文件只存放目录名称，不继承或替代任何 ROS 环境变量。

## 6. 板端终端 B：检查端点并监听动作反馈

在新 SSH 终端执行以下完整块，先记录端点和实际 Runtime 库，再持续监听反馈。必须在发送动作前开始监听，volatile QoS 不回放此前反馈。

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  set -o pipefail
  ros2 node info /topic_hardware_adapter | tee "$run_dir/hardware_endpoints.log"
  ros2 node info /rl_controller_wbc | tee "$run_dir/wbc_endpoints.log"
  /usr/bin/python3 - <<'PY' | tee -a "$run_dir/runtime_maps.log"
import datetime
import hashlib
from pathlib import Path

def digest(path):
    value = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()

print(datetime.datetime.now().isoformat(), flush=True)
found = False
for directory in Path('/proc').iterdir():
    if not directory.name.isdigit():
        continue
    try:
        command = (directory / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
        if '/rl_controller' not in command or '__node:=rl_controller_wbc' not in command:
            continue
        found = True
        print('PID:', directory.name, command)
        print('EXE:', (directory / 'exe').resolve(), digest(directory / 'exe'))
        print('FD count:', len(list((directory / 'fd').iterdir())))
        for line in (directory / 'status').read_text().splitlines():
            if line.startswith(('VmRSS:', 'VmSize:', 'Threads:')):
                print(line)
        libraries = set()
        for line in (directory / 'maps').read_text().splitlines():
            if 'librknnrt' in line:
                print(line)
                libraries.add(line.split(maxsplit=5)[5])
        for library in sorted(libraries):
            print('LIB:', library, digest(library))
    except (FileNotFoundError, PermissionError, ProcessLookupError) as error:
        print('READ ERROR:', directory.name, error)
if not found:
    raise SystemExit('WBC worker not found; inspect launch.log before sending actions')
PY
  ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String \
    --qos-reliability reliable --qos-durability volatile | tee -a "$run_dir/actions.log"
)
```

硬件桥的电机、IMU 等硬件端点应指向 `/wbc_isolated/...`，不能仍连接真实 `/actuator/motor/...`。WBC 命令仍使用正式控制话题。库身份以进程 `/proc/<pid>/maps` 为准，不根据 `LD_LIBRARY_PATH` 推测。

动作反馈应包含 `started`、`completed`；如出现 `failed`、`rejected`，结合主日志中的 reason 排查。

如需观察 WBC 命令帧，可在另一个板端 SSH 终端执行下列块。该监听增加终端/磁盘负载，不用于严格性能采样：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  set -o pipefail
  ros2 topic echo /control/policy/command/wbc luvbot_control_interfaces/msg/PolicyCommand \
    --qos-reliability best_effort --qos-durability volatile --field sequence_id \
    | tee -a "$run_dir/wbc_sequence.log"
)
```

## 7. 板端终端 C：进入 Walking、逐个触发动作

通过原手柄 **B+Y** 从 Standby 进入 Walking，保持零速度。确认主日志出现进入 Walking 的信息，AMP 持续产生新鲜输出；Standby 或 EmergencyStop 下不能开始 WBC 动作。

无手柄且 `/joy` 没有其他发布者时，可执行下列输入替代手柄。命令会检查发布者数量，有其他发布者时停止，避免两路输入互相覆盖。

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  joy_publishers=$(ros2 topic info /joy | awk '/^Publisher count:/ {print $3}')
  if [[ "$joy_publishers" != 0 ]]; then
    printf '已有 /joy 发布者或无法确认数量，请使用原手柄 B+Y。\n' >&2
    exit 1
  fi
  ros2 topic pub --once /joy sensor_msgs/msg/Joy \
    '{axes: [0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
  ros2 topic pub --once /joy sensor_msgs/msg/Joy \
    '{axes: [0,0,0,0,0,0], buttons: [0,1,0,0,1,0,0,0,0,0,0,0,0,0,0]}'
  ros2 topic pub --once /joy sensor_msgs/msg/Joy \
    '{axes: [0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
)
```

随后执行完整动作块。每次仅修改 `action_id` 后重跑整个块，自动生成新的毫秒时间戳，不能复用旧 payload：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  service_type=$(ros2 service type /control/policy/wbc/prepare)
  if [[ "$service_type" != luvbot_control_interfaces/srv/ControlWbcMotion ]]; then
    printf 'WBC prepare 服务未就绪，请先检查 launch.log 中的 worker 初始化错误。\n' >&2
    exit 1
  fi
  action_id=L0_Hugme
  payload=$(/usr/bin/python3 -c 'import json,sys,time; print(json.dumps({"data": json.dumps({"timestamp_ms": time.time_ns()//1000000, "action_id": sys.argv[1]})}))' "$action_id")
  printf '%s\n' "$payload" | tee -a "$run_dir/action_requests.log"
  ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String "$payload"
)
```

每次等待 `started`、`completed` 和交回 AMP，再发下一条，不用固定 sleep 代替反馈确认。一次失败后先恢复，不继续发后续动作。

| 检查 | 依次设置的 action_id |
| --- | --- |
| LoRA 互切 | `L0_Hugme` → `L0_act_cute_1` → `L0_Hugme` |
| 重复动作 | `L0_Hugme` → `L0_Hugme`，每次生成新时间戳 |
| Base 首次加载 | 重启整套链路、重新监听并进入 Walking，首个动作发 `L0_act_cute_2` |
| LoRA / Base 互切 | `L0_Hugme` → `L0_act_cute_2` → `L0_Hugme` |
| Base / LoRA 互切 | `L0_act_cute_2` → `L0_Hugme` → `L0_act_cute_2` |

## 8. 板端新终端：反馈中断与恢复

故障检查与性能采样分开，主链路须仍为隔离模式。下列块默认中断电机反馈；若单独检查 IMU，将 `fault_parameter` 改为 `imu_feedback_enabled`。一次只检查一种故障。

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  fault_parameter=feedback_enabled
  printf '%s set %s false\n' "$(date -Is)" "$fault_parameter" | tee -a "$run_dir/faults.log"
  set -o pipefail
  ros2 param set /isolated_hpm "$fault_parameter" false | tee -a "$run_dir/faults.log"
)
```

应看到原 watchdog/故障路径拒绝或停止动作，保留 reason、反馈与停机日志。恢复反馈可在任意新板端终端执行：

```bash
(
  set -e
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  source /opt/ros/humble/setup.bash
  source "$robot_ws/install/setup.bash"
  export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros"
  set -o pipefail
  printf '%s restore feedback\n' "$(date -Is)" | tee -a "$run_dir/faults.log"
  ros2 param set /isolated_hpm feedback_enabled true | tee -a "$run_dir/faults.log"
  ros2 param set /isolated_hpm imu_feedback_enabled true | tee -a "$run_dir/faults.log"
)
```

恢复反馈不会自动解除 EmergencyStop。故障消失后按原手柄 **LB+RB** 复位，释放后再按 B+Y 进入 Walking。没有可用手柄时，停止主链路和监听，重新执行第 5～7 节的完整块。若 RKNN 回退失败、Context 已禁用，需要重启链路，不能仅复位状态机。

## 9. 停止、归档与日志回收

先在终端 A 按 Ctrl+C，等待主 launch 退出；原脚本按主节点、硬件桥、隔离端的顺序清理，使 disable 仍有接收端。随后停止所有监听终端，确认没有进程继续写本次日志，再归档。主动 Ctrl+C 的退出码须结合日志判断，不能单独作为动作通过或失败证据。

在新的板端终端执行以下完整块，不需要 ROS 环境。它复制本次构建日志，生成日志校验清单并记录已归档路径：

```bash
(
  set -e

  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  run_root="$robot_ws/smoke_logs/wbc_lora_resident"
  run_dir=$(cat "$run_root/current_run_dir")
  [[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
  test -f "$run_dir/launch.exitcode"
  if [[ -f "$run_dir/build_dir.txt" ]]; then
    build_dir=$(cat "$run_dir/build_dir.txt")
    [[ "$build_dir" == "$robot_ws/smoke_logs/wbc_lora_build/"* && -d "$build_dir" ]]
    if [[ ! -e "$run_dir/build" ]]; then
      cp -a "$build_dir" "$run_dir/build"
    fi
  fi
  printf '%s\n' "$(date -Is)" > "$run_dir/ARCHIVED_AT.txt"
  list=$(mktemp)
  trap 'rm -f -- "$list"' EXIT
  cd "$run_dir"
  set -o pipefail
  find . -type f ! -name LOGS.sha256 -print0 | sort -z | xargs -0 -r sha256sum > "$list"
  mv "$list" LOGS.sha256
  sha256sum -c LOGS.sha256
  printf '%s\n' "$run_dir" > "$run_root/last_completed_run_dir"
  du -sh "$run_dir"
  printf '已归档：%s\n' "$run_dir"
)
```

在**本地主机的新终端**执行以下块，读取板端最后一次已归档运行路径并取回日志，无需沿用此前变量：

```bash
(
  set -e

  local_ws=/home/luvbot/SC/l0_deploy_v2_ws
  robot_target=forlinx@192.168.5.2
  robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
  remote_root="$robot_ws/smoke_logs/wbc_lora_resident"
  remote_run_dir=$(ssh "$robot_target" "cat '$remote_root/last_completed_run_dir'")
  [[ "$remote_run_dir" == "$remote_root/"* ]]
  run_id=${remote_run_dir##*/}
  [[ "$run_id" =~ ^[0-9]{8}_[0-9]{6}_[[:alnum:]]{6}$ ]]
  [[ "$remote_run_dir" == "$remote_root/$run_id" ]]
  local_logs="$local_ws/smoke_logs/wbc_lora_resident/$run_id"
  mkdir -p "$local_logs"
  rsync -rtv "$robot_target:$remote_run_dir/" "$local_logs/"
  cd "$local_logs"
  sha256sum -c LOGS.sha256
  printf '日志已回收：%s\n' "$local_logs"
)
```

启动时的 `SOURCE.sha256` 是工作目录源码快照，`ASSETS.sha256` 是已安装策略目录的快照；不再从 delivery 复制。前者不能单独证明二进制的构建来源，仍须结合二进制哈希和已有构建日志。未找到构建日志时仍可启动，但本次归档不含 `build/`。

主要证据：`launch.log`、`actions.log`、`action_requests.log`、端点记录、`runtime_maps.log`、二进制/模型哈希、`conversion/`、`build/` 和 `ros/`。仅重建代码时，`build/` 只含该次重建日志；首次转换、安装的原构建目录也应保留，转换记录已随运行单独收集。日志没有自动轮转，不承诺固定容量上限。

## 10. 验收口径与常见问题

| 项目 | 通过条件 / 待取得证据 |
| --- | --- |
| 版本身份 | 源码/资产哈希通过；保留实际二进制、实际加载库、模型哈希和 Runtime/driver |
| 构建产物 | 四个正式 RKNN、安装和 colcon 均成功；pending_runtime 不等于运行通过 |
| 动作链路 | A、B、Base 完整完成；首个 Base、重复 A、混合切换有反馈与 AMP 交接 |
| 参数常驻 | 共享 policy `contexts=1`；比较 prepare 后和 completed 的累计计数，参数写入/转换/bind/sync 不随帧数增长；正常每帧只新增两次观测同步和一次输出同步 |
| 数值 | 同板同模型、相同固定观测/历史，对比旧 FP32 输入提交、C++ 常驻路径及随包期望，覆盖三动作及切换首帧；Python verifier 不能替代 C++ 路径 |
| 时序 | 保存完整动作样本数、reference/policy/cycle 的平均、P95/P99、最大值和失败计数；保持原 20 ms 门限，与同负载旧基线比较 |
| 故障恢复 | 反馈中断、输入过期、加载时取消、0.5 秒准备超时后再 prepare、损坏包、schema/归一化不匹配、绑定/回退失败及退出清理 |
| 回归与资源 | AMP/static 同模型回归；反复切换的 RSS、DMA/驱动内存、句柄变化及退出释放 |

累计计数包含 prepare 成本；取消的加载可能已写备用 slot，这不属于稳态逐帧上传。P95/P99 是 0.1 ms 直方图区间上界，溢出桶表示至少 100 ms；cycle 不覆盖外层锁等待和调度，仍需结合命令时效判断。SDK profiling 另跑，避免改变性能采样负载。

反馈中断可按本文操作；其余故障注入和 C++ 同输入数值对照需要专门的板端验证安排，不要篡改正式资产或放宽校验。若参数计数符合要求但仍超 20 ms，分别报告“参数常驻通过”和“周期预算未通过”。隔离动作完成不能替代真实闭环验收。

| 现象 | 处理 |
| --- | --- |
| 新终端找不到 ros2 / 话题 | 执行完整块，检查 underlay、本工作空间 install、domain 97、localhost 1 |
| `current_run_dir` 不存在 | 先完成主启动块，不要把 run_dir 设为空或改到根目录 |
| 安装目录已存在 | 资产未变则仅重建代码；更新资产用新目录名并调整启动参数 |
| 源码或资产哈希失败 | 核对失败文件、板端修改及 SDK，重新同步对应版本，不跳过校验 |
| `Unsupported resident WBC layout` | 当前支持 UNDEFINED、1/2 维、FP16/NONE、FP16/AFFINE、FP32/NONE 及受检末维 padding/stride；旧版本会误拒绝 FP16/AFFINE，需同步 C++ 并重建；其他情况保存逻辑/native 属性及版本，不改 attr 绕过 |
| prepare 服务不存在 / worker did not respond | 先检查 WBC worker 是否已退出，修复初始化错误后重启整套链路；该故障不能靠重复发 B+Y 或动作解决 |
| 动作被拒绝 | 核对三个动作 ID、安装目录、启动清单、Walking、零速度与 AMP 新鲜输出 |
| 命令过期或去重 | 重跑动作块生成时间戳；过去 5 秒、未来 1 秒时限，相同 timestamp/action 去重窗口 6 秒 |
| 恢复反馈仍不能开始 | EmergencyStop 需要复位；Context 禁用需要重启；随后重新检查 Walking 和 AMP |
| 启动成功但没有 completed | 保留完整 reason 与时序；启动成功或少量推理帧不能判定验收通过 |

## 11. 后续正式部署与数值验收边界

正式模型部署路径已确定：源码策略目录 → 工作空间 colcon build → source install → sim2real 正式启动链 → Walking → 原动作话题。日常运行不使用 delivery，HPM 隔离与真实 HPM 使用同一 WBC 模型加载和推理实现。

后续接入真实电机时，入口选 `hpm_mode:=real`，由既有 HPM 提供真实电机和 IMU 反馈；此模式不会自动启动隔离节点，也不会替终端设置 ROS domain。启动、监听、触发终端需要明确设置与真实 HPM 相同的 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 和通信配置，不能直接继承隔离验收的 domain 97。真实 HPM 环境值尚未在本次会话中确认，因此本文没有假定它为 domain 0，也未执行真实硬件启动。

上次日志只能确认程序执行与时序，无法补算残差。要验证推理计算，后续需另外取得同一帧的 policy[399]、command_short[240]、裁剪前 action[21]，以及当时的 adapter/模型身份；reference 则需 time_step 与各输出数组。本地用相同输入、相同完整参数包运行对应 ONNX，分别统计 policy 和 reference 的最大绝对误差、平均绝对误差、RMSE、各动作及切换首帧结果。还需区分 FP16 输入舍入误差和 RKNN 算子计算误差。相同 nominal 时间、动作名或“静止反馈”不足以对齐包含历史动作的输入。

本次仅核查旧日志并更新流程说明，没有新增数值采集代码。正式数值通过结论须基于实际可对齐的数据；当前仍保留 Base、数值对照、AMP 偶发时效与退出确认问题的未通过状态。
