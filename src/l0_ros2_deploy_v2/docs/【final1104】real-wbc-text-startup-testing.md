# 真机 WBC 与文本生成：启动及测试步骤

更新：2026-11-04。适用于当前部署源码、L0 V5、RK3588、ROS 2 Humble。

该代码已集成wbc、文本生成两部分部署功能。

本文分两轮测试：先运行普通 WBC 的固定动作，再退出该控制会话，切换到文本生成动作。两套入口使用相同控制话题和硬件接口，不同时运行。2026-11-04 已在 `forlinx@192.168.5.2` 的 `/userdata/forlinx/wbc_text_workspace1104` 完成上传、编译、安装检查，以及 3 条文本的板端 NPU 离线生成测试；未启动硬件控制程序、未发送动作指令。第 4～6 节的真机执行验收仍待现场完成，第 7 节记录已完成的验证。

## 1. 当前默认配置

| 项目 | 普通 WBC | 文本生成 WBC |
| --- | --- | --- |
| 启动入口 | `wbc_lora.launch.py` | `text_wbc.launch.py` |
| AMP | `l0_v5_amp_10281` | `l0_v5_amp_10281` |
| WBC 包 | `wbc_lora_fp16` | `text_wbc_bundle` |
| 参数选择 | 对应动作的 LoRA | `UpperBody_Enhanced` LoRA |
| 动作参考 | 包内 reference RKNN | 文本生成的动态 JSON reference |
| 默认预加载 | 86 个固定动作 | 单一 `UpperBody_Enhanced` 参数/reference 基准 |
| 推理 | RKNN | 文本 Lite2 + WBC RKNN |

预加载 86 个动作不会自动执行或轮播动作。两条入口均继承真机 roll 30°、pitch 360°、连续违规 10 次的默认设置；当前 pitch 计算范围为 ±90°，360° 不会触发该项 pitch 越限保护。

## 2. 创建新工作区、上传和编译

新工作区根目录固定为 `/userdata/forlinx/wbc_text_workspace1104`，直接在其下放 `src/`、`model_sources/`、`deploy_config/`，不再嵌套 `l0_deploy_v2_ws`。以下命令明确区分本机与真机终端。旧 `wbc_workspace`、`zy_workspace` 不作为复制或编译目标。

### 2.1 本机终端：在真机创建目录

以下使用此前地址 `192.168.5.2`；如果当前机器人地址不同，只修改 `ROBOT_HOST`。密码由 SSH 交互提示输入，不写入命令。关闭公钥尝试以避免 `Too many authentication failures`。

```bash
export ROBOT_HOST=forlinx@192.168.5.2
ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  "$ROBOT_HOST" 'mkdir -p /userdata/forlinx/wbc_text_workspace1104'
```

如返回 Permission denied，应由现场负责人为 forlinx 准备该目录及写入权限，不要改旧工作区权限。

### 2.2 本机终端：上传当前代码和模型

在本机 `/home/luvbotic/l0_deploy_v5_ws` 执行。推荐 rsync，两端需有 rsync 命令；可重复执行续传，不使用 `--delete`。

```bash
cd /home/luvbotic/l0_deploy_v5_ws
export ROBOT_HOST=forlinx@192.168.5.2

rsync -avz --partial --progress \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  -e 'ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password' \
  src model_sources deploy_config PACKAGE_INFO.json TEXT_MODELS_SHA256SUMS 文本模型放置说明.txt \
  "$ROBOT_HOST:/userdata/forlinx/wbc_text_workspace1104/"
```

只传上述指定项目，不上传本机 `build/`、`install/`、`log/`、`.aws/`、`.codex/` 等内容。`src/` 已包含本部署源码、匹配的 `luvbot_ros_interfaces` 和 `odin_ros_driver_rev1`；模型与配置目录不可漏传。源目录参数这里不加尾部斜杠，目标保留完整的 `src/`、`model_sources/` 和 `deploy_config/` 层级。

如果两端无法使用 rsync，可在本机改用以下 scp 命令，二选一即可；scp 不支持同样的增量续传体验：

```bash
cd /home/luvbotic/l0_deploy_v5_ws
export ROBOT_HOST=forlinx@192.168.5.2
scp -o PubkeyAuthentication=no -o PreferredAuthentications=password -r \
  src model_sources deploy_config PACKAGE_INFO.json TEXT_MODELS_SHA256SUMS 文本模型放置说明.txt \
  "$ROBOT_HOST:/userdata/forlinx/wbc_text_workspace1104/"
```

预期目录如下：

```text
/userdata/forlinx/wbc_text_workspace1104/
├── src/
│   ├── l0_ros2_deploy_v2/
│   ├── luvbot_ros_interfaces/
│   └── odin_ros_driver_rev1/
├── deploy_config/
│   ├── wbc-production.env
│   └── wbc-record-qos.yaml
├── model_sources/text_to_motion/
│   ├── compiled/              最新文本模型、manifest、词表和素材
│   ├── deps/rknnlite/          配套 RKNNLite2
│   └── runtime/librknnrt.so    配套私有 Runtime
├── PACKAGE_INFO.json
├── TEXT_MODELS_SHA256SUMS
└── 文本模型放置说明.txt
```

### 2.3 登录真机并检查上传结果

在本机执行登录：

```bash
ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  forlinx@192.168.5.2
```

登录后，以下为真机终端命令：

```bash
export ROBOT_WS=/userdata/forlinx/wbc_text_workspace1104
cd "$ROBOT_WS"
ls src
sha256sum -c TEXT_MODELS_SHA256SUMS
```

文本模型清单应全部显示 OK；缺失或不匹配时先检查上传结果。该清单校验文本交付文件，不代表整个工作区的所有源码均被它覆盖。普通 WBC 和文本 WBC 资产随源码存放于 `src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/`。

### 2.4 真机终端：在新工作区构建

使用板端 ROS Humble 和现有编译依赖。在机器人不执行动作的维护时段构建；新终端不要手动 source 旧工作区。如果 shell 启动文件自动 source 旧工作区，先在此次构建环境中排除该 overlay。

```bash
export ROBOT_WS=/userdata/forlinx/wbc_text_workspace1104
cd "$ROBOT_WS"
env -i HOME="$HOME" USER="$USER" LOGNAME="$USER" \
  PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 ROBOT_WS="$ROBOT_WS" \
  bash --noprofile --norc <<'BUILD'
set -eo pipefail
cd "$ROBOT_WS"
source /opt/ros/humble/setup.bash
export MAKEFLAGS='-j2 -l2' PYTHONDONTWRITEBYTECODE=1
colcon build --executor sequential \
  --packages-up-to legged_real legged_deploy legged_controllers l0_v5_description \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DLEGGED_DEPLOY_BACKEND=RKNN \
    -DLEGGED_DEPLOY_WBC_SIM=OFF \
    -DRKNN_ROOT="$ROBOT_WS/src/l0_ros2_deploy_v2/legged_deploy/third_party/rknn_runtime"
BUILD
```

这一步编译 ROS 程序并安装已交付的 RKNN/LoRA/reference，不需要重新转换文本模型或 WBC 模型。遇到缺失依赖按报错补齐板端环境后再构建，不通过 source 旧部署 install 掩盖新工作区漏编译问题。依赖含系统 ROS、yaml/OpenSSL 等以及文本运行所需 NumPy、PyYAML、psutil、ruamel.yaml；文本入口会选择本工作区自带的 RKNNLite2 和 Runtime。

构建通过后生成新工作区的 `build/`、`install/`、`log/`。AMP 模型路径包含编译时源码位置，因此部署后保留这份 `src/`，不要构建后移动工作区。后续更新代码/模型后仍需重新安装构建，再使用新终端。

## 3. 每个终端的公共环境

后续终端 A、B、C 都先执行：

```bash
export ROBOT_WS=/userdata/forlinx/wbc_text_workspace1104
source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

ros2 pkg prefix legged_deploy
ros2 pkg prefix legged_controllers
```

两个包的输出路径必须分别为新工作区的 `install/legged_deploy`、`install/legged_controllers`（默认隔离安装）。若指向其他工作区，先修正终端环境再启动。所有终端与 HPM 使用现场相同的 ROS domain，不套用 MuJoCo 的 localhost/domain 77 设置。

HPM、电机校准、使能和手柄急停按现场原流程准备。`hpm_mode:=real` 启动部署控制链，不代替 HPM 电机通信节点的启动。

## 4. 第一轮：普通 WBC 固定动作

### 4.1 终端 A：启动

```bash
ros2 launch legged_controllers wbc_lora.launch.py hpm_mode:=real
```

等待日志显示 WBC worker 就绪、86 个动作预加载完成。按原手柄流程进入 Walking，保持零速度并确认机器人站稳。

### 4.2 终端 B：查看动作反馈

```bash
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

### 4.3 终端 C：请求一个动作

修改第一行的动作 ID，每次重新执行整段，生成新的请求时间戳：

```bash
action_id='L0_strange_new_1_A'

wbc_request=$(/usr/bin/python3 -c \
  'import json,sys,time; print(json.dumps({"timestamp_ms":time.time_ns()//1000000,"action_id":sys.argv[1]}))' \
  "$action_id")

ros2 topic pub --once \
  /algorithm/brain/wbc_action_cmd \
  std_msgs/msg/String \
  "{data: '$wbc_request'}"
```

其他指定动作包括 `L0_strange_1`、`L0_excited_8`、`L0_angry_new_2`、`A-C04_mod`。可选动作完整列表见工作区 `deploy_config/wbc-production.env`。

终端 B 应看到同一请求的 `started → completed`。同时检查实际动作和返回 AMP 后的姿态。每次完成并恢复稳定后再发送下一个请求；`rejected` 或 `failed` 需结合终端 A 日志定位。

### 4.4 可选：使用运行记录脚本和交互控制台

这一组入口替代 4.1 的直接 launch，不同时启动。启动终端执行：

```bash
cd "$ROBOT_WS/src/l0_ros2_deploy_v2"
bash scripts/start_sim2real.sh
```

另一个终端执行：

```bash
cd "$ROBOT_WS/src/l0_ros2_deploy_v2"
bash scripts/wbc_action_console.sh
```

输入编号或完整动作 ID，`list` 显示列表，`q` 退出。启动脚本读取正式 env 并生成 `runs/wbc_production/`；控制台读取本次运行快照。不要把依赖 run 的 shell 控制台直接接到没有生成 run 的 4.1 启动流程上。

本次 rsync 上传排除了 Git 元数据，新工作区默认按 4.1 的直接 ROS 入口测试。此可选记录入口会调用 Git 保存源码版本和 diff，需要另行准备有效的 Git 工作树；纯文件复制目录不保证该入口成功。直接 ROS launch 不依赖这项版本记录逻辑。控制台/启动脚本的 `--dry-run` 可用于配置预览，但不能替代实际模型加载验证。

## 5. 切换到文本生成测试

普通动作完成、机器人恢复稳定后，按现场流程退出控制会话，在普通 WBC 启动终端按 Ctrl+C，等待子进程退出。随后再启动文本入口，不保留两套部署控制程序同时运行。

### 5.1 终端 A：启动文本 WBC

完成第 3 节环境设置后执行：

```bash
ros2 launch legged_controllers text_wbc.launch.py hpm_mode:=real
```

等待 JSON WBC 接入启用、worker 就绪。此时只选择 `UpperBody_Enhanced`，不是预加载普通包中的 86 个动作。按原手柄流程进入 Walking，保持零速度站稳。

### 5.2 终端 C：设置文本模型和输出路径

完成第 3 节环境设置后执行：

```bash
unset TEXT_WBC_MODEL_ROOT RKNN_RUNTIME_LIB TEXT_WBC_PYTHON

export TEXT_WBC_MODEL="$ROBOT_WS/model_sources/text_to_motion/compiled/manifest.json"
deploy_share=$(ros2 pkg prefix --share legged_deploy)
mapping_share=$(ros2 pkg prefix --share system_command_arbiter)
export TEXT_WBC_BUNDLE="$deploy_share/policy/l0_v5/text_wbc_bundle/base_manifest.json"
export TEXT_WBC_MAPPING="$mapping_share/config/l0_v5.yaml"

export TEXT_WBC_RUN="$ROBOT_WS/records/text_$(date +%Y%m%d_%H%M%S_%N)"
mkdir -p "$TEXT_WBC_RUN"
```

`text_wbc_generate` 自动选择所属工作区的配套 RKNNLite2 和 Runtime。模型路径应指向最新 `compiled/manifest.json`，不要改成旧 deliveries 的 manifest 或单个 `.rknn`。

### 5.3 生成并执行

修改 `motion_text` 即可更换动作描述。每次重新执行整段，使用新输出文件名：

```bash
motion_text='头以小幅度慢速点头。'
motion_output="$TEXT_WBC_RUN/action_$(date +%Y%m%d_%H%M%S_%N).json"

ros2 run legged_controllers text_wbc_generate \
  --model "$TEXT_WBC_MODEL" \
  --bundle "$TEXT_WBC_BUNDLE" \
  --mapping "$TEXT_WBC_MAPPING" \
  --base-action UpperBody_Enhanced \
  --text "$motion_text" \
  --seed 42 \
  --output "$motion_output" \
  --execute
```

后续可替换为此前用户反馈可用的描述，例如：

```text
头以大幅度慢速点头两次。
头以小幅度快速左右倾斜三次。
头以中幅度慢速向左转。
左手以大幅度抬起。
右手以小幅度慢速前后摆动两次。
左手摸肚子两次。
头点头；右手放后面。
头以小幅度慢速点头；左手以大幅度抬起。
左手以小幅度前后摆动两次；右手放后面。
头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。
```

这些是已有用户反馈样例，不表示本次已逐项通过真机测试。短提示词“点头”曾生成额外右手动作，测试需要核对实际语义表现。

仅生成 JSON、不请求执行时，去掉 `--execute`；上述实时用法仍需运行中的控制链提供起始观测。生成时会采集全身起姿，当前 JSON 有短时有效期，每次执行重新生成，不能直接把旧 JSON 当成可反复播放的固定动作。

真机默认根部参考取 `UpperBody_Enhanced` reference 第 0 帧；本流程不添加 `--root-height`，也不把 XML 初始高度 0.4575 m 写成固定目标高度。

## 6. 观察结果和定位失败阶段

文本终端正常状态顺序：

```text
generating → generated → ready → started → completed
```

| 状态/现象 | 含义与检查点 |
| --- | --- |
| `generated` | JSON 已保存，尚不代表动作执行 |
| `ready` | WBC 已校验并登记动态 reference，尚不代表机器人已运动 |
| `started` | 控制器开始该次动作流程 |
| `completed` | 动作流程完成、进入回交 AMP；还需观察实际恢复状态 |
| `no_motion` | 没有选中的活动部位，未提交执行 |
| `rejected` / `failed` | 读取同一请求 ID 的 worker/controller 原因，不连续重试掩盖问题 |
| 没有实时观测 | 检查目标工作区、ROS domain、控制器状态和观测是否使能 |
| 生成期间姿态变化 | 旧起姿不再适用，稳定后重新采集生成 |
| 模型/依赖缺失或来源不匹配 | 检查工作区 `model_sources/text_to_motion`、安装源码及 manifest 是否属于同次交付 |
| 超时、倾角或电机异常 | 保存首个异常及前后日志，结合硬件和调度状态定位 |

终端 B 可继续查看 `/system/state_machine/wbc_action_fdb`。文本 JSON 位于本次 `$TEXT_WBC_RUN`；直接 launch 日志使用当前 `ROS_LOG_DIR` 或 ROS 默认日志目录，记录脚本入口另有 `runs/wbc_production/`。

验收同时包含流程和实际表现：普通动作需完成并恢复稳定；文本动作还需符合输入描述。`completed` 本身不证明动作语义、跟踪质量或长期稳定性合格。

测试结束按现场流程结束控制会话，再 Ctrl+C 退出 launch，等待其子进程清理。文本取消接口和详细协议见下方文本部署说明。

## 7. 2026-11-04 真机部署与编译记录

本次目标主机为 `forlinx@192.168.5.2`，新工作区是 `/userdata/forlinx/wbc_text_workspace1104`。上传、构建和检查写入均位于这个新工作区，没有修改旧 `wbc_workspace` 或 `zy_workspace`。

| 检查项 | 实际结果 |
| --- | --- |
| 构建 | `9 packages finished [11min 49s]`，退出码 `0` |
| 构建模式 | `RelWithDebInfo`、`LEGGED_DEPLOY_BACKEND=RKNN`、`LEGGED_DEPLOY_WBC_SIM=OFF` |
| 文本交付校验 | `TEXT_MODELS_SHA256SUMS` 的 100 项全部通过，包含根目录的 `文本模型放置说明.txt` |
| WBC 交付校验 | 普通包清单内 433 个文件、文本 WBC 包清单内 8 个文件均通过 SHA256 校验 |
| source/install 一致性 | 普通 WBC 包 434 个文件、文本 WBC 包 9 个文件、AMP `l0_v5_amp_10281` 的 4 个文件全部一致；此处数量包含包的安装清单 |
| ROS 包来源 | `legged_deploy`、`legged_controllers`、`legged_real`、`robot_interfaces`、`l0_v5_description` 均解析到新工作区 install |
| 动态链接依赖 | `rl_controller`、`topic_hardware_adapter` 的 `ldd` 检查无 `not found` |
| 文本 Python 依赖 | NumPy、PyYAML、psutil、ruamel.yaml、随包 RKNNLite 可导入；仅导入检查，未执行模型推理 |
| 编译内嵌策略路径 | 指向新工作区 `src/l0_ros2_deploy_v2/legged_deploy/policy` |

构建有 CMake 提示：部分包不使用统一传入的 RKNN/WBC 变量。该提示未使构建失败；`legged_deploy` 的 CMake 缓存已确认采用上述 RKNN 配置。

本次构建及检查文件保存在真机：

```text
/userdata/forlinx/wbc_text_workspace1104/
├── build_real_1104.sh     本次实际构建脚本
├── build-output.log      完整构建输出
├── build.exitcode        本次构建退出码：0
├── text-model-check.log  文本交付 SHA256 校验输出
├── post-build-check.log  包路径、动态库、Python 依赖和编译配置检查
└── log/latest_build/     colcon 分包构建日志
```

上述表格记录初次构建后的安装检查。随后已完成下节的文本模型实际推理；普通 WBC 的 NPU 推理、86 个动作预加载、动态 reference 登记和真机执行仍需按第 4～6 节在现场验收。不能把编译或离线生成成功视为动作执行已经通过。

### 7.1 文本模型板端 NPU 推理实测

2026-11-04 使用新工作区安装后的 `ros2 run legged_controllers text_wbc_generate`，从干净环境加载本工作区 `model_sources/text_to_motion/compiled/manifest.json`，实际运行 `lite2` 后端。日志确认 Python `3.10.12`、RKNNLite `2.3.2`、私有 Runtime `2.3.2`、板端 RKNN Driver `0.9.8`，两份文本神经网络模型成功加载并完成推理。

测试采用 `--snapshot`，起姿关节值来自 `UpperBody_Enhanced` 配置的默认值（21 个零值），姿态为单位四元数，base position 为 `[0,0,0]` 占位，根部参考标记为 `wbc_base_action`。这是明确标记 `offline_test_only` 的离线测试输入，不是实机实时观测，也没有修改部署默认高度或控制配置。未传 `--execute`，未创建 ROS 动作事务、未发布执行指令。

| 输入文本 | 输出帧数 | 生成耗时 `inference_ms` | 入口总耗时 | 结果 |
| --- | --- | --- | --- | --- |
| 头以小幅度慢速点头。 | 65 | 19.79 ms | 1.658 s | 退出码 0，生成 JSON |
| 左手以大幅度抬起。 | 79 | 26.06 ms | 1.668 s | 退出码 0，生成 JSON |
| 头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。 | 136 | 32.13 ms | 1.676 s | 退出码 0，生成 JSON |

`inference_ms` 是已加载模型后的文本识别与动作合成耗时，包括相应 CPU 处理，不是单一 NPU 算子的耗时；入口总耗时还包含 ROS 命令入口、模型加载和 JSON 保存。这是单次样例结果，不代表高负载下的时延保证。

三个 JSON 均通过 `[帧数,9]` 形状、数值有限、当前关节限位检查，没有发生输出限位裁剪；均包含 21 关节起姿及 `UpperBody_Enhanced` 根部参考标记。点头样例双臂六关节的轨迹变化量为 0；抬左手样例右臂和头部的变化量为 0。组合样例三个部位均有输出变化；具体动作幅度、次数和实机跟踪质量仍需现场验收。

日志中的 `Query dynamic range failed / RKNN_ERR_MODEL_INVALID` 出现在静态形状模型的动态范围查询处，随后两模型推理及输出校验均成功；此次不属于模型加载失败。测试工具最初调用板上不存在的 `/usr/bin/time`，当时尚未进入推理；改用 Bash 内置计时后完成上述测试，未安装系统软件、未改推理代码。

原始结果保存在真机：

```text
/userdata/forlinx/wbc_text_workspace1104/records/text_inference_check_20261104_152557/
├── run_check.sh          离线推理命令；默认输出已存在，复测需传新的文件名前缀
├── offline_snapshot.json
├── nod.json
├── nod_retry.log
├── nod_retry.exitcode
├── left_arm.json
├── left_arm.log
├── left_arm.exitcode
├── combined.json
├── combined.log
├── combined.exitcode
└── validation.json       输出校验和关节变化量
```

这些 JSON 只作离线测试记录，不能作为实机执行请求复用。正式执行须按第 5 节从新鲜实时观测重新生成。

## 8. 相关文档

- [真机 WBC 86 动作启动说明](wbc-real-startup.md)：同步配置、入口和下载版原件说明。
- [文本动作部署说明](text-motion-deployment.md)：模型依赖、协议、取消和详细排查。
- [本地 WBC 正式部署手册](wbc-lora-production-deployment.md)：模型制作、编译安装与运行记录。
- [下载版启动文档原件](wbc-lora-production-deployment-downloaded-20261103.md)：保留历史步骤和示例，当前配置以本文及正式 env 为准。
