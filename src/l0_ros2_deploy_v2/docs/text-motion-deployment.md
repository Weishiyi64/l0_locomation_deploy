# 板端文本动作生成使用说明

适用于 RK3588 上生成九关节动作并通过 JSON 交给正式 WBC 的链路。最初验证工作区为 `l0_text_wbc_board_ws_20261028_203257`；当前工作空间已集成 `text_wbc_bundle`、`text_generation_models_20261102` 的文本模型、源码 baseline、运行依赖及 `l0_v5_amp_10281` 源码资产。旧 `text_motion_node`、`text_motion_request` 和上肢直接覆盖接口不适用于此交付版本。

## 1. 部署目录与依赖

下面的工作区结构以原验证目录为例。部署到新板端路径时，将当前源码、工作空间根目录的 `model_sources/text_to_motion/`、WBC/AMP 资产和接口包一起迁移，并在新路径构建；不能只复制旧 `install/`。

```text
l0_text_wbc_board_ws_20261028_203257/
├── src/
│   ├── l0_ros2_deploy_v2/
│   │   ├── texttomotion/baseline/          原始 ONNX 交付与编译器
│   │   └── legged_deploy/policy/l0_v5/
│   │       ├── text_wbc_bundle/           共享 WBC 模型和动作参数
│   │       └── l0_v5_amp_10281/           当前 AMP 资产
│   └── luvbot_ros_interfaces/             匹配版本的接口包
├── model_sources/text_to_motion/
│   ├── compiled/                         manifest、两个 RKNN、词表与素材
│   ├── deps/rknnlite/                     RKNNLite2 2.3.2
│   ├── deps/rknn_toolkit_lite2-2.3.2.dist-info/
│   └── runtime/librknnrt.so               Runtime 2.3.2
└── records/                              输出 JSON 与运行日志
```

`text_wbc_bundle` 从 `lora_actions_20261102` 的交付包精简为仅有 `UpperBody_Enhanced` 的文本运行包；`model_sources/text_to_motion/`、`texttomotion/baseline/` 来自 `text_generation_models_20261102`；`text_wbc_1029_code_update_20261102` 的代码补丁已应用。工作空间根目录保留 `PACKAGE_INFO.json` 和 `TEXT_MODELS_SHA256SUMS`，可在根目录执行 `sha256sum -c TEXT_MODELS_SHA256SUMS` 核对模型包。`l0_v5_amp_10281` 已另行放入源码策略目录。文本 JSON 使用 `UpperBody_Enhanced`，其 `parameter_source=lora`，LoRA A/B 有非零权重。普通 WBC 动作由独立的 `wbc_lora_fp16` 包提供。LoRA 模型制作及安装见 [LoRA 正式部署说明](wbc-lora-production-deployment.md)。

推理使用系统 `/usr/bin/python3`，已部署环境为 Python 3.10 / aarch64、ROS Humble。进入工作目录或新开终端不会自动激活虚拟环境。`text_wbc_generate` 安装入口自动使用所属工作区的 `deps/rknnlite` 和 `runtime/librknnrt.so`；NumPy、PyYAML、psutil、ruamel.yaml 和 ROS Python 依赖仍由板端环境提供。

`TEXT_WBC_PYTHON`、`TEXT_WBC_MODEL_ROOT`、`RKNN_RUNTIME_LIB` 可覆盖默认选择。要使用当前工作区自己的依赖，保持这三项未设置。直接运行 Python 源文件不会经过安装入口的环境设置。

## 2. 模型编译与软件构建

当前工作空间已有 2026-11-02 模型包的 `compiled/manifest.json`、`qwen_tokenizer.rknn`、`inbetweener.rknn`、词表、素材、RKNNLite2 2.3.2 依赖和私有 Runtime。生成器的 `--model` 指向这份 compiled manifest；不需要为本次集成重新编译 RKNN。原始 ONNX 和配套后处理在 `texttomotion/baseline/`，只在重新编译时使用。

```bash
export TEXT_WBC_WS=/userdata/forlinx/wbc_text_workspace1104
export TEXT_WBC_REPO="$TEXT_WBC_WS/src/l0_ros2_deploy_v2"
# 仅当需要从当前 baseline 重新编译时执行；输出目录使用新名称。
RKNN_PYTHON=/usr/bin/python3 bash "$TEXT_WBC_REPO/texttomotion/baseline/compile.sh" \
  --output "$TEXT_WBC_WS/model_sources/text_to_motion/compiled_new" --build-only
```

重新编译会生成 `manifest.json`、两个 RKNN 和配套素材。选择新产物时，将下节 `TEXT_WBC_MODEL` 指向 `compiled_new/manifest.json`；使用本次已交付的 `compiled/` 时保持下节默认路径。RKNNLite 与 Runtime 已包含在本工作空间的 `model_sources/text_to_motion/` 中。

软件构建在板端目标工作区执行：

```bash
cd "$TEXT_WBC_WS"
source /opt/ros/humble/setup.bash
export MAKEFLAGS='-j2 -l2'
colcon build --executor sequential --packages-up-to legged_real legged_deploy legged_controllers l0_v5_description \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DLEGGED_DEPLOY_BACKEND=RKNN -DLEGGED_DEPLOY_WBC_SIM=OFF \
  -DRKNN_ROOT="$TEXT_WBC_REPO/legged_deploy/third_party/rknn_runtime"
source install/setup.bash
```

修改源码或安装资产后重新构建，再使用新终端加载目标工作区。仅修改本说明不需要构建。

## 3. 每个新终端的环境设置

```bash
export TEXT_WBC_WS=/userdata/forlinx/wbc_text_workspace1104
source /opt/ros/humble/setup.bash
source "$TEXT_WBC_WS/install/setup.bash"
unset TEXT_WBC_MODEL_ROOT RKNN_RUNTIME_LIB TEXT_WBC_PYTHON
export PYTHONDONTWRITEBYTECODE=1
export TEXT_WBC_MODEL="$TEXT_WBC_WS/model_sources/text_to_motion/compiled/manifest.json"
export TEXT_WBC_BUNDLE="$TEXT_WBC_WS/install/legged_deploy/share/legged_deploy/policy/l0_v5/text_wbc_bundle/base_manifest.json"
export TEXT_WBC_MAPPING="$TEXT_WBC_WS/install/system_command_arbiter/share/system_command_arbiter/config/l0_v5.yaml"
export TEXT_WBC_RUN="$TEXT_WBC_WS/records/text_$(date +%Y%m%d_%H%M%S_%N)"
mkdir -p "$TEXT_WBC_RUN"
```

所有 ROS 终端使用同一个 ROS 域。日志中的解释器应为所选 Python，Runtime 路径应位于该工作区的 `model_sources/text_to_motion/runtime/`。
`TEXT_WBC_MODEL` 必须指向当前工作空间的 2026-11-02 compiled manifest；生成器会对照源码 baseline 的 manifest 哈希，WBC 接收端会检查 JSON 中的来源标识。不要混用旧工作区的 manifest 或 ROS 安装目录。

`PACKAGE_INFO.json` 记录的源码身份与已应用的 2026-11-02 代码补丁一致。生成器和 WBC JSON 接收端使用相同的模型来源标识；JSON 动作的控制参数由 `wbc_json_base_action=UpperBody_Enhanced` 选择 LoRA adapter。板端仍需完成模型加载、JSON 生成与 WBC 接收检查。

## 4. 只生成 JSON

离线生成使用显式 `--snapshot`，不创建 ROS 节点或发送执行请求。snapshot 包含按 Base 配置排序的 21 个 `joint_names`、21 维 `joint_positions`、3 维 `base_position` 和 wxyz 顺序的 `base_quaternion_wxyz`。位置需有限且在机械限位内，四元数需有效。离线 snapshot 的 root reference 高度必须有效；合成输入应明确标记为合成数据。

已有工作区提供 `records/HOST_REFERENCE_SNAPSHOT.json` 合成示例。迁移时若未携带该文件，将下面路径替换为准备好的 snapshot：

```bash
ros2 run legged_controllers text_wbc_generate \
  --model "$TEXT_WBC_MODEL" --bundle "$TEXT_WBC_BUNDLE" --mapping "$TEXT_WBC_MAPPING" \
  --base-action UpperBody_Enhanced \
  --snapshot "$TEXT_WBC_WS/records/HOST_REFERENCE_SNAPSHOT.json" \
  --text '左手抬起' --seed 42 --output "$TEXT_WBC_RUN/left_arm.json" \
  > "$TEXT_WBC_RUN/left_arm.log" 2>&1
echo "exitcode=$?"
cat "$TEXT_WBC_RUN/left_arm.log"
```

成功时退出码为 0，日志包含 `state=generated`，JSON 包含 `model_diagnostics.backend=lite2`、`fps=50` 和完整帧数组。输出路径必须不存在。省略 `--snapshot` 时会采集实时观测，需要正在运行的控制器和 WBC；未加 `--execute` 仍只生成文件。

常用参数：

| 参数 | 用途 |
| --- | --- |
| `--text` | 自然语言动作请求 |
| `--tokens HEAD LEFT RIGHT` | 直接指定三组分类 ID，跳过文本分类 |
| `--seed` | 素材随机选择种子 |
| `--speed` | 速度，默认 1.2，范围 0.1–4.0 |
| `--no-return` | 关闭模型回正 |
| `--return-frames` | 回正帧数，取 −1、30、60 或 90 |
| `--limit-policy reject\|clip` | 最终输出边界策略，默认 reject |
| `--execute` | 生成后提交正式 WBC 执行；不能与 snapshot 同用 |

## 5. 实时生成并交给 WBC

现场按原流程独立维护 HPM、校准、使能和急停。以下入口启动部署控制链，不启动 HPM 电机通信节点：

```bash
ros2 launch legged_controllers text_wbc.launch.py hpm_mode:=real
```

默认机型 `l0_v5`，AMP 为 `l0_v5_amp_10281`，共享包为 `text_wbc_bundle`，预加载名单仅为 `[UpperBody_Enhanced]`。JSON 动作使用其 LoRA 参数。覆盖 `wbc_policies` 或 `wbc_json_base_action` 时，两者必须仍指向该包中存在的动作；普通 WBC 动作应使用 `wbc_lora.launch.py` 与 `wbc_lora_fp16`。

按原手柄流程进入 Walking 并保持站立。启动检测采用正式 WBC 的状态、观测、prepare 和 active 流程。实时生成默认使用所选 `UpperBody_Enhanced` 动作 reference 首帧的 root position/orientation；无里程计导致测得 z=0 时，不要求额外填写 `--root-height`。IMU、观测新鲜度与使能检查仍然保留。`--root-height` 仅用于明确需要覆盖参考高度的场景。

在另一个已完成第 3 节设置的终端执行：

```bash
ros2 run legged_controllers text_wbc_generate \
  --model "$TEXT_WBC_MODEL" --bundle "$TEXT_WBC_BUNDLE" --mapping "$TEXT_WBC_MAPPING" \
  --base-action UpperBody_Enhanced --text '左手抬起' --seed 42 --execute \
  --output "$TEXT_WBC_RUN/left_arm_execute.json" \
  > "$TEXT_WBC_RUN/left_arm_execute.log" 2>&1
echo "exitcode=$?"
cat "$TEXT_WBC_RUN/left_arm_execute.log"
```

执行顺序为：采集全身起姿 → NPU 生成 → 保存完整 JSON → 发布 `/text_motion/json` → 等待 worker `ready` → 发布正式动作命令 → 控制器 prepare/active → WBC 执行 → 返回 AMP。

| Topic | 内容 |
| --- | --- |
| `/control/policy/observation` | 现有 21 关节与 IMU 观测 |
| `/text_motion/json` | 完整轨迹 JSON，`std_msgs/msg/String` |
| `/text_motion/wbc_status` | worker 的 request_id/state/detail |
| `/algorithm/brain/wbc_action_cmd` | 正式动作命令，timestamp_ms/action_id |
| `/system/state_machine/wbc_action_fdb` | 控制器 started/completed/cancelled/rejected/failed |
| `/text_motion/cancel` | 要取消的原始 `T_*` 请求 ID 字符串 |

同一请求应出现 worker `ready`、控制器 `started` 和控制器 `completed`。worker 的 `ready` 只代表 JSON 已登记；控制器终态代表控制流程进入回交，仍需观察 AMP 恢复和实际机器人状态。

JSON 使用 `gmr_upper_body_joint_trajectory_v1`，50 Hz、rad，九关节顺序为左臂三轴、右臂三轴、颈 yaw/pitch/roll。WBC 将其展开为完整 21 关节参考，附加 100 帧保持和过渡。请求有效期为 5 秒，不能直接重播旧文件；每次执行应重新采集起姿生成。

## 6. 取消与结果检查

在另一个终端读取正在执行请求的 JSON 路径，发送取消：

```bash
TEXT_WBC_JSON=/填写本次输出目录/left_arm_execute.json
TEXT_WBC_ID=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["request_id"])' "$TEXT_WBC_JSON")
ros2 topic pub --once /text_motion/cancel std_msgs/msg/String "{data: '$TEXT_WBC_ID'}"
```

观察 `/system/state_machine/wbc_action_fdb` 中相同 ID 的 `cancelled` 和 AMP 恢复。仅收到 worker 的取消登记反馈不代表活动动作已经退出。

记录文本、seed、JSON、请求 ID、退出码及完整 stdout/stderr。检查左右侧、非活动部位、文本 `UpperBody_Enhanced` LoRA 的执行及取消后恢复；普通 WBC 动作在独立的 `wbc_lora_fp16` 入口验收。

## 7. 限位与常见问题

生成流程读取 `--mapping` 对应的 l0_v5 九关节限位，并用于素材处理、插帧和回正。颈 pitch 为 `[-0.349,0.175]` rad。最终默认 `reject` 继续拒绝异常越界；显式 `clip` 只允许最后适配边界裁切。`safety_adjustments` 不统计 composer 内部处理，计数为 0 不表示内部从未裁切。

| 现象 | 检查与处理 |
| --- | --- |
| `No module named rknnlite` | 确认通过当前工作区的安装入口运行，`deps/rknnlite` 和对应 aarch64/Python 3.10 扩展完整；重新 source 正确 install。 |
| 静态模型的 dynamic range warning | 单独出现该警告不等于失败；依据退出码、最终 state 和后续异常判断。 |
| `Model exceeds robot limits` | 核对安装脚本已更新、mapping 来自 l0_v5；保留异常，不通过扩大机械限位处理。 |
| `A valid measured root height...` | 该错误来自旧安装版本；当前实时入口使用 Base root reference。核对所 source 的工作区及构建产物。 |
| 等待端点超时、观测无效或姿态变化 | 确认 ROS 域、唯一 worker/控制器、Walking 和观测使能；生成期间保持姿态稳定，再重新生成。 |
| `rejected`、重复或过期请求 | 查 worker/controller 的 detail；使用新请求 ID、实时起姿和新输出文件。 |
| 程序退出码非零但终端没有异常 | 查看重定向日志；保留 stdout 和 stderr，不以 RKNN 初始化成功判断整条生成事务成功。 |
