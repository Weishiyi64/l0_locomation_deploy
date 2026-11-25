# WBC-LoRA 训练结果交付与部署资产生成规范

更新：2026-10-27。适用于当前 L0 V5 WBC-LoRA 导出器及 RK3588 部署链。本文只说明目录组织、文件来源、交付责任与生成命令，不改动训练或部署代码。

**训练完成后的原始包、部署导出包、RKNN 编译产物是三个阶段。** `base_manifest.json`、`weights.bin`、`rl_config.yaml` 不是训练 checkpoint 中可以直接复制出来的同名文件；它们由现有部署导出器生成。训练端不必生成 RKNN。

```text
训练原始交付包 training_inputs/<批次>/
  Base checkpoint + 原生 ONNX/YAML + 各动作 LoRA/完整 checkpoint + reference ONNX
                │ prepare_wbc_deploy.py --mode lora
                ▼
部署导出包 model_sources/<包名>/
  base_lora.onnx + manifests + weights.bin + 动作配置/reference ONNX + 验证资料
                │ batch_convert_onnx_to_rknn_rk3588.py --build-only
                ▼
同一 model_sources/<包名>/ 中新增 *.rknn 和 *.rknn.json
                │ prepare_wbc_deploy.py --mode install-lora
                ▼
src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/<包名>/
  运行资产 + INSTALLATION.json
                │ colcon build
                ▼
ROS package share 中的 WBC 策略目录
```

本文采用新包名 `wbc_lora_trained_20261027`，三个示例动作均使用各自训练出的 LoRA，避免覆盖此前含 Base 对照动作的 `wbc_lora_resident`。

## 1. 训练端需要交付什么

### 1.1 当前导出器所需的最小原始包

以下是训练结果交给部署准备端时的目录。`<emotion>` 只是分类目录；动作文件路径由 `emotion_index.json` 指定。可以只交付本次选择的动作，不必携带全部 80 个动作。

```text
training_inputs/
└── L0_V5_WBC10000_LoRA_20261027/
    ├── emotion_index.json                    # 动作索引，必需
    ├── base/
    │   ├── base_ckpt.pt                       # 原始共同 Base checkpoint，必需
    │   ├── policy.onnx                        # 同一 Base 的原生策略导出，必需
    │   └── policy.yaml                        # 与该导出匹配的部署语义配置，必需
    └── lora/
        └── affectionate/
            ├── L0_Hugme/
            │   ├── lorackpt_2000.pt           # 本动作原始紧凑 LoRA checkpoint，必需
            │   ├── checkpoint_2000.pt         # 同一训练节点的完整 checkpoint，必需
            │   └── motion/
            │       └── reference_motion.onnx  # 本动作参考轨迹导出，必需
            ├── L0_act_cute_1/
            │   ├── lorackpt_2000.pt
            │   ├── checkpoint_2000.pt
            │   └── motion/reference_motion.onnx
            └── L0_act_cute_2/
                ├── lorackpt_2000.pt
                ├── checkpoint_2000.pt
                └── motion/reference_motion.onnx
```

`base/base_ckpt.pt`、`base/policy.onnx`、`base/policy.yaml` 和根目录 `emotion_index.json` 是当前脚本固定读取的路径。动作文件名可以不同于 `*_2000.pt`，但索引必须记录实际相对路径；不要从文件名猜测保存轮次。

| 原始交付文件 | 为什么需要 | 能否由当前部署工具补齐 |
| --- | --- | --- |
| `base/base_ckpt.pt` | 提供 Base actor、归一化统计和 Base 身份；用于验证共享图及 LoRA 合成 | 不能从动作 reference 或 RKNN 还原，必须提供原始文件 |
| `base/policy.onnx` | 提供与 checkpoint 对应的原生网络图；导出器在图上增加动态参数输入 | 当前工具不会仅凭 `.pt` 重新构建此原生图，需训练侧既有导出流程提供 |
| `base/policy.yaml` | 提供关节顺序、默认位置、PD、action scale 等部署语义 | 当前工具不会从 checkpoint 自动提取成此配置，需训练侧提供匹配导出 |
| 各动作 `lorackpt_*.pt` | 提供 A/B、bias、LayerNorm 等动作参数及归一化统计 | 必须提供；不能只给最终融合 ONNX，或只给 A/B 两个矩阵 |
| 各动作 `checkpoint_*.pt` | 当前导出器验证 Base+LoRA 重建结果，并生成独立数值期望 | **当前实现必须提供**；虽然运行时不读取，不能在导出前省略 |
| 各动作 `motion/reference_motion.onnx` | 提供参考关节/机体轨迹和帧率、帧数、关节/机体名称等元数据 | 当前部署工具只校验、复制和编译它，不从 NPZ 生成它 |
| `emotion_index.json` | 将动作 ID 对应到上述文件，并记录 adapter/checkpoint 哈希 | 可以在整理交付时生成，不需要重新训练；必须依据实际文件与动作映射 |

**如果训练目录只有 `.pt` 和 `.npz`，尚不足以直接执行当前部署流程。** 需先用训练项目已有导出流程补齐 Base ONNX、Base policy YAML 和各动作 reference ONNX。当前仓库中的 `prepare_wbc_deploy.py --mode lora` 不负责这一步，也不能通过给文件改名补齐。

原始 checkpoint 按字节复制，允许改文件名并更新索引，但不要 `torch.load` 后重新保存来“精简”。Adapter 内 `base_sha256` 绑定原 Base 文件内容，重新保存 Base 会改变身份。`lorackpt` 与完整 checkpoint 也必须来自同一个动作、同一保存节点。

### 1.2 正式交付建议额外保留的资料

以下不是当前 LoRA 导出器的必需读取项，但建议随训练原始包交付，支持追溯和重新导出；不必全部安装到机器人运行目录。

```text
L0_V5_WBC10000_LoRA_20261027/
├── README_交付说明.md                 # 批次、机器人、代码版本、动作选择与导出环境
├── MANIFEST.sha256                   # 整包完整性清单，覆盖实际交付文件
├── base/
│   ├── training_config.yaml
│   └── export_provenance/            # Base 导出配置、命令、日志
├── lora/<emotion>/<motion>/
│   ├── training_config.yaml
│   ├── training_outcome.json         # 训练完成状态/保存节点
│   └── motion/
│       ├── <motion>.npz              # 原始参考轨迹，用于追溯/训练侧重新导出
│       └── manifest.json             # 数据集原始元信息
└── validation/                       # 已完成的训练/导出验证报告
```

`emotion_index.csv`、训练视频和可视化页面可选。训练脚本、仿真器、优化器恢复所需环境属于训练归档；正式 C++ worker 不加载这些文件。原始 checkpoint 内附带的 critic/optimizer 不因此变成运行时依赖。

### 1.3 `emotion_index.json` 的最小契约

这是一个 JSON 数组，每个选中动作至少包含：

| 字段 | 含义 |
| --- | --- |
| `motion` | 唯一动作 ID，例如 `L0_Hugme`；1～32 个字母、数字、下划线或连字符 |
| `adapter` | 相对包根目录的 `lorackpt` 路径 |
| `adapter_sha256` | 该原始 adapter 文件实际 SHA256 |
| `checkpoint` | 相对包根目录的完整 checkpoint 路径 |
| `checkpoint_sha256` | 该原始完整 checkpoint 文件实际 SHA256 |
| `motion_onnx` | 相对包根目录的 reference ONNX 路径 |

已有训练交付包中的 `emotion_index.json` 可直接使用，其 `emotion`、`fps`、`motion_npz`、`rank`、`num_updates` 等附加字段无需删除。只复制选中动作时，保留完整索引也可用于显式 `--actions`；为了交付清晰，建议索引只包含本次交付动作。所有路径应位于包内，不保留跨机器绝对路径或指向包外的软链接。

如果首次整理的包没有索引，可在**整理机**上按 1.1 的命名执行下列命令。它只创建缺失的索引，读取现有文件计算哈希，不修改 checkpoint。保存轮次不同则先修改这里的两个文件名；它不判断模型训练质量或 Base 兼容性，后续导出仍需验证。

```bash
(
set -e -o pipefail
training_root=/home/luvbot/SC/l0_deploy_v2_ws/training_inputs/L0_V5_WBC10000_LoRA_20261027
python3 - "$training_root" <<'PY'
import hashlib, json, pathlib, re, sys
root = pathlib.Path(sys.argv[1]).resolve()
destination = root / 'emotion_index.json'
if destination.exists():
    raise SystemExit('Index already exists; keep and verify the supplied index')
entries, seen = [], set()
def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()
for directory in sorted((root / 'lora').glob('*/*')):
    if not directory.is_dir():
        continue
    action = directory.name
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', action) or action in seen:
        raise SystemExit('Invalid or duplicate action ID: ' + action)
    adapter = directory / 'lorackpt_2000.pt'
    checkpoint = directory / 'checkpoint_2000.pt'
    reference = directory / 'motion/reference_motion.onnx'
    for path in (adapter, checkpoint, reference):
        if not path.is_file() or not path.resolve().is_relative_to(root):
            raise SystemExit('Missing or external asset: ' + str(path))
    seen.add(action)
    entries.append({
        'motion': action,
        'adapter': str(adapter.relative_to(root)),
        'adapter_sha256': digest(adapter),
        'checkpoint': str(checkpoint.relative_to(root)),
        'checkpoint_sha256': digest(checkpoint),
        'motion_onnx': str(reference.relative_to(root)),
    })
if not entries:
    raise SystemExit('No action directories found')
with destination.open('x', encoding='utf-8') as stream:
    json.dump(entries, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
print(destination, 'actions=', len(entries))
PY
)
```

## 2. 你列出的部署文件，分别从哪里来

| 部署文件 | 生成来源 | 生成阶段 / 工具 | 原始训练交付时必须已有吗 |
| --- | --- | --- | --- |
| `base_manifest.json` | Base/ONNX 身份、动态输入 schema、动作目录及生成文件哈希 | `prepare_wbc_deploy.py --mode lora` 调用导出器生成 | 不需要 |
| `base_lora.onnx` | `base/policy.onnx` 与匹配 Base 权重/图结构 | 同上，生成含 31 个参数输入的共享图 | 不需要，不能由原生 policy 简单改名 |
| `adapters/<动作>/manifest.json` | 参数名称、形状、偏移、Base/归一化/schema 身份、权重哈希 | 同上 | 不需要 |
| `adapters/<动作>/weights.bin` | 原始 LoRA 中的 31 个部署参数张量 | 同上，打包 FP16 little-endian 参数与对齐信息 | 不需要，不能直接把 `.pt` 改名 |
| `actions/<动作>/rl_config.yaml` | `base/policy.yaml` + 该动作 reference 元数据 | 同上，生成 `RobotCfg` / `WbcCfg` | 不需要，不能直接把训练 YAML 改名 |
| `actions/<动作>/reference_motion.onnx` | 索引 `motion_onnx` 指向的训练侧参考导出 | 同上，校验后复制，保持源文件内容 | **原始包中必须提供源 reference ONNX** |
| `base_lora.rknn` | 共享动态 ONNX | RK3588 policy 转换器 / 批量转换器 | 不需要 |
| `actions/<动作>/reference_motion.rknn` | 对应 reference ONNX | RK3588 reference 转换器 / 批量转换器 | 不需要 |
| 各 `*.rknn.json` | 实际转换版本、输入/产物哈希和验证状态 | 转换器在生成 RKNN 后写入 | 不需要，不能手写为“已验证” |
| `INSTALLATION.json` | 完整已转换资产的校验结果与文件列表 | `prepare_wbc_deploy.py --mode install-lora` | 不需要 |

`weights.bin` 不只包含 LoRA A/B，还包含各层完整 bias 和 LayerNorm gamma/beta；只交付 A/B 会丢失本次训练的部分参数。Base 固定大权重保留在共享 ONNX，动作包中的小参数在运行时切换。

`rl_config.yaml` 当前统一依据 **Base 的 `policy.yaml`** 生成，不读取每个动作的 `training_config.yaml` 来自动配置不同 PD 或 action scale。一个共享包需要满足当前导出器的相同关节/观测/控制契约；若动作训练使用不同配置，应先解决语义兼容性，不能认为导出器会自动适配。

## 3. 原始包到部署包：在哪里生成

推荐在训练机器或部署准备工作站上执行 CPU 导出，然后把完整导出包上传 RK3588；RK3588 负责 RKNN 转换、ROS 构建和运行。这个分工允许板端不安装 PyTorch。

也可以把 1.1 的原始包复制到板端后运行同一导出入口。条件是板端有能读取该批 checkpoint 的 Python/PyTorch 环境，以及 NumPy、PyYAML、ONNX、ONNX Runtime 和 checkpoint 反序列化所需模块。导出使用 CPU，不要求 CUDA；Toolkit2 可用不代表这些导出依赖也已安装。本文未验证板端 PyTorch 环境，不将它作为默认前提。

### 3.1 已有本批训练结果：准备机生成部署包

下面对应当前已有训练结果目录。输出使用新的目录名，不覆盖之前的模型包。准备机的 Python 路径为已有训练环境示例；其他机器应改成实际环境。导出器读取 `.pt`，应只处理来源可信的训练交付。

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
deploy_repo="$local_ws/src/l0_ros2_deploy_v2"
training_root='/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果'
output_bundle="$local_ws/model_sources/wbc_lora_trained_20261027"
export_python=/home/luvbot/miniconda3/envs/active_adaption/bin/python
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
test ! -e "$output_bundle"
"$export_python" -c 'import torch, numpy, yaml, onnx, onnxruntime; print("CPU export dependencies import OK")'
"$export_python" -B "$deploy_repo/prepare_wbc_deploy.py" \
  "$training_root" "$output_bundle" --mode lora \
  --actions L0_Hugme L0_act_cute_1 L0_act_cute_2 --samples 33
)
```

本命令的三个动作全部使用训练 LoRA。**没有 `--base-actions`。** 前一份 `wbc_lora_resident` 示例中的 `L0_act_cute_2` 曾被显式指定为 Base 对照，用于检查参数恢复；这是导出选择，不是训练结果固有属性。不能为缺失/损坏的 LoRA checkpoint 自动改用 Base。只有确实要导出 Base 对照时，才额外传入 `--base-actions L0_act_cute_2`，其余动作仍用各自 LoRA。

`--actions` 可列出本次需要的多个动作，导出器统一生成一份共享图和每个动作的参数/reference。它要求样本数至少 20，示例为 33；验证包含合成输入上的 FP32 一致性、FP16 参数打包误差及切换检查，不是机器人实际轨迹的数值验收。

当前导出器针对特定 L0 网络：7 个 Linear、5 个 LayerNorm、rank=16、alpha=16、policy[399]、command_short[240]、action[21]。动作归一化统计必须与共享 Base 相同；不符合时会拒绝，不能直接套到任意训练结构或归一化不同的动作上。

### 3.2 导出成功、尚未编译 RKNN 时的目录

```text
model_sources/wbc_lora_trained_20261027/
├── base_manifest.json
├── base_lora.onnx
├── adapters/
│   ├── L0_Hugme/{manifest.json,weights.bin}
│   ├── L0_act_cute_1/{manifest.json,weights.bin}
│   └── L0_act_cute_2/{manifest.json,weights.bin}
├── actions/
│   ├── L0_Hugme/{rl_config.yaml,reference_motion.onnx}
│   ├── L0_act_cute_1/{rl_config.yaml,reference_motion.onnx}
│   └── L0_act_cute_2/{rl_config.yaml,reference_motion.onnx}
├── validation_report.json
├── verification_inputs.npz
├── verification_expected.npz
├── probe_encoder.onnx
├── probe_encoder_manifest.json
├── probe_encoder_inputs.npz
├── probe_encoder_expected.npz
├── probe_decoder.onnx
├── probe_decoder_manifest.json
├── probe_decoder_inputs.npz
├── probe_decoder_expected.npz
├── probe_output.onnx
├── probe_output_manifest.json
├── probe_output_inputs.npz
└── probe_output_expected.npz
```

大括号表示该目录内的两个独立文件。此阶段没有 `.rknn`、`.rknn.json` 或 `INSTALLATION.json`，这是正常的。

若训练端已完成本次 CPU 导出，**训练向板端实际交付的可以直接是这个完整目录**，板端无需再接收或加载 `.pt`。原始 checkpoint、原始 YAML、轨迹与索引留在训练归档。正式流转推荐保留全部导出验证资料；其中 `verification_*.npz` 是数值对照基准，不要因 C++ worker 不读取就从交付归档中丢弃。

仅为了 `--build-only` 编译及运行，核心最小集是共享 ONNX+manifest、每个 adapter 的 manifest/weights、每个动作的 config/reference ONNX。probe 与验证样本不参与正式动作执行；但是删去它们后，相应离线验证或 `--include-probes` 功能将不可用。正常上传时直接复制整个导出目录最清楚。

### 3.3 可选：部署板上完成 CPU 导出

采用此方案时，将 1.1 的原始包放在板端 `training_inputs/L0_V5_WBC10000_LoRA_20261027`。**这与 3.1 是两种执行位置，选一个即可。** 板端 `/usr/bin/python3` 只有通过依赖导入并能读取该批 checkpoint 后才能使用；否则改成实际准备好的导出环境，或回到准备机导出。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
deploy_repo="$robot_ws/src/l0_ros2_deploy_v2"
training_root="$robot_ws/training_inputs/L0_V5_WBC10000_LoRA_20261027"
output_bundle="$robot_ws/model_sources/wbc_lora_trained_20261027"
export_python=/usr/bin/python3
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
test ! -e "$output_bundle"
"$export_python" -c 'import torch, numpy, yaml, onnx, onnxruntime; print("CPU export dependencies import OK")'
"$export_python" -B "$deploy_repo/prepare_wbc_deploy.py" \
  "$training_root" "$output_bundle" --mode lora \
  --actions L0_Hugme L0_act_cute_1 L0_act_cute_2 --samples 33
)
```

## 4. 准备机导出后：上传板端、编译、安装

以下命令每块独立定义变量；在新终端复制完整块即可。采用板端 CPU 导出时跳过上传步骤。下面只提供操作命令，未在本次文档编写中执行导出、编译或安装。

### 4.1 本地主机上传已导出的整个包

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
bundle_name=wbc_lora_trained_20261027
local_bundle="$local_ws/model_sources/$bundle_name"
remote_bundle="$robot_ws/model_sources/$bundle_name"
test -f "$local_bundle/base_manifest.json"
test -f "$local_bundle/validation_report.json"
ssh "$robot_target" "test ! -e '$remote_bundle' && mkdir -p '$remote_bundle'"
rsync -rtv --itemize-changes "$local_bundle/" "$robot_target:$remote_bundle/"
)
```

### 4.2 RK3588 板端批量生成 RKNN 和转换记录

板端需已有 Toolkit2 2.3.2 及当前部署仓库转换工具；此步骤不需要 PyTorch。三动作包编译一份共享 policy 加三份 reference，不为每个动作编译独立 policy。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
deploy_repo="$robot_ws/src/l0_ros2_deploy_v2"
bundle_name=wbc_lora_trained_20261027
source_bundle="$robot_ws/model_sources/$bundle_name"
toolkit_python=/usr/bin/python3
mkdir -p "$robot_ws/build_logs/wbc_models"
conversion_log="$robot_ws/build_logs/wbc_models/${bundle_name}_$(date +%Y%m%d_%H%M%S_%N)"
"$toolkit_python" -B "$deploy_repo/batch_convert_onnx_to_rknn_rk3588.py" \
  "$source_bundle" --manifest "$source_bundle/base_manifest.json" \
  --build-only --log-dir "$conversion_log"
)
```

生成：`base_lora.rknn`、`base_lora.rknn.json`，以及每个 `actions/<动作>/reference_motion.rknn` 和对应 `.rknn.json`。`--build-only` 的验证状态仍为 `not_run`，不代表 RK3588 数值通过。

### 4.3 RK3588 板端生成安装清单，再构建

源策略目录必须不存在；如安装成功、colcon 构建失败，修复构建后只执行 source/colcon 部分，不重复安装。ROS 构建在工作空间根目录执行，并已有匹配的接口源码与依赖。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
deploy_repo="$robot_ws/src/l0_ros2_deploy_v2"
bundle_name=wbc_lora_trained_20261027
/usr/bin/python3 -B "$deploy_repo/prepare_wbc_deploy.py" \
  "$robot_ws/model_sources/$bundle_name" \
  "$deploy_repo/legged_deploy/policy/l0_v5/$bundle_name" --mode install-lora
source /opt/ros/humble/setup.bash
test -d "$robot_ws/src/luvbot_ros_interfaces"
cd "$robot_ws"
colcon build --packages-up-to legged_deploy legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN
source "$robot_ws/install/setup.bash"
deploy_prefix=$(ros2 pkg prefix legged_deploy)
[[ "$deploy_prefix" == "$robot_ws/install" || "$deploy_prefix" == "$robot_ws/install/"* ]]
test -f "$deploy_prefix/share/legged_deploy/policy/l0_v5/$bundle_name/INSTALLATION.json"
)
```

正式启动选择 `wbc_bundle=wbc_lora_trained_20261027`，并将本次要启用的动作列入 `wbc_actions`。新包的 `L0_act_cute_2` 是 LoRA，其含义不同于旧 Base 对照包，不应只按动作 ID 把两次结果视作同一策略。

## 5. 交付验收时应确认的边界

1. **原始包是否齐全**：Base 三件套、索引、每个动作的 adapter/完整 checkpoint/reference ONNX 均存在，路径和哈希一致。Base 文件必须匹配 adapter 内绑定的 Base 身份。
2. **导出是否成功**：只接收导出器完整生成的包；查看 `validation_report.json` 中实测结果，区分合成样本检查与板端实测。无法从缺失的训练语义或模型身份推测正确 manifest。
3. **RKNN 是否生成**：检查每个 RKNN 及转换记录；记录中的误差阈值不是实测误差。
4. **安装和构建是否成功**：安装器检查完整资产并生成 `INSTALLATION.json`，colcon 将其安装到 ROS share；运行时不需要原始训练 checkpoint。
5. **板端验收另行进行**：加载成功、动作完成、控制时序、停机确认与同输入数值精度分别检查，不能以任一阶段的文件存在代替其余验收。

因此，部署端可以生成你列出的 manifests、参数二进制、动作配置、动态 ONNX、RKNN 和安装记录；前提是先拿到第 1 节完整的原始输入，以及执行对应阶段所需的环境。若希望 RK3588 环境最简单，就把第 3.2 节完整导出目录作为训练/准备端向板端的交付边界。
