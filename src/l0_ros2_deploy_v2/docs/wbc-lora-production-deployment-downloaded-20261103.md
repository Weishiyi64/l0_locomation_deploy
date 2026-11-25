# RK3588 WBC-LoRA 正式部署操作手册

更新：2026-10-28。对象：L0 V5、RK3588 Linux/aarch64、ROS2 Humble、常驻参数 WBC-LoRA。本文给出训练交付、文件前处理、RKNN 编译、模型安装与构建，以及通过原正式 sim2real 入口加载模型、独立的终端动作交互和可选录包流程。硬件通信和 HPM 启动由硬件侧原流程维护。

运行资产直接放在工作空间内，不依赖 `deliveries`、交付包日志指针或隔离启动入口。WBC 从 ROS 安装目录加载；当前 AMP 的模型路径仍指向编译时的源码策略目录，必须保留该目录。本文是操作流程，不代表真实机器人闭环或数值精度已验收。

## 0. 最终交付约定与文件前处理

**最终版本：训练端统一完成 Base 模型、LoRA 参数和 reference motion 的 FP16 转换后交付；ONNX 对外输入/输出接口保留 FP32。部署端不执行数值精度验证，不承担训练参数的 FP16 导出工作，只做资产完整性/接口检查、RKNN 编译、安装、构建和运行检查。** 本节取代此前 reference ONNX 保留 FP32 的交付约定。

下文板端分步命令沿用示例名 `wbc_lora_fp16`；统一入口默认使用 `wbc_lora_<时间戳>`，需将示例包名统一替换为脚本实际输出。旧包不会因为改名而变成符合本规范的新包。若新目录也已存在，使用新的版本目录名，并一致修改各步骤中的包名。

### 0.1 精度与接口规范

| 交付内容 | 参数/内部数据精度 | 对外接口及约束 |
| --- | --- | --- |
| 共享 `base_lora.onnx` | Base 固定权重采用 FP16；图内通过必要的 Cast 衔接 FP32 接口 | `policy[399]`、`command_short[240]`、31 个动态参数输入及 `action[21]` 输出均保持 FP32，名称、顺序和形状不变 |
| 各动作 `weights.bin` | FP16 little-endian；覆盖 A/B、完整 bias、LayerNorm gamma/beta，共 31 个参数张量 | manifest 的 `weights.dtype` 为 `float16-le`，各张量偏移按现有 64 字节对齐契约生成；加载器按既有方式解包并写入 RKNN 原生张量 |
| 各动作 `reference_motion.onnx` | **轨迹表等浮点数据统一 FP16**；不再要求保留 FP32 轨迹表 | 输入 `time_step[1,1]` 与六个输出仍为 FP32，保留关节/机体顺序、50 Hz、帧数、wxyz 四元数和边界取帧语义 |
| manifest / `rl_config.yaml` | 保持 JSON/YAML 元数据形式，不对文件做“半精度转换” | 保留真实参数与接口描述；哈希必须对应最终交付字节 |
| 板端 `.rknn` | 按现有 RK3588、FP16、非量化目标编译 | 不启用 Python Runtime 数值对照，统一使用 `--build-only` |

“统一 FP16”指模型参数、LoRA 参数和 reference 浮点轨迹数据的交付精度，不是把 shape、索引或全部 ONNX 输入输出改成 FP16。图中的整数类型及索引保持原类型。`application_dtype` 和图的输入输出 `dtype` 仍为 `float32`，`target_precision` 为 `float16`；内部精度和应用接口精度是不同字段。

上表是本机 FP16 导出归档的存储契约。统一入口实际交付的是完成编译前处理的 `model_sources`，其中 reference 表为 FP16 值展开的 FP32 编译表示，见 0.4；不改变轨迹值或接口。

reference 六个输出契约保持为：`joint_pos[1,21]`、`joint_vel[1,21]`、`body_pos_w[1,23,3]`、`body_quat_w[1,23,4]`、`body_lin_vel_w[1,23,3]`、`body_ang_vel_w[1,23,3]`。精度转换不得改变这些形状及含义。

### 0.2 训练端保留的原始输入

以下文件留在训练/导出环境中，用于生成最终交付包和追溯；正式部署端无需接收原始 `.pt`，也不需安装 PyTorch 来重新导出。

```text
training_inputs/<训练批次>/
├── emotion_index.json
├── base/
│   ├── base_ckpt.pt                      # 原始共同 Base，保持原字节
│   ├── policy.onnx                       # 与原始 Base 匹配的原生导出
│   └── policy.yaml                       # 关节顺序、PD、action scale 等配置
└── lora/<分类>/<动作ID>/
    ├── lorackpt_2000.pt                  # 原始 LoRA 参数
    ├── checkpoint_2000.pt                # 可选：同一训练节点的完整 checkpoint，供额外对照
    └── motion/
        ├── reference_motion.onnx        # 用于训练侧前处理的原生 reference
        └── <动作ID>.npz                 # 建议归档的原始参考轨迹
```

`2000` 仅为示例保存轮次，索引应填写实际文件名。`emotion_index.json` 为数组，至少包含每个动作的 `motion`、`adapter`、`adapter_sha256`、`motion_onnx`；路径相对包根目录。`checkpoint` 和 `checkpoint_sha256` 可选，前者指向的文件存在时必须有匹配哈希。Base 三个文件的路径为当前导出器固定读取路径。

原始目录的索引可用仓库脚本 `scripts/generate_wbc_emotion_index.py` 自动生成。它只读目录及文件字节，不加载 `.pt`，只依赖 Python 3.10+ 标准库；默认扫描全部 `lora/<分类>/<动作>/`，按文件名轮次选择最新的 `lorackpt_N.pt`，也可用 `--step` 指定轮次。同轮次 `checkpoint_N.pt` 存在时记录路径和哈希，不存在时省略；不会为匹配完整 checkpoint 而改选旧 LoRA。它检查 reference 文件存在并计算哈希，不判断模型训练质量。

在训练结果所在机器的新终端先预览三个动作，不写文件：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
training_root='/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果'
python3 -B "$local_ws/src/l0_ros2_deploy_v2/scripts/generate_wbc_emotion_index.py" \
  "$training_root" --actions L0_Hugme L0_act_cute_1 L0_act_cute_2 --dry-run
)
```

实际写入自定义文件名 `emotion.json` 的示例；不加 `--actions` 表示扫描全部动作。目标已存在会拒绝覆盖：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
training_root='/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果'
python3 -B "$local_ws/src/l0_ros2_deploy_v2/scripts/generate_wbc_emotion_index.py" \
  "$training_root" --step 2000 --output emotion.json
)
```

相对 `--output` 路径以训练根目录为基准。现有导出器固定读取 `emotion_index.json`，新包首次生成时省略 `--output` 即使用该名称；自定义 `emotion.json` 不会自动替代已有索引。输出记录保存轮次、动作分类、文件相对路径和实际哈希，不从文件名推断 rank、alpha 或精度。这个脚本用于训练原始目录，板端最终 FP16 交付包仍使用 `base_manifest.json`，不需要训练索引。

导出前可以省略逐动作完整 `checkpoint_*.pt`，共享 `base/base_ckpt.pt` 和紧凑 `lorackpt_*.pt` 仍必需。缺少完整动作 checkpoint 时，导出器用 Base+LoRA 重建的 FP32 actor 生成数值期望，并记录 `expected_source: base_plus_lora_actor`；完整文件存在时继续校验其哈希、actor 和归一化统计，失败仍报错。旧索引指向的完整文件缺失也支持，报告会记录 `full_checkpoint_verification: missing`，不声称完成独立 checkpoint 对照。原始 `.pt` 不提前转换或重新保存为 FP16，以免改变 Base 身份哈希并破坏原有匹配；**FP16 转换作用于导出副本和最终交付参数包。** 如果原始输入只有 `.pt` / `.npz`，训练端先通过训练项目的导出流程补齐原生 Base ONNX/YAML 和 reference ONNX。

### 0.3 本机：一个路径完成训练交付处理

相关工具已统一移到仓库 `scripts/`，旧根目录命令须增加 `scripts/`。在本机工作空间执行下面一个命令；唯一必填参数是第 0.2 节约定的训练目录，默认处理索引内全部动作：

```bash
/home/luvbot/miniconda3/envs/active_adaption/bin/python -B \
  /home/luvbot/SC/l0_deploy_v2_ws/src/l0_ros2_deploy_v2/scripts/process_wbc_lora_delivery.py \
  /home/luvbot/SC/l0_deploy_v2_ws/training_inputs/L0_V5_WBC10000_LoRA80_20261026
```

其他机器使用安装了 PyTorch、NumPy、PyYAML、ONNX、ONNX Runtime 的 Python 3.10+ 环境。脚本以仓库所在工作空间为默认输出位置，与运行命令时的当前目录无关。默认生成带时间戳的新包名，也可用 `--name wbc_lora_20261028` 指定名称、`--actions L0_Hugme L0_act_cute_1 L0_act_cute_2` 选择子集、`--workspace <目录>` 指定独立验证工作空间。所有目标必须不存在，失败后查看日志并使用新包名重试。

一次执行依次完成：输入路径/哈希检查 → FP16 导出与 CPU 数值检查 → 全量 SHA256 校验 → reference 编译前处理 → 板端文件白名单审计 → 编译命令 dry-run。默认每动作检查 33 个策略样本，reference 检查全部帧及边界。**不删除、重存或改写训练输入；不连接机器人，不执行 RKNN/ROS 编译、安装到现有策略目录或启动硬件。** 存在的可选完整 checkpoint 仍须通过校验，缺失时使用 Base+LoRA 重建，不替换成 Base 对照。

输出：

| 本机目录 | 内容与用途 | 是否传给机器人 |
| --- | --- | --- |
| `model_inbox/<包名>/` | FP16 完整导出归档，含 CPU 报告、probe、验证 NPZ | 否，留本机追溯 |
| `model_sources/<包名>/` | 已完成前处理的最小编译包，见 0.4 | **仅传此目录** |
| `smoke_logs/<包名>/` | 各步骤日志、退出码、`PROCESSING.json` | 否 |

板端编译包严格限制为共享 ONNX/manifest、每动作 adapter manifest/weights/config/reference，以及 `PREPROCESSING.json`、`ASSETS.sha256`，合计 `4 + 4 × 动作数` 个文件；不存在任何 `.pt`（包括 Base、紧凑 LoRA 和完整 checkpoint）、训练日志、视频、原始 NPZ、probe、验证样本或无关文件。每阶段失败立即退出，报告只有在全部步骤成功后才为 `passed`。完整导出归档也不含 `.pt`。

### 0.4 向板端交付的轻量编译目录

```text
model_sources/<包名>/
├── base_manifest.json
├── base_lora.onnx                        # Base 权重 FP16，33 输入/输出接口 FP32
├── PREPROCESSING.json                   # 原始导出与编译副本的来源/哈希
├── ASSETS.sha256
├── adapters/<动作ID>/
│   ├── manifest.json
│   └── weights.bin                      # FP16，完整 31 张量
└── actions/<动作ID>/
    ├── rl_config.yaml
    └── reference_motion.onnx            # FP16 值展开的 FP32 编译表，接口 FP32
```

每个动作各有一组 adapter 和 reference，共享 policy 只有一份。FP16 原始 reference 保留在本机 `model_inbox`，前处理仅将量化后的值展开以兼容现有编译器，不能再次对 `model_sources` 执行该前处理。来源身份和数据精度契约不变。验证样本及 probe 留在本机归档，不发送到板端。

板端后续仅需编译共享 policy/各 reference，并执行 `scripts/prepare_wbc_deploy.py --mode install-lora`。安装器校验 ONNX、RKNN 与转换记录的一致性后，只复制 C++ 实际读取的 RKNN、转换记录、manifest、weights、config 和 `INSTALLATION.json`；ONNX 留在 `model_sources` 用于重编译，不重复安装到 `legged_deploy/policy` 和 ROS share。来源 ONNX 哈希仍保留在 manifest 和转换记录中，运行时继续验证这些身份。运行目录不包含 `.pt`、ONNX、probe 或验证样本。

### 0.5 部署端前处理与检查范围

统一入口已在本机通过前处理脚本完成文件完整性、哈希、模型结构、张量名称/形状/类型、动作 ID、关节顺序与 reference 元数据检查，并生成供编译器使用的 reference 副本。板端收到 `model_sources` 后直接编译、安装；只有收到原始 FP16 导出归档时才需要另行前处理。ONNX checker、哈希与有限值检查属于格式及完整性检查，仍然保留；它们不等于数值精度验证。

板端转换命令全部使用 `--build-only`，不附加 `--target`、`--device-id` 或数值验证样本，不调用 `scripts/verify_wbc_lora_rknn.py`，不通过移除 `--build-only` 运行 Python Runtime 对照。`verification_backend: not_run`、`verification: null` 是本流程的预期状态，无需补成 `passed`。`max_abs_error` 即使仍存在于工具记录，也只是默认阈值，不是实测结果；`converted_pending_runtime` 对应后续加载、动作和控制链运行检查。

### 0.6 可选：单独生成编译工作包（统一入口已包含）

以下是本次在本地主机执行的命令。脚本检查 FP16 交付包，将 reference 规整成现有 RKNN 转换器支持的结构，逐字节保留共享 policy、LoRA 参数和配置，最后生成 `PREPROCESSING.json` 与新的 `ASSETS.sha256`。它不读取 `.pt`，不运行模型推理。

```bash
(
set -e
cd /home/luvbot/SC/l0_deploy_v2_ws
/home/luvbot/miniconda3/envs/active_adaption/bin/python -B src/l0_ros2_deploy_v2/scripts/preprocess_wbc_lora_fp16.py \
  model_inbox/wbc_lora_fp16 model_sources/wbc_lora_fp16
(cd model_sources/wbc_lora_fp16 && sha256sum -c ASSETS.sha256)
)
```

输出必须是新目录；追加 `--dry-run` 可只检查。reference 编译表展开为 FP32，但数值仍是已交付的 FP16 值，帧索引和元数据保持原样。来源及输出哈希均保存在前处理记录中。

本次 80 个动作的前处理及两份资产清单校验已通过，两个输出包均无 `.pt`。训练端最终共享策略最大绝对误差为 `0.001841`（阈值 `0.05`）；reference 共检查 26,773 个帧索引，最大误差为 `0.011273`（阈值 `0.02`）。执行日志位于工作空间 `smoke_logs/wbc_lora_preprocess_20261027/`。

以上为 2026-10-27 的历史记录。当前统一入口已自动完成本节；按第 2.3 节仅上传 `model_sources` 编译包，跳过板端第 3.0 节。CPU 导出检查与编译 dry-run 不代表 RK3588 编译或机器人运行验收。

### 0.7 本次本机验证（2026-10-28）

- 三动作轻量输入：从现有训练资产复制 Base、紧凑 LoRA 和 reference，省略全部完整动作 checkpoint；覆盖索引省略 checkpoint 和索引保留缺失路径两种情况。一次处理通过，板端编译包 16 文件、14.60 MiB、0 PT。
- 80 动作完整处理：只传 `training_inputs/L0_V5_WBC10000_LoRA80_20261026` 路径，自动生成 `wbc_lora_20261028_105159_584983`；324 文件、83.02 MiB、0 PT，输入哈希未变。每动作 33 个策略样本、reference 共 26,773 个帧索引通过；最终 policy 最大绝对误差 0.0018404（阈值 0.05），reference 0.0112725（阈值 0.02），动作切换复现检查通过。
- 利用本机已有 80 动作 RKNN 包安装到独立验证目录：404 文件、73.14 MiB，全部文件 SHA256 通过；省去 47.01 MiB ONNX，不包含 PT、ONNX 或验证 NPZ。只检验安装内容，未运行 RKNN。
- 大写 `.PT`、无关文件、损坏参数、reference 哈希不符、训练路径越界、输出目录重用和未知动作均被拒绝；迁移后的板端构建包装脚本 dry-run 通过。

记录：工作空间 `smoke_logs/wbc_lora_20261028_105159_584983/PROCESSING.json`、`validation/wbc_lora_pipeline_20261028/VALIDATION.json`，数值明细在同名 `model_inbox` 的 `validation_report.json`。本次没有 SSH、rsync、RKNN 编译、ROS 构建或机器人操作；本机通过不代表板端运行验收。

## 1. 工作位置与 Remote VS Code

| 项目 | 本文约定 |
| --- | --- |
| 本地主机工作空间 | `/home/luvbot/SC/l0_deploy_v2_ws` |
| 板端连接 | `forlinx@192.168.5.2`，不同地址请修改同步命令 |
| 板端工作空间 | `/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws` |
| 部署源码 | `<工作空间>/src/l0_ros2_deploy_v2` |
| 匹配接口源码 | `<工作空间>/src/luvbot_ros_interfaces` |
| 本机 FP16 导出归档 | `<本机工作空间>/model_inbox/<包名>`，不上传 |
| 前处理后的编译工作包 | `<工作空间>/model_sources/wbc_lora_fp16` |
| 安装策略目录名 | `wbc_lora_fp16` |
| AMP 策略 | `l0_v5_amp_07090` |
| Toolkit2 Python | 板端 `/usr/bin/python3`，已使用版本 `rknn-toolkit2==2.3.2` |
| ROS underlay | `/opt/ros/humble/setup.bash` |
| 真实 HPM 启动 | 由机器人原有接口、脚本或服务独立管理；部署侧连接已有实例 |

在本地主机安装 VS Code 的 Remote - SSH 扩展，执行 **Remote-SSH: Connect to Host...**，连接 `forlinx@192.168.5.2`，打开板端工作空间目录。确认左下角显示该 SSH 主机；通过 Terminal → New Terminal 新建的终端才在板端执行。构建在工作空间根目录进行。普通本地窗口中的终端仍在本地主机运行。[VS Code 官方 Remote SSH 说明](https://code.visualstudio.com/docs/remote/ssh)

下文明确标注“本地主机”的块才在本地主机执行，其他块均在板端执行。普通 SSH 也可使用：

```bash
ssh forlinx@192.168.5.2
```

每个代码块都重新设置必要路径；每个 ROS 块都重新 source underlay、工作空间和正式运行配置。**新终端不能继承另一个终端的 shell 变量。请复制完整块。** 外层 `( ... )` 为子 shell，变量不保留到下一个块。不要在 source ROS 前启用 `set -u`。

已有板端工作空间须具备 ROS2/colcon、匹配的 `luvbot_ros_interfaces` 与 `robot_interfaces`、`joy`、机器人描述、RKNN 头文件/运行库及仓库声明的构建依赖。HPM 的电机和 IMU 传输由其独立工作空间负责。本手册不替代操作系统、HPM、关节标定和驱动的首次安装。

## 2. 动作放在哪里，复制哪些文件

### 2.1 完整 LoRA 动作包的结构

默认接收第 0.3 节生成的 `model_sources/<包名>` 轻量编译包。以下以 `wbc_lora_fp16` 为示例，实际使用脚本输出的包名；`model_inbox` 只在本机保留。此处展示前处理、板端 RKNN 编译与安装后的目录；`.rknn`、`.rknn.json` 和 `INSTALLATION.json` 在上传时尚不存在，属于后续生成文件。

Base 固定权重、LoRA 参数及 reference 浮点轨迹数据均由训练端以 FP16 交付，ONNX 输入输出接口保持 FP32。板端不接收原始 checkpoint 来重新导出；第 0.3 节已经在本机规整 reference，第 3.1 节生成 RKNN，第 4 节安装资产并构建 ROS。

```text
l0_deploy_v2_ws/
├── model_inbox/
│   └── wbc_lora_fp16/                    # 仅本机：完整导出归档，不上传
├── model_sources/
│   └── wbc_lora_fp16/                 # 前处理输出；编译后在原处产生 RKNN
│       ├── PREPROCESSING.json            # 原包/编译副本哈希与处理记录
│       ├── base_manifest.json             # 共享图、动作目录与文件哈希
│       ├── base_lora.onnx                  # 唯一共享 policy；Base 权重 FP16，接口 FP32
│       ├── base_lora.rknn                  # RK3588 编译产物
│       ├── base_lora.rknn.json             # 转换记录，安装时必须存在
│       ├── adapters/
│       │   ├── L0_Hugme/
│       │   │   ├── manifest.json
│       │   │   └── weights.bin             # FP16 参数包
│       │   ├── L0_act_cute_1/              # 同样包含 manifest.json / weights.bin
│       │   └── L0_act_cute_2/              # 本版使用该动作训练 LoRA，非旧 Base 对照
│       └── actions/
│           ├── L0_Hugme/
│           │   ├── rl_config.yaml
│           │   ├── reference_motion.onnx   # FP16 交付值展开的 FP32 编译副本
│           │   ├── reference_motion.rknn
│           │   └── reference_motion.rknn.json
│           ├── L0_act_cute_1/              # 同上
│           └── L0_act_cute_2/              # 同上
├── src/
│   ├── luvbot_ros_interfaces/
│   └── l0_ros2_deploy_v2/
│       └── legged_deploy/policy/l0_v5/
│           ├── l0_v5_amp_07090/            # AMP，与 WBC 包分开
│           │   ├── policy.rknn
│           │   ├── rl_config.yaml
│           │   └── joy_config.yaml
│           └── wbc_lora_fp16/          # install-lora 生成；保留上述包结构
│               ├── INSTALLATION.json
│               ├── base_manifest.json
│               ├── base_lora.rknn
│               ├── base_lora.rknn.json
│               ├── adapters/...
│               └── actions/...           # 运行包无 ONNX；仅 config/RKNN/转换记录
├── install/legged_deploy/share/legged_deploy/policy/l0_v5/
│   ├── l0_v5_amp_07090/                    # 安装的 AMP 配置；当前 AMP 模型仍从源码目录加载
│   └── wbc_lora_fp16/                  # 正式运行读取这里
├── deploy_config/                         # 模型和动作选择、可选录包 QoS
└── runs/wbc_production/                    # 每次运行独立日志与 bags
```

合并安装工作空间的 share 路径可能不同，应以 `ros2 pkg prefix legged_deploy` 为准。验证样本、probe 和训练原始结果由训练端归档，正式部署不要求将它们放入源包或安装目录。

LoRA 动态加载指切换参数值：共享 policy 有 2 个观测输入和 31 个参数输入，形状固定。`weights.bin` 不转 RKNN；所有动作复用一份 `base_lora.rknn`，每个动作的 reference 单独转换。不要从其他导出批次单拷一个 adapter、手改 `rl_config.yaml` 或拼接 manifest；这些文件受哈希和 schema 校验约束。

### 2.2 源码交付范围：只合并模型加载相关改动

正式交付沿用硬件侧维护的部署入口与通信代码。不要用当前整个工作区覆盖板端源码；工作区含有独立的键盘功能删改、机器人默认值调整、硬件配置改动和历史隔离测试代码。旧 `wbc-lora-sync-files.txt` 含隔离端点及硬件包文件，也不能作为本次正式交付的整包同步清单。

| 范围 | 本次处理 |
| --- | --- |
| 模型推理与资产加载 | 审查并合并 `legged_deploy/include/{rknn_model,rl_controller_wbc,wbc_lora_assets}.h`、对应 `src/*.cpp` 及 `legged_deploy/CMakeLists.txt` 中必要改动 |
| 原启动链模型参数 | 只合并 `sim2real.launch.py → sequential_launch_real.sh → one_start.launch.py` 中 `wbc_mode`、`wbc_lora_bundle`、`wbc_policies` 的传递及 manifest 解析，保留硬件侧原有配置和其他参数 |
| 前处理、转换、模型安装 | 交付对应 Python 工具及其依赖模块；模型资产按第 2.3 节单独传输 |
| 硬件维护范围 | 保留板端 HPM 启动、ROS/DDS 环境、`legged_real`、`robot_interfaces`、电机映射、方向和标定配置；本次不覆盖 |
| 隔离测试范围 | `isolated_hpm`、隔离域设置和测试重映射不作为正式模型加载所需改动交付 |

仓库当前提交版本与工作区之间存在非模型差异，不能简单用 `git restore` 整体恢复，也不能将所有差异一并交付。以硬件侧原有正式版本为合并基线，保留其既有修改；本手册只说明模型加载所需接入，不修改硬件启动参数。若板端已合并模型加载代码，则直接继续模型安装与构建。

### 2.3 本地主机：上传轻量编译包（手动操作）

一次处理结束后，将下面的包名改成脚本实际输出的名称，仅传输 `model_sources`。板端目标目录要求新建，不覆盖正在运行的资产。下面是用户后续手动执行的命令，处理脚本不会执行 SSH/rsync。

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
local_assets="$local_ws/model_sources/wbc_lora_fp16"
remote_assets="$robot_ws/model_sources/wbc_lora_fp16"
test -f "$local_assets/base_manifest.json"
test -f "$local_assets/base_lora.onnx"
ssh "$robot_target" "test ! -e '$remote_assets' && mkdir -p '$remote_assets'"
rsync -rtv --itemize-changes "$local_assets/" "$robot_target:$remote_assets/"
)
```

如果传输中断，只对确认属于本次上传、尚未用于运行的目录重试最后一条 rsync；先重新定义此块中的路径变量。批量转换和安装会校验 manifest 中的源文件哈希。已经安装好当前包、只更新程序时，可直接跳至第 4.2 节构建，不必重新上传或转换。

AMP 目录也必须存在。当前源码中的 `l0_v5_amp_07090` 可能是指向 v4 目录的软链接，单独复制链接会导致板端缺文件。板端已部署 AMP 时保留原目录。首次导入 AMP 可在本地主机执行以下可选块，`-L` 解引用模型链接，目标目录必须尚不存在：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
local_amp="$local_ws/src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_07090"
remote_amp="$robot_ws/src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_07090"
test -f "$local_amp/policy.rknn"
test -f "$local_amp/joy_config.yaml"
ssh "$robot_target" "test ! -e '$remote_amp' && test ! -L '$remote_amp' && mkdir -p '$remote_amp'"
rsync -rtLv --itemize-changes "$local_amp/" "$robot_target:$remote_amp/"
)
```

### 2.4 仅更新日常操作脚本与文档

[wbc-operation-sync-files.txt](wbc-operation-sync-files.txt) 仅列出本次六个操作文件、对应文档和 README，不含模型、C++、硬件配置或 HPM 文件。三个入口依赖同目录的公共 Bash、Python 和 QoS 文件，不能只复制入口 `.sh`。在本地主机执行；已有同名文件会备份到板端独立目录，不使用 `--delete`：

```bash
(
set -e -o pipefail
local_repo=/home/luvbot/SC/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
backup="$robot_ws/build_logs/operator_scripts_backup_$(date +%Y%m%d_%H%M%S_%N)"
rsync -rtv --backup --backup-dir="$backup" \
  --files-from="$local_repo/docs/wbc-operation-sync-files.txt" \
  "$local_repo/" "$robot_target:$robot_ws/src/l0_ros2_deploy_v2/"
)
```

脚本直接使用已安装的 ROS Humble、系统 `/usr/bin/python3`（PyYAML、rclpy）、Bash、flock、setsid 和现有模型。仅此清单变更无需重编译，不会修改 `deploy_config/wbc-production.env` 或开始运行节点。后续从第 5 节选择模型，再分别使用三个入口。

## 3. 板端：RKNN 模型编译

**统一入口已生成 `model_sources` 编译包，上传后直接执行第 3.1 节，跳过 3.0。只有单独收到未经前处理的 FP16 导出归档时才执行 3.0。不要直接用原始 FP16 reference 调用旧转换器。**

这些命令在 RK3588 板端执行，使用 Toolkit2 环境，与 ROS C++ 构建分开。关闭正在运行的部署任务后转换；不在实时控制期间进行模型编译。当前转换器固定目标 `rk3588`、FP16、非量化；`--build-only` 仅编译，不初始化 Python NPU Runtime，不与 `--target` / `--device-id` 同用。

先确认工具链：(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
source /opt/ros/humble/setup.bash
source "$robot_ws/install/setup.bash"
source "$robot_ws/deploy_config/wbc-production.env"
ros2 launch legged_controllers sim2real.launch.py \
  "robot_type:=$robot_type" "subpolicy:=$amp_policy" \
  wbc_enabled:=true wbc_mode:=lora \
  "wbc_lora_bundle:=$wbc_bundle" \
  "wbc_policies:=$wbc_actions"(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
source /opt/ros/humble/setup.bash
source "$robot_ws/install/setup.bash"
source "$robot_ws/deploy_config/wbc-production.env"
ros2 launch legged_controllers sim2real.launch.py \
  "robot_type:=$robot_type" "subpolicy:=$amp_policy" \
  wbc_enabled:=true wbc_mode:=lora \
  "wbc_lora_bundle:=$wbc_bundle" \
  "wbc_policies:=$wbc_actions"
)l; print("ONNX / NumPy / YAML imports OK")'
test -f "$robot_ws/model_sources/wbc_lora_fp16/base_manifest.json"
)
```

设备树应包含 `rk3588`，当前已用 Toolkit2 为 2.3.2；不要把 Toolkit Lite2 当成支持编译的 Toolkit2。板端 C++ Runtime 也需与模型兼容。命令失败时先解决环境，不在其他 Python 环境中盲目重试。

### 3.0 可选：板端生成编译工作包（统一入口交付时跳过）

此脚本只需要 ONNX、NumPy、PyYAML，不需要 PyTorch、ONNX Runtime 或 RKNN Runtime。源目录应是训练端实际交付的 FP16 包；不能拿旧 FP32 模型包改名代替。输出目录不能提前创建；需要预览时，给 Python 命令追加 `--dry-run`，不产生任何输出文件，也不继续执行后续编译步骤。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_inbox/wbc_lora_fp16"
prepared_bundle="$robot_ws/model_sources/wbc_lora_fp16"
/usr/bin/python3 -B "$robot_repo/scripts/preprocess_wbc_lora_fp16.py" \
  "$source_bundle" "$prepared_bundle"
)
```

完成后继续第 3.1 节。前处理成功而 RKNN 编译失败时，修复编译问题后只重跑编译；不重复对同一输出目录执行前处理。更换训练资产或前处理脚本时使用新的工作目录，保留旧记录。

### 3.1 推荐：一个包内所有动作批量编译

下面依次编译一份共享 policy 和 manifest 中每个动作的 reference。本次 80 动作包将生成 81 个 RKNN。逐模型日志、退出码和 `conversion_summary.json` 保存在新目录中。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_sources/wbc_lora_fp16"
mkdir -p "$robot_ws/build_logs/wbc_models"
conversion_log="$robot_ws/build_logs/wbc_models/$(date +%Y%m%d_%H%M%S_%N)"
/usr/bin/python3 -B "$robot_repo/scripts/batch_convert_onnx_to_rknn_rk3588.py" \
  "$source_bundle" --manifest "$source_bundle/base_manifest.json" \
  --build-only --log-dir "$conversion_log"
printf '转换日志：%s\n' "$conversion_log"
)
```

可选参数：`--dry-run` 校验源包并打印待执行命令；`--skip-existing` 仅在源文件、RKNN、转换器、Toolkit 版本等 provenance 匹配时跳过；`--policy-only` 只编译共享 policy。本版不使用 `--include-probes`，不运行数值验证。`--log-dir` 必须是新目录。

`--build-only` 产生的 `verification_backend: not_run` / `verification: null` 是本版预期记录：部署端按约定不做数值验证，不需要补跑对照。仍需完成模型加载、动作执行及控制链运行检查；编译成功不记作数值精度通过。

### 3.2 可选：单独编译共享 policy

用于修复单模型转换或检查图结构；完成批量转换后无需重复执行。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_sources/wbc_lora_fp16"
/usr/bin/python3 -B "$robot_repo/scripts/convert_onnx_to_rknn_rk3588_wbc_policy.py" \
  "$source_bundle/base_lora.onnx" --manifest "$source_bundle/base_manifest.json" \
  --build-only
)
```

### 3.3 可选：单独编译一个已前处理动作的 reference

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_sources/wbc_lora_fp16"
action_id=L0_Hugme
/usr/bin/python3 -B "$robot_repo/scripts/convert_onnx_to_rknn_rk3588_wbc_reference.py" \
  "$source_bundle/actions/$action_id/reference_motion.onnx" --build-only
)
```

将 `action_id` 改成 manifest 中的另一个动作即可。reference 转换器没有 `--dry-run` 参数。

### 3.4 可选：多个独立 LoRA 包批量前处理和编译

首次处理 `model_inbox` 下的多个完整训练包时，使用此块代替逐包执行 3.0/3.1。每个一级子目录必须有自己的 `base_manifest.json`，对应 `model_sources` 目的目录均须尚不存在；按包串行处理，任何包失败立即停止。部分完成后应逐个继续未完成的包，避免覆盖既有工作目录。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
mkdir -p "$robot_ws/build_logs/wbc_models"
batch_log=$(mktemp -d "$robot_ws/build_logs/wbc_models/multi_$(date +%Y%m%d_%H%M%S)_XXXXXX")
found=0
for training_bundle in "$robot_ws"/model_inbox/*; do
  test -f "$training_bundle/base_manifest.json" || continue
  found=$((found + 1))
  source_bundle="$robot_ws/model_sources/$(basename "$training_bundle")"
  /usr/bin/python3 -B "$robot_repo/scripts/preprocess_wbc_lora_fp16.py" \
    "$training_bundle" "$source_bundle"
  /usr/bin/python3 -B "$robot_repo/scripts/batch_convert_onnx_to_rknn_rk3588.py" \
    "$source_bundle" --manifest "$source_bundle/base_manifest.json" \
    --build-only --skip-existing --log-dir "$batch_log/$(basename "$source_bundle")"
done
test "$found" -gt 0
printf '完成 %s 个包，日志：%s\n' "$found" "$batch_log"
)
```

### 3.5 可选：旧式独立 WBC 动作编译

仅适用于每个动作自带 `policy.onnx`、`reference_motion.onnx`、`rl_config.yaml` 的静态 WBC，目录示例为 `model_sources/wbc_static/L0_动作名/...`。它不属于 LoRA 动态参数模式，不与主流程混用。

转换器扫描根目录的子目录；不要把整个 `policy/l0_v5` 作为根目录，以免把 AMP 等其他模型送入 WBC 转换器。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
/usr/bin/python3 -B "$robot_repo/scripts/batch_convert_onnx_to_rknn_rk3588.py" \
  "$robot_ws/model_sources/wbc_static" --build-only
)
```

静态模式的 `--skip-existing` 只依据文件存在，不做 LoRA 模式的 provenance 检查；源模型更新后应重新编译。静态模式不支持 `--log-dir`，可用 shell 重定向另存日志。后续第 4～8 节的命令针对 LoRA 包。

## 4. 板端：安装动作、colcon build、source

### 4.1 将编译好的整个包安装到源码策略目录

先完成第 3 节全部 RKNN 编译，再运行本节安装；缺失 reference RKNN 时不能安装完整包。安装器校验模型、转换记录、adapter、reference、配置与哈希，并生成 `INSTALLATION.json`。目标必须不存在；更新动作时用版本化的新目录，而不是覆盖旧包。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_sources/wbc_lora_fp16"
bundle_name=wbc_lora_fp16
/usr/bin/python3 -B "$robot_repo/scripts/prepare_wbc_deploy.py" \
  "$source_bundle" "$robot_repo/legged_deploy/policy/l0_v5/$bundle_name" \
  --mode install-lora
)
```

### 4.2 构建并安装 ROS 包

在板端新的干净终端执行。仅源码变更、资产已安装时也执行此块。若上一步成功而本步失败，修复后只重跑构建，不重复 install-lora。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source /opt/ros/humble/setup.bash
test -d "$robot_ws/src/luvbot_ros_interfaces"
test -f "$robot_repo/legged_deploy/policy/l0_v5/wbc_lora_fp16/INSTALLATION.json"
mkdir -p "$robot_ws/build_logs/wbc_colcon"
build_dir=$(mktemp -d "$robot_ws/build_logs/wbc_colcon/$(date +%Y%m%d_%H%M%S)_XXXXXX")
cd "$robot_ws"
colcon build --packages-up-to legged_deploy legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN 2>&1 | tee "$build_dir/build.log"
source "$robot_ws/install/setup.bash"
for package in legged_deploy legged_real legged_controllers luvbot_control_interfaces; do
  prefix=$(ros2 pkg prefix "$package")
  [[ "$prefix" == "$robot_ws/install" || "$prefix" == "$robot_ws/install/"* ]]
  printf '%s: %s\n' "$package" "$prefix"
done
deploy_prefix=$(ros2 pkg prefix legged_deploy)
test -f "$deploy_prefix/share/legged_deploy/policy/l0_v5/wbc_lora_fp16/base_lora.rknn"
ldd "$deploy_prefix/lib/legged_deploy/rl_controller" | tee "$build_dir/rl_controller.ldd.log"
if grep -q 'not found' "$build_dir/rl_controller.ldd.log"; then exit 1; fi
printf '构建成功，日志：%s\n' "$build_dir"
)
```

普通 colcon 安装会把 `legged_deploy/policy` 复制到 package share。WBC 的模型、manifest 和动作配置均通过该安装目录定位；只改源码策略目录、只复制 RKNN、或只执行 source，都不会替代这次构建安装。

**当前 AMP 的模型加载有例外：** `legged_deploy/CMakeLists.txt` 将 `LEGGED_POLICY_PATH` 固定为编译时的源码 `policy` 目录，AMP 从那里读取 `policy.rknn`，而其启动配置从 package share 读取。因此必须保留板端 `src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_07090` 及软链接实际目标；更新 AMP 时确保源码模型与安装配置属于同一版本。不要只搬走 `install` 目录后删除源码资产；改变工作空间路径后需要重新构建。本文按现有代码说明，不修改加载逻辑。

## 5. 板端：模型配置与运行记录

### 5.1 保存模型选择

日常操作使用仓库 `scripts/` 中的三个独立入口：sim2real 启动、可选录包、交互动作发布。HPM 由操作者按硬件侧原流程手动启动和停止，没有 HPM 包装脚本。脚本不设置 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 或 `RMW_IMPLEMENTATION`；各终端沿用原来与 HPM 正常通信的环境。

模型配置仍为 `<工作空间>/deploy_config/wbc-production.env`。已配置的板端无需重建此文件；查看后按实际安装包调整，不能直接套用旧示例覆盖现有 AMP。2026-10-28 bushu1028 编译交付后的配置为：

```bash
robot_type=l0_v5
amp_policy=l0_v5_amp_10273_mjlab_no_torsional_friction
wbc_bundle=wbc_lora_bushu1028_20261028
wbc_actions='["L0_angry_10", "L0_crouch_off", "L0_strange_new_5", "L0_strange_new_5_1"]'
```

`wbc_actions` 为要预加载的动作 ID 数组，可以是已安装 manifest 的子集，不能有重复或未知 ID；这里不是自动执行列表。配置变更后按原流程重启部署。交互控制台读取该次启动的配置快照，并核对运行中 worker 的实际参数，因此只修改当前配置文件不能改变正在运行的动作名单。

共享 `base_ckpt` 不决定动作完成后的状态。本次只增加操作脚本，未增加动作级完成状态配置；`L0_crouch_off` 成功结束后仍按现有代码返回 AMP，不会自动进入 EmergencyStop。

### 5.2 运行记录由启动脚本创建

`start_sim2real.sh` 每次启动自动创建 `runs/wbc_production/<时间戳>_<随机串>/`，并更新 `current_run_dir`。不再手动执行长块创建 run。启动前检查 WBC 安装清单哈希、AMP 源码/安装配置一致性及 worker 文件，记录模型清单、环境快照、源码版本和 diff、worker/AMP 哈希、启动命令、ROS 日志与终端输出。

源码 diff 不含未跟踪文件的完整内容，应另保留构建所用源码。`model_sources/<包名>/PREPROCESSING.json` 存在时归档，只有轻量运行包时允许省略。启动脚本不会编译、安装或改写模型。

录包和动作控制台读取 `current_run_dir`，各自固定到打开时的运行目录；后续重启部署后应重新打开这些终端。已封存的记录（存在 `LOGS.sha256`）拒绝追加写入。本次目录被清理时，应在结束旧部署后重新启动创建新记录，不能将指针改到任意其他目录。

## 6. 板端：独立启动 sim2real

### 6.1 正式启动入口

先按原硬件流程手动准备 HPM，再在独立终端执行：

```bash
cd /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
bash scripts/start_sim2real.sh
```

脚本自动 source ROS Humble 和当前工作空间，读取第 5.1 节的配置。实际调用仍为 `ros2 launch legged_controllers sim2real.launch.py`，显式传入机器人、AMP、WBC 包及动作名单，并固定 `wbc_enabled:=true wbc_mode:=lora hpm_mode:=real bag_enabled:=false`。其余硬件、手柄、安全参数沿用板端入口默认值。`hpm_mode:=real` 依赖外部 HPM，不会启动真实或隔离 HPM。

部署保持原 `sim2real → sequential_launch_real → 硬件桥与就绪检查 → one_start → AMP/WBC` 顺序。该脚本不启动录包或动作控制台、不发送动作、不跳过就绪检查。HPM 停止时，硬件桥可能初始化成功，但 IMU/硬件就绪检查会失败，后面的 AMP/WBC 不会正常启动；不能将其当作完整运行链通过。

本终端保持前台运行。完成原有停机步骤后 Ctrl+C，由原启动链清理子进程；日志保存到本次目录的 `deploy.log` 和 `ros/`。`launch.exitcode` 只记录外层 launch 返回值；返回 0 仍须检查日志中的子节点退出与模型初始化结果。已有部署节点运行时脚本拒绝再启动第二套。

只预览配置与命令，不新建运行目录或启动节点：

```bash
bash scripts/start_sim2real.sh --dry-run
```

三个 Bash 入口都支持 `--help`、`--workspace /绝对/工作空间` 和 `--dry-run`。默认从脚本位置推导工作空间，不依赖当前目录；预览仍需已有 ROS 环境、工作空间和配置。它们直接从源码 `scripts/` 执行，只更新这些脚本无需重新 colcon build。

2026-10-28 操作脚本交付检查：本机与板端 Bash/Python/YAML 语法、文档命令块、四动作 manifest/config 哈希和输入解析已检查；板端三类入口的 `--dry-run`（录包覆盖 A/B/C/D）通过。此次检查未启动 HPM、sim2real 或录包节点，未发布动作；真实反馈与 Ctrl+C 进程回收仍需在后续授权运行时确认。

### 6.2 只读检查模型加载与既有通信端点

任意新板端终端执行：

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source /opt/ros/humble/setup.bash
source "$robot_ws/install/setup.bash"
source "$robot_ws/deploy_config/wbc-production.env"
ros2 node list
for node in /topic_hardware_adapter /rl_controller /rl_controller_wbc; do
  ros2 node info "$node"
done
service_type=$(ros2 service type /control/policy/wbc/prepare)
test "$service_type" = luvbot_control_interfaces/srv/ControlWbcMotion
ros2 topic info /control/policy/observation --verbose
ros2 topic info /control/policy/command/rl --verbose
ros2 topic info /control/policy/command/wbc --verbose
ros2 topic info /joy --verbose
ros2 topic info /actuator/motor/enable --verbose
ros2 topic info /actuator/motor/state --verbose
ros2 topic info /imu/data --verbose
ros2 param get /rl_controller_wbc wbc_mode
ros2 param get /rl_controller_wbc wbc_lora_manifest
ros2 param get /rl_controller_wbc available_wbc_policies
)
```

节点在 Standby 下可能已建立端点，但策略输出不是持续有效的行走命令。进入 Walking 后还需确认 AMP 持续提供新鲜命令，且无持续的 `Policy command expired or unavailable`。WBC 未活动时 `/control/policy/command/wbc` 暂无消息是正常现象。服务类型查询只证明服务已发现，不等于一次准备调用成功。

## 7. 可选 rosbag：按运控用途分别录包

**所有录包均可选；不录全部话题。** 常规动作回归建议选 A+C，电机反馈/跟踪问题加 B，排查日志时短时加 D。每组用一个独立终端和独立 bag 目录；可以同时开多个组选项。A/B 为高频数据，会增加磁盘和 CPU 负载，性能测量应同时注明录包选择。

rosbag2 按 topic 记录，不按节点自动筛选。下表给出角色到话题的映射；同一话题有多个发布者时都会录入。HPM 的实际节点名用 `ros2 topic info --verbose` 查看。`/rosout` 是共享日志话题，D 组不是只含这几个运控节点的日志。

| 选项 | 关联节点/角色 | 本组显式录制话题 | 用途 |
| --- | --- | --- | --- |
| A，策略输入输出，可选 | 实机运控节点 + AMP + WBC | `/control/policy/observation`、`/control/policy/command/rl`、`/control/policy/command/wbc` | 检查观测、策略序号、有效期、目标关节位置与 AMP/WBC 交接 |
| B，电机与惯导，可选 | HPM + 实机运控节点 | `/actuator/motor/state`、`/actuator/motor/command`、`/imu/data` | 实际电机反馈与下发命令、IMU、看门狗前后的数据 |
| C，动作与安全事件，可选 | 动作请求方 + 实机运控节点 + WBC + 手柄 | `/algorithm/brain/wbc_action_cmd`、`/system/state_machine/wbc_action_fdb`、`/control/policy/wbc/active`、`/control/policy/wbc/done`、`/hardware_status/motors_ready`、`/system/safety/emergency_stop`、`/system/safety/hardware_fault`、`/actuator/motor/enable`、`/joy` | 请求/完成、控制权、急停、故障、使能与操作者输入 |
| D，ROS 日志，可选 | 所有向 `/rosout` 发布日志的节点 | `/rosout` | 跨节点错误时序；也保留各终端文本日志 |

| 话题 | 消息类型 |
| --- | --- |
| `/control/policy/observation` | `luvbot_control_interfaces/msg/PolicyObservation` |
| `/control/policy/command/rl`、`/control/policy/command/wbc` | `luvbot_control_interfaces/msg/PolicyCommand` |
| `/actuator/motor/state` | `robot_interfaces/msg/MotorStateArray` |
| `/actuator/motor/command` | `robot_interfaces/msg/MotorCommand` |
| `/actuator/motor/enable` | `robot_interfaces/msg/MotorEnableCommand` |
| `/imu/data` | `sensor_msgs/msg/Imu` |
| `/joy` | `sensor_msgs/msg/Joy` |
| 动作请求、动作反馈 | `std_msgs/msg/String`，data 为 JSON |
| active、done、motors_ready、emergency_stop、hardware_fault | `std_msgs/msg/Bool` |
| `/rosout` | `rcl_interfaces/msg/Log` |

`/control/policy/wbc/prepare` 是服务，普通 Humble topic 录包不直接录制其调用。C 组记录服务前后的动作请求、反馈与状态。可选扩展 `/odom`（`nav_msgs/msg/Odometry`）只有实际存在发布者时才加进 B 组；当前关闭 RViz 时不要用 `/joint_states` 是否存在判断反馈是否正常。

### 7.1 录包 QoS

默认 QoS 文件随脚本交付：[scripts/wbc-record-qos.yaml](../scripts/wbc-record-qos.yaml)。如果工作空间已有 `deploy_config/wbc-record-qos.yaml`，录包脚本优先使用该文件，不覆盖用户配置。每次录制保存实际使用的 QoS 快照到本次运行目录。

策略与传感器采用 best_effort/volatile；`wbc/active` 和 `motors_ready` 采用 reliable/transient_local；请求与反馈沿用 reliable/volatile。安全事件采用 best_effort/volatile，需要在动作发生前开始录制。QoS 不兼容时查看录包终端和快照，不修改发布者设置来适配录包。

### 7.2 A 组：策略输入输出

在一个独立板端终端执行：

```bash
cd /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
bash scripts/record_wbc_rosbag.sh A
```

### 7.3 B 组：电机命令、反馈和 IMU

另开终端，进入同一仓库后执行：

```bash
bash scripts/record_wbc_rosbag.sh B
```

### 7.4 C 组：动作与安全事件

另开终端，进入同一仓库后执行：

```bash
bash scripts/record_wbc_rosbag.sh C
```

### 7.5 D 组：ROS 日志

另开终端，进入同一仓库后执行：

```bash
bash scripts/record_wbc_rosbag.sh D
```

每个终端只录制所选的一组；可同时打开 A 和 C 等不同组，同一 run 的同一组有重复录制锁。录包不启动 sim2real/HPM，不发送动作，也不下载文件。`--dry-run` 打印话题和命令，不创建 bag 或 ROS 节点。

bag 位于当前 run 的 `bags/<组名>_<时间戳>/`，使用 sqlite3，缓存 16 MiB，单文件达到 256 MiB 或 300 秒分片。分片不限制总磁盘占用，脚本不自动停止录制。Ctrl+C 后等待 recorder 写完 `metadata.yaml`，终端将输出目录；录包日志、退出码和 QoS 快照保存在 run 根目录。

未执行动作时 WBC 命令、done 或动作事件为空可属正常。录包不是完整性保证，仍需按第 9 节检查消息数量、QoS、丢帧与磁盘情况。不要在连接真实 HPM 的环境中直接回放包含 motor command、enable、joy 或动作请求的 bag。

## 8. 独立交互页面：输入一次，发布一个动作

在新的板端终端执行：

```bash
cd /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
bash scripts/wbc_action_console.sh
```

这是持续运行的终端交互页面，不需要浏览器，也不会随 sim2real 自动打开。动作列表来自该次启动的 `wbc_actions` 与安装 manifest，不在脚本中写死。当前四动作配置示例：

```text
WBC 动作控制台
   1. L0_angry_10
   2. L0_crouch_off
   3. L0_strange_new_5
   4. L0_strange_new_5_1
动作> 2
```

输入 `2` 或完整的 `L0_crouch_off` 均对应同一动作；编号随本次加载名单顺序变化。只接受编号和完整 ID，不进行模糊匹配或中文名称猜测。`list`/`help`/`?` 重新显示列表，`q`/`quit`/`exit` 退出。空输入和未知输入不会发布；不接受管道或批量输入。

发送前按原手柄 **B+Y** 进入 Walking 并保持零速度，确认 AMP 有新鲜输出；控制台不切换状态、不解除急停或使能电机。它在每次输入后检查唯一 worker 的 mode、manifest、实际加载名单、prepare 服务和请求/反馈端点；实际状态是否允许动作仍由原控制器判断。

每个有效输入生成新的毫秒时间戳，并显示、执行一次 `ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String ...`。`data` 内保持原有 JSON 契约 `{"timestamp_ms": ..., "action_id": "..."}`，使用 reliable/volatile，禁止重试和自动连发。发布子进程最长等待 3 秒，超时即退出控制台且不重发，避免无限等待订阅者后发送过期请求。

同一页面订阅 `/system/state_machine/wbc_action_fdb`，按 **timestamp_ms 与 action_id 同时匹配** 显示 `started`、`completed`、`rejected` 或 `failed`。发布进程退出不代表动作完成。收到终态前不接收下一动作，执行期间提前输入的内容会丢弃，防止形成动作队列。`failed`、发布结果不确定或等待终态超过参考时长加 30 秒时，控制台退出；`rejected` 显示后返回输入页，由操作者排查原因后决定下一次输入。

请求尝试写入 `action_requests.jsonl`，反馈写入 `action_feedback.jsonl`，发布命令输出写入 `action_publish.log`，都保存在本次 run。只预览列表可使用 `bash scripts/wbc_action_console.sh --dry-run`，不创建 ROS 节点或发布消息。

退出页面、Ctrl+C 或反馈超时都不会替代机器人停机，也不会取消已经交给控制器的动作。停止机器人仍使用原有手柄/硬件流程。`L0_crouch_off` 完成后返回 AMP 的现有逻辑保持不变。

## 9. 停机、检查录包与回收

### 9.1 停机顺序

1. 停止动作请求，等待正在执行的动作完成并回到 AMP。按现场已有停机流程进入安全的非 Walking 状态；需要急停时使用原手柄 RB，不用杀进程代替机器人停机。
2. 按硬件侧原有正式停机流程，在 `sim2real` 所在终端退出部署，由原启动脚本回收其管理的进程。保持 HPM 运行到运控退出和 disable 完成，核对原部署输出及实际电机状态；不要在正常运动中直接中断策略。
3. 需要停止 HPM 时，通过其原有接口或服务管理方式操作；也可按现场要求保留已有公共 HPM 服务。本模型加载交付不接管 HPM 生命周期。
4. 如有录包，保留到上述停机事件结束，再逐个 Ctrl+C，等每个 recorder 写完 metadata。最后退出动作控制台。

### 9.2 检查并封存本次记录

所有写入本次记录目录的进程结束后，在新板端终端执行：

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source /opt/ros/humble/setup.bash
source "$robot_ws/install/setup.bash"
source "$robot_ws/deploy_config/wbc-production.env"
run_root="$robot_ws/runs/wbc_production"
run_dir=$(cat "$run_root/current_run_dir")
[[ "$run_dir" == "$run_root/"* && -d "$run_dir" ]]
export ROS_LOG_DIR="$run_dir/ros"
for bag_path in "$run_dir"/bags/*; do
  test -d "$bag_path" || continue
  test -f "$bag_path/metadata.yaml"
  ros2 bag info "$bag_path" | tee "$run_dir/bag_info_$(basename "$bag_path").txt"
done
/usr/bin/python3 - "$run_dir" <<'PYCODE'
import hashlib, pathlib, sys
root = pathlib.Path(sys.argv[1])
lines = []
for path in sorted(root.rglob('*')):
    if path.is_file() and path.name != 'LOGS.sha256':
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        lines.append(f'{digest.hexdigest()}  {path.relative_to(root)}\n')
(root / 'LOGS.sha256').write_text(''.join(lines))
PYCODE
(cd "$run_dir" && sha256sum -c LOGS.sha256)
printf '%s\n' "$run_dir" > "$run_root/last_completed_run_dir"
printf '已封存记录：%s\n' "$run_dir"
)
```

`bag info` 应包含所选 topic、正确类型、合理 duration 和非零的预期消息数量。动作从未运行时 WBC/动作事件为空是可解释的；声称动作已运行却没有相应记录时不能作完整证据。单个 bag 目录含 `metadata.yaml` 和一个或多个 `.db3`，回收时必须整体复制。[ROS2 官方 rosbag2 用法](https://github.com/ros2/rosbag2/tree/humble)

### 9.3 本地主机：回收完整记录

新建本地主机终端执行，不能在 Remote SSH 的板端终端执行此块：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
remote_root="$robot_ws/runs/wbc_production"
remote_run=$(ssh "$robot_target" "cat '$remote_root/last_completed_run_dir'")
[[ "$remote_run" == "$remote_root/"* ]]
run_name=${remote_run##*/}
[[ "$run_name" =~ ^[0-9]{8}_[0-9]{6}_[A-Za-z0-9_]{6,64}$ ]]
local_root="$local_ws/robot_logs/wbc_production"
mkdir -p "$local_root/$run_name"
rsync -rtv --itemize-changes "$robot_target:$remote_run/" "$local_root/$run_name/"
(cd "$local_root/$run_name" && sha256sum -c LOGS.sha256)
)
```

## 10. 完成条件与常见问题

| 检查项 | 应看到的结果 / 处理方式 |
| --- | --- |
| 训练交付与前处理 | 原始 FP16 包检查通过，独立编译目录和 `PREPROCESSING.json` 已生成，reference 副本符合现有转换器结构 |
| 资产安装 | manifest、adapter、reference、RKNN 和转换记录匹配；安装清单校验通过 |
| 构建 | colcon 成功，package prefix 指向本工作空间，动态库无 `not found` |
| HPM 通信 | 正式 ROS 环境一致，真实 motor state 与 IMU 到达，motors_ready 为 true |
| 策略启动 | AMP 与 WBC 均保持运行，prepare 服务存在，共享模型与 reference 加载无异常 |
| Walking | 状态切换成功，AMP 持续输出；不能只凭 WALK 横幅判定控制链正常 |
| 动作执行 | 每条请求对应 started/completed，动作完成后恢复 AMP；无未解释的 rejected/failed/超时 |
| 录包 | 所选话题与消息类型正确，实际动作对应的消息存在，无未处理的 QoS/磁盘错误 |
| 停机 | disable 成功且真实电机状态符合现场停机要求，无未处理的退出异常 |

- **prepare service unavailable / worker did not respond**：先查看正式 sim2real 启动输出中 WBC 是否初始化失败或退出，再按硬件侧原有流程检查通信；反复按 B+Y 不会恢复死亡的 worker。
- **Unsupported resident WBC layout，FP16/AFFINE**：检查板端是否构建并 source 了包含 FP16/AFFINE 兼容修复的部署源码，核对安装前缀和实际 worker；不要删除张量形状、布局或类型校验来绕过错误。
- **Policy command expired or unavailable**：检查 AMP 进程、观测和策略命令端点、命令时间戳/有效期及调度；持续出现时不继续动作。
- **节点/话题看不到**：先检查是否曾加载旧版配置中的 ROS 环境覆盖；将模型配置保持为第 5.1 节的四项，在新终端使用硬件侧原有正常通信环境。编辑文件不能修改已启动进程的环境；不要自行改电机、HPM 或硬件参数来迁就模型加载。
- **录包零消息**：核对是否已进入对应工作状态，以及 `ros2 topic info --verbose` 中发布者和 QoS；不要把录制可靠性改为 reliable 去订阅 best_effort 发布者。
- **Failed to set realtime priority**：保留警告并核查板端已有实时调度权限配置，性能结论应标明此条件；不要通过降低安全限制掩盖延迟问题。

### 本版部署的验证范围

部署端按约定不进行 ONNX/RKNN 数值对照、不计算训练端与板端输出残差，也不采集逐张量数据作为数值验收任务。训练端负责最终 FP16 交付资产及其精度；板端保留文件/哈希/ABI 检查和加载、动作、控制时序、故障与停机检查。`verification: null` / `not_run` 原样保存，不伪造数值通过状态。

A～D 组 rosbag 是可选的运控记录和问题排查资料，不是本流程的数值精度验证步骤。此前 `20261027_103346_3Rz7nA` 等隔离日志来自旧资产，既不能证明新 FP16 包已运行，也不用于补算本版数值误差。
