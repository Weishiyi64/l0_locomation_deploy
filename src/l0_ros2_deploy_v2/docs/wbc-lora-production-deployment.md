# RK3588 WBC-LoRA 正式部署操作手册

更新：2026-10-28（末尾新增本机 FP16 与 80 动作资产复查）。对象：L0 V5、RK3588 Linux/aarch64、ROS2 Humble、常驻参数 WBC-LoRA。本文独立给出训练交付与文件前处理、源码与动作复制、RKNN 编译、安装与构建、真实 HPM 接入、逐节点启动、动作触发、可选分组录包及停机回收流程。

运行资产直接放在工作空间内，不依赖 `deliveries`、交付包日志指针或隔离启动入口。WBC 从 ROS 安装目录加载；当前 AMP 的模型路径仍指向编译时的源码策略目录，必须保留该目录。本文是操作流程，不代表真实机器人闭环或数值精度已验收。

## 0. 最终交付约定与文件前处理

**最终版本：训练端统一完成 Base 模型、LoRA 参数和 reference motion 的 FP16 转换后交付；ONNX 对外输入/输出接口保留 FP32。部署端不执行数值精度验证，不承担训练参数的 FP16 导出工作，只做资产完整性/接口检查、RKNN 编译、安装、构建和运行检查。** 本节取代此前 reference ONNX 保留 FP32 的交付约定。

本版采用新目录名 `wbc_lora_fp16`，区别于此前的 `wbc_lora_resident` 隔离测试包；后文命令统一使用新目录名。旧包不会因为改名而变成符合本规范的新包。若新目录也已存在，使用新的版本目录名，并一致修改各步骤中的包名。

### 0.1 精度与接口规范

| 交付内容 | 参数/内部数据精度 | 对外接口及约束 |
| --- | --- | --- |
| 共享 `base_lora.onnx` | Base 固定权重采用 FP16；图内通过必要的 Cast 衔接 FP32 接口 | `policy[399]`、`command_short[240]`、31 个动态参数输入及 `action[21]` 输出均保持 FP32，名称、顺序和形状不变 |
| 各动作 `weights.bin` | FP16 little-endian；覆盖 A/B、完整 bias、LayerNorm gamma/beta，共 31 个参数张量 | manifest 的 `weights.dtype` 为 `float16-le`，各张量偏移按现有 64 字节对齐契约生成；加载器按既有方式解包并写入 RKNN 原生张量 |
| 各动作 `reference_motion.onnx` | **轨迹表等浮点数据统一 FP16**；不再要求保留 FP32 轨迹表 | 输入 `time_step[1,1]` 与六个输出仍为 FP32，保留关节/机体顺序、50 Hz、帧数、wxyz 四元数和边界取帧语义 |
| manifest / `rl_config.yaml` | 保持 JSON/YAML 元数据形式，不对文件做“半精度转换” | 保留真实参数与接口描述；哈希必须对应最终交付字节 |
| 板端 `.rknn` | 按现有 RK3588、FP16、非量化目标编译 | 不启用 Python Runtime 数值对照，统一使用 `--build-only` |

“统一 FP16”指模型参数、LoRA 参数和 reference 浮点轨迹数据的交付精度，不是把 shape、索引或全部 ONNX 输入输出改成 FP16。图中的整数类型及索引保持原类型。`application_dtype` 和图的输入输出 `dtype` 仍为 `float32`，`target_precision` 为 `float16`；内部精度和应用接口精度是不同字段。

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
    ├── checkpoint_2000.pt                # 同一训练节点的完整 checkpoint
    └── motion/
        ├── reference_motion.onnx        # 用于训练侧前处理的原生 reference
        └── <动作ID>.npz                 # 建议归档的原始参考轨迹
```

`2000` 仅为示例保存轮次，索引应填写实际文件名。`emotion_index.json` 为数组，至少包含每个动作的 `motion`、`adapter`、`adapter_sha256`、`checkpoint`、`checkpoint_sha256`、`motion_onnx`；路径相对包根目录。Base 三个文件的路径为当前导出器固定读取路径。

原始目录的索引可用仓库脚本 `generate_wbc_emotion_index.py` 自动生成。它只读目录及文件字节，不加载 `.pt`，只依赖 Python 3.10+ 标准库；默认扫描全部 `lora/<分类>/<动作>/`，按文件名轮次选择最新一对 `lorackpt_N.pt` / `checkpoint_N.pt`。最新轮次不完整时直接报错，使用 `--step` 才能明确指定其他轮次。它检查 reference 文件存在并计算哈希，不判断模型训练质量。

在训练结果所在机器的新终端先预览三个动作，不写文件：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
training_root='/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果'
python3 -B "$local_ws/src/l0_ros2_deploy_v2/generate_wbc_emotion_index.py" \
  "$training_root" --actions L0_Hugme L0_act_cute_1 L0_act_cute_2 --dry-run
)
```

实际写入自定义文件名 `emotion.json` 的示例；不加 `--actions` 表示扫描全部动作。目标已存在会拒绝覆盖：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
training_root='/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果'
python3 -B "$local_ws/src/l0_ros2_deploy_v2/generate_wbc_emotion_index.py" \
  "$training_root" --step 2000 --output emotion.json
)
```

相对 `--output` 路径以训练根目录为基准。现有导出器固定读取 `emotion_index.json`，新包首次生成时省略 `--output` 即使用该名称；自定义 `emotion.json` 不会自动替代已有索引。输出记录保存轮次、动作分类、文件相对路径和实际哈希，不从文件名推断 rank、alpha 或精度。这个脚本用于训练原始目录，板端最终 FP16 交付包仍使用 `base_manifest.json`，不需要训练索引。

现有导出器需要完整 checkpoint 来核对 Base+LoRA 重建结果，导出前不能省略。原始 `.pt` 不提前转换或重新保存为 FP16，以免改变 Base 身份哈希并破坏原有匹配；**FP16 转换作用于导出副本和最终交付参数包。** 如果原始输入只有 `.pt` / `.npz`，训练端先通过训练项目的导出流程补齐原生 Base ONNX/YAML 和 reference ONNX。

### 0.3 训练端文件前处理顺序

1. **确定动作与来源。** 选择同一 Base、相同关节/观测契约的动作，准备上述原始输入及索引。正式训练动作使用各自 LoRA；不使用 `--base-actions` 把缺失或异常的 LoRA 替换成 Base。
2. **生成动态共享图及动作资产。** 在训练/导出环境中使用 `prepare_wbc_deploy.py <原始包> <新的中间目录> --mode lora --actions <动作ID...> --samples 33`。此阶段生成共享图、adapter manifest/weights、动作配置和 reference 副本。当前脚本自带的 CPU 重建/样本检查属于训练端导出过程，不在部署端重跑。
3. **完成 Base 和 reference ONNX 的 FP16 前处理。** 训练端对中间目录的 `base_lora.onnx` 固定权重和每个 `reference_motion.onnx` 的浮点轨迹数据进行 FP16 转换，保留 FP32 输入输出，通过图内 Cast 衔接。不得先把动作 LoRA 合并进共享 Base；31 个参数仍是动态输入，不能通过常量折叠固化为某个动作。
4. **确认 LoRA 参数包为 FP16。** 现有导出器已将 31 个张量打包成 FP16 `weights.bin`，通常不需要二次转换。按 manifest 校核名称、形状、偏移和长度，不将 `.pt` 直接改名为 `.bin`，不遗漏 bias 或 LayerNorm 参数。
5. **保留控制语义与 reference 元数据。** 动作 `rl_config.yaml` 由 Base `policy.yaml` 与对应 reference 元数据生成；不能用训练 YAML 改名替代。现有导出器不按每个动作的训练配置自动生成不同 PD/action scale，同包动作需符合共同部署契约。reference 转换只改变浮点精度，不改变取帧、坐标、关节顺序或帧率。
6. **最后更新清单和哈希。** 完成所有精度转换后，重新计算共享模型的 `model.sha256`、各动作的 `reference_onnx_sha256`，以及确有字节变化的配置/权重哈希。`base_checkpoint_sha256`、`source_onnx_sha256` 等原始来源身份继续指向原始输入，不用最终共享图哈希替代。输入输出 ABI 未改变时，不把 schema 的 FP32 接口声明改成 FP16。旧 `.rknn`、旧转换记录和旧安装清单不进入新交付包。
7. **封装最终包。** 将精度转换后的完整包交给部署端。训练端保存原始输入、导出工具版本、转换过程和已有验证结论；转换前的验证报告不能当作转换后模型已验证的证明。最终模型的精度责任留在训练/交付端，板端流程不追加数值对照任务。

现有 `export_wbc_lora.py` 已实现 LoRA 参数 FP16 打包，**它生成的共享 ONNX 不能仅凭这一点视作 Base/reference 都已完成 FP16 前处理**。2026-10-28 起，当前 `export_wbc_lora.py` 在导出完成前自动将 Base/probe 固定参数与全部 reference 表保存为 FP16、保留 FP32 ABI，并再次验证最终 FP16 图；无需附加 `--fp16` 参数。第 3 步仍属于训练/导出侧工作。板端新增的 `preprocess_wbc_lora_fp16.py` 接收已经完成训练端导出的 FP16 包，不代替此处的训练端精度转换。

### 0.4 训练端向部署端交付的最终目录

```text
wbc_lora_fp16/
├── base_manifest.json                    # 最终共享图与动作目录/哈希
├── base_lora.onnx                        # Base 权重 FP16，33 个输入及输出接口 FP32
├── adapters/
│   ├── L0_Hugme/
│   │   ├── manifest.json
│   │   └── weights.bin                  # FP16，包含完整 31 张量
│   ├── L0_act_cute_1/{manifest.json,weights.bin}
│   └── L0_act_cute_2/{manifest.json,weights.bin}
└── actions/
    ├── L0_Hugme/
    │   ├── rl_config.yaml
    │   └── reference_motion.onnx        # 浮点轨迹数据 FP16，输入输出接口 FP32
    ├── L0_act_cute_1/{rl_config.yaml,reference_motion.onnx}
    └── L0_act_cute_2/{rl_config.yaml,reference_motion.onnx}
```

大括号表示目录内的独立文件。上述文件均由训练端前处理完成后随包交付。部署端保留原包，并由前处理脚本生成独立编译副本及其清单；不重新推导训练参数或训练来源。三个示例动作在此包中均为训练 LoRA，`L0_act_cute_2` 不再沿用旧包的 Base 对照含义。probe、NPZ 验证样本和训练报告可保留在训练归档；本版板端流程不依赖它们，也不执行 `--include-probes`。

| 文件 | 负责生成的一端 | 到板端后是否需要生成 |
| --- | --- | --- |
| 完整原始 FP16 交付包 | 训练/导出端，完成 FP16 前处理和最终哈希更新 | 不需要，保存到 `model_inbox/<包名>` |
| 编译工作包及 `PREPROCESSING.json` | `preprocess_wbc_lora_fp16.py` | 需要，生成到 `model_sources/<包名>`，仅规整 reference 表/输出分支并更新编译副本清单 |
| `base_lora.rknn`、各动作 `reference_motion.rknn` | RK3588 部署端 | 需要，批量或单模型编译 |
| 各 `*.rknn.json` | 对应 RKNN 转换器 | 需要，随实际编译自动生成 |
| `INSTALLATION.json` | `prepare_wbc_deploy.py --mode install-lora` | 需要，安装完整资产后生成 |
| ROS `install/` | 板端 colcon | 需要，从板端源码构建 |

### 0.5 部署端前处理与检查范围

收到包后使用独立前处理脚本完成文件完整性、哈希、模型结构、张量名称/形状/类型、动作 ID、关节顺序与 reference 元数据检查，并生成供编译器使用的 reference 副本，再执行 RKNN 编译和安装。ONNX checker、哈希与有限值检查属于格式及完整性检查，仍然保留；它们不等于数值精度验证。

板端转换命令全部使用 `--build-only`，不附加 `--target`、`--device-id` 或数值验证样本，不调用 `verify_wbc_lora_rknn.py`，不通过移除 `--build-only` 运行 Python Runtime 对照。`verification_backend: not_run`、`verification: null` 是本流程的预期状态，无需补成 `passed`。`max_abs_error` 即使仍存在于工具记录，也只是默认阈值，不是实测结果；`converted_pending_runtime` 对应后续加载、动作和控制链运行检查。

### 0.6 独立前处理脚本与完整流程

`preprocess_wbc_lora_fp16.py <原始交付目录> <新编译目录>` 集中处理部署前的适配工作，支持 `--dry-run` 只检查不写文件。原始交付目录只读，输出必须是不存在的、与输入互不嵌套的新目录；发生错误时不发布不完整的输出包。

脚本逐字节保留共享 FP16 policy、LoRA 参数及动作配置。对 reference，只接受原生六路 Gather 轨迹图，以及轨迹表/输出之间的 FP16/FP32 Cast 或 Identity 包装；将已经是 FP16 的轨迹值精确展开成 FP32 编译表，恢复现有编译器要求的六个直接 Gather 输出，保留原来的帧索引计算子图和元数据。**这是编译副本的表示适配，不是恢复训练时 FP32 数值，也不改变原包的 FP16 交付约定。** 后续 RKNN 仍按 FP16、非量化编译。

不支持的图变换、融合结构、外部 ONNX 权重、缺失文件、哈希不符或接口不匹配会报错。脚本不会为错误的训练交付重算哈希来绕过检查，也不会读取 `.pt`、生成训练权重、调用 ONNX Runtime/RKNN 推理或执行数值验证。已有 policy/reference 转换器和 C++ 加载器保持不变。

编译副本中的 `reference_onnx_sha256` 对应规整后的 reference；原包哈希、各 reference 的输入/输出哈希、工具版本和处理方式保存于 `PREPROCESSING.json`。历史 RKNN、转换记录、probe 和数值验证样本不复制到新工作包；共享图中的诊断项也相应移除。该记录保留在编译目录和运行日志归档中，现有安装器不把它当作运行必需资产。

完整顺序为：`训练 FP16 包 → model_inbox → 前处理脚本 → model_sources → --build-only RKNN 编译 → install-lora → colcon build/source → ROS 节点运行检查`。第 3.0 节给出可直接执行的前处理命令，随后按第 3.1、4.1、4.2 节继续。这个流程已接入现有工具，但尚未以新的训练 FP16 实包完成 RK3588 编译和运行验收。

## 1. 工作位置与 Remote VS Code

| 项目 | 本文约定 |
| --- | --- |
| 本地主机工作空间 | `/home/luvbot/SC/l0_deploy_v2_ws` |
| 板端连接 | `forlinx@192.168.5.2`，不同地址请修改同步命令 |
| 板端工作空间 | `/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws` |
| 部署源码 | `<工作空间>/src/l0_ros2_deploy_v2` |
| 匹配接口源码 | `<工作空间>/src/luvbot_ros_interfaces` |
| 训练原始 FP16 交付包 | `<工作空间>/model_inbox/wbc_lora_fp16` |
| 前处理后的编译工作包 | `<工作空间>/model_sources/wbc_lora_fp16` |
| 安装策略目录名 | `wbc_lora_fp16` |
| AMP 策略 | `l0_v5_amp_07090` |
| Toolkit2 Python | 板端 `/usr/bin/python3`，已使用版本 `rknn-toolkit2==2.3.2` |
| ROS underlay | `/opt/ros/humble/setup.bash` |
| 真实 HPM 启动 | `bash "$HOME/hpm_start.sh"`，由机器人侧提供 |

在本地主机安装 VS Code 的 Remote - SSH 扩展，执行 **Remote-SSH: Connect to Host...**，连接 `forlinx@192.168.5.2`，打开板端工作空间目录。确认左下角显示该 SSH 主机；通过 Terminal → New Terminal 新建的终端才在板端执行。构建在工作空间根目录进行。普通本地窗口中的终端仍在本地主机运行。[VS Code 官方 Remote SSH 说明](https://code.visualstudio.com/docs/remote/ssh)

下文明确标注“本地主机”的块才在本地主机执行，其他块均在板端执行。普通 SSH 也可使用：

```bash
ssh forlinx@192.168.5.2
```

每个代码块都重新设置必要路径；每个 ROS 块都重新 source underlay、工作空间和正式运行配置。**新终端不能继承另一个终端的 shell 变量。请复制完整块。** 外层 `( ... )` 为子 shell，变量不保留到下一个块。不要在 source ROS 前启用 `set -u`。

已有板端工作空间须具备 ROS2/colcon、匹配的 `luvbot_ros_interfaces` 与 `robot_interfaces`、`joy`、机器人描述、RKNN 头文件/运行库及仓库声明的构建依赖。HPM 的电机和 IMU 传输由其独立工作空间负责。本手册不替代操作系统、HPM、关节标定和驱动的首次安装。

## 2. 动作放在哪里，复制哪些文件

### 2.1 完整 LoRA 动作包的结构

将训练端已完成第 0 节 FP16 前处理的**整个交付包根目录**放入 `model_inbox/wbc_lora_fp16`，再由独立脚本生成 `model_sources/wbc_lora_fp16` 编译工作包。此处展示前处理、板端 RKNN 编译与安装后的目录；`.rknn`、`.rknn.json` 和 `INSTALLATION.json` 在上传时尚不存在，属于后续生成文件。

Base 固定权重、LoRA 参数及 reference 浮点轨迹数据均由训练端以 FP16 交付，ONNX 输入输出接口保持 FP32。板端不接收原始 checkpoint 来重新导出；第 3.0 节只在编译副本中规整 reference，第 3.1 节生成 RKNN，第 4 节安装资产并构建 ROS。

```text
l0_deploy_v2_ws/
├── model_inbox/
│   └── wbc_lora_fp16/                    # 训练原始 FP16 交付包，只读保留
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
│               ├── base_lora.onnx
│               ├── base_lora.rknn
│               ├── base_lora.rknn.json
│               ├── adapters/...
│               └── actions/...
├── install/legged_deploy/share/legged_deploy/policy/l0_v5/
│   ├── l0_v5_amp_07090/                    # 安装的 AMP 配置；当前 AMP 模型仍从源码目录加载
│   └── wbc_lora_fp16/                  # 正式运行读取这里
├── deploy_config/                         # 本机 ROS domain、动作选择、录包 QoS
└── runs/wbc_production/                    # 每次运行独立日志与 bags
```

合并安装工作空间的 share 路径可能不同，应以 `ros2 pkg prefix legged_deploy` 为准。验证样本、probe 和训练原始结果由训练端归档，正式部署不要求将它们放入源包或安装目录。

LoRA 动态加载指切换参数值：共享 policy 有 2 个观测输入和 31 个参数输入，形状固定。`weights.bin` 不转 RKNN；所有动作复用一份 `base_lora.rknn`，每个动作的 reference 单独转换。不要从其他导出批次单拷一个 adapter、手改 `rl_config.yaml` 或拼接 manifest；这些文件受哈希和 schema 校验约束。

### 2.2 本地主机：同步源码

Remote VS Code 直接修改板端源码时可跳过此步。通过本地主机同步时，先停止本次部署进程并确认不会覆盖板端独有修改。以下用于同步完整部署仓库和匹配的接口仓库，保留板端现有策略目录；不传本地 `build/install/log`，不使用删除选项。先执行预览：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
test -d "$local_ws/src/luvbot_ros_interfaces"
ssh "$robot_target" "git -C '$robot_ws/src/l0_ros2_deploy_v2' status --short"
rsync -rtlvn --itemize-changes --exclude='.git' --exclude='__pycache__/' \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' \
  --exclude='/legged_deploy/policy/' \
  "$local_ws/src/l0_ros2_deploy_v2/" "$robot_target:$robot_ws/src/l0_ros2_deploy_v2/"
rsync -rtlvn --itemize-changes --exclude='.git' --exclude='__pycache__/' \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' \
  "$local_ws/src/luvbot_ros_interfaces/" "$robot_target:$robot_ws/src/luvbot_ros_interfaces/"
)
```

预览确认后，在本地主机执行实际同步：

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
test -d "$local_ws/src/luvbot_ros_interfaces"
ssh "$robot_target" "git -C '$robot_ws/src/l0_ros2_deploy_v2' status --short"
rsync -rtlv --itemize-changes --exclude='.git' --exclude='__pycache__/' \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' \
  --exclude='/legged_deploy/policy/' \
  "$local_ws/src/l0_ros2_deploy_v2/" "$robot_target:$robot_ws/src/l0_ros2_deploy_v2/"
rsync -rtlv --itemize-changes --exclude='.git' --exclude='__pycache__/' \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' \
  "$local_ws/src/luvbot_ros_interfaces/" "$robot_target:$robot_ws/src/luvbot_ros_interfaces/"
)
```

这里要求板端已有部署仓库；全新系统请先备齐第 1 节依赖与工作空间。rsync 会更新同名文件，但不会移除板端额外文件。板端有独立修改或已停用的旧源码时，先通过版本管理审查、合并，避免混用不同版本。

### 2.3 本地主机：上传完整动作源包

先将训练端交付的完整 FP16 包放到本地主机的 `model_inbox/wbc_lora_fp16`，或将下面的 `local_assets` 改成实际导出包根目录。板端目标目录要求新建，更新时改用版本目录名，如 `wbc_lora_20261028`；同时修改后续编译、安装与运行配置中的目录名。不要覆盖正在运行的资产。

```bash
(
set -e -o pipefail
local_ws=/home/luvbot/SC/l0_deploy_v2_ws
robot_target=forlinx@192.168.5.2
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
local_assets="$local_ws/model_inbox/wbc_lora_fp16"
remote_assets="$robot_ws/model_inbox/wbc_lora_fp16"
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

## 3. 板端：RKNN 模型编译

**执行顺序：先运行第 3.0 节前处理，再编译其生成的 `model_sources` 工作包。不要直接用 `model_inbox` 中的原始 FP16 reference 调用旧转换器。**

这些命令在 RK3588 板端执行，使用 Toolkit2 环境，与 ROS C++ 构建分开。关闭正在运行的部署任务后转换；不在实时控制期间进行模型编译。当前转换器固定目标 `rk3588`、FP16、非量化；`--build-only` 仅编译，不初始化 Python NPU Runtime，不与 `--target` / `--device-id` 同用。

先确认工具链：

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
test "$(uname -m)" = aarch64
tr '\0' '\n' < /proc/device-tree/compatible
/usr/bin/python3 -c 'import importlib.metadata as m; from rknn.api import RKNN; print("rknn-toolkit2=" + m.version("rknn-toolkit2"))'
/usr/bin/python3 -c 'import onnx, numpy, yaml; print("ONNX / NumPy / YAML imports OK")'
test -f "$robot_ws/model_inbox/wbc_lora_fp16/base_manifest.json"
)
```

设备树应包含 `rk3588`，当前已用 Toolkit2 为 2.3.2；不要把 Toolkit Lite2 当成支持编译的 Toolkit2。板端 C++ Runtime 也需与模型兼容。命令失败时先解决环境，不在其他 Python 环境中盲目重试。

### 3.0 板端：生成独立编译工作包

此脚本只需要 ONNX、NumPy、PyYAML，不需要 PyTorch、ONNX Runtime 或 RKNN Runtime。源目录应是训练端实际交付的 FP16 包；不能拿旧 FP32 模型包改名代替。输出目录不能提前创建；需要预览时，给 Python 命令追加 `--dry-run`，不产生任何输出文件，也不继续执行后续编译步骤。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_inbox/wbc_lora_fp16"
prepared_bundle="$robot_ws/model_sources/wbc_lora_fp16"
/usr/bin/python3 -B "$robot_repo/preprocess_wbc_lora_fp16.py" \
  "$source_bundle" "$prepared_bundle"
)
```

完成后继续第 3.1 节。前处理成功而 RKNN 编译失败时，修复编译问题后只重跑编译；不重复对同一输出目录执行前处理。更换训练资产或前处理脚本时使用新的工作目录，保留旧记录。

### 3.1 推荐：一个包内所有动作批量编译

下面依次编译一份共享 policy 和 manifest 中每个动作的 reference。当前三动作包共生成四个 RKNN。逐模型日志、退出码和 `conversion_summary.json` 保存在新目录中。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source_bundle="$robot_ws/model_sources/wbc_lora_fp16"
mkdir -p "$robot_ws/build_logs/wbc_models"
conversion_log="$robot_ws/build_logs/wbc_models/$(date +%Y%m%d_%H%M%S_%N)"
/usr/bin/python3 -B "$robot_repo/batch_convert_onnx_to_rknn_rk3588.py" \
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
/usr/bin/python3 -B "$robot_repo/convert_onnx_to_rknn_rk3588_wbc_policy.py" \
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
/usr/bin/python3 -B "$robot_repo/convert_onnx_to_rknn_rk3588_wbc_reference.py" \
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
  /usr/bin/python3 -B "$robot_repo/preprocess_wbc_lora_fp16.py" \
    "$training_bundle" "$source_bundle"
  /usr/bin/python3 -B "$robot_repo/batch_convert_onnx_to_rknn_rk3588.py" \
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
/usr/bin/python3 -B "$robot_repo/batch_convert_onnx_to_rknn_rk3588.py" \
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
/usr/bin/python3 -B "$robot_repo/prepare_wbc_deploy.py" \
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

## 5. 板端：首次配置正式运行环境

### 5.1 保存与 HPM 一致的 ROS 设置

已确认 HPM 入口为 `bash "$HOME/hpm_start.sh"`。其 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 尚未提供，**不能沿用隔离验证的 domain 97**。先在 Remote VS Code 中查看该脚本及它 source 的配置，确认 HPM 实际使用值；若脚本和环境均未设置，才按该环境默认值确认。外部脑控或其他计算机需要接入时，`ROS_LOCALHOST_ONLY=1` 会限制跨主机发现。

下面交互式填写已确认的值，一次生成配置。`RMW_IMPLEMENTATION` 留空表示保持系统默认，只有 HPM 也使用该默认时才留空。若 HPM 还有自定义 DDS XML/profile，其他终端也应在此配置中设置相同的必要环境。配置文件是 shell 文件，只保存可信内容。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
mkdir -p "$robot_ws/deploy_config"
config_file="$robot_ws/deploy_config/wbc-production.env"
if test -e "$config_file"; then
  printf '配置已存在，请在 Remote VS Code 中检查/修改：%s\n' "$config_file"
  exit 0
fi
read -r -p 'HPM 实际 ROS_DOMAIN_ID（整数 0..232）: ' hpm_domain
[[ "$hpm_domain" =~ ^[0-9]{1,3}$ ]] && (( 10#$hpm_domain <= 232 ))
read -r -p 'HPM 实际 ROS_LOCALHOST_ONLY（0 或 1）: ' hpm_localhost
[[ "$hpm_localhost" == 0 || "$hpm_localhost" == 1 ]]
read -r -p 'HPM 的 RMW_IMPLEMENTATION（系统默认则留空）: ' hpm_rmw
[[ -z "$hpm_rmw" || "$hpm_rmw" =~ ^rmw_[a-zA-Z0-9_]+$ ]]
(
  set -C
  {
    printf 'export ROS_DOMAIN_ID=%q\n' "$((10#$hpm_domain))"
    printf 'export ROS_LOCALHOST_ONLY=%q\n' "$hpm_localhost"
    if test -n "$hpm_rmw"; then
      printf 'export RMW_IMPLEMENTATION=%q\n' "$hpm_rmw"
    else
      printf 'unset RMW_IMPLEMENTATION\n'
    fi
    printf 'robot_type=%q\n' l0_v5
    printf 'amp_policy=%q\n' l0_v5_amp_07090
    printf 'wbc_bundle=%q\n' wbc_lora_fp16
    printf 'wbc_actions=%q\n' '[L0_Hugme, L0_act_cute_1, L0_act_cute_2]'
  } > "$config_file"
)
cat "$config_file"
)
```

示例配置选择 `L0_Hugme`、`L0_act_cute_1`、`L0_act_cute_2` 三个训练 LoRA 动作；均须存在于本次训练交付的 manifest。安装包可以包含更多动作，`wbc_actions` 只选择本次 worker 预加载、可请求的动作。新 FP16 包与旧隔离测试包不同，旧运行记录不表示新包已完成板端运行检查。

正式切换到其他动作包时修改 `wbc_bundle` 和 `wbc_actions`，再重启受影响的部署进程。它们不是 ROS 动态参数；修改配置不会改变已启动进程。环境中的 ROS domain 也不能在进程运行中通过修改文件切换。

### 5.2 每次运行创建独立记录目录

确认上一轮进程已停止后执行一次。其他终端将读取本次目录指针，不自行创建第二个 run。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source /opt/ros/humble/setup.bash
source "$robot_ws/install/setup.bash"
source "$robot_ws/deploy_config/wbc-production.env"
deploy_prefix=$(ros2 pkg prefix legged_deploy)
[[ "$deploy_prefix" == "$robot_ws/install" || "$deploy_prefix" == "$robot_ws/install/"* ]]
installed_bundle="$deploy_prefix/share/legged_deploy/policy/$robot_type/$wbc_bundle"
test -f "$installed_bundle/INSTALLATION.json"
test -f "$installed_bundle/base_lora.rknn"
amp_source="$robot_repo/legged_deploy/policy/$robot_type/$amp_policy"
amp_installed="$deploy_prefix/share/legged_deploy/policy/$robot_type/$amp_policy"
test -f "$amp_source/policy.rknn"
cmp "$amp_source/rl_config.yaml" "$amp_installed/rl_config.yaml"
cmp "$amp_source/joy_config.yaml" "$amp_installed/joy_config.yaml"
run_root="$robot_ws/runs/wbc_production"
mkdir -p "$run_root"
run_dir=$(mktemp -d "$run_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
mkdir -p "$run_dir/ros" "$run_dir/bags"
cp "$robot_ws/deploy_config/wbc-production.env" "$run_dir/environment.env"
printf '%s\n' "$installed_bundle" > "$run_dir/installed_bundle.txt"
cp "$installed_bundle/INSTALLATION.json" "$run_dir/INSTALLATION.json"
cp "$installed_bundle/base_manifest.json" "$run_dir/base_manifest.json"
cp "$robot_ws/model_sources/$wbc_bundle/PREPROCESSING.json" "$run_dir/PREPROCESSING.json"
/usr/bin/python3 - "$installed_bundle" "$run_dir/models.sha256" <<'PYCODE'
import hashlib, json, pathlib, sys
root, dest = map(pathlib.Path, sys.argv[1:])
files = json.loads((root / 'INSTALLATION.json').read_text())['files']
lines = []
for relative, expected in sorted(files.items()):
    actual = hashlib.sha256((root / relative).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit('Installed asset checksum mismatch: ' + relative)
    lines.append(f'{actual}  {relative}\n')
dest.write_text(''.join(lines))
PYCODE
{
  git -C "$robot_repo" rev-parse HEAD
  git -C "$robot_repo" status --short
  uname -a
  printf 'ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s RMW=%s\n' \
    "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "${RMW_IMPLEMENTATION:-system_default}"
} > "$run_dir/version.txt"
git -C "$robot_repo" diff HEAD --binary > "$run_dir/source_changes.patch"
sha256sum "$deploy_prefix/lib/legged_deploy/rl_controller" > "$run_dir/worker.sha256"
sha256sum "$amp_source/policy.rknn" "$amp_installed/rl_config.yaml" \
  "$amp_installed/joy_config.yaml" > "$run_dir/amp.sha256"
printf '%s\n' "$run_dir" > "$run_root/current_run_dir"
printf '本次记录目录：%s\n' "$run_dir"
)
```

`version.txt` 与 diff 不包含未跟踪文件的完整内容，应另保留本次构建对应的源码版本或源码归档。WBC 模型身份由安装清单与哈希记录确定，AMP 模型与配置另记入 `amp.sha256`。

## 6. 板端：逐个启动节点

每行角色使用一个独立终端，按表中顺序启动。表中的 launch 仅启动该角色；不要同时启动 `sim2real.launch.py`、`one_start.launch.py` 或此前的隔离链路，否则会重复启动硬件桥、策略或手柄。

| 终端 | 启动对象 | ROS 角色 |
| --- | --- | --- |
| 1 | HPM 脚本 | 真实电机/IMU 传输，节点名由 HPM 决定 |
| 2 | `legged_real.launch.py` | `/topic_hardware_adapter`；内含 `LeggedController`、状态机、策略选择与安全检查 |
| 检查终端 | 硬件就绪检查 | 检查 IMU 和电机 ready，不是常驻节点 |
| 3 | `rl_controller.launch.py` | `/rl_controller`，AMP 行走策略 |
| 4 | `rl_controller` + WBC 参数 | `/rl_controller_wbc`，共享 LoRA policy 与动作 reference |
| 5 | `joy_teleop.launch.py` | `/joy_node_robot`，物理手柄输入 |
| 可选终端 | rosbag recorder | 每组选项独立一个 bag 目录，见第 7 节 |

`LeggedController` 是硬件桥内部的控制组件，不另起一个同名节点。`system_command_arbiter` 在本链路提供关节映射配置，不需要再启动一个命令仲裁节点。

常驻命令中的 tee 子进程忽略 Ctrl+C，控制进程仍收到 SIGINT 并正常清理；tee 会在日志管道关闭后退出，以保留停机阶段的输出。每个角色仅启动一次，重启时创建新的运行记录，避免覆盖旧日志。

### 6.1 终端 1：真实 HPM

若机器人上 HPM 已由服务或现有终端运行，确认其环境和话题后使用现有实例，不重复启动。未启动时执行：

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
test -f "$HOME/hpm_start.sh"
bash "$HOME/hpm_start.sh" 2>&1 | (trap '' INT; exec tee "$run_dir/hpm.log")
)
```

脚本是否前台常驻、是否自行启动后台进程，以板端脚本为准；退出 SSH 或关闭 VS Code 窗口不能代替确认 HPM 已停止。运行模式是 `real`；后续硬件桥启动会发送电机使能，必须按真实机器人现场操作条件进行。

### 6.2 终端 2：硬件桥与运控状态机

以下显式传入底层 launch 的必填参数，并沿用当前上层启动链的姿态阈值、阻尼与辅助功能设置。

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
controllers_share="$(ros2 pkg prefix legged_controllers)/share/legged_controllers"
ros2 launch legged_real legged_real.launch.py \
  robot_type:="$robot_type" subpolicy:="$amp_policy" hpm_mode:=real \
  rviz:=false imu_enabled:=true wbc_enabled:=true \
  head_script_enabled:=false \
  head_script_config:="$controllers_share/pcmd_yaml/l0_keepalone.yaml" \
  joystick_imitation_enabled:=false joystick_sequence_enabled:=false \
  joystick_back_emergency_sequence_enabled:=false radar_control_enabled:=false \
  arm_swing_enabled:=false arm_swing_stop_vx_threshold:=0.1 \
  arm_swing_high_vx_threshold:=0.8 arm_swing_vx_hysteresis:=0.03 arm_swing_blend_sec:=0.5 \
  emergency_damping_kd:=0.2 \
  safety_orientation_roll_limit_deg:=30.0 safety_orientation_pitch_limit_deg:=30.0 \
  safety_orientation_consecutive_violation_count:=10 \
  2>&1 | (trap '' INT; exec tee "$run_dir/hardware_bridge.log")
)
```

### 6.3 检查终端：等待真实反馈就绪

此处跳过电池检查，保持 IMU 检查；如已有电池发布链路并要检查，将第一个 `false` 改为 `true`。这不会启动电池节点。

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
ready_script="$(ros2 pkg prefix legged_controllers)/lib/legged_controllers/check_hardware_ready.bash"
bash "$ready_script" false true /imu/data 2>&1 | tee "$run_dir/readiness.log"
ros2 topic info /actuator/motor/state --verbose
ros2 topic info /imu/data --verbose
ros2 topic echo /hardware_status/motors_ready std_msgs/msg/Bool \
  --qos-reliability reliable --qos-durability transient_local --once
)
```

要求脚本成功，`motors_ready` 为 true，标准 `/actuator/motor/state` 和 `/imu/data` 有真实 HPM 发布者。`/wbc_isolated/...` 不是本流程的数据源。发现错误先处理，不能通过关闭 IMU 或 readiness 检查绕过。

### 6.4 终端 3：AMP 行走策略

启动后核对 `RKNN: Loaded RK3588 model from ...`，当前代码应显示编译时的源码策略路径。它与 WBC 的安装目录加载方式不同。

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
ros2 launch legged_deploy rl_controller.launch.py \
  robot_type:="$robot_type" subpolicy:="$amp_policy" policy_type:=rl \
  2>&1 | (trap '' INT; exec tee "$run_dir/amp.log")
)
```

### 6.5 终端 4：WBC-LoRA worker

使用与 `one_start` 相同的参数直接启动单个 worker；先从 manifest 中解析所选首动作的配置，不把策略 YAML 当成 ROS 参数文件传入。

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
deploy_share="$(ros2 pkg prefix legged_deploy)/share/legged_deploy"
policy_root="$deploy_share/policy"
installed_bundle="$policy_root/$robot_type/$wbc_bundle"
wbc_config=$(/usr/bin/python3 - "$installed_bundle" "$wbc_actions" <<'PYCODE'
import json, pathlib, sys, yaml
root = pathlib.Path(sys.argv[1]).resolve()
selected = yaml.safe_load(sys.argv[2])
catalog = {a['action_id']: a for a in json.loads((root / 'base_manifest.json').read_text())['actions']}
if not isinstance(selected, list) or not selected or any(a not in catalog for a in selected):
    raise SystemExit('wbc_actions must be a nonempty list of manifest action IDs')
config = (root / catalog[selected[0]]['config']).resolve()
if not config.is_relative_to(root) or not config.is_file():
    raise SystemExit('Invalid action config path')
print(config)
PYCODE
)
ros2 run legged_deploy rl_controller --ros-args -r __node:=rl_controller_wbc \
  -p "robot_type:=$robot_type" -p "robot_cfg_path:=$wbc_config" \
  -p policy_type:=rl_wbc -p wbc_mode:=lora \
  -p "wbc_lora_manifest:=$installed_bundle/base_manifest.json" \
  -p "subpolicy:=''" -p "policy_root_path:=$policy_root" \
  -p "available_wbc_policies:=$wbc_actions" \
  -p "rl_controller_config_path:=$deploy_share/config/rl_controller_config.yaml" \
  2>&1 | (trap '' INT; exec tee "$run_dir/wbc.log")
)
```

worker 应完成共享 RKNN 与所选 reference/adapter 加载、保持运行并提供 prepare 服务。初始化失败或进程退出时，后续动作一定会被拒绝；先检查 `wbc.log` 中第一处错误，不能把服务不存在归因于未进入 Walking。

### 6.6 终端 5：手柄

本链路只保留一个有效 `/joy` 发布者；如果 HPM 或既有进程已经提供物理手柄话题，检查后跳过此步骤。否则执行：

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
ros2 launch legged_controllers joy_teleop.launch.py \
  2>&1 | (trap '' INT; exec tee "$run_dir/joystick.log")
)
```

当前 launch 使用 deadzone 0.01、autorepeat_rate 10 Hz、coalesce_interval 0.05 s。查看日志确认打开的是现场目标手柄；不要向真实机器人额外注入合成 `/joy` 按键来代替物理操作。

### 6.7 检查节点、接口与参数

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
| A，策略输入输出，可选 | 硬件桥 + AMP + WBC | `/control/policy/observation`、`/control/policy/command/rl`、`/control/policy/command/wbc` | 检查观测、策略序号、有效期、目标关节位置与 AMP/WBC 交接 |
| B，电机与惯导，可选 | HPM + 硬件桥 | `/actuator/motor/state`、`/actuator/motor/command`、`/imu/data` | 实际电机反馈与下发命令、IMU、看门狗前后的数据 |
| C，动作与安全事件，可选 | 动作请求方 + 硬件桥 + WBC + 手柄 | `/algorithm/brain/wbc_action_cmd`、`/system/state_machine/wbc_action_fdb`、`/control/policy/wbc/active`、`/control/policy/wbc/done`、`/hardware_status/motors_ready`、`/system/safety/emergency_stop`、`/system/safety/hardware_fault`、`/actuator/motor/enable`、`/joy` | 请求/完成、控制权、急停、故障、使能与操作者输入 |
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

### 7.1 一次创建录包 QoS 配置

策略与传感器订阅用 best_effort/volatile，兼容其发布 QoS；`wbc/active` 和 `motors_ready` 用 reliable/transient_local 获取最近状态。安全话题可能有多个来源，录包采用 best_effort/volatile 兼容采集，需在动作前开始录制，不能指望补录启动前的安全事件。订阅 QoS 不会改变控制节点的发布设置。

下列 YAML 使用标准锚点复用配置；不是新增 ROS 话题。Humble 支持 `--qos-profile-overrides-path`，其字段及兼容规则见 [ROS2 官方 QoS 覆盖说明](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.rst)。

```bash
(
set -e -o pipefail
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
mkdir -p "$robot_ws/deploy_config"
cat > "$robot_ws/deploy_config/wbc-record-qos.yaml" <<'YAML'
/control/policy/observation: &sample
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/control/policy/command/rl: *sample
/control/policy/command/wbc: *sample
/actuator/motor/state: *sample
/imu/data: *sample
/odom: *sample
/actuator/motor/command: &event
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
/actuator/motor/enable: *event
/algorithm/brain/wbc_action_cmd: *event
/system/state_machine/wbc_action_fdb: *event
/control/policy/wbc/done: *event
/joy: *event
/control/policy/wbc/active: &latched
  history: keep_last
  depth: 10
  reliability: reliable
  durability: transient_local
/hardware_status/motors_ready: *latched
/system/safety/emergency_stop: *sample
/system/safety/hardware_fault: *sample
/rosout: *sample
YAML
cat "$robot_ws/deploy_config/wbc-record-qos.yaml"
)
```

### 7.2 A 组，可选：策略输入输出

在准备触发动作前的新终端执行并保持运行：

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
test -f "$robot_ws/deploy_config/wbc-record-qos.yaml"
mkdir -p "$run_dir/bags"
bag_path="$run_dir/bags/policy_$(date +%Y%m%d_%H%M%S_%N)"
ros2 bag record --storage sqlite3 --node-name record_wbc_policy \
  --output "$bag_path" --max-cache-size 16777216 \
  --max-bag-size 268435456 --max-bag-duration 300 \
  --qos-profile-overrides-path "$robot_ws/deploy_config/wbc-record-qos.yaml" \
  /control/policy/observation \
  /control/policy/command/rl \
  /control/policy/command/wbc \
  2>&1 | (trap '' INT; exec tee "$run_dir/record_policy_$(date +%Y%m%d_%H%M%S_%N).log")
)
```

### 7.3 B 组，可选：电机命令、反馈和 IMU

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
test -f "$robot_ws/deploy_config/wbc-record-qos.yaml"
mkdir -p "$run_dir/bags"
bag_path="$run_dir/bags/actuator_$(date +%Y%m%d_%H%M%S_%N)"
ros2 bag record --storage sqlite3 --node-name record_wbc_actuator \
  --output "$bag_path" --max-cache-size 16777216 \
  --max-bag-size 268435456 --max-bag-duration 300 \
  --qos-profile-overrides-path "$robot_ws/deploy_config/wbc-record-qos.yaml" \
  /actuator/motor/state \
  /actuator/motor/command \
  /imu/data \
  2>&1 | (trap '' INT; exec tee "$run_dir/record_actuator_$(date +%Y%m%d_%H%M%S_%N).log")
)
```

### 7.4 C 组，可选：动作与安全事件

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
test -f "$robot_ws/deploy_config/wbc-record-qos.yaml"
mkdir -p "$run_dir/bags"
bag_path="$run_dir/bags/events_$(date +%Y%m%d_%H%M%S_%N)"
ros2 bag record --storage sqlite3 --node-name record_wbc_events \
  --output "$bag_path" --max-cache-size 16777216 \
  --max-bag-size 268435456 --max-bag-duration 300 \
  --qos-profile-overrides-path "$robot_ws/deploy_config/wbc-record-qos.yaml" \
  /algorithm/brain/wbc_action_cmd \
  /system/state_machine/wbc_action_fdb \
  /control/policy/wbc/active \
  /control/policy/wbc/done \
  /hardware_status/motors_ready \
  /system/safety/emergency_stop \
  /system/safety/hardware_fault \
  /actuator/motor/enable \
  /joy \
  2>&1 | (trap '' INT; exec tee "$run_dir/record_events_$(date +%Y%m%d_%H%M%S_%N).log")
)
```

### 7.5 D 组，可选：ROS 日志

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
test -f "$robot_ws/deploy_config/wbc-record-qos.yaml"
mkdir -p "$run_dir/bags"
bag_path="$run_dir/bags/diagnostics_$(date +%Y%m%d_%H%M%S_%N)"
ros2 bag record --storage sqlite3 --node-name record_wbc_diagnostics \
  --output "$bag_path" --max-cache-size 16777216 \
  --max-bag-size 268435456 --max-bag-duration 300 \
  --qos-profile-overrides-path "$robot_ws/deploy_config/wbc-record-qos.yaml" \
  /rosout \
  2>&1 | (trap '' INT; exec tee "$run_dir/record_diagnostics_$(date +%Y%m%d_%H%M%S_%N).log")
)
```

每组启动后应出现所选话题的订阅信息，无 QoS 不兼容错误。尚未发生动作时 WBC 命令和 done 消息数量为零不等于录包失败；触发完整动作后再检查数量。

这些参数使用 Humble 的 sqlite3 存储，单文件达到 256 MiB 或 300 秒便分片，缓存上限 16 MiB。**分片不限制总磁盘占用，也不会自动停止录制。** 不预先创建 `bag_path` 本身；rosbag2 要求输出目录尚不存在。默认不开启压缩，减少板端实时负担。[ROS2 Humble 官方 record 参数实现](https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/record.py)

可选限时录包：在完整命令的 `ros2 bag record` 前加 `timeout --signal=INT 120`，120 秒后发送 SIGINT 收尾；timeout 返回 124 属于到时结束，仍须检查 metadata 和录包日志。手工结束则在对应录包终端按 Ctrl+C，等待写盘完成。

磁盘接近耗尽时先停止非必要录包。best_effort 与有限缓存不保证无丢帧；控制时序排查需检查序号、时间戳、实际记录数量和 recorder 的丢包警告。不要在接着真实 HPM 的 domain 中直接回放包含 motor command、enable、joy 或动作请求的话题。

## 8. 启动录包后：进入 Walking 并逐个执行动作

先在新终端持续监听动作反馈；记录其 `timestamp_ms`、`action_id`、`status` 与动作请求对应关系。

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
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String \
  --qos-reliability reliable --qos-durability volatile \
  2>&1 | (trap '' INT; exec tee "$run_dir/action_feedback.log")
)
```

现场操作原手柄 **B+Y** 从 Standby 进入 Walking，保持零速度，确认硬件桥输出 `FSM: Standby -> Walking`，AMP 开始持续提供有效命令。进入 Walking 成功和 WBC worker 就绪是两个独立条件；有前者而 prepare 服务不存在时，仍不能执行 WBC。

随后在另一个新终端执行一个动作：

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
action_id=L0_Hugme
/usr/bin/python3 - "$wbc_actions" "$action_id" <<'PYCODE'
import sys, yaml
if sys.argv[2] not in yaml.safe_load(sys.argv[1]):
    raise SystemExit('Action not selected in wbc_actions')
PYCODE
service_type=$(ros2 service type /control/policy/wbc/prepare)
test "$service_type" = luvbot_control_interfaces/srv/ControlWbcMotion
payload=$(/usr/bin/python3 -c 'import json,sys,time; print(json.dumps({"data": json.dumps({"timestamp_ms": time.time_ns()//1000000, "action_id": sys.argv[1]})}))' "$action_id")
printf '%s\n' "$payload" | tee -a "$run_dir/action_requests.log"
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String "$payload"
)
```

每次等待 `started` → `completed`、WBC 退出且 AMP 恢复，再发下一动作。将 `action_id` 改成 `L0_act_cute_1` 后重跑整个块即可；不要复用旧 payload，也不要固定 sleep 后无条件连发。当前时间窗口为过去 5 秒、未来 1 秒，相同 timestamp/action 会去重。`rejected` 或 `failed` 时先检查原因，暂停后续动作。

## 9. 停机、检查录包与回收

### 9.1 停机顺序

1. 停止动作请求，等待正在执行的动作完成并回到 AMP。按现场已有停机流程进入安全的非 Walking 状态；需要急停时使用原手柄 RB，不用杀进程代替机器人停机。
2. 在终端 4、3、5 分别 Ctrl+C 停止 WBC、AMP、手柄。操作前已完成上一项；不要在正常运动中直接中断策略。
3. 保持 HPM 运行，在终端 2 Ctrl+C 停止硬件桥，让其发送 disable。检查 `hardware_bridge.log` 和 HPM 实际停机状态，确认禁用命令被接收执行；出现 disable 确认超时不能记作正常停机。
4. 根据 HPM 脚本的前台/后台方式停止 HPM，或按现场要求保留已有公共 HPM 服务。关闭终端不会自动证明后台进程退出。
5. 录包进程保留到上述停机事件结束，再逐个 Ctrl+C，等每个 recorder 写完 metadata。最后停止动作反馈监听。

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
cp "$robot_ws/deploy_config/wbc-record-qos.yaml" "$run_dir/record-qos.yaml" 2>/dev/null || true
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
[[ "$run_name" =~ ^[0-9]{8}_[0-9]{6}_[A-Za-z0-9]{6}$ ]]
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

- **prepare service unavailable / worker did not respond**：先查看 `wbc.log` 是否有初始化失败或进程退出，再检查 DDS 环境；反复按 B+Y 不会恢复死亡的 worker。
- **Unsupported resident WBC layout，FP16/AFFINE**：检查板端是否构建并 source 了包含 FP16/AFFINE 兼容修复的部署源码，核对安装前缀和实际 worker；不要删除张量形状、布局或类型校验来绕过错误。
- **Policy command expired or unavailable**：检查 AMP 进程、观测和策略命令端点、命令时间戳/有效期及调度；持续出现时不继续动作。
- **节点/话题看不到**：先比较各终端与 HPM 的 domain、localhost、RMW 和自定义 DDS 配置。重新 source 后也只影响该终端随后启动的进程，旧进程须按流程停止后重启。
- **录包零消息**：核对是否已进入对应工作状态，以及 `ros2 topic info --verbose` 中发布者和 QoS；不要把录制可靠性改为 reliable 去订阅 best_effort 发布者。
- **Failed to set realtime priority**：保留警告并核查板端已有实时调度权限配置，性能结论应标明此条件；不要通过降低安全限制掩盖延迟问题。

### 本版部署的验证范围

部署端按约定不进行 ONNX/RKNN 数值对照、不计算训练端与板端输出残差，也不采集逐张量数据作为数值验收任务。训练端负责最终 FP16 交付资产及其精度；板端保留文件/哈希/ABI 检查和加载、动作、控制时序、故障与停机检查。`verification: null` / `not_run` 原样保存，不伪造数值通过状态。

A～D 组 rosbag 是可选的运控记录和问题排查资料，不是本流程的数值精度验证步骤。此前 `20261027_103346_3Rz7nA` 等隔离日志来自旧资产，既不能证明新 FP16 包已运行，也不用于补算本版数值误差。

## 11. 2026-10-28 本机 80 动作与三条功能路径复查

实际训练资产来自 `/home/luvbotic/下载/training_inputs.zip`，其中 `L0_V5_WBC10000_LoRA80_20261026/emotion_index.json` 含 80 条动作。`l0_deploy_v2_source_20261028_135934` 是不含权重的源码快照，不能替代训练资产。

完整导出包已放入本仓库 `legged_deploy/policy/l0_v5/wbc_lora_fp16/`：一个共享 `base_lora.onnx/.rknn`、80 个 FP16 `weights.bin`、80 个 `reference_motion.onnx/.rknn`、对应配置和 manifest。七层 Base 固定矩阵、31 个动态 LoRA/bias/LayerNorm 张量、参考浮点轨迹表均采用 FP16 参数存储；动态参数的应用输入 ABI 保持 FP32，resident loader 按 RKNN 原生 FP16/FP32 类型打包。不要把所有接口、帧索引、时间或校准值都改成 half。

本次修改复用了现有工具，没有再新增部署 shell 脚本：

- `export_wbc_lora.py`：在最终落盘前转换 Base、probe 固定参数和 80 条参考为 FP16，更新最终模型哈希，并用所有 80 个动作各 33 组样本复核最终图。源 checkpoint 不修改，原始重建验证和最终半精度验证分别记录。
- `convert_onnx_to_rknn_rk3588_wbc_policy.py`：编译临时图显式存储 FP16 参数并用 Cast 维护 FP32 接口；仍使用 `float_dtype=float16`、`do_quantization=False`，不声称每个内部算子都强制 half。
- `convert_onnx_to_rknn_rk3588_wbc_reference.py`：接受带 Cast 的 FP16 原始参考，规范化取帧后将合并参考表以 FP16 存储；输出保持六个 FP32 张量。
- 原 `preprocess_wbc_lora_fp16.py` 的编译副本可暂时将 half 值展开为 FP32 以兼容六个 Gather；这不恢复丢失的精度，新转换器随后明确保存 FP16 表。原始 FP16 交付包与编译临时图应区分。

81 个部署 RKNN（共享策略 1 + 参考 80）均编译成功，记录位于各 `.rknn.json`。完整训练导出/最终精度结果在包内 `validation_report.json`；编译汇总原始日志 `/tmp/l0_lora80_build_models/conversion_summary.json`。运行时依旧一个共享 LoRA policy，按动作切换参数，不是把 80 个动作融合进 80 份独立 policy。

若在板端按本手册复现，仍采用第 0～3 节的 FP16 包完整性检查、编译和安装步骤，并使用 `--build-only`。本次 x86 Toolkit 模拟器的数值测试属于开发验证，不意味着板端实时性能或真机闭环已通过，也不改变板端不做数值对照的约定。

五条直接 WBC 在 `policy/l0_v5/wbc/`，与这 80 个 LoRA 是两个独立包。文本生成含 x86 自定义分词算子，当前不能直接编译为可用 RKNN；文本 FP16 参数副本使用主机 ONNX Runtime，板端执行文本 JSON。三条路径的实际支持情况及文本/五条 WBC 真机操作，见 [文本与 WBC 部署文档第 9～10 节](text-motion-deployment.md#9-2026-10-28-fp16限位与手臂跟随复查以本节为准)。

最终补充：共享 RKNN 已完成 Toolkit 模拟器对 80 条动作各 33 组、共 2640 组输入的数值检查，最大绝对输出误差 0.00344467，最小余弦相似度 0.99999682。80 条资产的 `install-lora` 完整性/哈希/ABI 检查通过；工作区最终 7 包构建通过。模拟器耗时不可作为板端实时性能结论。
