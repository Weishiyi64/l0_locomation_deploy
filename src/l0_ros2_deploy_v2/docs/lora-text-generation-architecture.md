# LoRA 与文本动作生成：部署结构和运行流程

本文基于当前 `src/l0_ros2_deploy_v2` 源码及 2026-11-02 集成的动作资产，说明 L0 V5 在 RK3588、ROS 2 Humble 下的两条动作链路。这里的“文本生成”是**文本生成动作轨迹**：输入自然语言，生成九关节、50 Hz 的动作 JSON，再由 WBC 执行。具体板端命令见 [动作集成与验收](actions-20261102-deployment.md)、[LoRA 操作手册](wbc-lora-production-deployment.md) 和 [文本动作使用说明](text-motion-deployment.md)。

## 1. 总体关系

```text
                         ┌─ LoRA 动作 ID ─────────────────────────────┐
用户/控制台 ──────────────┤                                            ├─ 控制器 prepare/active ─ WBC ─ 电机命令
                         └─ 文本 → RKNNLite 生成 → 轨迹 JSON → 动态 ID ┘
                                                               │
实时观测、IMU、电机反馈 ────────────────────────────────────────────┘
AMP 行走策略 ──────────────── Walking 常态控制 ←─ WBC 动作结束后回交
```

两条链路都使用 `legged_controllers` 的动作状态机与 `legged_deploy` 的 `rl_wbc` worker。普通 WBC 选择预加载动作及对应 adapter/reference；文本链路为每次请求临时登记一条 JSON reference，默认加载 **`UpperBody_Enhanced` LoRA 参数**推理。两套运行包的 `base_checkpoint_sha256` 相同，但各自安装一份共享 `base_lora.rknn`，不能把文件交叉混用。AMP 是独立的行走策略，不是 WBC LoRA 的 Base 模型。控制器负责从 AMP 切入 WBC、选择有效命令和增益，并在完成、取消或故障后处理回交。

| 链路 | 入口及运行包 | 策略参数 | 动作参考 |
| --- | --- | --- | --- |
| 普通 WBC | `wbc_lora.launch.py` → `wbc_lora_fp16` | 共享 Base RKNN + 所选动作的 LoRA 参数 | 所选动作的 `reference_motion.rknn` |
| 文本生成 | `text_wbc.launch.py` → `text_wbc_bundle`；另起 `text_wbc_generate` | 同源 Base RKNN + `UpperBody_Enhanced` 的 LoRA 参数 | 每次新生成的 JSON reference，不编译逐请求 RKNN |

## 2. 源码与资产位置

| 位置 | 职责 |
| --- | --- |
| `scripts/process_wbc_lora_delivery.py`、`scripts/export_wbc_lora.py`、`scripts/preprocess_wbc_lora_fp16.py` | 训练交付检查、FP16 导出与 reference 编译前处理；产生板端轻量编译包。 |
| `scripts/batch_convert_onnx_to_rknn_rk3588.py`、`scripts/prepare_wbc_deploy.py` | 板端编译共享 policy 与逐动作 reference，校验并安装 LoRA 运行包。 |
| `scripts/start_sim2real.sh`、`scripts/wbc_action_console.sh`、`scripts/record_wbc_rosbag.sh` | LoRA 正式启动、单次动作请求、可选录包；HPM 生命周期由硬件侧维护。 |
| `legged_controllers/launch/sim2real.launch.py`、`legged_controllers/scripts/sequential_launch_real.sh`、`legged_controllers/launch/one_start.launch.py` | 实机启动传参，按硬件桥与就绪检查、主控制器、AMP/WBC worker 的顺序启动。 |
| `legged_controllers/launch/text_wbc.launch.py` | 文本动作场景的入口：指定 L0 V5、LoRA 模式、共享包、JSON 参数动作和预加载名单，再包含 `sim2real.launch.py`。 |
| `legged_controllers/launch/wbc_lora.launch.py` | 普通 WBC 的入口：指定 `wbc_lora_fp16` 和预加载动作名单，再包含 `sim2real.launch.py`。 |
| `legged_controllers/scripts/text_wbc_generate`、`texttomotion/generate_wbc_json.py` | 配置当前工作区的 RKNNLite/Runtime，采集姿态、生成完整 JSON，按需发起 WBC 执行。 |
| `texttomotion/motion_adapter.py`、`texttomotion/baseline/` | 文本模型推理与九关节轨迹适配；baseline 内含 ONNX、素材、词表及 RKNN 编译器。 |
| `legged_deploy/src/wbc_lora_assets.cpp`、`legged_deploy/src/rl_controller_wbc.cpp` | 校验 LoRA manifest/参数，加载共享 WBC policy、逐动作 reference，准备和执行动作。 |
| `legged_deploy/src/rl_controller_wbc_json.cpp`、`legged_deploy/src/wbc_json_reference.cpp` | 校验并登记文本 JSON，把九关节轨迹扩成 WBC 使用的 21 关节 reference。 |
| `legged_controllers/src/LeggedControllerWbc.cpp`、`legged_controllers/src/PolicyMux.cpp` | 处理动作请求、prepare 服务、AMP/WBC 切换、反馈及取消。 |

### 2.1 LoRA 模型包

训练端输入是共享 Base checkpoint、每动作 LoRA checkpoint、reference motion 及配置。`process_wbc_lora_delivery.py` 在本机生成 `model_sources/<包名>/`；板端只接收这个轻量编译目录，不需要原始 `.pt`：

```text
model_sources/<包名>/
├── base_manifest.json              # 共享图、动作索引、身份与哈希
├── base_lora.onnx                   # 共享 WBC policy
├── adapters/<动作 ID>/
│   ├── manifest.json
│   └── weights.bin                  # 31 个动态参数张量；不转换为 RKNN
└── actions/<动作 ID>/
    ├── rl_config.yaml
    └── reference_motion.onnx
```

板端编译后增加**一份** `base_lora.rknn` 和**每动作一份** `reference_motion.rknn`，并保留转换记录。`prepare_wbc_deploy.py --mode install-lora` 安装到 `legged_deploy/policy/l0_v5/<包名>/`，`colcon build` 再将其安装到 `legged_deploy` 的 package share。运行时 `one_start.launch.py` 在 package share 内解析 `base_manifest.json`，检查所选动作文件；`WbcLoraAssets` 继续校验身份、哈希、张量布局及配置。AMP 资产与此包分开；当前 AMP 模型路径仍关联编译时的源码策略目录，迁移工作区后需重新构建。

### 2.2 文本模型与运行资产

文本模型由 `texttomotion/baseline/` 的两个 ONNX（`qwen_tokenizer`、`inbetweener`）编译成两个 RKNN；词表、动作素材和编排逻辑仍在 Python/NumPy 中。当前工作空间已纳入 `text_generation_models_20261102` 的 `model_sources/text_to_motion/compiled/manifest.json`、RKNNLite2 依赖和 `librknnrt.so`。安装入口 `text_wbc_generate` 会从其所属工作区寻找这些运行依赖。新模型的来源标识为 `35918078c0dab7cf1845c87de72c33554c138e576ff781dab7ab5dfa905b8aff`，生成器与 WBC JSON 接收端已同步该标识。

文本链路要求 WBC LoRA 包中有已预加载的 JSON 参数动作，以及匹配的 AMP 资产、L0 V5 电机 mapping。`text_wbc.launch.py` 默认引用 `text_wbc_bundle`、`l0_v5_amp_10281` 和 `[UpperBody_Enhanced]`，JSON 动作选择 `parameter_source=lora` 的 `UpperBody_Enhanced`。当前 `text_wbc_bundle` 只保留这个动作及共享 Base RKNN；普通 WBC 的 86 个动作保留在独立的 `wbc_lora_fp16` 包。文本模型、运行依赖与 `l0_v5_amp_10281` 源码资产均已放入工作空间；目标板仍需构建和加载验证。

当前 `sim2real.launch.py` 默认使用 `wbc_mode:=lora` 和 `wbc_lora_fp16`，其默认动作名单均在该包中；`wbc_lora.launch.py` 与 `text_wbc.launch.py` 则显式传入各自的包名和较小的预加载名单。旧逐动作 `policy.rknn`/`reference_motion.rknn` 目录仍可通过显式 `wbc_mode:=static` 选择，但需要提供对应资产。

## 3. LoRA 从交付到执行

1. **训练交付**：训练端导出共享 Base 图、逐动作 adapter 和 reference；脚本检查索引、哈希、模型接口、关节顺序并完成 FP16 交付前处理。ONNX 的外部输入/输出保持 FP32。
2. **板端编译与安装**：批量转换器编译共享 policy 和各动作 reference；安装器校验转换记录后生成运行包；在板端工作空间执行 `colcon build --packages-up-to ...` 并 source 目标工作空间。
3. **启动**：硬件侧先准备 HPM。执行 `ros2 launch legged_controllers wbc_lora.launch.py hpm_mode:=real`；默认选择 `wbc_lora_fp16` 的 `L0_Hugme` 与 `L0_act_cute_1`，可通过 `wbc_policies` 选其他动作。入口经 `sim2real` 和 `one_start` 创建唯一 WBC worker；启动时预加载所选动作的配置与 reference，并初始化一份共享 policy，不自动执行动作。原有 `start_sim2real.sh` 也可按其配置文件指定包名和名单。
4. **动作请求**：操作者先使机器人进入 Walking 并保持零速。控制台向 `/algorithm/brain/wbc_action_cmd` 发布 `{"timestamp_ms": ..., "action_id": "<动作 ID>"}`；动作 ID 必须在实际加载的 manifest 名单中。
5. **准备和切换**：控制器调用 `/control/policy/wbc/prepare`。worker 将选中动作的 `weights.bin` 读入备用缓冲并提交到共享 RKNN context，准备首帧；成功后控制器通过 `control/policy/wbc/active` 切换，AMP 暂停推理，WBC 输出 `control/policy/command/wbc`。每周期查询该动作的 reference RKNN，再运行共享 policy。
6. **结束**：WBC 发出完成事件，控制器发布 `/system/state_machine/wbc_action_fdb`，回交 AMP。准备、执行或交接期间不排队执行其他动作；急停与故障走现有安全路径。

当前 `wbc_lora_fp16` 包含 86 个动作，均标为 `parameter_source=lora`。**一个运行包内只初始化一份共享 policy**，每个预加载的普通动作有自己的 reference RKNN 和 adapter；`weights.bin` 在 prepare 时加载并切换，不会为每个动作再建立一份 Base policy。

## 4. 文本生成到 WBC 执行

```text
文本 / 三组 token + 起姿
  → qwen_tokenizer / 素材编排 / inbetweener（RKNNLite）
  → 九关节 50 Hz 轨迹 + 限位检查
  → gmr_upper_body_joint_trajectory_v1 JSON（新 T_* 请求 ID）
  → /text_motion/json → worker ready
  → /algorithm/brain/wbc_action_cmd → 控制器 prepare/active
  → 共享 Base WBC policy + UpperBody_Enhanced LoRA + JSON reference → 完成/取消 → AMP
```

1. **生成入口**：`ros2 run legged_controllers text_wbc_generate` 接收 `--model`、`--bundle`、`--mapping`、`--base-action`、`--text`（或 `--tokens`）、`--output` 等参数。直接运行 Python 源文件不会经过入口脚本的依赖路径配置。
2. **起姿来源**：带 `--snapshot` 时离线读取显式 21 关节、机体位置和四元数，只生成文件，不创建 ROS 节点；不带它时订阅 `/control/policy/observation`，要求观测新鲜、推理已使能，并在生成后再次检查姿态变化。`--execute` 必须使用实时观测与板端 Lite2 后端。
3. **轨迹生成**：模型输出九关节动作，适配层处理左右臂顺序、活动部位、初末帧和机械限位；默认越界拒绝。完整 JSON 写入新文件，包含 50 Hz 帧、起姿、Base 身份、有效期及生成诊断。输出文件已存在会拒绝覆盖。
4. **登记与执行**：只生成 JSON 时流程到此结束。带 `--execute` 时先发布 `/text_motion/json`；WBC worker 校验大小、身份、时间、关节、起姿等条件，登记动态 `T_*` 动作并回 `ready`。生成进程收到 `ready` 后才发送正式动作命令。worker 对此动作加载 `UpperBody_Enhanced` 的 LoRA adapter 和 JSON reference，不创建逐请求 reference RKNN。普通 WBC 动作由另一套运行包及启动入口提供。
5. **参考轨迹与反馈**：worker 将九关节轨迹扩成 21 关节参考，加入起始保持、进入/退出过渡和结束保持共 100 帧。`/text_motion/wbc_status` 的 `ready` 仅说明 JSON 已登记；动作是否开始、完成或取消，以控制器 `/system/state_machine/wbc_action_fdb` 和 AMP 恢复为准。
6. **取消**：向 `/text_motion/cancel` 发布该次原始 `T_*` ID。worker 标记取消；如果动作已交给控制器，仍需等待控制器的终态反馈和 WBC→AMP 回交。JSON 有短有效期，且请求不能重播；每次执行重新采集起姿并生成新文件。

文本生成模型（`qwen_tokenizer`、`inbetweener`）与 WBC Base 是不同模型：前者负责产生轨迹，后者负责根据实时观测和轨迹输出 21 关节控制动作。`text_wbc_bundle` 仅保留 `UpperBody_Enhanced` LoRA 和它的 reference；文本动作本身使用每次生成的 JSON reference。文本包与普通 WBC 包的 Base checkpoint 身份一致，但两份 `base_lora.rknn` 的文件哈希不同，应分别按各自 manifest 加载。

## 5. Base 与训练轮次的可追溯范围

两套运行包的 `base_checkpoint_sha256` 均为 `f02d4845019346b577fb0babceb8c288e1b5e1694f7888907ae8cfd63d7e4142`。它与本机 `training_inputs/L0_V5_WBC10000_LoRA80_20261026/base/base_ckpt.pt` 的实际哈希相同，因此可确认这两套包的 WBC Base 与该训练交付目录一致。部署包不包含原始 `.pt`。

原训练目录的 `emotion_index.json` 收录 80 个动作，路径均为 `lorackpt_2000.pt`，并记录 `num_updates=2000`。其中 4 个动作已用 `bushu_4motion_1102` 的新 checkpoint 和 reference 替换；新 checkpoint 内部同样记录 `num_updates=2000`、`iteration=1999`，来源哈希见[四动作更新记录](actions-20261102-deployment.md#四动作替换2026-11-02)。普通 WBC 包还包含 6 个其他动作；其中 5 个 `A-*` 动作的 adapter 权重与 `UpperBody_Enhanced` 逐字节相同。文本包仅保留 `UpperBody_Enhanced`。运行包的 adapter manifest **没有保存 LoRA checkpoint 轮次或原始 `.pt` 哈希**，因此不能仅从部署资产断言“所有 LoRA 动作都是 2000 轮”；应结合对应训练交付记录。

## 6. 关键接口与检查点

| 接口 | 发布方 → 接收方 | 用途 |
| --- | --- | --- |
| `/control/policy/observation` | 硬件/控制链 → AMP、WBC、文本生成进程 | 21 关节、IMU 等实时观测。 |
| `/algorithm/brain/wbc_action_cmd` | LoRA 控制台或文本生成进程 → 控制器 | 带时间戳的动作 ID 请求。 |
| `/control/policy/wbc/prepare` | 控制器 → WBC worker | 选择动作、加载参数、准备首帧。 |
| `control/policy/wbc/active` | 控制器 → AMP/WBC worker | AMP/WBC 推理切换。 |
| `control/policy/command/rl`、`control/policy/command/wbc` | AMP/WBC → 控制器 | 控制器按状态选择有效策略输出。 |
| `/text_motion/json`、`/text_motion/wbc_status` | 文本生成进程 ↔ WBC worker | 提交完整轨迹、返回登记状态。 |
| `/text_motion/cancel` | 操作端 → worker 与控制器 | 取消文本请求。 |
| `/system/state_machine/wbc_action_fdb` | 控制器 → 操作端 | `started`、`completed`、`cancelled`、`rejected`、`failed` 等动作反馈。 |

启动前先核对实际工作区的资产与配置，再看 worker 的 `wbc_mode`、`wbc_lora_manifest`、`available_wbc_policies`。看到 `ready`、模型初始化或 ROS 话题存在，均不等于实机动作已完成；运行验收仍需检查动作反馈、AMP 回交和真实机器人状态。本文由静态源码检查整理，未在机器人上编译或启动。
