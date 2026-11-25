# RK3588 WBC-LoRA 部署开发规划与代码修改范围

> **2026-10-26 后续实施更新：** 本文保留为历史方案/证据；交付边界以用户最新指令为准。当前已开发正式 sim2real 链路中的 base_lora 加载及 HPM 隔离入口，C++ 仅涉及模型加载/推理，远端尚未验收。现行操作见 [正式部署同步与日志](wbc-lora-robot-sync-and-logs.md)，不再执行本文旧的独立 smoke 步骤。

日期：2026-10-26。状态：P0 离线工具开发及 CPU 验证已完成；RKNN 转换/真板门槛由用户手动验收，P1–P4 未开始。具体产物及命令见 [P0 交付](wbc-lora-p0-handoff.md)。

依据：[部署方案](wbc-lora-deployment-plan.md)、[验证工作流](wbc-lora-validation-workflow.md)、[可行性审阅](wbc-lora-feasibility-review.md)。本文将这些要求落实到当前代码的文件、函数、参数传递、线程职责和验收顺序。

## 1. 本轮开发目标与边界

交付一条可验证的链路：共享基础 policy 图常驻，按 emotion action 加载完整 adapter；reference/config 与 adapter 成套选择；最终 policy/reference 都是 FP16 RKNN。本地 OpenVINO 与板端 RKNN 共用加载及 WBC 计算逻辑。板端主要 smoke 只运行模型，不启动 HPM 或任何电机通信。

基础行走继续使用 AMP。保留原部署步骤及启动顺序，第一阶段不改对外 `wbc_action_cmd`、反馈话题、`PolicyObservation`、`PolicyCommand` 或 `ControlWbcMotion.srv`。沿用 Walking 前置条件、速度/姿态检查、准备后激活及返回 AMP 的平滑交接。

核心修改集中在 `legged_deploy`，其次是 `legged_controllers` 的参数传递及 WBC 准备逻辑。`legged_real`、仿真环境的 launch 仅传入控制器新参数；硬件运行逻辑、HPM、标定、映射、增益数值和电机协议不属于本轮开发范围。

只使用轻量包中实际存在的部分动作，首批至少两个，例如 `L0_Hugme`、`L0_act_cute_1`；随后覆盖本地 10 个动作。不将补齐 80 个动作作为开发前提。

## 2. 先固定的实现选择

| 项目 | 本阶段选择 |
| --- | --- |
| 动态参数 | 14 个 A/B、7 个 Linear bias、10 个 LayerNorm gamma/beta，共 31 个张量；rank=alpha=16 |
| 基础策略 | worker 级持有一个共享 policy Context；动作条目不再各自拥有一份 LoRA policy 模型 |
| reference | 首阶段预加载选定动作子集的 reference；仍各有自己的 Context，不同时引入 reference 按需初始化 |
| 图输入 ABI | 首版验证 2 个原始观测输入 + 31 个参数输入；先不增加打包拆包算子。若 SDK 不接受该图，记录阻塞并修订 ABI 后再集成 |
| 模型精度 | policy/reference RKNN 均为 FP16；源 ONNX 和数值对照为 FP32 |
| 参数文件与运行 I/O | 参数文件明确记录 dtype；第一版可用 FP16 参数文件，在非实时加载阶段转为现有 FP32 应用输入，由 Runtime 处理模型输入转换 |
| 归一化 | 本地子集与 Base 相同，图内共享；manifest 校验归一化标识，不再外部归一化一次 |
| 参数驻留 | 两个固定活动/备用参数槽位；不预加载全部 adapter，不先实现通用 LRU 或后台预取 |
| 板端 smoke | 独立 `wbc_model_smoke` 可执行程序，复用正式加载器、执行器和输入组装逻辑，无硬件对象 |
| 本地 | 项目已有 OpenVINO CPU 后端；ONNX Runtime 仅用于离线 oracle，不增加第三条正式 ROS WBC 运行链 |
| 旧部署兼容 | 显式区分 `static` 与 `lora` 资产模式，先保留旧静态 WBC 入口用于对照/回退，不自动改现有默认名单 |

FP16 文件格式与 FP32 Runtime I/O 是独立约定，不能把 half 字节按 `float*` 解读。输入类型或布局与模型不符必须报错。首版先保证正确性，再依据实测决定是否使用设备原生 FP16/零拷贝缓冲；不把低拷贝作为首个 smoke 的前提。

## 3. 现有文件的具体修改范围

路径均相对于部署仓库 `src/l0_ros2_deploy_v2/`。

### 3.1 离线导出、打包及 FP16 转换

| 文件 | 当前行为 | 具体调整 |
| --- | --- | --- |
| [prepare_wbc_deploy.py](../prepare_wbc_deploy.py) | 读取单动作规范导出，校验两输入 policy，生成单动作目录 | 增加明确的 LoRA 打包入口/模式；读取 Base、所选 adapter 和对应 reference，复用现有配置字段/关节值处理，生成共享图和逐动作资产；旧两输入打包入口保留 |
| [convert_onnx_to_rknn_rk3588_wbc_policy.py](../convert_onnx_to_rknn_rk3588_wbc_policy.py) | `check_model()` 仅允许 2 个输入 | 保留静态校验，新增按 graph manifest 校验动态输入的名称、完整形状和数量；保持 `policy[399]`、`command_short[240]`、`action[21]`；参数按名称映射，不依赖图输入位置 |
| 同一 policy 转换器的构建/验证部分 | 非量化构建，对照单模型输出 | 明确固定目标版本与 FP16 浮点配置；读取至少两个 adapter 的完整参数及固定观测验证；沿用现有 Mish/opset 兼容处理，保留 FP32 oracle |
| [convert_onnx_to_rknn_rk3588_wbc_reference.py](../convert_onnx_to_rknn_rk3588_wbc_reference.py) | 参考图兼容改写、逐帧对照；实际误差判据硬编码 0.02 | 保留现有 Gather 兼容处理和帧语义；明确 FP16 构建；将实际容差统一为一个参数来源并记录，取消“声明 0.005、实际 0.02”的不一致 |
| [batch_convert_onnx_to_rknn_rk3588.py](../batch_convert_onnx_to_rknn_rk3588.py) | 递归匹配子目录 `policy.onnx` / `reference_motion.onnx` | 增加按部署 manifest 编排的 LoRA 模式：共享 `base_lora.onnx` 只转换一次，选中 reference 各转换一次；保持原静态批处理。新模式不能只看输出文件存在就跳过，需核对源模型、构建参数及版本 |

新增导出工具负责生成动态图与参数包，不修改训练仓库。真实层名、639/1423/21 等维度、5 处 LayerNorm/Mish 和拼接关系依据已交付模型与参考实现核对，不按“七层等宽 MLP”重建。

动态图参数名采用固定短名并在 manifest 映射回训练层名；A/B、bias、gamma/beta 的形状、方向及替换/增量语义分别声明。actor_std 不进入现有确定性 action 图，记录该处理。

### 3.2 板端模型执行与公共 WBC 核心

| 文件 | 具体调整 |
| --- | --- |
| [legged_deploy/include/rl_controller_wbc.h](../legged_deploy/include/rl_controller_wbc.h) | 将 LoRA policy 所有权提升到 worker；`Motion` 保留 config、reference、adapter 标识和时序。移除公共逻辑对 RKNN 头文件的直接依赖；拆出可独立调用的 WBC 核心，ROS worker 保留订阅/服务/线程职责 |
| [legged_deploy/src/rl_controller_wbc.cpp](../legged_deploy/src/rl_controller_wbc.cpp)：构造、`loadConfig()`、`initializeModel()` | 分离共享 policy 初始化与逐动作 metadata/reference 初始化；LoRA 模式不再逐动作初始化完整 policy；严格核对 Base/ABI、关节顺序、参数布局及 reference 信息 |
| 同文件：`makePolicyInput()`、`makeCommandInput()`、`inferReferenceFrame()` | 移到公共核心供 worker 和 smoke 共用；保持历史顺序、参考偏移、朝向锚点及周期内重复帧复用。提供受控单步输入，不在 smoke 中复制组装算法 |
| 同文件：`inferAndPublish()` | 通过统一执行接口绑定 2 个观测和 31 个动作参数；按动作配置裁剪/缩放；保留有限数值、观测新鲜度、命令有效期和 20 ms 预算判断；发布前核对请求代次仍有效 |
| 同文件：`prepareCallback()`、`activeCallback()`、`runLoop()` | 引入独立加载线程、准备代次和明确阶段；文件读取不持有运行锁。共享 Context 只由推理所有者调用；准备成功才生成新鲜首帧，取消和迟到完成按代次处理 |
| [legged_deploy/include/rknn_model.h](../legged_deploy/include/rknn_model.h) / [src/rknn_model.cpp](../legged_deploy/src/rknn_model.cpp) | 增加必要的输入/输出形状、dtype、字节数/stride 查询；复用输入描述及输出空间；在保留原 `run()` 接口的基础上供 WBC 执行适配层调用，保持错误及有限数值检查 |

`RknnModel` 还被 AMP、imitation 和 planner 使用。首版不强迫这些调用者改用新接口，不改变它们的模型/策略行为；若内部缓冲管理调整，必须确认其单输入、多输出等现有调用仍成立。

首版取消应用层每周期的大块临时分配：399/240 输入使用固定工作区；17 帧状态历史、3 帧动作历史及 8 份参考临时数据预分配；输入描述与动作输出复用。SDK 内部分配情况按实际接口和测量记录，不能因应用层预分配就宣称整个 Runtime 无分配。

### 3.3 构建与安装

| 文件 | 具体调整 |
| --- | --- |
| [legged_deploy/CMakeLists.txt](../legged_deploy/CMakeLists.txt) | 建立小型内部 WBC 公共库，供 `rl_controller` 和 `wbc_model_smoke` 链接；公共核心不再只在 RKNN 条件内编译。RKNN/OpenVINO 分别编译对应适配器，复用既有后端选择 |
| 同文件 | 安装独立 smoke 目标及实际需要的工具；整理 `rknn_model` 的编译/链接，避免同一目标内重复定义；共用 manifest 校验所需的 OpenSSL 在 OpenVINO 构建下也可链接 |
| [legged_deploy/package.xml](../legged_deploy/package.xml) | 依赖检查项：已有 yaml-cpp、OpenSSL、OpenVINO；不预先增加第三方 JSON/NPZ/C++ 推理依赖，除非实施发现明确缺项 |
| [legged_deploy/src/main.cpp](../legged_deploy/src/main.cpp) | 并发检查项：已有 `MultiThreadedExecutor`。确保 WBC 回调可并行处理取消/观测；通常只需修改 worker 的 callback group，不默认重写公共 main 或影响 AMP |

### 3.4 控制器准备、取消及时间预算

| 文件 | 具体调整 |
| --- | --- |
| [legged_controllers/include/legged_controllers/LeggedController.h](../legged_controllers/include/legged_controllers/LeggedController.h) | 增加独立的 `wbc_prepare_timeout_sec_`；不改变现有交接状态、急停入口或速度阈值 |
| [legged_controllers/src/LeggedControllerInit.cpp](../legged_controllers/src/LeggedControllerInit.cpp) | 声明、读取和验证准备超时；与命令等待超时分别记录。缺省先沿用 0.5 s，板端冷加载测量后通过 launch 配置调整准备预算 |
| [legged_controllers/src/LeggedControllerWbc.cpp](../legged_controllers/src/LeggedControllerWbc.cpp) | START 准备等待使用新预算；CANCEL、有效首帧/返回 AMP 的等待继续使用原命令预算。等待期间保留 AMP，准备返回后仍重新检查 generation、Walking、姿态/速度及命令有效性 |
| [legged_controllers/src/PolicyMux.cpp](../legged_controllers/src/PolicyMux.cpp) | 复用 `prepareWbcMotion(command,id,timeout)` 的现有签名；核对调用处各预算；`loadWbcWalkingGains()` 使用本次动作响应对应的生成配置，不改增益数值。读取仍在非控制循环回调内 |

`PolicyMux.h`、`LeggedControllerRosIo.cpp` 不列为默认必须修改：现有服务调用签名和控制器侧 Reentrant 回调组已能复用；只有实施新增成员声明或实际回调接线确需调整时才改对应位置。

### 3.5 launch 及脚本参数传递

| 文件 | 具体调整 |
| --- | --- |
| [sim2real.launch.py](../legged_controllers/launch/sim2real.launch.py) / [sim2mujoco.launch.py](../legged_controllers/launch/sim2mujoco.launch.py) | 作为用户入口集中声明新参数；实机期望 RKNN、本地期望 OpenVINO，LoRA 模式和动作名单显式给出，保留现有默认启动顺序 |
| [sequential_launch_real.sh](../legged_controllers/scripts/sequential_launch_real.sh) / [sequential_launch_sim.sh](../legged_controllers/scripts/sequential_launch_sim.sh) | 只增加参数传递及必要的参数数量/帮助说明校验；不增加 HPM 启动，不修改现有进程顺序和清理逻辑 |
| [one_start.launch.py](../legged_controllers/launch/one_start.launch.py) | 按模式/明确后端检查资产；static 沿用现有名单，lora 检查共享模型及所选 action 的 adapter/reference/config。仍只启动一个 WBC worker，拒绝后端与编译能力不匹配，不自动 CPU 回退 |
| [legged_real.launch.py](../legged_env_real/legged_real/launch/legged_real.launch.py) / [legged_mujoco_sim.launch.py](../legged_env_mujoco/legged_mujoco/launch/legged_mujoco_sim.launch.py) | 只将准备超时传入控制器节点；不把模型后端/资产参数无意义地传入硬件实现，不修改电机反馈、使能和仿真桥行为 |

后续旧静态模型完全退出时可单独清理兼容分支，本阶段不以此扩大重构范围。

## 4. 计划新增的代码文件

以下是拟新增位置，不是已经存在的实现。可在实际编码时合并小文件，但职责不能复制到多个入口。

| 拟新增文件 | 职责 |
| --- | --- |
| `export_wbc_lora.py` | 从交付 Base/adapter 导出完整动态图及标准参数包，生成层映射、ABI、哈希和归一化标识；不依赖修改训练过程 |
| `prepare_wbc_smoke_data.py` | 读取已有 NPZ/记录观测，生成固定状态、独立 reference/policy 以及组合链期望值和来源信息；板端格式采用 YAML/JSON 数组，复用 yaml-cpp，避免新增 C++ NPZ 解析依赖 |
| `legged_deploy/include/wbc_model_backend.h` | 仅定义 WBC 所需模型元信息、按名称绑定输入、执行及读取输出的薄接口 |
| `legged_deploy/src/wbc_model_rknn.cpp` | 复用 `RknnModel`，核对 FP16 模型/应用 I/O，提供正式板端执行路径 |
| `legged_deploy/src/wbc_model_openvino.cpp` | 使用已有 OpenVINO 依赖加载同源 ONNX，提供相同的输入/输出契约 |
| `legged_deploy/include/wbc_policy_core.h` / `src/wbc_policy_core.cpp` | 共用状态历史、参考查询、399/240 输入构造、单步推理及动作解码；不创建硬件对象或 ROS 控制话题 |
| `legged_deploy/include/wbc_lora_adapter.h` / `src/wbc_lora_adapter.cpp` | manifest、哈希、层/形状/范围校验、参数读取及固定槽位管理；不直接调用或改写在运行的 NPU Context |
| `legged_deploy/src/wbc_model_smoke.cpp` | 独立模型 smoke，调用同一公共核心及加载器，生成逐 case 精度和耗时报告；无 HPM、硬件适配器、motor command/enable 接口 |

Python 离线工具可以继续使用已有 PyTorch/ONNX/ONNX Runtime 环境。板端运行时不加载 `.pt`，不为这次改造安装 PyTorch。JSON manifest 用现有结构化 YAML 解析能力读取，所有 shape、dtype、偏移、长度及有限数值显式校验；不另造文本扫描解析器。

## 5. 参数及资产契约

| 参数 | 消费方与计划含义 |
| --- | --- |
| `wbc_enabled` | 沿用；关闭时不启动 worker、不校验重资产 |
| `wbc_mode` | 新增 `static/lora`，首阶段缺省 `static`，显式选择新方案，不自动改现有行为 |
| `wbc_backend` | 新增期望值 `rknn/openvino`；入口根据环境明确传入，worker 与编译能力交叉校验。它不意味着一个二进制可以任意切换未编入的 Runtime |
| `wbc_lora_root` | 新增 LoRA 部署根目录，按 package share/robot_type 定位；不引用训练机绝对路径 |
| `wbc_actions` | 新增 LoRA 动作 ID 列表；非空、无重复、合法 ID，且全部资产存在；只注册实际选定子集 |
| `wbc_policies` | 旧 static 目录名单，语义不改；lora 模式只使用 `wbc_actions`，不同时加载旧 policy 集合 |
| `wbc_prepare_timeout_sec` | 新增加载/准备等待预算，初始缺省 0.5 s，与现有行为对齐；可依据冷加载实测单独配置 |
| `wbc_command_timeout_sec` | 沿用有效命令、返回和清理等待预算，不因磁盘加载而放宽；原有值 0.5 s |

参数沿现有链路分流：模型模式、后端、LoRA 根目录和动作名单由 `sim2* → sequential_launch_* → one_start → RLWbc` 传递，`one_start` 将名单映射为 worker 的 `available_wbc_actions`；准备超时由 `sim2* → sequential_launch_* → 环境 launch → LeggedController` 传递。环境节点不消费模型后端参数，worker 不消费控制器的交接超时。

LoRA worker 继续使用 `rl_controller` 可执行程序及 `policy_type=rl_wbc`。`robot_cfg_path` 指向选定名单首个动作的生成配置，以满足现有 `main.cpp` 的启动读取；真正执行时按请求切换成套动作配置。AMP 的 `subpolicy`、配置路径和启动节点不因上述传参而改变。

资产沿用部署方案中的 `wbc_lora/base_lora.*`、`base_manifest.json`、`adapters/<action_id>/` 和 `actions/<action_id>/` 布局。每动作保留含 `RobotCfg/WbcCfg` 的 `rl_config.yaml`，worker 的准备响应继续返回该动作配置路径，因而无需改变现有增益加载接口。

共享图 manifest 固定图 ABI/层映射/归一化标识并记录 ONNX 与 RKNN 各自哈希。adapter 绑定 Base checkpoint 哈希、ABI 和张量布局；打包发布时补齐实际 RKNN 哈希，避免 adapter 导出阶段被尚未生成的 RKNN 文件阻塞。模型、参数、reference 和配置的版本关系不能仅靠文件名判断。

manifest 的相对路径限制在部署根目录内；未知 action 或不完整/不兼容资产直接拒绝，不从磁盘按外部请求拼接任意路径。static 的严格资产检查继续保留，LoRA 模式也不采用“跳过坏动作继续启动”的隐式降级。

## 6. 动态加载的线程与状态调整

职责明确分为三部分：ROS 回调接受命令和观测；加载线程执行文件读取、哈希、解析及类型转换；推理所有者在安全边界选择参数、调用共享 Context、维护历史及生成结果。公共 WBC 核心不自带线程，独立 smoke 可同步单步调用相同实现。

```text
Idle → Loading(epoch) → Ready(epoch) → Prepared(epoch) → Active(epoch) → Idle
                  失败/取消/超时：使 epoch 失效，拒绝提交，维持 AMP
```

关键实现要求：

1. START 先保留唯一准备槽并分配代次；加载器仅写备用槽。禁止在当前 `prepareCallback()` 的整段运行互斥锁内读文件或等待加载。
2. 服务可以有界等待结果，但必须使用允许 CANCEL 并发进入的服务回调组；观测更新、active 控制不能被同一个默认互斥回调组拖住。已有 MultiThreadedExecutor 不会自动解除同组串行限制。
3. Ready 仅表示参数已准备。随后推理所有者核对代次，选择成套资源、采集新鲜观测、重置 17/3 帧历史及朝向锚点，并在原预算内生成首帧；响应成功前不暂停 AMP。
4. CANCEL、超时和急停使旧代次失效。已进入设备的调用不要求强制中止，但其结果不得发布或激活；活动槽在设备不再读取后才可复用。确认取消不等于立即释放仍在途的内存。
5. 新请求只有在旧请求不能再生效、相关槽位可安全复用后才能提交。执行中/交接中不排队执行其他 emotion；保留现有控制器清理确认机制。
6. 选定动作的参数、reference、增益和历史必须属于同一代。准备失败保留 AMP；运行中无效输出/过期/故障仍按原急停路径退出，不能复用过期目标。

第一阶段不增加服务命令枚举或新 ROS 消息。上述代次在单 worker 内维护，并与现有控制器 generation/准备槽机制配合。若后续要开放多个外部加载客户端或执行中换 adapter，应另立协议，不能直接扩展当前单请求假设。

## 7. 板端无 HPM smoke 的代码及数据范围

`wbc_model_smoke` 链接纯 WBC 公共库和 RKNN 执行器，通过 `ros2 run` 所需的 ament 安装入口调用，但不需要创建控制节点。它不调用 `sim2real`、`one_start` 或硬件就绪脚本，不创建 `LeggedReal`，不发布 actuator command/enable。

测试数据在离线阶段生成，部署时与模型一起显式传入。通过 YAML/JSON 保存固定状态及期望数组，原始 NPZ 作为来源保留；不让 C++ smoke 另引入 NPZ 库。至少两个动作、每动作不少于现有验证所需的 20 组代表输入，然后扩展到 10 个动作子集。

分三层调用同一执行代码：

| 层次 | 输入与对照 |
| --- | --- |
| reference | 指定 frame index，核对 6 组输出、首/中/末帧及边界钳位 |
| policy | 固定 399/240 输入和 adapter，比较 action 与 q_target，排除 reference 数值影响 |
| 组合链 | 固定机器人状态、17 帧历史、3 帧 previous actions、锚点和帧索引，分别以 CPU reference/板端 FP16 reference 组装 command 后比较策略结果 |

要求 `A → B → A` 全程共享 policy 初始化次数为 1；reference 初始化次数单独记录。期望输出由已核对的 FP32 模型生成，不用待测 RKNN 自己生成 oracle，不让两条链分别反馈输出形成闭环漂移后再做逐帧精确比较。

首版精度按既定数量级记录与判断：policy 最大绝对误差 0.05、reference 0.02，均可配置并输出实际值，统计 q_target 弧度误差。结构错误、错参数/错帧、非有限输出直接失败；不为追求 FP32 末位一致反复调参。

报告字段包括：模型/数据/adapter 哈希、case/action ID、实际后端、Toolkit/Runtime/driver、模型精度与 I/O dtype、每层误差、加载/首帧/稳态周期耗时、Context 初始化次数、内存及失败原因。数值失败与性能失败分别汇总；无 HPM smoke 不能代替后续运动验收。

## 8. 开发拆分、依赖及完成条件

| 批次 | 内容与修改面 | 产物及完成条件 |
| --- | --- | --- |
| P0：导出契约与目标算子验证 | 离线导出/打包工具、动态 policy 校验；reference FP16 转换参数 | 两个真实 adapter 的完整参数图与 FP32 oracle 对齐；目标 RK3588 真实 block 在同 Context 切换通过。未通过时先记录 SDK 阻塞，不展开完整 ROS 改造 |
| P1：公共核心与主要板端 smoke | RKNN 执行适配、adapter 加载、公共单步核心、CMake、smoke/数据准备工具 | policy/reference 均为 FP16；无 HPM 的三层对照及 `A → B → A` 通过；共享 policy 不重建。形成第一份真实设备数值报告 |
| P2：正式 worker 与控制链接入 | RLWbc 加载线程/代次/缓冲、准备预算、控制器及实机 launch 参数 | 复用服务/topic，冷加载期间 AMP 保持，取消/迟到结果不误激活；旧 static 入口及 AMP 等调用保持兼容 |
| P3：本地同源 ROS 2 / MuJoCo | OpenVINO 适配、公共 worker 编译、仿真参数链 | 与板端使用同一参数包和配置，完成加载/取消/返回及 `AMP → A → AMP → B → AMP → A`，不以 CPU 结果替代 RKNN 报告 |
| P4：周期、资源和子集回归 | 仅针对实测瓶颈调整缓冲/输入绑定；补充报告和操作文档 | 正式 C++ 完整 reference/input/policy 周期满足原 20 ms 判定；记录冷/热加载及 10 动作重复切换，检查资源增长和回归影响 |

依赖顺序为 `P0 → P1 → P2 → P3 → P4`。P0/P1 是板端可行性门槛，P3 不阻塞首次无 HPM smoke。低拷贝、参考轨迹跨周期缓存、NPU 绑核和全部 80 动作扩展不列为首个可用版本的前置工作。

每批代码、配置/资产格式说明及相应验证记录一起交付。正式构建从父工作空间进行，使用匹配版本的接口包；本规划不执行构建、仿真或实机启动。

## 9. 明确不纳入默认修改的范围

| 范围 | 处理 |
| --- | --- |
| HPM 仓库、驱动、CAN/电机协议与节点启动 | 不改；本轮主要 smoke 完全不启动它们 |
| `legged_env_real/legged_real/src/LeggedReal.cpp`、硬件 watchdog/enable/校准 | 不改；新增参数只在 launch 传到共享控制器，不改变硬件状态机 |
| 电机 mapping、关节限位、标定与 PD 数值 | 不改；使用原训练/部署配置并核对动作绑定 |
| AMP、imitation、planner 的策略算法 | 不改；仅对共用 RknnModel 的内部调整做兼容验证 |
| `ControlWbcMotion.srv` 及外部消息定义 | 首阶段不改字段/枚举，保持当前调用方可用 |
| 训练实现、rank、LoRA 注入位置及数学结构 | 不改；只增加推理导出包装和部署参数格式 |
| 工作空间 `deploy_workflow` 与原始方案归档 | 保留，新的开发状态在本文件及修订方案记录 |

## 10. 本轮规划的静态核对依据

已核对 worker 的初始化/2 输入约束、Motion 资源所有权、首帧准备和锁；`RknnModel` 的 AMP/imitation/planner 调用；控制器 START/CANCEL/返回逻辑；两条顺序脚本和环境 launch 的参数消费；构建条件及已有依赖。以上内容据当前工作区源码整理，包含用户尚未提交的修改。

P0 已实现动态图及完整参数包导出、FP16 转换接入、规范资产校验和手动 RKNNLite 检查工具，并交付两动作 CPU 对照结果。正式 C++ 公共核心、ROS worker/launch 和 P1 组合 smoke 尚未改造；未运行真板验证或 HPM。
