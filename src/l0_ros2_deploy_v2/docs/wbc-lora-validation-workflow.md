# WBC-LoRA 本地 ROS 2 验证与板端无 HPM smoke 工作流

> **2026-10-26 后续实施更新：** 本文保留为历史方案/证据；交付边界以用户最新指令为准。当前已开发正式 sim2real 链路中的 base_lora 加载及 HPM 隔离入口，C++ 仅涉及模型加载/推理，远端尚未验收。现行操作见 [正式部署同步与日志](wbc-lora-robot-sync-and-logs.md)，不再执行本文旧的独立 smoke 步骤。

日期：2026-10-26。状态：验证架构及实施要求已确定，相关后端改造和 smoke 入口待实现，本文不代表测试已执行。

关联：[部署方案](wbc-lora-deployment-plan.md)、[可行性审阅](wbc-lora-feasibility-review.md)、[现有部署流程](deploy-workflow.md)。

文件/函数级修改范围、参数传递和批次依赖见[开发规划](wbc-lora-development-plan.md)。P0 离线代码及 CPU 结果已交付；真板由用户手动执行，入口与命令见 [P0 交付](wbc-lora-p0-handoff.md)。本文所述正式 C++ 组合 smoke 及 ROS 集成仍待 P1 之后实现。

用户约束：AMP 保留；本地 sim2mujoco 尽可能复用实际加载流程；主要 smoke 在机器人 RK3588 上执行且不启动 HPM 电机通信节点；只用特定状态下的 policy/reference 输入输出对照验证模型。最终 policy 和 reference_motion 均部署为 FP16 RKNN，接受合理的 FP16 误差。

## 1. 决定：保留轻量的本地同源测试，板端模型 smoke 优先

**本地 ROS 2 开发有价值，应当做，但复用加载和控制逻辑，不追求在本地运行 RKNN。** 本地后端选择项目已有的 OpenVINO CPU 路径，读取同源 ONNX；板端使用 RKNN。ONNX Runtime 继续作为离线对照工具，不额外扩展为第三条正式 WBC ROS 运行链。

理由是动作映射错误、参数包不匹配、reference 绑定错误、历史重置、准备/取消竞态和 ROS 2 交接问题，不依赖 NPU 就可以在本地发现。共用这些代码，才能让本地测试覆盖板端要运行的加载流程。已有 OpenVINO 依赖和两条 launch 链可以复用，将后端差异限定在推理适配层，避免维护两套控制器。

本地无法验证 RKNN 编译、设备布局、NPU 实际精度与性能；增加大量本地仿真测试也不能补足这部分证据。因此先以至少两个动作完成板端动态参数及 FP16 smoke，再补齐本地完整 sim2mujoco 集成。若板端单图路线被 SDK 阻断，避免先投入大量本地仿真改造。

| 路径 | 推理后端与资产 | 主要价值 | 不作为哪项结论的依据 |
| --- | --- | --- | --- |
| 本地 ROS 2 + sim2mujoco | OpenVINO CPU，动态 policy/reference ONNX | 加载、动作切换、取消、状态机、历史及闭环接线；观察动作是否连续 | RKNN 精度、NPU 支持及 20 ms 性能 |
| RK3588 无 HPM 模型 smoke | 正式 FP16 policy/reference RKNN，同一部署参数包 | 指定输入上的精度、真实加载/切换、运行库及模型耗时 | 物理运动效果、稳定性和底层闭环性能 |
| 最终运动验收 | 后续按既有实机部署流程进行 | 运动与整机闭环 | 不属于本轮无 HPM smoke |

## 2. 尽可能相同的加载流程，具体共用哪些代码

本地与板端使用相同的动作目录规范、action ID、manifest 和 `weights.bin`、参数校验、适配器管理、请求代次/取消、动作与 reference 绑定、399/240 维输入组装、历史重置、动作缩放及交接协议。manifest 同时记录对应 ONNX 与 RKNN 的版本/哈希，便于确认是同一次导出。

仅在推理调用边界区分 OpenVINO 与 RKNN：按后端选择模型文件、核对 I/O 属性并执行。共用的参数包可保留 FP16 数据，本地在非实时准备阶段转换成执行器需要的类型；板端同样按查询到的 tensor 类型处理。不能让本地加载训练 `.pt`、板端加载另一套参数格式而各自实现完整动作加载器。

同一动作的 ONNX/RKNN 及其 reference、配置来自同一批导出。不能用动作名相同但帧数或轨迹不同的旧资产替代。两端显式使用同一个 `robot_type` 和所选动作子集；当前仿真默认为 l0_v3，不能依赖该默认值代替本批 l0_v5 配置。

两种运行模式显式记录实际后端。板端 smoke/部署指定 RKNN，缺失资产或后端不可用时失败，不能自动退回 CPU。当前环境静态检查为 x86_64、安装 ROS 2 Humble，Python 能发现 OpenVINO/ONNX Runtime；这不代替 C++ 构建与动态 WBC 图的本地运行验证。

当前代码存在以下明确的改造点，因此“启动已有 sim2mujoco 即完成 LoRA 测试”目前不成立：

- `legged_deploy/CMakeLists.txt` 只在 RKNN 构建时加入 WBC 源文件；`rl_controller_wbc.h` 也直接限定 RKNN，并持有 `RknnModel`。
- `one_start.launch.py` 虽被仿真和实机共同调用，但 WBC 资产检查写死为 `.rknn`。
- 基础 AMP 已有 OpenVINO 分支，可以复用其依赖与调用方式，但 WBC 多输入、reference 多输出及动态参数仍需接入。

实现上只抽取必要的模型执行接口和 WBC 公共输入/加载逻辑，不另写一套 Python 仿真控制器，也不建立通用的多后端框架。

## 3. 建议执行顺序

1. 使用现有部分训练结果选择至少两个实际存在的动作，例如 `L0_Hugme` 与 `L0_act_cute_1`；固定 Base、完整 adapter 和各自 reference/config。
2. 离线导出动态参数 ONNX 及统一参数包，对照原 checkpoint 确认完整 A/B、bias、LayerNorm 和归一化语义。生成固定输入及期望输出数据。
3. 先在目标 RK3588 验证真实参数化 block；使用同一个 Context 切换两个动作参数，确认目标路线能运行。
4. 转换共享 policy 和各动作 reference 为 FP16 RKNN，记录实际构建精度及 I/O 类型；将模型、参数包及同一份对照数据部署到板端。
5. 执行无 HPM 的 reference、policy 独立对照，再执行两者组合对照；至少完成 `A → B → A`，随后覆盖现有子集。
6. 板端模型路线通过后，完成 OpenVINO WBC 执行适配，在本地 ROS 2 先验证同源加载，再通过 sim2mujoco 检查 `AMP → A → AMP → B → AMP → A` 及取消/故障路径。
7. 用最终 C++ RKNN 路径补测稳定周期、冷加载和切换耗时。模型 smoke 完成不等于已完成带电机的整机验收。

板端早期验证可以先直接调用纯模型执行路径；本地 ROS 2 的少量公共加载开发可穿插进行，但不能把完整仿真改造作为首次板端 smoke 的前提。

## 4. 板端主要 smoke：独立推理入口，无 HPM 依赖

推荐增加独立的模型 smoke 可执行程序，作为 `legged_deploy` 的测试入口构建和安装，直接复用正式的模型执行器、adapter 加载器、reference 查询和 WBC 输入组装代码。它读取测试数据并单步调用模型，不建立实际机器人硬件层。入口名称在实现时确定，当前没有已实现的 smoke 命令。

该入口不调用 `sim2real.launch.py`、`sequential_launch_real.sh` 或 `one_start.launch.py`，不创建 `LeggedReal/topic_hardware_adapter`，不启动 HPM、motor arbiter、AMP、手柄或真实传感器。它不创建电机 command/enable 发布接口，不依赖电机反馈或 motors_ready；输入来自对照数据，输出只用于比对及报告。

这是独立模型验证场景，而不是关闭正式部署的使能逻辑或绕过就绪检查。当前 sim2real 会创建硬件适配器并等待 HPM/电机就绪，直接用它做本次 smoke 不合适。

如需要额外验证板端 ROS 消息接线，可再增加纯回放、WBC worker、结果收集的隔离运行模式，输入话题使用 smoke 命名空间；它仍不包含硬件适配器。主要数值 smoke 保持确定性单步，避免 ROS 调度导致历史、参考帧和观测配对不一致。

### 4.1 reference 独立对照

输入明确的 `time_step`，对照 FP32 ONNX 的全部 6 组输出：`joint_pos`、`joint_vel`、`body_pos_w`、`body_quat_w`、`body_lin_vel_w` 和 `body_ang_vel_w`。

覆盖首帧、中间帧、末帧及越界钳位；小型参考动作可直接沿用已有转换器的全帧检查。核对帧数、关节/body 顺序、四元数 wxyz 及有限数值，不能仅看某一关节是否接近。

### 4.2 policy 独立对照

给同一个共享模型输入固定的 `policy[399]`、`command_short[240]` 及选定 adapter，比较 `action[21]` 和缩放后的 `q_target`。两端都使用同一份 command 数据，先将 reference 的误差排除，便于定位 policy 转换或参数切换问题。

选择真实记录或可追溯的代表状态，包含静止、参考首/中/末附近及若干运动状态；至少覆盖现有转换验证需要的 20 组输入。随机输入仅作为补充。按 `A → B → A` 验证加载结果分别对应各动作，返回 A 后可复现同输入输出；不重建共享 policy Context。

### 4.3 reference → command → policy 组合对照

用相同机器人状态、参考帧索引、朝向锚点、历史和 previous actions 分别运行 CPU 对照链及板端链。此时 CPU 使用 ONNX reference，板端使用 FP16 RKNN reference，均经同源输入组装再推理 policy，比较中间 command、action 和 q_target。

这是固定输入下的开环数值对照，不让模型输出驱动电机，也不让两条链各自反馈自己的输出再要求逐帧精确对齐。previous actions 和历史由同一份测试数据明确给定，避免把闭环误差积累当作格式转换错误。

## 5. 对照数据和结果

数据优先来自已有训练评估或本地仿真采样，保存原始输入而非仅保存最终目标。独立 policy 测试可以直接读取现有 `--verification-data` 风格的 NPZ，再增加动作和来源元信息。

| 数据 | 用途 |
| --- | --- |
| Base、ONNX、RKNN、adapter、reference、配置和数据哈希 | 确认版本及成套关系 |
| case/action ID、参考帧索引、joint_names、机器人姿态/角速度、关节 q/dq | 定位具体状态及输入顺序 |
| 明确的历史、3 帧 previous actions、朝向锚点及输入组装版本 | 确保 399/240 输入可复现 |
| 固定 policy/command 输入、reference 6 组期望值、action/q_target 期望值 | 支持独立及组合对照 |

期望值从可信 FP32 模型生成，并先与对应最终 checkpoint 核对；不能由待测 RKNN 自己生成期望值。离线数据的旧时间戳只作为来源记录。本地 ROS 回放进入现有 worker 时需产生有效、递增的新时间戳，不得因回放旧包而触发现有观测时效拒绝。

报告至少保存每模型/动作的实际后端、SDK/driver、构建精度、I/O 类型、误差、失败 case 和耗时。准备、首次推理及稳定周期分别记录；带调试/性能采集的结果不混同于正式时延结果。

## 6. FP16 转换与适度精度要求

**最终共享 policy 和各 reference_motion 均为 FP16 RKNN；源 ONNX 和 CPU 对照保留 FP32。** 不需要先把整个 ONNX 的所有 tensor 强制改成 FP16；由固定版本的 RKNN 转换工具完成模型内部浮点精度转换，保持现有非 INT8 路径 `do_quantization=False`，在配置、构建日志和模型信息中显式确认 FP16。

Rockchip 的[官方转换示例](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov8/python/convert.py)以浮点构建分支关闭量化；本项目应固定实际 Toolkit 版本及浮点配置，而不是把“未启用 INT8”误写成 FP32 RKNN。policy 与 reference 必须分别留存转换及验证记录。

FP16 模型内部计算与外部 I/O 类型分别处理。现有 C++ 封装使用 FP32 输入及 `want_float=1` 的输出接口，可在查询和核对模型属性后继续使用，由 Runtime 做必要转换；不能把 float32 指针按 FP16 字节解释。采用设备原生 FP16 缓冲时，则按实际 dtype、布局和 stride 明确转换及同步。此接口语义可参照[官方 Runtime 头文件](https://github.com/airockchip/rknn-toolkit2/blob/v2.3.2/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h)。

reference 的帧索引、Clip/Cast/Gather 等控制语义必须保持正确。FP16 模型不要求帧索引也强制使用 half；尤其长动作的整数索引不能因 half 舍入而查错帧。这属于功能正确性检查，不是追求更小的浮点误差。

接受合理的 FP16 误差，不要求逐位相同或 CPU FP32 级容差。初期沿用当前数量级：policy 最大绝对误差 0.05，reference 最大绝对误差 0.02；报告均值/最大值及 q_target 的弧度误差，余弦相似度作辅助指标，近零向量不只凭余弦判错。参数应可配置、实际值写入报告，不为消除末位差异反复调精度。

注意当前 reference 转换脚本声明 `--max-abs-error` 默认 0.005，但实际比较使用硬编码 0.02。实施时统一为明确的参数来源并打印实际阈值，不声称命令行阈值已生效。结构、参数错配、非有限输出、明显偏差和查错帧必须报错，不因允许 FP16 舍入而忽略。

精度和性能分开判定。模型 smoke 先回答加载和数值是否正确，再用正式 C++ 路径测完整 reference/input/policy 周期是否满足现有 20 ms；未通过时如实报告。无 HPM 的模型周期结果不代表带传感器和电机通信的整机调度已经验收。

## 7. 相对部署方案增加的改造范围

| 落点 | 新增要求 |
| --- | --- |
| WBC 公共代码 | 共用动作解析、完整 adapter 加载、reference 绑定、历史及输入组装；薄执行接口适配 OpenVINO/RKNN |
| `legged_deploy/CMakeLists.txt`、WBC 头文件及源文件 | 将公共 WBC 从 RKNN-only 编译限制中分离；本地 OpenVINO 与板端 RKNN 选择各自执行器，不改变 AMP 原有后端 |
| `one_start.launch.py`、`sim2mujoco.launch.py`、`sequential_launch_sim.sh` 及实机参数链 | 统一配置传递、显式后端及按格式检查资产；保持原仿真/实机启动顺序，复用同一 WBC worker 协议 |
| policy/reference 转换工具 | 明确两者 FP16；保留 FP32 oracle；记录 I/O 类型、构建版本和统一的验证阈值 |
| 板端 smoke 入口 | 独立的纯模型程序，调用正式加载器/执行器及组装逻辑；不实例化任何硬件控制对象或电机发布接口 |
| 本地 ROS 2 回放与 sim2mujoco | 验证同源加载、准备/取消/激活/返回；回放保持可控状态和有效时间戳，不复制模型加载实现 |

这些是后续实施要求，本轮只更新文档，没有新建 smoke 程序、数据文件或执行 ROS/硬件测试。
