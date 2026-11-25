# WBC-LoRA 动态加载方案可行性审阅

审阅日期：2026-10-26。

审阅对象：[原始总体方案归档](wbc-lora-proposal-original.md)，原路径为 `/home/luvbot/下载/RK3588_WBC_LoRA_动态加载部署方案.md`。

训练资产：`/home/luvbot/SC/L0_V5_WBC10000_LoRA80_20261026训练结果/`。

范围已由用户确认：AMP 继续负责基础行走，emotion 动作共用一份 WBC 基础策略并动态加载适配器。本轮核对方案可行性，部署代码只用作约束依据；未修改原方案、既有 `deploy_workflow`、部署代码或启动配置。

归档认定：初步可行，可进入验证开发；板端执行与实时性能尚未验收。[修订后的部署方案](wbc-lora-deployment-plan.md)汇总实施要求，[现有部署流程](deploy-workflow.md)记录改造前的运行机制。

后续验证要求已确定：policy 和 reference_motion 均部署为 FP16 RKNN；主要数值 smoke 在 RK3588 上独立执行，不启动 HPM/硬件适配器；本地使用 OpenVINO/ROS 2 和 sim2mujoco 验证同源加载及切换。见[验证工作流与理由](wbc-lora-validation-workflow.md)。本报告下述静态检查不代表这些测试已经执行。

## 1. 审阅结论

**“一份常驻基础策略图 + 按动作动态加载完整适配参数”的方向在模型表达及现有控制架构上成立，但原方案不能原样实施。修正参数范围后，RK3588 单图 NPU 执行能力及 20 ms 周期仍是必须实测的门槛，当前不能认定已经通过。**

本次已能确定的关键问题是：原方案把 bias 固定在基础图中，只将 A/B 作为动态参数；实际训练同时改变了 Linear bias 和 LayerNorm 参数。按原文实现会遗漏训练结果，无法作为对应 emotion 策略的等价部署。

| 维度 | 当前判断 |
| --- | --- |
| 共享基础权重 | 可行；本地适配器声明同一 Base，实查 Base SHA256 与交付值一致 |
| 只切换 A/B，固定其余参数 | 不等价；本地 10 个动作的 7 个 Linear bias、5 层 LayerNorm 的 gamma/beta 均与 Base 不同 |
| 单图显式输入完整动作参数 | 数学上可表达；包含动态 MatMul、动态 bias 和动态 LayerNorm 参数，需 RKNN 编译及板端验证 |
| 沿用 AMP/WBC 控制权交接 | 可复用；动态加载应发生在 WBC 激活前，AMP 在此期间继续控制 |
| 只用 LoRA 包确定动作 | 不完整；必须同时绑定该动作的参考轨迹、配置及时间语义 |
| 当前板端性能是否合格 | 未验证；不能由 LoRA 参数比例推导时延 |
| 轻量包的部分动作能否支持开发 | 足够；本地 10 个 affectionate 动作可支持适配器切换及集成验证 |

本地 `emotion_index.json` 保留 80 条原始索引，但实际交付子集为 10 套 adapter、最终 checkpoint、NPZ 和 reference ONNX。用户确认这是轻量化交付的预期范围，本审阅不将其作为阻塞。验证时只注册实际存在的动作；索引其余条目不代表本轮应加载的资产。

## 2. 原方案必须修正的参数语义

### 2.1 共享的是基础矩阵 W，bias 和 LayerNorm 随动作切换

检查依据：

- `integration_reference/lora.py:12`：LoRALinear 的真实计算为基础 Linear 加低秩残差。
- 同文件 `inject_lora()`：Linear bias、LayerNorm、actor std 可独立训练。
- `lora/affectionate/L0_Hugme/training_config.yaml:1080`：rank=16、alpha=16，且三个训练开关均开启。
- 对本地 10 个 `lorackpt_2000.pt` 与 Base 的张量逐项比较：每个动作都有上述 bias、LayerNorm 参数变化；不是仅依据配置推测。

适配器应表达：

```text
linear_k(x) = x @ W_base.T + b_k
              + (alpha / rank) * ((x @ A_k.T) @ B_k.T)

在真实网络存在 LayerNorm 的位置：
LayerNorm(linear_k(x), gamma_k, beta_k, eps=1e-5)
随后按原图执行 Mish；并保留 encoder/decoder 的拼接关系。
```

因此原方案第 3 节的固定 `b_l` 应改为动作对应的 `b_{l,k}`；第 4 节参数包除了 14 个 A/B 张量，还必须包含 7 个 bias 和 10 个 LayerNorm gamma/beta 张量。它们是训练后保存的替换值，不能误当成相对 Base 的增量重复相加。

LayerNorm 的动作相关仿射部分可以在同一个图内显式表示为 `u = LayerNorm_without_affine(x)`、`y = gamma_k * u + beta_k`，核心归一化的 epsilon 及轴保持原样。这是在单图内等价表达已有训练计算，可避免把“LayerNormalization 算子直接接受动态 gamma/beta”作为唯一实现方式；动态 Mul/Add、编译器融合及有限精度误差仍需在阶段 A 中验证。

`actor_std` 在 10 个适配器中也发生变化，但当前 ONNX 输出只有确定性的 `action`，其图不使用 std。因此确定性均值推理可不将 std 接入 NPU 前向；参数导出规范必须明确这一点。若改为随机采样动作，需要另行保留并验证其语义。

### 2.2 归一化必须校验绑定，本地子集可以共享

Base ONNX 已在图内执行 `policy` 和 `command_short` 的归一化。不能再在 C++ 端无条件归一化一次，也不能假定任意 adapter 都能使用任意 Base 的统计。

本次直接比较本地 10 个 adapter 与 Base：它们的全部 `vecnorms` 张量均完全一致。训练配置也记录 `freeze_rollout_vecnorm: true`。因此，对本次验证集，可以保留共享的归一化常量，减少动态输入和算子改造。

方案应写成明确约束：manifest 记录归一化版本/哈希；仅接受与共享图统计一致的 adapter。以后若接入不同统计的 adapter，必须将对应均值/标准差也参数化，或拒绝不兼容的包，不能静默沿用 Base 统计。

交付参考实现的统计换算为 `mean=sum/count`，`std=sqrt(max(ssq/count-mean², 1e-5))`。如需参数化，应在离线导出阶段生成 mean/std，避免在控制周期内解析训练统计。

### 2.3 参数包不能直接等同于训练 .pt

`lorackpt_2000.pt` 的格式为 `mimic_lite_lora_adapter_v2`，包含 actor、vecnorms、配置快照及来源等；它不是 RKNN 可直接消费的 tensor buffer。

离线导出器需将推理必需参数转换成确定的二进制布局，并记录训练 Base SHA256、动态 ONNX/RKNN 的哈希或版本、层名、形状、rank、alpha、dtype、端序、偏移、大小和归一化标识。板端只读此部署格式，不在 ROS 2 控制回调中解释 PyTorch checkpoint。

本批 Base checkpoint 的 SHA256 为：

```text
f02d4845019346b577fb0babceb8c288e1b5e1694f7888907ae8cfd63d7e4142
```

## 3. 原方案的网络近似不能用作实际导出依据

静态读取 `base/policy.onnx`，并核对 adapter 层名/矩阵形状，实际 7 层 Linear 如下。表中尺寸采用 PyTorch 的 `[out, in]`。

| 序号 | actor 层名 | W 形状 | rank=16 的 A / B 形状 |
| --- | --- | --- | --- |
| 1 | `module.0.module.1.module.0.0` | `[1024,639]` | `[16,639]` / `[1024,16]` |
| 2 | `module.0.module.1.module.0.3` | `[1024,1024]` | `[16,1024]` / `[1024,16]` |
| 3 | `module.0.module.1.module.1` | `[1024,1024]` | `[16,1024]` / `[1024,16]` |
| 4 | `module.0.module.3.module.0` | `[1024,1423]` | `[16,1423]` / `[1024,16]` |
| 5 | `module.0.module.3.module.3` | `[1024,1024]` | `[16,1024]` / `[1024,16]` |
| 6 | `module.0.module.3.module.6` | `[1024,1024]` | `[16,1024]` / `[1024,16]` |
| 7 | `module.0.module.4.module.actor_mean` | `[21,1024]` | `[16,1024]` / `[21,16]` |

原始输入为 `policy[399]`、`command_short[240]`，先分别归一化，再拼接为 639 维；encoder 输出与原始归一化后的 399 维 policy 再拼接为 1423 维进入 decoder。图中共有 5 个 LayerNormalization、5 个 Mish；不能给每个 Linear 机械追加同一种激活。

本批 rank=alpha=16，scale=1，均为固定形状。这里的“动态加载”是运行时改变参数值，不要求动态 tensor shape，也不要求启用 RKNN 的 `dynamic_input` 形状功能。

按真实形状，基础 Linear 权重为 6,327,296 个元素，A/B 共 213,552 个元素。A/B 的 FP16 原始体积为 417.09 KiB；加上 7 个 bias 和 5 层 LayerNorm 参数，确定性 actor 适配参数约 449.13 KiB，不含 manifest、对齐及可选归一化数据。最终 policy/reference RKNN 已明确使用 FP16；上述只是适配器容量估算，文件及外部 I/O dtype 仍须显式处理，保留 FP32 对照并接受合理的 FP16 舍入误差。

A/B 理论增量约为基础 Linear MAC 的 3.375%，不包含 LayerNorm、激活、输入传输等；不能用它推导真实 NPU 时延。原方案中的七个 1024×1024 和存储表只宜保留为一般示例。

## 4. RK3588 可行性的真正门槛

当前仓库自带 `librknnrt.so` 的可读版本字符串为 `2.3.2 (429f97ae6b@2025-04-09T09:09:27)`；这不能证明目标设备加载了同一运行库，板端还需查询实际 SDK/driver 版本。

Rockchip v2.3.2 的[官方 ONNX 算子表](https://github.com/airockchip/rknn-toolkit2/blob/v2.3.2/doc/RKNNToolKit2_OP_Support-2.3.2.md)列出 MatMul 和 LayerNormalization，并说明还有编译器限制。该列表不足以证明本方案中“运行时 A/B + 动态 LayerNorm gamma/beta”的完整图能在目标 NPU 上执行。因此可行性仍须通过目标版本的编译、数值和设备执行验证。

方案第 7 节阶段 A 应扩展为验证**真实参数化 block**，而不只是一个等宽 Linear：

1. 按本批 rank=16，覆盖 639→1024、1423→1024、1024→21 等实际形状的动态 A/B；保持基础 W 为常量。
2. 包含随动作变化的 bias 和 LayerNorm gamma/beta，并保持真实激活位置与 epsilon；LayerNorm 可采用前述核心归一化与动态仿射分离的单图表达。若采用打包输入，Slice/Reshape/Transpose 等拆包操作也须包含在验证图中。
3. 在同一个 RKNN Context 上交替输入至少两个真实 adapter 的参数，核对输出分别对应各自参考值；同时检查全零 A/B、Base bias/LayerNorm 时能恢复 Base 结果。
4. 核对输入确实仍是运行时数据，未被常量折叠；从转换日志、执行明细及设备报告确认新增计算的后端。
5. 单 block 通过后仍需完整图验证；小图通过不能排除全图布局、工作区、编译优化及多输入限制。

逐张量传入时，14 个 A/B + 7 个 bias + 10 个 LayerNorm 参数，再加两个观测输入，共 33 个输入；若后续归一化也参数化，还会增加输入。可以评估固定布局的打包参数输入，但不能预先假定打包及拆包在目标 NPU 上没有额外限制或开销。

原方案所述预分配双缓冲、显式内存同步和禁止并发覆盖方向正确。仓库头文件及[官方 v2.3.2 Runtime 头文件](https://github.com/airockchip/rknn-toolkit2/blob/v2.3.2/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h)提供内存绑定和同步相关 API，但接口存在不等于当前推理封装已经采用，也不保证任意缓冲切换没有拷贝。设备布局、stride、dtype 和同步方式仍按实际模型属性及调用模式验证。

当前转换器明确使用 `rknn.build(do_quantization=False)`。原方案“恢复当前 INT8/量化配置”的描述应改为沿用现有**非量化构建路径**；INT8 或混合精度属于额外验证范围，不是本项目既有 WBC 的事实。

## 5. 参考动作与适配器必须成套生效

现有 WBC 不是仅输入机器人状态就能确定动作的网络。它每周期还通过 reference RKNN 查询目标动作，在 8 个参考偏移处组装 `command_short[240]`。

因此共享 policy 不意味着可以删掉每个动作的 reference 和配置。建议方案明确一个动作条目绑定以下内容：

```text
action_id
  → 共享 Base / 图 ABI 版本
  → 完整 adapter 参数包 / 归一化标识
  → 对应 reference_motion / 帧数 / 50 Hz 时序
  → joint_order / default_joint_angles / action_scale / clip_actions / kps / kds
```

同名动作也不保证新旧参考资产相同。例如旧部署 `L0_excited_1` 配置为 207 帧，新训练索引中该动作是 237 帧；本地轻量包没有该动作实体，因此后者是索引记录，不能直接拿旧参考替代。当前可验证的 `L0_Hugme` reference 模型为 279 帧、50 Hz，接口为 `time_step[1,1]` 及 6 组参考输出，与现有查询接口一致。

“一个 Context”应限定为一个共享 **policy Context**。若继续复用当前 reference RKNN 方案，reference 仍有自己的 Context。保留各 reference 或按需准备均可，但需单独统计它们的驻留内存、加载时延和周期查询耗时；这不违背共享基础策略的目标。

最小集成验证可以保留选定子集 reference 的现有预加载方式，只让适配参数动态读取。reference 动态加载可后续按测量结果决定，避免把它与 LoRA 动态加载混为同一个必要条件。

## 6. 与现有控制代码结合后的约束

### 6.1 动态加载阶段与激活阶段必须分开

现有流程已经是 AMP 控制 → WBC 准备 → 有效首帧 → 平滑进入 WBC → 完成后返回 AMP，可以继续沿用。

适配器文件读取、哈希、布局转换及备用缓冲准备应在非实时加载线程完成。随后在单一推理所有者掌握的安全边界，成套选择 adapter/reference/config，读取**新鲜**机器人观测，重置历史并预推理首帧，再完成原有激活交接。

不应在磁盘加载开始时就发布首帧或暂停 AMP，否则首帧可能在返回准备响应前过期。当前策略命令默认有效期 100 ms；控制器准备服务/等待输出的默认超时是 0.5 s。冷加载能否满足该准备预算须测量；若不能，应采用预准备/缓存或单独明确加载等待协议，不能把磁盘等待放入 20 ms 推理周期。

### 6.2 失败回退要符合本项目状态

当前执行中、交接中不会接受第二个 emotion 动作。保留这一约束即可满足动态加载：按需加载发生在动作激活前，而不是要求执行中即时切换 emotion。

准备新动作失败时，应保持 AMP 继续控制，并保留原有可用资源；这与“旧 emotion 继续执行”不是相同业务状态。WBC 已运行时出现过期输出、推理失败或传感器错误，仍走既有急停/故障路径，不能因 LoRA 失败而继续使用陈旧目标。

后台加载必须携带请求代次或取消令牌。超时、取消或急停后，迟到的加载完成不能重新提交旧动作；取消时不能长时间等待持有运行互斥锁的磁盘线程。

### 6.3 周期预算必须覆盖 reference 和数据传输

现有 `RLWbc::inferAndPublish()` 在完整采样/推理路径上检查 20 ms 预算，包含参考查询及 policy 推理。LoRA 方案应把输入绑定、同步、拆包及输出处理也计入，而不是只测新增 MatMul 的 NPU 时间。

当前 `RknnModel::run()` 每次提交全部输入，固定按 FP32 指针和字节数处理，并临时创建输入描述数组、由 Runtime 返回输出。即使 adapter 值未变化，也不能假定现在的封装只在切换时传输一次参数。

若声明实时路径无分配，还需处理现有输入描述、观测/参考向量等分配；仅新增一个预分配 AdapterManager 不会自动让整条现有路径达到该要求。低拷贝优化须在功能验证后按实际耗时决定，并保留现有 AMP、reference 和 planner 的调用兼容性。

## 7. 方案修订后的最小改造范围

下表是通过前述模型/平台验证后才进入的实现范围，用于说明落点，不是本轮已经执行的改动。

| 落点 | 必要内容及理由 |
| --- | --- |
| 离线动态图及适配器导出 | 按真实层名导出共享 W、动态 A/B/bias/LayerNorm；归一化采用已审计的共享值或受控参数化；不修改训练数学结构 |
| `prepare_wbc_deploy.py`、WBC policy 转换脚本及批量编排 | 原检查仅接受 2 输入静态策略；需增加动态参数 ABI/manifest 校验。转换一次共享 policy，逐动作导出参数及对应 reference，不继续逐动作生成完整 policy RKNN |
| `legged_deploy/include/rl_controller_wbc.h`、`src/rl_controller_wbc.cpp` | 将 `Motion::model_` 的逐动作 policy 所有权移到 worker 级；Motion 保留动作元数据/reference/adapter 标识；按需加载、取消和提交完整动作资源 |
| `legged_deploy/include/rknn_model.h`、`src/rknn_model.cpp` | 保留现有安全校验，并提供所需的 dtype/shape/布局查询与参数输入；当前仅按元素数、FP32 输入处理的接口不足以直接绑定 FP16 设备缓冲 |
| `legged_controllers/launch/one_start.launch.py` 及上游参数传递 | 文件预检查改为“共享 policy + 实际选定动作的 adapter/reference/config”；不再要求每个动作目录各有一份完整 `policy.rknn`。维持既有启动顺序 |
| `LeggedControllerWbc.cpp` / `PolicyMux.cpp` | 复用外部 action_id、状态校验及平滑交接；按准备阶段实测结果处理异步加载和过期请求。现有增益读取也要与最终动作配置对应 |

外部 `wbc_action_cmd`、反馈话题和下游电机接口不因共享模型而必然改变。现有 `ControlWbcMotion.srv` 已能以 `motion_id` 选择动作；若采用后台预准备后再走原 START，可继续沿用。只有决定新增公开加载/查询协议时，才需要另行版本化 ROS 接口，不应把新增服务作为当前方案成立的前提。

## 8. 可执行的可行性判定标准

| 验证层次 | 应取得的证据 | 判定 |
| --- | --- | --- |
| 参数完整性 | 用本地至少两套 adapter，覆盖 A/B、bias、LayerNorm、归一化约束；核对 Base 哈希和真实层形状 | 当前材料足够开展，不能仅比对 A/B |
| 离线等价性 | 动态图分别加载各动作完整参数，与各自最终 checkpoint 的确定性均值输出对齐；包含代表性观测与边界输入 | 未执行；修正导出后必须通过 |
| RKNN 单 block | 对真实形状和全部动态参数完成转换、同 Context 参数切换、板端数值及后端确认 | 未执行；决定单图路线能否继续 |
| RKNN 完整图 | 完整网络含归一化、拼接、LayerNorm/Mish、动态参数；检查精度、设备执行明细、内存及输入开销 | 未执行；不能用单层结果替代 |
| 现有 WBC 周期 | 相同 reference、SDK/driver、线程/NPU 条件下，完整周期及准备/切换分别测量；记录 P95/P99/max 及超时 | 未执行；完整周期须满足现有 20 ms 判定 |
| 控制集成 | `AMP → A → AMP → B → AMP → A`，并核对坏包、取消、迟到加载、急停、超时和重复切换 | 未执行；本地 10 个动作足够支持该验证 |

若单 block 因目标 SDK 的动态权重或动态 LayerNorm 路径受限而无法通过，应记录具体算子、形状、构建/运行日志与 SDK/driver 版本，再决定替代路线。不能自动改成 CPU LoRA、逐动作完整 RKNN 或删除已训练参数，并将其当作原方案已经实现。

## 9. 本轮证据范围

已执行：读取方案、部署源码及训练交付说明；静态读取 Base ONNX 拓扑/常量形状和 reference 元信息；读取本地 10 套 checkpoint 张量并比较额外 actor 参数及归一化；检查本地资产和轻量 ZIP 的目录索引；读取仓库 Runtime 二进制版本字符串；核对 Rockchip v2.3.2 官方算子表和接口声明。

未执行：新模型导出、RKNN 转换、NPU 推理、ROS 启动、仿真、训练或实机动作。本报告中的参数差异和形状是本轮直接检查结果；交付包已有 rollout/CPU 精度报告不等于动态 RKNN 部署已经验证。

部署代码的关键依据：

- [WBC Motion 资源所有权](../legged_deploy/include/rl_controller_wbc.h)：`Motion` 内各自持有 policy/reference 模型。
- [WBC 加载与推理](../legged_deploy/src/rl_controller_wbc.cpp)：逐动作预加载、2 输入校验、参考查询、首帧准备、时效与周期约束。
- [RKNN 推理封装](../legged_deploy/src/rknn_model.cpp)：输入提交、FP32 类型、Runtime 输出获取及有限数值检查。
- [现有转换器](../convert_onnx_to_rknn_rk3588_wbc_policy.py)：2 输入 ABI、Mish 兼容改写及非量化构建。
- [动作请求及交接](../legged_controllers/src/LeggedControllerWbc.cpp)：AMP 前置、取消确认、激活、返回及故障处理。
- [部署节点组织](../legged_controllers/launch/one_start.launch.py)：模型资产预检查和单 WBC worker 的启动方式。
