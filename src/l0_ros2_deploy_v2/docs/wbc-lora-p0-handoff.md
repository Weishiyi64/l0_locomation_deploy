# P0 交付：动态图、完整 LoRA 参数包与手动板端验证

> **2026-10-26 后续实施更新：** 本文保留为历史方案/证据；交付边界以用户最新指令为准。当前已开发正式 sim2real 链路中的 base_lora 加载及 HPM 隔离入口，C++ 仅涉及模型加载/推理，远端尚未验收。现行操作见 [正式部署同步与日志](wbc-lora-robot-sync-and-logs.md)，不再执行本文旧的独立 smoke 步骤。

> **2026-10-26 板端问题交接：** Python 验证与正式 C++ 部署加载了不同的 Runtime；真板验收尚未通过。后续统一复用既有部署环境与 C++ 加载链路，修复尚未实现。请先阅读 [Runtime 问题与开发交接](wbc-lora-runtime-handoff.md)；旧 Python 目标验证 / Lite2 smoke 不再作为当前操作入口。

日期：2026-10-26。**P0 离线开发及 CPU 验证完成；RKNN 转换、真板数值和算子执行验收由用户手动完成，当前不标记为已通过。** P1 的正式 C++ 公共 WBC 核心、ROS worker 和无 HPM C++ smoke 尚未开发。

关联：[开发规划](wbc-lora-development-plan.md)、[部署方案](wbc-lora-deployment-plan.md)、[验证工作流](wbc-lora-validation-workflow.md)。

## 1. 实际交付

交付目录位于工作空间根目录：/home/luvbot/SC/l0_deploy_v2_ws/deliveries/wbc_lora_p0_20261026/。

| 内容 | 说明 |
| --- | --- |
| base_lora.onnx、base_manifest.json | 一份共享基础图；2 个观测 + 31 个动态参数输入 |
| adapters/ACTION_ID/manifest.json、weights.bin | 完整 A/B、bias、LayerNorm 参数；little-endian FP16，64 字节对齐；包含 Base/布局/归一化和文件校验 |
| actions/ACTION_ID/ | 对应 reference ONNX 及 RobotCfg/WbcCfg；现有部署资产保持原样 |
| probe_encoder/decoder/output.onnx 及 manifest | 639→1024、1423→1024、1024→21 真实维度探针，包含适用的 bias/LayerNorm/Mish |
| verification_*.npz、probe_*_inputs/expected.npz | 固定 FP32 输入及最终 checkpoint 期望值，包含校验信息 |
| validation_report.json、qa_report.json、DELIVERY_STATUS.json | 实际 CPU 结果、检查范围及未执行项 |
| tools/ | 7 个完整工具脚本，可随整个目录复制 |
| code_changes.patch、MANIFEST.sha256 | 相对本次开发前文件的变更及交付校验清单 |

首批为 L0_Hugme、L0_act_cute_1。导出器支持选择其他实际存在的动作，不要求原始索引的 80 个动作全部就绪。本次没有生成 .rknn；FP16 参数包验证不等于 FP16 RKNN 验证。

## 2. 已实现的代码

- [export_wbc_lora.py](../export_wbc_lora.py)：在原 Base ONNX 上参数化 31 个张量；导出与 CPU 对照通过后原子发布新目录，拒绝覆盖非空目录。
- [wbc_lora_assets.py](../wbc_lora_assets.py)：检查 ABI、路径、Base/归一化、形状、dtype、字节区间、校验值及有限数据；兼容 Python 3.10。
- [prepare_wbc_deploy.py](../prepare_wbc_deploy.py)：新增 --mode lora --actions，旧静态入口仍为默认。
- [policy 转换器](../convert_onnx_to_rknn_rk3588_wbc_policy.py)：新增 --manifest，支持共享图及探针；保留两输入静态 ABI，指定 FP16 非 INT8 构建。
- [reference 转换器](../convert_onnx_to_rknn_rk3588_wbc_reference.py)：指定 FP16；实际 0.02 容差改由 --max-abs-error 生效，保留 Gather 兼容改写。
- [批量转换器](../batch_convert_onnx_to_rknn_rk3588.py)：--manifest 模式只转换一次共享图和所选 reference；支持 dry-run，缓存跳过检查来源/模型/构建版本和参数验证记录。
- [verify_wbc_lora_rknn.py](../verify_wbc_lora_rknn.py)：已实现的旧 RKNNLite 检查器，支持固定期望值及同一 Context 的 A→B→A；实际板端未安装 Lite2，当前标准流程暂停使用此入口，后续改由同部署链路的 C++ 工具验证。

成功转换并验证后生成 MODEL.rknn.json，记录源/输出哈希、FP16 配置、Toolkit 版本、验证位置和误差。--target rk3588 表示使用可连接目标，--device-id 选择具体设备；不传 target 时用 Toolkit 模拟器，报告不会将其记为真板。

## 3. 已完成的验证

每动作 33 组固定转换样本，按 Base 归一化范围生成；**不是实机状态或新 rollout**。

| 动作 | 动态 FP32 图相对最终 checkpoint 最大绝对误差 | FP16 参数包相对最终 checkpoint 最大绝对误差 |
| --- | ---: | ---: |
| L0_Hugme | 8.64e-7 | 6.68e-4 |
| L0_act_cute_1 | 1.16e-6 | 4.26e-4 |

零 A/B 配合 Base bias/LayerNorm 恢复 Base 的最大误差为 8.34e-7；同一 ONNX Session 的 A→B→A 可复现。三个探针的 FP16 参数包最大误差分别约为 0.00221、0.000862、0.0000126。

系统 Python 3.10 的 15 项错误资产拒绝检查通过，包括越界路径、错误 Base、缺失张量、错误维度、重叠区间、错误校验值及动态模型误用旧 ABI；非有限输出拒绝检查通过。转换 dry-run、旧静态 ABI、7 个脚本语法及检查器 CPU 接线检查通过。接线检查使用 CPU 替身，不计作 RKNN/NPU 结果。

## 4. 用户手动执行：当前状态

继续采用“同步 ONNX、LoRA 和转换工具 → 真机生成 FP16 RKNN → 真机手动 smoke”的总体顺序。源包同步和日志拉取见 [机器人同步与板端转换说明](wbc-lora-robot-sync-and-logs.md)。旧 Python 目标验证已暴露 Runtime 加载路径问题，原 Lite2 smoke 也不匹配实际环境；修复及后续验收以 [Runtime 问题与开发交接](wbc-lora-runtime-handoff.md) 为准。

源包包括 base_lora.onnx、三个探针、两动作 reference ONNX/配置、FP16 adapters、固定输入/期望值及转换工具；同步阶段不要求本地有 .rknn 或 .rknn.json。使用 BOARD_SOURCE_MANIFEST.sha256 校验全部 29 个源文件，约 35.04 MiB。

原始两输入 base/policy.onnx 是导出来源；动态验证使用两个观测加 31 个参数输入的 base_lora.onnx。Base 固定权重已包含其中，无需把训练 base_ckpt.pt 放入板端。

用户已手动尝试板端转换，反馈加载失败；不能继续把它描述为“尚未尝试”，也不能记为通过。下一步实现 Toolkit2 候选导出与同部署链路 C++ 验证，沿用现有 colcon build 和 source install/setup.bash。当前没有可直接执行的新 C++ smoke 命令。

所有板端 smoke 由用户手动控制，只运行模型，不启动 ROS、HPM 或电机通信。数值通过也不能自动证明没有 CPU 回退或完整控制周期达标。

## 5. 阶段边界

上文 CPU 检查属于原离线交付记录，不代表远程环境。实际板端已安装 Toolkit2 2.3.2，用户已手动运行转换并反馈 Runtime 加载失败；详见最新交接文档。本次文档更新未执行板端模型或启动 ROS、MuJoCo、HPM。真板验收仍待完成，不以 CPU 结果宣布 NPU 支持或完整 20 ms 周期达标。

本阶段不改 C++ worker、launch 默认值、ROS 接口、增益、映射或原 deploy_workflow。先交付 P0 代码与离线结果，不将 P1 正式运行接入标记为已开始。
