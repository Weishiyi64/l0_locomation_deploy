# WBC-LoRA 板端 Runtime 问题与开发交接

> **2026-10-26 后续实施更新：** 本文保留为历史方案/证据；交付边界以用户最新指令为准。当前已开发正式 sim2real 链路中的 base_lora 加载及 HPM 隔离入口，C++ 仅涉及模型加载/推理，远端尚未验收。现行操作见 [正式部署同步与日志](wbc-lora-robot-sync-and-logs.md)，不再执行本文旧的独立 smoke 步骤。

日期：2026-10-26。本文依据用户反馈的 RK3588 实际环境与日志，以及项目现有加载代码整理。它接替旧文档中的 Python 板端验证操作说明；原 deploy_workflow、正式部署启动顺序和底层方案不变。

## 1. 交接结论

**本次已定位到验证入口与正式部署使用了不同的 RKNN Runtime。当前错误发生在模型加载阶段，不能据此认定 ONNX 结构错误，也不能认定动态 LoRA 不可行。**

正式 C++ 部署解析到工作空间安装的 Runtime 2.3.2；失败的 Python Toolkit 验证实际加载 /usr/lib64/librknnrt.so，并报 Invalid RKNN model version 6。source install/setup.bash 后，这个差异仍然存在。更换 Python 解释器、重复 source、继续重跑全部转换，不是已经证实有效的修复。

下一步应沿用现有工作空间、构建和环境启动方式：Toolkit2 负责生成候选 RKNN，板端验证复用正式部署的 C++ RknnModel 和 Runtime。**这个修复尚未实现；当前交付为问题定位与明确的开发交接，不是“真板已通过”。**

原 Python 转换器将构建与目标验证绑定，暂不能作为当前环境的完整成功交付入口。旧 RKNNLite smoke 也不适用于已确认的板端环境。暂停照抄旧同步文档第 6、7 节继续测试；源包同步、文件校验和日志拉取仍可使用。

## 2. 已确认的远程证据

所有环境结论来自用户在远程执行的输出，不以开发机环境推断板端情况。

| 项目 | 实际值 / 证据 |
| --- | --- |
| SSH / 主机 | forlinx@192.168.5.2 / ok3588 |
| 工作空间 | /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws |
| 交付包 | 工作空间下 deliveries/wbc_lora_p0_20261026 |
| source 后 Python | /usr/bin/python3；3.10.12；prefix=/usr |
| 转换工具 | rknn-toolkit2 2.3.2 |
| 其他 Python 包 | onnx 1.16.1；onnxruntime 1.23.2；rknn-toolkit-lite2 未安装 |
| 正式可执行文件 | install/legged_deploy/lib/legged_deploy/rl_controller |
| 正式程序的 ldd 结果 | librknnrt.so → 工作空间 install/legged_deploy/lib/librknnrt.so |
| 上述工作空间 Runtime | 2.3.2，429f97ae6b@2025-04-09T09:09:27 |
| 失败 Python 的 LD_DEBUG 结果 | calling init: /usr/lib64/librknnrt.so |
| 失败位置 | rknn.init_runtime → rknn_init；Invalid RKNN model version 6 |
| 另行检查过的系统库 | /usr/lib/librknnrt.so 的版本字符串为 1.3.0，c193be371@2022-05-04T20:16:33 |

版本证据须精确区分：直接查出 1.3.0 的路径是 /usr/lib/librknnrt.so；失败进程实际加载的是 /usr/lib64/librknnrt.so。现有反馈未单独证明这两个路径指向同一文件，因此不将后者的精确版本写成已测值。已确认的是：Python 加载路径不同于正式部署，并且该 Runtime 拒绝当前模型格式。日志也不足以证明 SDK 源码是否写死了这个路径。

source 后的 LD_LIBRARY_PATH 已包含工作空间 install/legged_deploy/lib，因此不能再把问题简单归因于“没有 source”。rknn 模块位置显示 None 也不能推翻已经实际运行的 Toolkit2 编译日志。

关键日志位置：

~~~text
/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/smoke_logs/wbc_lora_p0/
├── 20261026_161724_vgci4S/
│   ├── convert_probe_encoder.log
│   ├── convert_probe_decoder.log
│   ├── convert_probe_output.log
│   ├── convert_policy.log
│   ├── convert_reference_L0_Hugme.log
│   └── convert_reference_L0_act_cute_1.log
└── runtime_trace_20261026_164650.log
~~~

六项转换均反馈 exit=1。已提供的 output probe 与 reference 详细日志显示：

~~~text
I rknn building done.
I target set by user is: rk3588
E Invalid RKNN model version 6
E rknn_init, load model failed!
E rknn-toolkit2 version: 2.3.2
~~~

因此，对这两份详细日志可确认“构建完成，目标加载失败，未进入数值验收”；其余项只有退出状态时，不外推每项内部构建均已完成。

output probe 编译表中的两次动态 exMatMul 为 FLOAT16 / NPU；reference 的 Cast、Clip、Gather 标为 CPU，Split 标为 NPU。这是编译阶段证据，不是板端执行成功或性能达标的证明，也不能把 reference 宣称为全 NPU 执行。

## 3. 正式部署与当前失败入口的区别

正常部署链路由既有流程启动 rl_controller（节点名 rl_controller_wbc）。它从模型路径确定同名 .rknn 文件，通过 C++ RknnModel 读取文件并调用 C API rknn_init。正式推理不通过 Python Toolkit 或 RKNNLite。

| 入口 | 模型加载 | 本次证据 |
| --- | --- | --- |
| 正常 C++ 部署 | RknnModel → C API → 工作空间安装的 Runtime | ldd 指向 install/legged_deploy/lib/librknnrt.so，版本 2.3.2 |
| 当前 Python 转换后的验证 | Toolkit2 init_runtime(target=rk3588) | 实际加载 /usr/lib64/librknnrt.so，模型版本拒绝 |
| 当前独立 Python smoke | verify_wbc_lora_rknn.py → RKNNLite | 板端未安装 Lite2；不是正式 C++ 加载链路 |

相关实现位置：

- legged_controllers/launch/one_start.launch.py：启动 legged_deploy 的 rl_controller。
- legged_deploy/src/rl_controller_wbc.cpp：初始化 policy/reference；现有 worker 对 policy 仍按两个输入检查。
- legged_deploy/src/rknn_model.cpp：.onnx 路径映射为 .rknn、读取文件、调用 RKNN C API。
- legged_deploy/CMakeLists.txt：选择 RKNN_LIBRARY，设置安装 RPATH，并安装 Runtime。
- 两个 convert_onnx_to_rknn_rk3588_wbc_*.py：构建后调用 Python init_runtime 做验证，成功才发布最终模型及记录。

正常 policy.onnx 对应的正式部署能运行，与这次 Python 验证失败不冲突。两条链路实际使用的 Runtime 不同；现有 worker 还没有接入 33 输入的动态 LoRA ABI，不能直接启动它替代新图的 smoke。

当前转换器使用临时候选文件，验证成功才发布 .rknn 和 .rknn.json。验证失败后没有最终文件，不等于 ONNX 无法编译；也不能把临时 check.rknn 的编译信息当作正式产物验收。

## 4. 已交付与未完成

| 工作 | 状态 |
| --- | --- |
| 共享 base_lora.onnx、三类真实维度探针、manifest | 已交付 |
| L0_Hugme、L0_act_cute_1 的动态参数包与 reference/config | 已交付，足够首轮验证开发 |
| 2 个观测 + 31 个参数的动态图、CPU 数值与切换检查 | 已完成离线验证 |
| 29 个文件的 ONNX/adapter 源包同步方案 | 已修正，不再要求本地先有 RKNN |
| 真板尝试 | 用户已执行；本轮加载失败，不能记为通过 |
| 与正式部署一致的 C++ 模型 smoke | 未实现，是下一步优先修复 |
| 正式 emotion worker 动态加载 LoRA、公共执行核心 | 尚未完成 |
| 本地 ROS2 / sim2mujoco 同加载流程验证 | 后续开发范围，不能替代板端 RKNN 验收 |

现有离线结果：每动作 33 组固定输入，按 Base 归一化范围生成，不是实机状态或新 rollout。动态 FP32 图相对最终 checkpoint 最大绝对误差为 L0_Hugme 8.64e-7、L0_act_cute_1 1.16e-6；FP16 参数包对应为 6.68e-4、4.26e-4。同一 ONNX Session 的 A→B→A 已通过。这些结果不能替代 FP16 RKNN 的真板结果。

交付包内 validation_report.json、qa_report.json、DELIVERY_STATUS.json 是原离线交付时的记录；其中 not_run / pending 反映当时状态。本轮用户反馈的板端失败及后续修复要求以本文为准，未将历史 CPU 记录改写为板端通过。

原始两输入 Base ONNX 是导出来源；共享图 base_lora.onnx 已包含 Base 固定权重。动态参数保存在 adapters/ACTION_ID/manifest.json 和 weights.bin，含 A/B、bias、LayerNorm 共 31 个张量。板端不需要训练 base_ckpt.pt。不要用原始两输入 policy.onnx 覆盖 33 输入的 base_lora.onnx。

## 5. 下一位开发者必须完成的修改

### 5.1 固定环境与职责

只维护既有部署工作空间。新增 C++ 工具纳入现有 colcon build；用户仍按原流程 source install/setup.bash。转换继续使用板端已有 Toolkit2 2.3.2；设备加载和数值验证使用与正式程序一致的 RknnModel、链接库和安装规则。

不另建 Conda/venv，不增加 RKNNLite 作为必需依赖，不引入 Python ctypes Runtime 替代实现，不覆盖系统全局 librknnrt.so。本轮未实施这些方案。

构建模型与加载模型是两个职责。Toolkit2 生成候选文件后，由同部署链路的 C++ 工具验证，全部通过才标为可发布产物。不要仅去掉 --target、把 Toolkit 模拟器结果当成真板通过。

### 5.2 文件级修改范围

下表是待实施清单，不代表当前已有这些新接口或命令。

| 文件 / 模块 | 具体调整 |
| --- | --- |
| convert_onnx_to_rknn_rk3588_wbc_policy.py | 将构建导出与 Python 目标 init_runtime 验证解耦；支持候选产物；保留静态两输入与动态 manifest 路径、FP16 配置及原有图适配 |
| convert_onnx_to_rknn_rk3588_wbc_reference.py | 同样拆分候选导出与目标验证；保留 reference Gather 改写、六输出顺序和边界用例 |
| batch_convert_onnx_to_rknn_rk3588.py | 接入候选→C++ 验证→发布的阶段；缓存跳过必须匹配来源、精度、模型与真实验证状态；任一步失败不能继续标为成功 |
| legged_deploy/src/rknn_model.cpp 及现有头文件 | 复用已有加载、输入输出、Context 管理；按需要补充可查询的模型 ABI、Runtime/driver 信息和有限值检查，避免另写一套加载器 |
| legged_deploy 内新增独立 C++ 模型验证入口 | 不初始化 ROS、不创建节点；读取固定用例及完整 adapter，验证三个探针、共享 policy、reference；在同一 policy Context 内执行 A→B→A |
| legged_deploy/CMakeLists.txt | 新工具链接与 rl_controller 相同的模型实现和 RKNN_LIBRARY，沿用安装位置/RPATH策略；避免独立发现另一个 Runtime |
| wbc_lora_assets.py / 公共资产接口 | 保留 manifest、hash、张量顺序、FP16 字节布局与校验规则；提供 C++ 可读取的固定样例格式或小型转换步骤，不重新依赖训练环境 |
| verify_wbc_lora_rknn.py 与相关文档 | 当前 Lite2 入口退出标准操作流程；如保留辅助用途，明确区分，不能作为板端交付必需入口 |
| prepare_wbc_deploy.py、产物记录 | 明确候选/离线验证/板端通过状态；只有对应产物通过所需验证后，才发布最终模型及可信 .rknn.json |

实现时优先复用现有模块，是否提取 CMake 公共目标由实际代码决定，不要求预先增加通用后端框架。NPZ 到 C++ 的固定用例接口尚需实现；不得把现有 Python 检查器当成已经具备 C++ 用例读取能力。

正式 worker 的动态 adapter 切换、reference 与动作绑定、预热与切换时序属于后续正式接入。在此之前，不通过放宽 worker 的两输入检查就直接启动新图。

### 5.3 产物和精度要求

policy、探针、reference 均以 float_dtype=float16、do_quantization=False 构建。动态参数包保持 FP16 存储；应用边界可沿用现有 C++ 的 FP32 输入输出，由 Runtime 完成所需转换。不能仅凭 ONNX/NPZ 的 FP32 dtype 就认定 RKNN 内部未采用 FP16。

候选产物应与可用产物分开标识。验证记录至少包括模型/源 ONNX/adapter 哈希、输入输出 ABI、构建精度、Toolkit 版本、实际加载 Runtime 路径与版本、可获取的 driver 版本、验证后端、样本数、误差和切换结果。加载失败应保留失败报告，不能留下看似已通过的记录。

沿用已定义的误差阈值，并在报告中记录；发现超限时先区分模型/张量映射/FP16 影响，不为了得到通过结果随意放宽。重点先完成正确链路，不扩展成无边界的数值精度研究。

## 6. 手动板端验收范围

验收全部由用户手动触发。最小顺序：三个探针 → 完整共享 policy → 两动作 reference → 固定状态的 reference→policy 组合对照。每项保存退出码和摘要，失败停止依赖该项的后续验证。

- 模型 smoke 不调用 rclcpp 初始化，不创建 ROS 节点，不启动 launch、HPM 或电机通信。无需为测试启动正式控制节点。
- 新可执行文件需确认实际加载的库与正式部署一致；source 本身不是充分证据。
- 两动作各使用现有 33 组 policy 用例，校验全部 33 个输入的顺序、形状和数据；切换 A→B→A 复用同一 Context，检测残留和污染。
- reference 对照所有帧与边界输入，检查六个输出。组合对照需明确指定状态、时间步、观测构造及期望来源；原有随机样例不能冒充完整组合对照。
- 同时记录数值结果和算子执行信息；CPU 回退需如实记录。仅模型 smoke 通过仍不代表完整控制周期 20 ms 达标。
- 报告以汇总误差、耗时、切换和少量失败样例为主，不逐帧打印完整参数、不录 rosbag。普通 smoke 日志按每轮 MiB 级预算设计；编译日志和临时模型单独统计。

AMP 继续负责行走。emotion 共用一个 WBC Base，各自绑定 adapter 与 reference。部分动作训练结果足以完成上述验证，不需要先补齐所有动作。

本地 ROS2 / sim2mujoco 开发有价值：可验证资产解析、动作绑定、输入构造、状态机和切换行为；应尽量共用相同加载流程与数据契约，沿用现有 OpenVINO 对 ONNX 的执行路径。它不能证明 RKNN 算子支持、板端精度、实际 Runtime 兼容或时延，因此主要 smoke 仍必须在真板完成。

## 7. 当前可执行操作与文档入口

**目前可直接做的是保留源包、同步文档、拉取已有日志。新的 C++ smoke 尚无可执行交付，本文不提供虚构的运行命令，也不要求用户继续采集同类环境信息。**

源包同步继续采用 [同步与日志说明](wbc-lora-robot-sync-and-logs.md) 第 2–4 节，校验 BOARD_SOURCE_MANIFEST.sha256。旧 BOARD_MANIFEST.sha256 曾在缺失 RKNN 时留下半清单，不能替代完整源包清单。29 个源文件约 35.04 MiB；同步阶段不要求本地存在 .rknn。重复 rsync 可补齐文件，不需先删除已传部分。

如仅将本次交接文档放到既有机器人目录，可在开发机执行：

~~~bash
rsync -rtv \
  /home/luvbot/SC/l0_deploy_v2_ws/src/l0_ros2_deploy_v2/docs/wbc-lora-runtime-handoff.md \
  forlinx@192.168.5.2:/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2/docs/

rsync -rtv \
  /home/luvbot/SC/l0_deploy_v2_ws/deliveries/wbc_lora_p0_20261026/RUNTIME_HANDOFF.md \
  forlinx@192.168.5.2:/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/deliveries/wbc_lora_p0_20261026/
~~~

这两条仅同步新交接文档，不同步代码、不执行模型。旧指南如也需更新，按同步说明同步项目文档；不要仅上传完整 MANIFEST.sha256 到只含源包子集的机器人目录。

拉取现有全部 P0 日志，在开发机执行以下整块。它包括轮次目录以及根目录下的 runtime_trace 文件，不拉取模型，不删除板端日志：

~~~bash
(
  set -e
  wbc_local_logs=/home/luvbot/SC/l0_deploy_v2_ws/smoke_logs_from_robot/wbc_lora_p0
  mkdir -p "$wbc_local_logs"
  rsync -rtv --partial \
    forlinx@192.168.5.2:/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/smoke_logs/wbc_lora_p0/ \
    "$wbc_local_logs/"
  du -sh "$wbc_local_logs"
)
~~~

已有轮次如带 LOGS.sha256，可进入该目录执行 sha256sum -c LOGS.sha256；没有清单时保留原始日志和退出码，不据此宣称完成验收。下载日志占用以实际 du 为准，失败时的编译 verbose 日志不套用旧 Lite2 摘要大小估算。

其余背景见 [P0 离线交付](wbc-lora-p0-handoff.md)、[开发规划](wbc-lora-development-plan.md)、[原部署流程归档](deploy-workflow.md)。本次只更新交接文档和交付包文档清单，未修改原 deploy_workflow、运行代码、模型、底层节点或远程环境。
