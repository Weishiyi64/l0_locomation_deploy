# sim2mujoco 测试记录

测试日期：2026-11-02。环境：本机 x86_64 Linux、ROS 2 Humble、OpenVINO 2024.6、MuJoCo 3.0.1、Python 3.10 / ONNX Runtime 1.23.2。全部 ROS 测试使用 localhost-only、domain 77，没有连接真机。

> 模型版本说明：下文原五动作及文本闭环测试使用旧 `l0_v5.xml`。2026-11-02 按用户指定切换到 `l0_base_hip_collision_split_experimental.xml`，新模型的验证范围单独记录在文末，旧结果不自动适用于新模型。

## 构建和兼容性

- 独立 `/tmp/l0-sim-build`、`/tmp/l0-sim-install` 构建通过，包含实际的 MuJoCo、控制器、接口包和 WBC CPU 后端。
- 五个共享 WBC C++ 文件使用 `USE_RKNN` 分支完成语法编译检查；本机不执行 RKNN，也不把此检查称为板端链接或真机测试。
- 与工作前 SHA256 快照比较，463 个关键真机入口、驱动和运行资产文件保持字节不变。
- Python launch/生成器语法及 shell 启动脚本语法通过。
- 修复了普通安装后的 MuJoCo 动态库搜索路径；安装可执行文件通过相对 RPATH 加载同包的 `libmujoco.so.3.0.1`。

## 资产验证

- 指定 WBC10000 checkpoint 身份与共享 Base 一致；两份原始 Base ONNX 虽然文件哈希不同，权重、LayerNorm、归一化逐项校验一致，20 组输入推理差值为 0。
- 四个新动作分别由指定 `lorackpt_2000.pt` 重新导出验证，weights 与仿真包逐字节相同，reference 源哈希与新转换记录相同。
- `A-C04_mod` 原始 JSON 来源字节一致；镜像坐标变换后全部 360 帧与归档 reference 的动作区间相同，最大差值 0。
- `A-C04_mod` 与仿真 `UpperBody_Enhanced` 使用同一份指定上半身 PT 参数，weights SHA256 为 `d089b409…`，与原真机文本包不同；真机包未替换。
- 文本 `qwen_tokenizer.onnx`、`inbetweener.onnx` 和 manifest 与 `/home/luvbotic/下载/text_generation_models_20261102` 的对应文件逐字节一致。

## 共享 Base / LoRA 数值与切换测试

最终五动作循环切换 10 轮，共 50 组有限随机观测，使用正式 `WbcLoraAssets` 和 OpenVINO WBC 后端。单一 Base 上完成 50 次 adapter prepare/commit，全部输出有限。与独立 ONNX Runtime CPU 对照的最大 action 绝对误差为 **4.0531158447265625e-6**。

此项是推理数值和参数切换测试，不能替代物理动作验收。

## 文本闭环

真实运行流程：最新文本模型 CPU 推理 → 60 帧九关节 JSON → `ready` → 正式动作 `started` → `completed` → AMP 平滑接管。请求 ID：`T_49254b45a2f54ddab3f6ad62eb54`。

- Base：共享 WBC10000，实例数 1。
- adapter：指定 PT 导出的 `UpperBody_Enhanced`，非全零 LoRA。
- JSON root：MuJoCo 实时快照，生成时高度约 0.4492 m。
- WBC 执行期 210 个控制周期，超时 0；平均/最大周期约 2.150/2.953 ms。
- 过期 JSON `T_sim_expired_check2` 被拒绝，原因 `expired/excessive JSON admission TTL`，未进入动作执行。

测试提示词为“挥挥右手打招呼”，本次模型实际返回 token `[5, 0, 0]`，仅头部 token 激活。本记录证明生成和控制链路，不将该结果当作提示词语义准确性验收。动作语义需要单独评价文本模型。

最终 0.004 s 软限位配置再次测试提示词“点头”：生成 **184 帧**九关节 JSON，请求 `T_0ee4fd2cf750443cb5b70f99051e`，完整经过 `ready → started → completed → AMP`。使用同一指定 Base / UpperBody_Enhanced，实时 snapshot root；334 个执行周期，超时 0，平均/最大周期 **1.956 / 3.471 ms**。原始 JSON 和日志分别为 `/tmp/l0-sim-text-live-final.json`、`/tmp/l0-sim-text-final-execute.log`、`/tmp/l0-sim-text-final-run.log`。这里同样只验收生成与执行链路，不对模型输出的动作语义作准确性保证。

## 普通动作物理验收

最终配置（软限位时间常数 0.004 s）下，五动作按顺序全部完成 `started → completed → AMP`，共享 Base 上累计五次 adapter prepare/commit，没有执行期推理超时。

| 动作 | 状态 | 执行周期 | 超时 | 平均/最大周期 ms |
|---|---|---:|---:|---:|
| L0_strange_new_1_A | completed | 428 | 0 | 2.538 / 5.193 |
| L0_strange_1 | completed | 452 | 0 | 2.143 / 3.811 |
| L0_excited_8 | completed | 452 | 0 | 2.081 / 4.341 |
| L0_angry_new_2 | completed | 434 | 0 | 2.378 / 3.933 |
| A-C04_mod | completed | 510 | 0 | 2.238 / 3.608 |

原 XML 软限位时间常数 0.02 s 下，`L0_angry_new_2` 两次触发左肘越限保护，最低反馈约 `-1.671959 rad`，而配置下限为 `-1.571 rad`、保护容差为 0.1 rad；控制器正确进入紧急阻尼。最终仅把仿真软限位约束时间常数设为 0.004 s，保留角度范围和保护逻辑。复测该动作左肘最低 `-1.602578 rad`，最大限位穿透约 0.031578 rad，低于原有保护容差；未截断或伪造反馈。此处是 MuJoCo 软约束穿透，并不等价于物理关节范围扩大。

最终五动作日志：`/tmp/l0-five-stiff-limit.log`、`/tmp/l0-five-stiff-results.log`。实际反馈采样：`/tmp/l0-five-feedback.json`（采样在首个动作完成附近才启动，覆盖后四个动作）。这些是本机临时证据，清理 `/tmp` 后需要重新运行。

测试结束已关闭本次启动的全部 MuJoCo、控制器及子 launch 进程；进程检查未发现 `/tmp/l0-sim-install` 的残留运行进程。

## 复现与范围

操作命令见 [组成与部署说明](sim2mujoco-deployment.md)。必须先启动所需入口，再进入零速度 AMP，稳定后逐个请求动作。每个动作应观察 `started → completed`，控制器完成 WBC 到 AMP 的切换，且无反馈故障或动作超时。文本 JSON 应同时观察 worker `ready` 和控制器 `completed`。

当前只覆盖用户指定的四个 wholebody1102 动作、A-C04_mod 和文本 JSON。其余 80 多动作不属于这次仿真验收范围，原真机动作包保持不变。本次不是长时间稳定性、全速度扰动、全提示词或真机安全验收。

## 用户动作表现反馈（2026-11-02）

用户反馈：输入“点头”时，出现点头同时右手抬起。核对之前保留的同提示词 JSON `/tmp/l0-sim-text-live-final.json`，token 顺序 head/left/right，输出 `[40, 0, 35]`，右手已被文本模型选中。JSON 中右肩 pitch 范围约 -2.5684～0.0070 rad，右肘范围约 -0.2959～1.3481 rad。因此该次附带右手动作已存在于生成 reference 中，不能归因为 WBC 执行阶段自行添加。此文件是之前保存的同提示词样本，不是用户本次运行抓取的数据。

用户反馈以下描述可以执行；这是用户观察记录，未由本轮自动测试重新逐项验收幅度、速度和次数：

1. 头以大幅度慢速点头两次。
2. 头以小幅度快速左右倾斜三次。
3. 头以中幅度慢速向左转。
4. 左手以大幅度抬起。
5. 右手以小幅度慢速前后摆动两次。
6. 左手摸肚子两次。
7. 头点头；右手放后面。
8. 头以小幅度慢速点头；左手以大幅度抬起。
9. 左手以小幅度前后摆动两次；右手放后面。
10. 头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。

当前短提示词“点头”的语义准确性问题仍未修复。上述长描述可作为后续文本模型回归测试样例，不构成所有短提示词或组合动作均正确的保证。

## 指定 v5 碰撞拆分模型复测（2026-11-02）

两个仿真入口现默认加载 `l0_base_hip_collision_split_experimental.xml`。XML、49 个分块碰撞网格和引用的 22 个原有网格均与用户指定源逐字节核对一致。XML SHA256 为 `b1a54eac002742e718781a77ab3056a9e9528108e2c9fdd057ad6de881d5d254`。原 `l0_v5.xml` 未覆盖，463 个关键真机文件复核仍未变化。

已安装 description、MuJoCo launch 和控制器 launch 到 `/tmp/l0-sim-install`，Python/shell 语法检查通过。实际进程参数确认加载安装目录内的新 XML；完成 MuJoCo 启动、实时反馈、进入 AMP、WBC 五动作预加载。

本次只重新执行 `L0_angry_new_2`：`started → completed → AMP`，434 个执行周期，推理超时 0，平均/最大周期 2.173/4.033 ms。保留仿真 `joint_limit_time_constant=0.004` 设置，未修改 WBC 参数或控制器保护。日志为 `/tmp/l0-sim-split-model.log` 和 `/tmp/l0-sim-split-angry.log`。

该复测验证模型加载及动作生命周期，不代表“手卡住”的视觉/接触问题已解决。其余四动作和文本闭环尚未在此新 XML 上逐项重新验收；文本入口通过包含同一 sim2mujoco launch 继承新模型选择。

## 指定动作完成后进入 Emergency（2026-11-09，仅 MuJoCo）

本次使用当前碰撞拆分 XML、localhost ROS domain 77 和终端 `mujoco_control`。在 `/tmp/l0-sim-build`、`/tmp/l0-sim-install` 独立构建通过，共 11 个包；Python/shell 语法检查通过。与本次修改前 SHA256 快照比较，459 个真机入口、驱动和运行资产文件未变化。共享控制器新增分支由 `isSim` 限定，未添加手柄动作绑定。

仿真包新增 `L0_crouch_off`：reference ONNX 的 SHA256 与真机 manifest 中记录的源哈希一致，LoRA 参数和动作配置从匹配 Base 的真机包复制到仿真包，原真机资产未修改。reference 为 126 帧、50 Hz，含配置中的 1 秒起始保持，共执行 176 个周期。

| 场景 | 实测结果 |
|---|---|
| 默认配置，终端执行 `L0_strange_new_1_A` | `started → completed`，回到 AMP；观测确认 AMP 推理启用且电机命令 Kp 大于 0 |
| 默认配置，终端执行 `L0_crouch_off` | `started → completed`，FSM 进入 EmergencyStop，没有 WBC→AMP 过渡；完成后的 705 个电机命令样本均为 Kp=0、dq=0、tau=0、Kd=5，AMP 推理关闭 |
| Emergency 后请求 `L0_strange_1` | `rejected`，终端返回非零退出码 |
| `wbc_emergency_after_actions:='[]'`，执行 `L0_crouch_off` | `started → completed`，恢复 AMP 推理和非零 Kp 输出 |
| 默认配置，`L0_crouch_off` 运行中终端执行 `stop` | 仅 `started → failed`，无 `completed`；控制器输出保持急停阻尼 |

主验证中普通动作 428 个执行周期、crouch 176 个执行周期，推理超时均为 0；平均/最大周期分别为 2.263/4.088 ms、2.293/4.348 ms。此处验收动作生命周期与控制器输出，不代表蹲下姿势或接触质量的视觉验收，也未进行真机测试。

临时证据：`/tmp/wbc_emergency_build_20261109.log`、`/tmp/wbc_emergency_sim_20261109.log`、`/tmp/wbc_emergency_terminal_result_20261109.json`、`/tmp/wbc_emergency_empty_check_20261109.log`、`/tmp/wbc_emergency_abort2_check_20261109.log`。中途停止验证的首次会话与上一仿真清理过程重叠，动作被拒绝，不计为通过；清空进程后独立复验通过。启动进入 AMP 时存在既有的短暂 command unavailable 阻尼提示，手动 stop/关闭桥接时也存在禁能命令相关警告，本记录不将运行称为完全无告警。

## LT/RT + 十字键 WBC 请求链路（2026-11-09）

本次按用户要求同时接入 MuJoCo/真机启动参数，但只在本机运行 MuJoCo。未连接真机、未上传或执行板端编译。控制器 C++、真机驱动、`wbc_lora_fp16` 和安全参数保持不变。新增独立 Python 请求节点复用 `/joy` 和现有 WBC 命令/反馈协议，终端入口保留。

本机未发现可用物理手柄设备；以下通过发布与既有手柄布局一致的 Joy 消息验证，包含实际 OpenVINO 推理与 MuJoCo 动力学执行，不是物理手柄 USB/蓝牙验收。11 包仿真构建、Python/shell 语法检查、真机/仿真普通及文本入口的 launch 参数和资产静态解析均通过；文本入口不创建固定动作手柄节点，关闭 WBC 的入口不创建 WBC 请求节点。

| 组合键 | 动作 | 执行周期 | 推理超时 | 平均/最大周期 ms | 结果 |
|---|---|---:|---:|---:|---|
| LT+上 | L0_fear_7_A | 463 | 0 | 1.710/3.957 | completed → AMP |
| LT+下 | L0_excited_1 | 287 | 0 | 1.295/2.898 | completed → AMP |
| LT+左 | L0_excited_new_1 | 446 | 0 | 1.661/3.434 | completed → AMP |
| LT+右 | L0_excited_new_5 | 338 | 0 | 1.481/3.865 | completed → AMP |
| RT+上 | L0_satisfied_10 | 561 | 0 | 1.457/3.987 | completed → AMP |
| RT+下 | L0_SayNo_17 | 322 | 0 | 1.510/3.799 | completed → AMP |
| RT+左 | L0_angry_9 | 352 | 0 | 1.479/2.910 | completed → AMP |
| RT+右 | L0_angry_new_1 | 313 | 0 | 1.462/3.296 | completed → AMP |

每个动作只有一条请求，反馈均为 `started → completed`；持续按住直到完成后也没有连发。启动时按住、双扳机冲突、斜向十字键、消息中断超过 0.5 秒后仍按住、动作忙碌期间切换并保持其他组合键，都没有补发请求。另以输入逻辑检查验证急停按钮混入、短消息和非有限值被忽略，以独立 ROS 节点检查验证不在预加载名单中的映射会拒绝启动。

8 个手柄动作后，通过原终端入口执行 `L0_strange_1`，仍正常完成并返回 AMP；再执行 `L0_crouch_off`，保持 `completed → EmergencyStop`，AMP 推理关闭、控制器输出急停阻尼，此时手柄请求被阻止。

主运行退出时发现新增节点未处理 ROS 外部 shutdown 异常，已补齐异常处理并重新构建，独立节点启动/SIGINT 退出复验返回 0。动作执行阶段无推理超时；原有进入 AMP 的短暂 command unavailable 提示及桥接退出的禁能相关警告仍存在。这里验证输入协议、动作生命周期和控制输出，不声称动作姿势/接触质量或真机效果已验收。

证据位于本机临时目录：`/tmp/wbc_joystick_mujoco_20261109.log`、`/tmp/wbc_joystick_check_20261109.log`、`/tmp/wbc_joystick_mujoco_result_20261109.json`、`/tmp/wbc_joystick_build_final_20261109.log`、`/tmp/wbc_joystick_shutdown_check_20261109.log`。操作和映射说明见 [WBC 手柄与终端并用](joystick-wbc-control.md)。
