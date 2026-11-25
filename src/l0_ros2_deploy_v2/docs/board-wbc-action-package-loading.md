# L0 V5 WBC 研发验证增量交付说明

本次软件合入的主要增量是固定 actor 的 Base 动作包加载。10 月 29 日交付已支持 Static 和 LoRA 多动作。新增的 `wbc_mode=base` 让一个包里的动作共用一份固定 actor RKNN，启动时加载所选动作的参考模型，执行时按动作 ID 选择参考轨迹和配置。这样，训练侧可以交付不同 actor 的包，软件侧沿用一套加载器。上层继续使用既有动作请求和反馈协议。

## 对比基线

| 记录 | 本次对比口径 |
| --- | --- |
| 2026-10-29 软件交付 | 以已归档的 `l0_v5` 交付版本 `cb79e50` 为起点。该版本含 Static、LoRA 动作加载及文本生成到 WBC 的 JSON reference 链路。 |
| 2026-11-02 文本动作补丁 | 针对 10 月 29 日交付的五文件补丁，文本默认动作更新为 `UpperBody_Enhanced`，并同步文本模型来源校验。 |
| 2026-11-04 WBC 板端验证 | 首次落地 Base 加载，随后将配置和预检查收口到 `sim2real`。这两份板端增量可用于定位加载改动。 |
| 2026-11-13 板端源码归档 | 用于核对后续实现。快照包含验证工作树改动，软件主分支的实际接收版本以软开当前分支为准。 |
| 2026-11-19 Sonic 包 | 最新可追溯的动作包验证，记录的是新 actor、沿用的 reference、资产哈希和预检查。 |

[10 月 29 日交付记录](../../../deliveries/l0_locomotion_deploy_push_20261029/DOCUMENTATION_PUSH_VERIFICATION.json) · [11 月 2 日补丁清单](../../../deliveries/text_wbc_1029_code_update_20261102/PATCH_INFO.json) · [Base 首版增量](../../../deliveries/wbc_base_20261104/code_update/incremental.patch) · [启动配置增量](../../../deliveries/wbc_startup_20261104/board_incremental.patch) · [11 月 13 日源码归档说明](../../../deliveries/board23_source_20261113/l0_deploy_v2_ws_source_only/README_SOURCE_ONLY.txt) · [Sonic 交付记录](../../../deliveries/wbc_sonic_dual64_245000_20261119/DEPLOYMENT.json)

如果目标分支已接入 11 月 4 日的 Base 加载，按下文核对配置、安装和动作 YAML 差异即可。如果目标分支已含 11 月 13 日对应实现，11 月 19 日 Sonic 记录对应的是资产增量，采用已有加载接口。

## 需要合入的动作包加载改动

| 层 | 上次交付 | 本次新增 |
| --- | --- | --- |
| 包类型 | `static` 每动作独立 policy。 `lora` 共享一个策略图，每动作有 reference 和 adapter。 | 新增 `base`，包内一个固定 actor，每动作有 reference 和 YAML。三种模式由同一个 worker 按启动配置选择。 |
| 资产索引 | `WbcLoraAssets` 读取 `wbc_lora_graph_v1`。 | 新增 `WbcBaseAssets` 读取 `wbc_base_graph_v1`，校验 actor 和 reference 的转换记录、配置哈希、动作 ID 和固定 actor 的跨动作配置。 |
| 加载与执行 | LoRA 动作开始时加载 adapter 并绑定常驻参数。 | Base 启动时创建一个普通 RKNN actor 并预加载所选 reference。动作准备阶段选择已加载的 reference 和配置。adapter 读取、提交、回滚仍由 LoRA 分支处理。 |
| 启动与安装 | 包和动作选择主要由原启动参数与外层脚本准备。 | `sim2real` 归并配置并预检查已安装资产。`one_start` 识别 Base manifest，传递解析后的名单和 manifest。CMake 将 policy 目录按目录复制安装，供同一个 ROS overlay 读取。 |

[10 月 29 日 WBC 加载实现](../../../workspaces/l0_locomotion_deploy_publish_20261029/src/l0_ros2_deploy_v2/legged_deploy/src/rl_controller_wbc.cpp) · [Base 资产加载](../legged_deploy/src/wbc_lora_assets.cpp) · [Base worker 首版](../../../deliveries/wbc_base_20261104/code_update/src/legged_deploy/src/rl_controller_wbc.cpp) · [CMake 安装差异](../../../deliveries/robot51_wbc_sync_20261109/SOURCE_CHANGES.patch)

按以下位置合并到软件公共分支中的对应模块。

1. **C++ 加载器。** 在 `legged_deploy/{include,src}/wbc_lora_assets.*` 合入 `WbcBaseAssets`，在 `rl_controller_wbc.*` 合入 Base 构造、配置路径、共享模型和 reference 分支。固定 actor 使用 `policy[399]`、`command_short[240]` 或 `command[240]`，输出 `action[21]`。沿用 21 关节、50 Hz 和六输出 reference 接口。 `WbcCfg.motion_id` 与 manifest 的 `action_id` 一致，动作配置的关节顺序、默认角、缩放和增益与固定 actor 配套。 [Base 加载增量](../../../deliveries/wbc_base_20261104/code_update/incremental.patch) · [Base manifest 示例](../../../deliveries/wbc_base_20261104/wbc_base_300w_s0/base_manifest.json)
2. **启动和预检查。** 在 `one_start.launch.py` 按 mode 检查 manifest 和资源。`sim2real.launch.py` 与 `deployment_config.py` 负责统一配置和运行快照，优先级为 launch 命令行、`deploy_config/wbc-production.env`、launch 默认值。Base 沿用 `wbc_lora_bundle` 选包，`wbc_policies` 选择动作子集或 `all`。`all` 在 `sim2real` 展开后以列表传给 `one_start`。预检查核对 `INSTALLATION.json` 中登记的文件哈希，子集只减少预加载项。外层脚本和直接启动共享这套解析。若发布保留 WBC 控制台和录包脚本，同步 `wbc_runtime_common.bash` 与 `wbc_action_console.*` 的 Base 支持，让它们从实际运行快照获取包和动作名单。 [one_start](../legged_controllers/launch/one_start.launch.py) · [启动预检查实现](../../../deliveries/board54_models_20261113/board_deployment_config.py) · [11 月 4 日启动改动](../../../deliveries/wbc_startup_20261104/使用与执行流程.txt)
3. **安装与包产物。** 将 `legged_deploy/CMakeLists.txt` 中 `policy/` 安装规则的 `USE_SOURCE_PERMISSIONS` 合入现有构建，使 WBC 的相对路径及哈希在 ROS 安装目录中可用。完整运行包随软件发布配置提供，包含 `base_manifest.json`、`INSTALLATION.json`、actor RKNN 和转换记录，以及每个动作的 reference RKNN、转换记录和 `rl_config.yaml`。需要自行构建包时，可复用 `build_wbc_base.py`，reference 复用要求来源 ONNX 哈希与转换记录匹配。包与名单在启动时确定，变更后重新启动部署。 [CMake 资产安装](../legged_deploy/CMakeLists.txt) · [Base 打包工具](../scripts/build_wbc_base.py)

对上层，`/algorithm/brain/wbc_action_cmd` 继续接收 `timestamp_ms + action_id`，`/system/state_machine/wbc_action_fdb` 继续按原请求键反馈普通动作的 `started`、`completed`、`rejected` 或 `failed`。Base 模式下，上层可通过 `available_wbc_policies` 读取本次注册的动作名单。 [板端动作请求和反馈实现](../../../deliveries/board23_source_20261113/l0_deploy_v2_ws_source_only/src/l0_ros2_deploy_v2/legged_controllers/src/LeggedControllerWbc.cpp)

## 随所选动作资产或行为合入的改动

| 改动 | 需要合入的条件和影响 |
| --- | --- |
| `WbcCfg.first_reference_frame` | 所选包使用非零起始帧时需要。默认 0。新版动作的参考帧基准从该值开始，有效时长为 `start_hold_sec + (reference_num_frames - first_reference_frame) × control_dt`。软体待机样例使用第 1 帧。 |
| `WbcCfg.emergency_stop_on_finish` | 正式交付包含正常完成后进入软件急停的动作时需要。默认 `false`。加载动作时由 `PolicyMux` 读取，完成分支由主控制器执行。 |
| AMP 与 WBC 的历史初始化和首帧匹配 | 需要复现 11 月 13 日板端切换行为时作为一组变更合入。两向交接均用源策略最后有效的关节目标初始化目标策略历史，并用 `handoff_id` 匹配新鲜首帧。11 月 4 日的 Base 加载已能独立工作，加载器合入可以沿用软件分支现有交接协议。 |

采用新版交接时，同步 `PolicyCommand.msg` 的 `handoff_id`、`ControlWbcMotion.srv` 的 `handoff_id` 与 `source_command`、新增的 `SeedPolicyHistory.srv` 和接口注册。同步 `PolicyMux`、主控制器、AMP 与 WBC worker 的生产消费逻辑，重编译使用共享 `PolicyCommand` 的节点，含现有 imitation 链路的消费者。需要单独验证文本 WBC 源策略、历史和返回 AMP 的兼容性。 [11 月 9 日动作配置增量](../../../deliveries/robot51_wbc_sync_20261109/SOURCE_CHANGES.patch) · [待机动作配置](../../../deliveries/board54_models_20261113/existing_standstill_config.yaml) · [板端交接接口](../../../deliveries/board23_source_20261113/l0_deploy_v2_ws_source_only/src/luvbot_ros_interfaces/luvbot_control_interfaces/CMakeLists.txt) · [板端 PolicyMux](../../../deliveries/board23_source_20261113/l0_deploy_v2_ws_source_only/src/l0_ros2_deploy_v2/legged_controllers/src/PolicyMux.cpp)

## 与软件公共分支的合并边界

- **以软件公共分支做差异合并。** 11 月 13 日 WBC 验证快照省去了此前的文本 JSON 路径。直接覆盖 `rl_controller_wbc.*`、启动文件或 CMake 会带走文本 reference 支持。保留公共分支的文本启动入口、`wbc_json_base_action`、`mapping_file`、JSON reference 分支和编译安装项，以及 `/text_motion/cancel` 和 `cancelled` 反馈。LoRA 分支继续按 `Motion::adapter_id_` 查找参数动作，文本请求 ID 与参数动作 ID 分开。合并 launch 时核对 `sim2real` 到顺序启动脚本再到 `one_start` 的传参顺序。现有文本入口显式选择 LoRA，Base 作为同一 worker 的另一种启动模式。 [10 月 29 日文本 WBC 启动](../../../workspaces/l0_locomotion_deploy_publish_20261029/src/l0_ros2_deploy_v2/legged_controllers/launch/text_wbc.launch.py) · [既有文本 reference 实现](../../../workspaces/l0_locomotion_deploy_publish_20261029/src/l0_ros2_deploy_v2/legged_deploy/src/rl_controller_wbc_json.cpp) · [文本取消反馈](../../../workspaces/l0_locomotion_deploy_publish_20261029/src/l0_ros2_deploy_v2/legged_controllers/src/LeggedControllerWbc.cpp)
- **机型配置和其他部门功能继续由软件公共分支维护。** 合并共享 launch 时采用该分支的 AMP 默认策略、姿态保护、硬件启动参数和感知配置。验证板的默认包、站立／下蹲策略和运行配置随对应验证资产交付。延迟埋点、固定帧长时站立和板端临时链接属于独立验证任务。 [板端源码归档范围](../../../deliveries/board23_source_20261113/l0_deploy_v2_ws_source_only/README_SOURCE_ONLY.txt) · [延迟与固定帧验证记录](../../../analysis/latency_implementation_20261109/STATUS.json)

## 集成验收和现有证据

| 验收项 | 软件目标分支应验证 |
| --- | --- |
| 兼容性 | Base 新包、既有 LoRA、Static 均能启动和选动作。原文本请求能完成 JSON 生成、接收、执行和取消。 |
| 资产与加载 | 已安装包可由目标 ROS overlay 解析。模式、清单、配置和哈希匹配。损坏包或未知动作被明确拒绝。worker 启动只初始化一个 Base actor，入选 reference 全部预加载。 |
| 实机回归 | 通过现有上层话题验证两个 Base 动作切换和返回 AMP，再回归原路径。核对动作反馈和 50 Hz 周期。目标版本采用起始帧、结束急停或新版交接时，覆盖相应分支。 |

11 月 4 日 Base 增量有板端编译与资产检查记录。11 月 19 日 Sonic 包复用 `wbc_base_partymimic1119` 的参考资产，168 个动作和 513 个资产通过启动预检查，actor 通过离线 NPU 数值对照。软件公共分支的合并构建、文本兼容性和实机闭环仍需在目标版本验收。 [11 月 4 日板端检查](../../../deliveries/wbc_base_20261104/BOARD_CHECKS.json) · [Sonic 安装与预检查](../../../deliveries/wbc_sonic_dual64_245000_20261119/DEPLOYMENT.json) · [Sonic NPU 数值对照](../../../deliveries/wbc_sonic_dual64_245000_20261119/NPU_CHECK.json)
