# l0_ros2_deploy_v2

## 运控部署文档

| 文档 | 内容与状态 |
| --- | --- |
| [L0 v5 文本生成动作部署](docs/text-motion-deployment.md) | 新 ONNX/自定义算子、sim2mujoco 离线验证、ROS 2 上半身轨迹接入、修改文件和真机操作 |
| [L0 v5 MuJoCo 动作查看](sim2mujoco/README.md) | 文本动作闭环仿真与 `legged_deploy/policy/l0_v5` 动作参考轨迹预览 |
| [LoRA 推理性能优化交接（新 agent 先读）](docs/wbc-lora-inference-performance-handoff.md) | 当前架构、核心文件、加载/线程路径、板端超时数据和修改边界；先采基线，优化尚未交付 |
| [正式部署同步、HPM 隔离与日志（操作入口）](docs/wbc-lora-robot-sync-and-logs.md) | 沿原 colcon/source/sim2real 流程；用户板端已执行 LoRA 推理，完整动作因时效超限未通过 |
| [板端 Runtime 原问题交接](docs/wbc-lora-runtime-handoff.md) | 保留原环境证据；其中旧开发待办和独立 smoke 入口已由当前方案替代 |
| [部署流程与 ROS 2 工作机制](docs/deploy-workflow.md) | 归档工作空间 `deploy_workflow`，原有部署步骤保持原文和顺序；补充现有模型加载及节点协作说明 |
| [WBC-LoRA 动态加载部署方案](docs/wbc-lora-deployment-plan.md) | 结合训练结果及实际部署代码修订；初步可行，可进入验证开发，板端执行和 20 ms 周期待验收 |
| [P0 交付与手动真板验证](docs/wbc-lora-p0-handoff.md) | 动态图、FP16 参数包和两动作 CPU 对照已交付；用户真板尝试遇到 Runtime 加载失败，尚未验收通过 |
| [WBC-LoRA 开发规划与代码修改范围](docs/wbc-lora-development-plan.md) | 文件/函数级修改清单、公共核心与动态加载职责、参数传递、P0–P4 历史规划；本轮实际范围以正式部署同步文档为准 |
| [本地 ROS 2 与板端无 HPM smoke](docs/wbc-lora-validation-workflow.md) | 历史验证方案，已由正式 sim2real + 隔离 HPM 入口替代 |
| [WBC-LoRA 可行性审阅](docs/wbc-lora-feasibility-review.md) | 参数差异、真实网络结构、代码修改落点及验证门槛 |
| [LoRA 原始总体方案](docs/wbc-lora-proposal-original.md) | 原文件逐字归档，供追溯；实施以修订方案和审阅结论为准 |

## 工作空间隔离

v1 和 v2 使用独立源码及构建目录，两个版本都基于 ROS 2：

| 版本 | 工作空间 | 部署仓库 / 分支 |
| --- | --- | --- |
| v1 | `~/zy_workspace/l0_deploy_v1_ws` | `src/l0_ros2_deploy_v1` / `feature/ground_promotion` |
| v2 | `~/zy_workspace/l0_deploy_v2_ws` | `src/l0_ros2_deploy_v2` / `feature/hpm_deploy` |

仓库分别为 `git@github.com:luvbotics/l0_ros2_deploy_v1.git` 和
`git@github.com:luvbotics/l0_ros2_deploy_v2.git`。每个工作空间独立生成
`build/`、`install/`、`log/`，新终端只加载所需版本的环境；不要同时 source 两个部署工作空间。
旧目录可保留用于回退，不复制其中的构建缓存、安装结果或日志。

v2 的 `src/` 还需匹配版本的 `luvbot_ros_interfaces`；启用雷达时需要
`odin_ros_driver_rev1` 包，工作空间中只能放置该包的一份源码。
v2 已移除眼睛表情消息触发动作、眼睛动画字段和串口包。
键盘/手柄触发的通用动作序列及头部控制继续保留。
v1 的眼睛、串口和旧硬件驱动依赖在 v1 工作空间内独立保留。

## L0 v4 HPM 功能迁移

当前实机默认机型为 `l0_v4`，启用全部 21 个关节；WBC、雷达和 MQTT 模拟电压测试
默认关闭。`l0_v4` 使用 RS05 双臂；`l0_v5` 使用 LS PJ2S-48XX22A 双臂，
两者配置独立。实机入口支持 `l0_v3/l0_v4/l0_v5`。v5 路由已按机器人上
`zyc_ws/ros2_ws/src/hpm_motor/config/motors.yaml` 补齐；详见
[v4/v5 配置说明](system_command_arbiter/README.md#l0-v4--v5-hpm-mappings)。
正式电机表和协议数值确认前不要上电运动。

## WBC 多动作（RK3588）

在 `legged_controllers/launch/sim2real.launch.py` 的 `wbc_policies` 默认值中集中维护
策略目录名单，或通过 launch 参数覆盖。`one_start` 只负责传参和启动唯一 WBC worker。
每个目录位于 `legged_deploy/policy/<robot_type>/<策略目录>/`，需包含
`rl_config.yaml`、`policy.rknn` 和 `reference_motion.rknn`；命令的 `action_id` 使用配置中的
`WbcCfg.motion_id`，可以与目录名不同，但不能重复。所有动作必须使用相同的观测关节名称和顺序。

worker 启动时加载全部配置、policy/reference RKNN 并校验，不在启动时执行动作。
任一动作缺失或不兼容都会使 WBC 启动失败；关闭 WBC 时不检查或加载这些资产。
当前提供的动作是 `L0_AfraidToLeave`（l0_v3/l0_v5）；旧 v4 WBC 资产已移除。
第二套动作需要提供兼容模型，不要只复制目录而保留重复的 motion_id。

以下命令仅供机器人端构建与验证，本地 macOS 不运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/zy_workspace/l0_deploy_v2_ws
# src/ 下需有匹配版本的 luvbot_ros_interfaces。
colcon build --symlink-install --packages-up-to legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN
source install/setup.bash
# 先按下文启动 HPM，再启动部署；多动作时将名单替换为实际已有的目录。
ros2 launch legged_controllers sim2real.launch.py robot_type:=l0_v3 \
  'wbc_policies:=[L0_AfraidToLeave]'
```

仿真入口默认 `wbc_enabled:=false wbc_policies:=[]`；显式开启时也需传入非空名单，
WBC 仍要求 RKNN 后端。直接调用 `one_start` 开启 WBC 时同样必须显式提供名单。

进入 AMP Walking 且运动指令接近零后，通过现有话题发送动作 ID 和当前 Unix 毫秒时间戳：

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(python3 -c 'import time; print(time.time_ns() // 1000000)'), \"action_id\": \"L0_AfraidToLeave\"}'}"
```

准备期间 AMP 继续控制；准备成功后暂停 AMP 推理，平滑切入选中的 WBC。
未选中的动作始终不推理；准备中、执行中和交接中不排队执行其他动作。
动作期间运动指令不打断 WBC，完成后交回 AMP，只使用届时仍有效的指令；急停和故障优先。
准备取消未确认时，新的请求必须先完成清理才能准备下一动作。

机器人端验收：

- 配置两套有效模型，执行 `A → AMP → B → AMP → A`，确认各自动作、增益和历史正确切换。
- 检查移动时启动、并发请求、未知 ID 都被拒绝；动作中输入运动指令不会中断动作。
- 检查缺失资产、重复 ID、关节顺序/ABI 不兼容会报错；准备超时后不误激活，清理后可重试。
- 检查动作中的传感器失效和急停走现有安全退出路径，不继续使用过期输出。
- 从日志确认只有一条 `WBC worker ready`，每个配置动作各有一条 `WBC preloaded`；
  对照 active 话题和 AMP 输出确认 AMP/WBC 稳态互斥，允许准备和交接边界短暂重叠。
- 记录进程内存及 NPU 负载；全部预加载会增加常驻内存，动作数量不增加持续推理路数。

日志在预加载、准备及动作结束/取消/失败时输出，不逐帧打印。
`WBC timing` 中 reference 是每周期所有 reference RKNN 调用的累计耗时，policy 是策略 RKNN 耗时；
cycle 从采集观测到输出发布及历史更新，均提供平均值/最大值和超预算次数。
准备首帧与动作执行分别统计；完整控制调度延迟仍需在机器人端结合实际频率测量。
周期应满足现有 20 ms 预算。reference 仍保持逐周期查询，本阶段没有参考帧缓存、
后台推理或 NPU 绑核调整。

## sim2mujoco（ROS 2 Humble）

感知、规划和策略公共接口位于独立仓库 `luvbot_ros_interfaces`，将匹配版本放在本工作空间
的 `src/luvbot_ros_interfaces`。实机 HPM 电机链路暂时使用本仓库根目录的
`robot_interfaces`；该包必须与 HPM 仓库中的同名包完全一致。

在 Ubuntu 22.04 / ROS 2 Humble 的新工作空间构建，避免旧版同名包的安装结果污染：

```bash
source /opt/ros/humble/setup.bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --symlink-install \
  --packages-up-to legged_mujoco mujoco legged_real legged_controllers system_command_arbiter robot_interfaces
source install/setup.bash

# 启动。默认组合为 l0_v3 + l0_v3_amp_06152
ros2 launch legged_controllers sim2mujoco.launch.py
```

启动后应当只有一套硬件契约：MuJoCo 提供
`actuator/motor/state`、`actuator/motor/set_enabled`、`imu/data` 和 `odom`；
Arbiter 提供 `actuator/motor/command`；Controller/Policy 使用
`control/policy/observation` 和 `control/policy/command/rl`。旧的
`LOWCMD`、`LOWSTATE`、`rlState`、`rlCmd_*` 不再属于该启动链路。

当前 MuJoCo bridge 要求模型为一个 free base、每个 actuator 对应一个单自由度
joint，并且 actuator 顺序与部署 mapping 中的 joint 顺序完全一致；不一致时会拒绝启动，
避免静默控制错关节。

## HPM 实机直连

实机启动前先确认两份接口包没有差异；有任何输出都不要联调：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
diff -ru robot_interfaces ~/zy_workspace/hpm_transport/ros2_ws/src/robot_interfaces
```

从 `l0_deploy_v2_ws` 父工作区使用全新的构建目录构建。`legged_real` 会直接使用
`robot_interfaces`，但仍复用 `system_command_arbiter` 包中的电机映射库；实机 launch 不启动
arbiter 节点。

```bash
source /opt/ros/humble/setup.bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --symlink-install \
  --packages-up-to robot_interfaces system_command_arbiter legged_real legged_controllers
```

HPM 和 deploy 使用两个终端，两个终端不要互相 source 对方的 workspace。

```bash
# 终端 1：先启动 HPM
source /opt/ros/humble/setup.bash
source ~/zy_workspace/hpm_transport/ros2_ws/install/setup.bash
ros2 launch hpm_system hpm_system.launch.py \
  enable_tcp:=true launch_motor:=true launch_imu:=true launch_tactile:=false

# 终端 2：再启动 deploy
source /opt/ros/humble/setup.bash
source ~/zy_workspace/l0_deploy_v2_ws/install/setup.bash
ros2 launch legged_controllers sim2real.launch.py
```

首次验证新 HPM 硬件时，不要启动完整 deploy 程序。架空并固定机器人、保持急停可用，
终端 1 只启动上述 HPM 节点；终端 2 运行以下测试。测试先发送一次全部禁用，然后按
`l0_v5.yaml` 的 index 0–20 顺序每秒只使能一个电机，不跳过任何电机。
每次发送同时打印关节名、HPM index、bus 和 motor ID：

```bash
source /opt/ros/humble/setup.bash
source ~/zy_workspace/l0_deploy_v2_ws/install/setup.bash
ros2 launch legged_real hpm_sequential_enable_test.launch.py allow_actuation:=true
```

日志中的 `Sent enable` 只表示 Reliable topic 已发布，不代表 HPM 返回了使能确认；
当前 `MotorEnableCommand` 协议没有 ACK。测试完成后电机保持使能，需通过急停、HPM
停机流程或断电退出，不要同时启动 `sim2real.launch.py`。

实机接口为 `/actuator/motor/command`、`/actuator/motor/state`、
`/actuator/motor/enable`，类型均来自 `robot_interfaces`；command 和 enable 为
Reliable depth 10，state 使用 SensorDataQoS。HPM 的 `/imu/data` 供 policy 使用；雷达发布的
`/odom` 仍只供 planner 使用。deploy 等到 enable topic 出现订阅者后只发布一次全电机 enable，
收到全部启用索引的首轮有效反馈后才允许发布运动命令。

电机缺包 watchdog 仅统计没有收到该电机新反馈的控制周期。新机器未校零导致关节位置或圈数
无法确认时，只报告位置未确认并限制运动命令，不累计通信缺包；非有限数据、无效状态和
持续电机错误仍按原有反馈故障处理。

电机反馈或 IMU 故障触发急停后，已使能且非校准状态下仍持续发布阻尼 MIT：
`kp=0`、`dq=0`、`tau=0`，`kd` 沿用 `emergency.damping_kd` 并经过现有电机映射限幅。
反馈故障和运动命令保护继续生效。失能、校准、使能切换或等待反馈确认期间继续发布
`kp=0`、`kd=0`、`dq=0`、`tau=0` 的 MIT；这类零增益帧不代表电机已执行失能。
这些帧的 `q` 取最近一次通过校验的关节位置，再经零偏、方向、减速比、圈数和限幅映射
成电机位置；没有新有效反馈时保持旧关节位置，从未获得有效反馈时使用初始关节位置 0。
调零核查期间也继续零增益 MIT，以持续获得位置反馈。核查只比较本次请求提交后的新反馈位置，
不使用 HPM 的应答状态码判定校零结果。离开校准或急停状态不会单独停止 MIT。

在 deploy 终端按 `Ctrl+C`（或向硬件桥发送 `SIGTERM`）时，硬件桥先停止处理 ROS 回调，
向 `/actuator/motor/enable` 发布全部配置中启用电机的 disable，最多等待 500 ms 的
DDS 投递确认。控制线程在等待期间继续发送零增益 MIT，随后直接终止进程，
以进程终止结束 MIT 发送，不提前析构控制循环或关闭 ROS。日志应依次出现 `Stopping hardware bridge`、
`Published HPM disable command` 和 `Shutdown disable acknowledged by DDS`。
请保持 HPM 运行到 deploy 退出；无订阅者、投递超时或发送异常会打印错误。
DDS 确认不代表电机已执行失能，仍需在机器人上核验电机状态；`SIGKILL` 无法执行该清理。

### 手柄零位校准

HPM 必须提供 `/actuator/motor/set_zero`，类型为 `robot_interfaces/srv/MotorSetZero`。
可先在机器人上执行只读检查：

```bash
ros2 service type /actuator/motor/set_zero
ros2 interface show robot_interfaces/srv/MotorSetZero
```

固定并支撑机器人，摆到机械标定姿态，停止其他控制端的运动及使能命令。急停状态下按
`LS+RS` 进入校准；等待 `Published HPM disable command`，并确认电机确实失能后再调零。
disable topic 没有设备 ACK；仅看到发布日志不能证明失能完成。

- `A+X`：左腿（含膝盖）；`B+Y`：右腿（含膝盖）。
- `LB+LT`：左臂；`RB+RT`：右臂。
- `START`：颈部；`BACK`：腰部（只有 mapping 中存在并启用了腰部关节时有效）。

只对所选组中 `enabled: true` 的关节发送请求，`motor_types` 使用 mapping 中的关节索引，
不是 CAN ID 或电机协议枚举。`request accepted` 只表示请求已提交；deploy 随后逐关节核查
`/actuator/motor/state` 中本次请求提交后的新反馈，原始电机 `q` 满足 `abs(q) <= 0.005 rad`
时打印 `POSITION_CONFIRMED`，对应映射后关节位置为该关节的 `zero_offset`（不含圈数补偿）。
核查只接受 `valid=true` 且有限的原始 `q`，不以关节限位或圈数确认状态代替原始位置核查。
HPM service 的 `CONFIRMED`、`TIMEOUT` 等结果不用于 deploy 的位置确认；HPM 内部发送策略未改。
deploy 不修改 mapping 的 `zero_offset`。

请求异步提交，等待期间继续零增益 MIT、反馈和手柄处理。未获得目标位置时继续等待并打印
当前收到的 `q`，没有调零核查超时，也不自动重发；目标未全部确认时仍禁止退出校准和使能。
服务不可用或请求提交异常仍报告错误。LS 电机提交请求时额外提示 `LS电机需要重启来保存参数`；
位置匹配不证明参数已经掉电保存，需按电机要求重启并核验。
所有位置确认后，按 `LS+RS` 退出并回到急停状态；退出会请求重新使能。调零会清除所选关节的
旧位置圈数补偿，并在后续有效反馈上重新计算；恢复运动前核验位置反馈与机械姿态一致。
整个校准过程禁止发送运动命令的提示是正常保护。

联调时检查类型、QoS 和实机 graph：

```bash
ros2 topic type /actuator/motor/command
ros2 topic type /actuator/motor/state
ros2 topic type /actuator/motor/enable
ros2 topic type /imu/data
ros2 topic info --verbose /actuator/motor/command
ros2 topic info --verbose /actuator/motor/state
ros2 topic info --verbose /actuator/motor/enable
ros2 node list | grep system_command_arbiter_node
ros2 service list | grep /actuator/motor/set_enabled
```

前三个类型应分别为 `robot_interfaces/msg/MotorCommand`、
`robot_interfaces/msg/MotorStateArray`、`robot_interfaces/msg/MotorEnableCommand`，IMU 应为
`sensor_msgs/msg/Imu`；最后两个命令应无输出。架空机器人并保持急停可用，先确认 HPM 的
`motor_type` 索引 0..20 均反馈正确、`/hardware_status/motors_ready` 为 true 且没有未知索引，
再允许进入控制状态。雷达 odom 与 planner 链路保持原样。

机器人升级后，以下旧配置可以人工检查并删除；本仓库脚本不会自动删除机器已有文件：

```text
/etc/modules-load.d/luvbot-gs-usb.conf
/etc/sudoers.d/can_setup
/etc/sudoers.d/candlelight_setup
/etc/udev/rules.d/candlelight.rules
```

## Todo List
* legged_real移植
* LeggedHardwareInterface中硬件接口底层实现
* 电机标定功能实现
* 接口延迟、系统稳定性测试


## ros1 ros2共存、切换 bash配置
```bash
 # ===== ROS environment switchers (ROS1 Noetic + ROS2 Foxy) =====
 export ROS1_SETUP=/opt/ros/noetic/setup.bash
 export ROS2_SETUP=/opt/ros/foxy/setup.bash

 # Unset ROS variables to avoid cross-contamination
 _ros_clean() {
   unset ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
   unset ROS_MASTER_URI ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
   unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
   unset CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
 }

 # Activate ROS1 (Noetic)
 set_ros1() {
   _ros_clean
   source "$ROS1_SETUP"
   echo "[OK] ROS1 activated: $ROS_DISTRO"
 }

 # Activate ROS2 (Foxy)
 set_ros2() {
   _ros_clean
   source "$ROS2_SETUP"
   echo "[OK] ROS2 activated: $ROS_DISTRO"
 }

 # Show current ROS env
 rosenv() {
   echo "ROS_DISTRO=$ROS_DISTRO"
   echo "ROS_VERSION=$ROS_VERSION"
   which roscore 2>/dev/null | sed 's/^/roscore: /' || true
   which ros2     2>/dev/null | sed 's/^/ros2: /' || true
 }
 # =============================================================
```



## ROS2北通手柄映射
```shell
axes
[0] - 左摇杆 左右
[1] - 左摇杆 上下
[2] - 右摇杆 左右
[3] - 右摇杆 上下
[4] - RT
[5] - LT
[6] - 十字键 左右
[7] - 十字键 上下
buttons
[0] - A
[1] - B
[2] -
[3] - X
[4] - Y
[5] -
[6] - LB
[7] - RB
[8] - LT
[9] - RT
[10] - back
[11] - start
[12] - home
[13] - 左摇杆 按下
[14] - 右摇杆 按下
```
雷达功能启动
最简日常启动顺序
以后设备已经安装、编译并配置好 IP 后，只需要：
# 终端0
cd ~/zy_workspace/l0_deploy_v2_ws
source install/setup.bash
ros2 launch legged_controllers sim2real.launch.py \
  subpolicy:=l0_v3_radar radar_control_enabled:=true

# 终端1
cd ~/zy_workspace/l0_deploy_v2_ws
source install/setup.bash
ros2 topic pub --once   /goal_pose   geometry_msgs/msg/PoseStamped   "{header: {frame_id: 'device0/odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}".
# 根据自己需求改数
