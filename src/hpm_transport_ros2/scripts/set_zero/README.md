<!-- Copyright (c) 2026 HPMicro. SPDX-License-Identifier: BSD-3-Clause -->
# set_zero：治具上的逐台电机标零

独立 ROS2 Python 包，所有新增内容位于 `deploy_ws/src/hpm_transport_ros2/scripts/set_zero`。只通过现有
ROS 接口工作，不直接访问 CAN/UDP、不修改 ID、不启动或重启 `hpm_system`。

## 上位机 Action 接口

板卡上直接执行 `~/start_motor_setzero.sh`，脚本在 UDP connected 后默认启动 Action 服务，等待客户端请求。也可显式指定 `--server`。

构建含 `luvbot_hardware_interfaces` 和 `set_zero` 的工作区并加载环境后，运行
`ros2 launch set_zero set_zero_server.launch.py`。服务器启动后等待请求。标零和测试分别由操作员触发：

```bash
ros2 action send_goal /set_zero/calibrate luvbot_hardware_interfaces/action/ModuleOperation "{module: left_leg}" --feedback
ros2 action send_goal /set_zero/motion_test luvbot_hardware_interfaces/action/ModuleOperation "{module: left_leg}" --feedback
```

上位机分别创建两个 ActionClient，用 `wait_for_server(timeout_sec=2.0)` 检查节点在线。
保存 `send_goal_async()` 返回的 goal handle，通过 feedback 显示阶段、当前关节、完成数和耗时。
`get_result_async()` 返回最终结果，错误包含 `error_code`、`message`、`failed_phase`、
`failed_index`（无对应电机时为 −1）及可用的日志路径。一个任务执行期间其他请求返回 `BUSY`。
运行中点击停止时调用该 handle 的 `cancel_goal_async()`，等待最终状态 `CANCELED` 和失能清理完成。
清理失败会返回故障，现有驱动接口没有物理失能确认。

标零成功后继续发送保持指令。测试必须指定当前保持的模组，测试完成并回位后可以重复测试。
新标零请求通过配置校验后会释放上一模组并重新预检。保持期间发生故障会清理电机，
下一次测试返回故障原因，需要重新标零。已结束的 goal 无法取消；退出服务器也会释放当前模组。

### 停止当前模组

```bash
ros2 topic pub --once /set_zero/stop std_msgs/msg/Empty '{}'
```

运行中收到停止消息，当前 Action 在停止和失能清理后返回 `ABORTED / STOP_REQUESTED`，
已发送的写零指令仍然有效。标零或测试成功后发停止消息也会释放保持并清除测试资格。
清理时新任务返回 `BUSY`，清理完成后可重新标零。失能请求发送出错时记录
`CLEANUP_FAILED`。单电机调试用 Ctrl+C 停止。

## 先使用不运动的入口

在工作区构建并加载环境（不要在已有控制任务运行时改动其环境）：

```bash
cd /home/a/HPM/my_project/99_09_motor_adc_imu/deploy_ws
colcon build --packages-up-to set_zero
source install/setup.bash
ros2 launch set_zero motor_auto_zero.launch.py motor_index:=14 dry_run:=true
```

`set_zero` 的普通增量构建会强制刷新 Python、YAML 和 launch 安装文件，避免远端时钟落后或旧产物时间戳在未来时漏更新。修改源码后仍需运行 `colcon build --packages-select set_zero`，再 `source install/setup.bash`；只执行 `source` 不会复制文件。请编辑源码目录中的配置，构建会用源码覆盖安装目录。此设置仅作用于 `set_zero`，其他包仍需保证系统时钟正确。

源码直接预览也不需要 ROS：

```bash
python3 /home/a/HPM/my_project/99_09_motor_adc_imu/deploy_ws/src/hpm_transport_ros2/scripts/set_zero/set_zero/motor_auto_zero.py --motor-index 14 --dry-run
```

`dry_run` 不创建节点、不访问设备、不写日志。配置中的 `null`/未确认方向会列入
`motion_errors`，预览仍返回 0。非法路由、未知目标、非法配置文件返回 2。

下面的在线查询会发送只读 ID 查询，须在设备连接/通讯工作已获授权时运行：

```bash
ros2 launch set_zero motor_auto_zero.launch.py motor_index:=14 check_only:=true
```

`config/motor_zero_calibration.yaml` 最外层（与 `defaults` 同级）控制是否写零，默认开启完整标定：

```yaml
write_zero: true
```

YAML 使用不带引号的 `true` / `false`，不是 launch 的 `:=` 写法。命令行显式指定优先于 YAML；未指定则读取 YAML，旧文件缺少此字段也默认为 `true`。
设为 `false` 时是寻位诊断：会使能和运动，检测到停滞后输出 `CONTACT_CANDIDATE`，随后失能退出；不写零、不回程、不报告标定成功。临时关闭可用：

```bash
ros2 launch set_zero motor_auto_zero.launch.py motor_index:=17 write_zero:=false
```

实机已出现中途摩擦卡住被误判限位并写零的问题。位置稳定、低速和力矩条件只能识别停滞，不能独立证明已碰到机械挡块。正式自动写零需要先在治具上验证判据，必要时接入独立限位开关或已知位置基准；仅延长等待或增大力矩不能消除该歧义。误写后的电机零坐标不能作为 URDF 零位使用。

使用默认配置启动完整标定（会写零并回到 URDF 零位，需完成对应关节的工艺验证）：

```bash
ros2 action send_goal /set_zero/calibrate luvbot_hardware_interfaces/action/ModuleOperation "{module: left_arm}" --feedback
ros2 launch set_zero motor_auto_zero.launch.py motor_index:=17
# 直接入口保留 --motor-index，不是 CAN ID：
ros2 run set_zero motor_auto_zero --motor-index 17
```

仍可用 `write_zero:=true/false` 临时覆盖；直接入口对应 `--write-zero` / `--no-write-zero`。

必须指定且只能指定 `module` 或 `motor_index`。模组为 `left_arm`、`right_arm`、
`left_leg`、`right_leg`、`neck`。`dry_run` 与 `check_only` 互斥。

## 工艺参数

编辑 `config/motor_zero_calibration.yaml`。所有目标的参数通过验证后才创建运动
流程；不采用“第一个能动就先动起来”的方式。未选关节的空参数不影响本次运行。

双臂各自的顺序及工艺方向（已按用户要求交换左右臂标定参数；电机与 URDF 正方向一致、反馈为输出轴弧度）：

| index | 关节 | 寻限位速度符号 | 接触角度 | 写零后回 URDF 零位的电机目标 |
|---|---|---|---|---|
| 14 | left_elbow_roll_joint | + | +90° | -90° |
| 13 | left_shoulder_roll_joint | - | -90° | +90° |
| 12 | left_shoulder_pitch_joint | + | +150° | -150° |
| 17 | right_elbow_roll_joint | - | -90° | +90° |
| 16 | right_shoulder_roll_joint | + | +90° | -90° |
| 15 | right_shoulder_pitch_joint | - | -150° | +150° |

`home_angle_deg` 是机械接触位置对应的 URDF 角度，不是电机当前反馈。
写零后 `q_urdf = q_motor + radians(home_angle_deg)`，因此回零目标自动取其相反数。
`seek_direction` 只决定寻位速度的正负，与 `home_angle_deg` 独立配置，不再由它选择 URDF 上限或下限。例如膝伸直处是 0°、电机须正转才能到该挡块时，可配置 `seek_direction: 1`、`home_angle_deg: 0.0`；写零后的回零目标就是电机坐标 0，不额外回转 113°或 135°。
到达后不会第二次写零。左右方向不要再叠加旧 `l0_v5.yaml` 中的 motor_direction。

左右臂对应关节的标定参数已互换，电机索引、ID、CAN 总线及模组内执行顺序不变。
双臂保留 `direction_confirmed: true`；腿、颈仍为
负方向碰图中下限的未确认假数据。不存在绕过确认标记的命令行开关。确认并修改逐关节配置后
才允许真实标定。`direction_confirmed` 只确认方向，不能替代驱动和治具验收。

公共检测、速度与安全阈值放在 `defaults`，逐关节同名字段覆盖公共值。
寻位、回程和保持的 `kd`、`bck_kp`、`bck_kd`、`hold_kp`、`hold_kd` 必须写在各自的 `joints` 条目内；
不再接受 `defaults` 中的公共增益，也不再提供 `kp:=...` / `kd:=...` 等批量增益覆盖。
调某个电机时修改它自己的条目，launch 的选模组/单电机用法不变。
其他数字参数仍可通过 CLI/launch 覆盖，例如 `seek_speed_rad_s:=...`、
`stall_window_s:=...`；CLI 对应 `--seek-speed-rad-s`。

寻限位和回零目标速度默认均为 `0.47 rad/s`（约 `26.9°/s`），所有关节均取消软件加减速斜坡，进入运动阶段直接发送对应目标速度，到达规划终点后速度指令归零。
`acceleration_rad_s2` 已从配置及 CLI/launch 参数中删除，旧配置含此字段会阻止运动。寻限位固定 `kp=0`，只使用该关节的 `kd`；
回零用该关节的 `bck_kp/bck_kd` 跟踪位置轨迹；配置 `final_mode: damping` 时到位后切为纯阻尼，持续发送 `q=0、dq=0、kp=0、kd=damping_kd、tau_ff=0`。`defaults.damping_kd` 默认为 0.2，允许逐关节覆盖；纯阻尼不锁位置、不静态承重，治具须托住模组。
因此配置中的 `bck_kp/hold_kp` 继续保留。前馈力矩通过逐关节 YAML 字段配置，默认均为零。

### 每关节前馈力矩（N·m）

在已有 `joints.<关节名>` 条目内修改这三个字段，不要新增重复关节键：

```yaml
seek_torque_ff_nm: 0.0  # 寻位、接触判停；接住时继承当时的合计支撑输出
move_torque_ff_nm: 0.0  # 回 URDF 零位，包括轨迹终点的到位判定
hold_torque_ff_nm: 0.0  # 已回零后的固定保持前馈（开启自适应时由自适应项替代）
```

三个字段只能逐关节填写，不能放在公共 `defaults`，未填写按 0 处理；不提供批量 launch 覆盖。
正数为电机/URDF 正方向，负数为负方向，**不再乘 `seek_direction`，回程也不自动反号**。
例如 `seek_torque_ff_nm: -0.1` 表示寻位阶段增加负向 0.1 N·m 前馈；这只是语法示例，不是已验收的调参值。
负载补偿方向由重力/摩擦决定，不能直接把寻位前馈复制为反号回程前馈。

除 LS 外，首次请求 SETZERO 后，写零提交和补发使能期间持续零输出 MIT，通过写零后回程检查后开始正常回程；失控时才切纯阻尼刹车。退出清理的前馈为 0。关闭自适应时，回程使用固定 `move_torque_ff_nm`；回位完成后前馈清零。
固定前馈不会自动爬升；它也不是额外的力矩额度。运行前拒绝非数值、NaN/Inf、超协议量程及超预算配置，
运行中按 `tau_ff + kp*(q_target-q) + kd*(dq_target-dq)` 检查合计估算值；MOVE 超预算切自动刹车，HOLD/RECOVER 采用保持限幅，其他阶段保留中止保护；反馈力矩保护继续保留。
日志的 `tau_ff_nm` 记录实际下发前馈。这里只提供接口，不自动加入 1 N·m，也不保证消除实机振荡。
修改远端源码 YAML 后，重新构建 `set_zero` 并 source 工作区，或用 `calibration_config:=绝对路径` 直接指定该 YAML。

### 逐关节力矩保护上限

`max_torque_nm` 已逐关节写入 `joints.<关节名>`，`defaults.max_torque_nm: 3.0` 继续保留，作为缺少关节字段时的缺省值。公共值不是再套一层的全局上限：例如右膝关节设为 36.0 时，解析结果就是 36.0，不会再被公共的 3.0 截小。显式 CLI/launch `max_torque_nm:=...` 仍按原规则覆盖本次所有选中关节。

本次初始值依据当前 `motors.yaml` 的协议正负量程取较小幅值：双臂 PA35 为 3 N·m；左右髋 Pitch 和膝关节 RS 为 36 N·m；左右髋 Roll、髋 Yaw、踝 Pitch 的 PA43 为 18 N·m；踝 Roll 和颈部 LS 为 23 N·m。这是软件协议上限，不是厂家连续额定力矩或经过实机验证的标定安全值，可在各关节中单独调低。未改 URDF、角度、方向、增益、前馈或方向确认状态。

现有测得力矩保护、指令估算预算及 HOLD/RECOVER 限幅都会使用该关节解析后的值；协议量程上限仍独立检查。`brake_max_torque_nm` 和 `adaptive_torque_limit_nm` 是另两项限制，不会随 `max_torque_nm` 自动增大，仍分别限制纯阻尼刹车估算值和自适应补偿幅值。

### 回零后的最终模式（可逐关节覆盖）

```yaml
defaults:
  final_mode: damping  # 默认纯阻尼；改成 position 则保持 URDF 零位对应的电机目标
```

仅 YAML 配置，支持在 `joints.<关节名>` 内覆盖公共模式。`damping` 保持现有纯阻尼流程；`position` 在到位后使用该关节已有的 `hold_kp`、`hold_kd` 锁住回程终点。它保持的是 `-radians(home_angle_deg)`，不是再次把电机目标置零。自适应开启时延续有界积分；关闭时使用固定 `hold_torque_ff_nm`。模式选择不改变回程中的自动刹车流程。
保持偏差触发恢复用 `hold_error_deg`，到位判据用 `arrival_tolerance_deg` / `arrival_velocity_rad_s` / `arrival_settle_s`；10° 是允许误差，不是禁止修正的小死区。所有最终模式都允许 `hold_kp >= 0`，负数及非有限数会被预检拒绝。`hold_kp: 0` 时 P 项不提供位置恢复力矩，实际输出由阻尼及固定/自适应补偿决定。位置保持仍可能因负载、增益或反馈延迟发生振荡，现有数值未经该姿态实机稳定性验证，不能仅增大 kp 或放宽超时。
`RETURN_START` / `MOTOR_PASS` 记录最终模式，`CALIBRATION_PASS` 的 `final_states` 给出每个关节的 HOLD/DAMPING 状态；混合模式的批次状态为 MIXED。

### 保持偏离后的支撑恢复

`final_mode: position` 时，HOLD 偏差超过现有 `hold_error_deg` 不再报 HOLD_POSITION_ERROR 退出，而进入 RECOVER：目标仍为回程终点，速度目标为 0，临时复用本关节已有的回程 `bck_kp/bck_kd`；配置中的 `hold_kp/hold_kd`、增益与力矩上限不变。位置和速度连续满足 `arrival_tolerance_deg`、`arrival_velocity_rad_s`、`arrival_settle_s` 后，尝试切回原 HOLD。

自适应补偿持续保留并按原有速率、幅值上限更新。恢复转 HOLD 所需的补偿超出 `adaptive_torque_limit_nm` 时，继续 RECOVER，不撤支撑、不提高积分上限。MOVE 首次到位遇到同类交接限制，也会保持 RECOVER；此时已满足到位条件，MOTOR_PASS 和最终状态如实记录 RECOVER，控制继续运行。

HOLD/RECOVER 的目标总力矩仍按原控制器计算；需求超过 `max_torque_nm` 或协议量程时，保持有界前馈，并按同一比例降低实际下发的 P/D 增益，使合计估算值落在允许范围，需求高于限值时取允许上限。YAML 中的增益、前馈和限值均不改，不会无条件额外增加固定力矩。自适应开启时，`hold_torque_ff_nm` 被自适应项替代；关闭时，HOLD 和 RECOVER 都使用该固定保持前馈。

`HOLD_RECOVER_START` 记录触发原因、实际增益、前馈和估算输出，`HOLD_RECOVERED` 标记恢复为原 HOLD；最近命令记录 `hold_limited`、`hold_requested_torque_nm` 和 `hold_applied_torque_nm`。不因保持偏差或保持指令估算限幅退出，但真实力矩超限、驱动故障、无效/超时/跳变反馈仍按硬保护退出，Ctrl+C 仍统一失能。持续力矩控制是否能承重、是否稳定仍需治具旁实机验证。

### 回程与可选位置保持的自适应补偿

全部 21 个关节已逐关节启用以下起始参数，仍可在各自 YAML 条目中独立调整。腿和颈原有的方向确认状态保留，尚未确认方向的关节仍会被运动预检拦截。旧配置缺少这些字段时维持固定前馈行为。这些数值是起始参数，不代表所有关节均已完成实机调参。

```yaml
adaptive_torque_enabled: true
adaptive_ki: 1.0                  # N·m/(rad·s)，位置误差积分增益
adaptive_torque_limit_nm: 1.0      # 正负补偿幅值上限，不是固定输出
adaptive_torque_rate_nm_s: 0.25   # 积分补偿每秒最大变化量
```

只允许逐关节填写，不支持公共 defaults 或 CLI 覆盖。开启后，MOVE 的 MIT 力矩字段由有界积分生成，**替代而非叠加**固定 `move_torque_ff_nm`；关闭自适应后，回程才使用固定回程前馈。`final_mode: damping` 时完成后前馈清零；`final_mode: position` 时 HOLD/RECOVER 继续自适应补偿，关闭自适应才使用 `hold_torque_ff_nm`。寻位/写零阶段的前馈不变。

首次进入 MOVE 时积分从 0 开始；刹车时冻结并保存原积分，恢复回程时继承刹车前的有界积分，只按新的、时间递增的反馈更新 `I += Ki * (q_target-q) * dt`；重复反馈不累积，反馈时间间隔超过超时阈值不补算。积分限幅、变化速率限制和总力矩饱和方向的积分冻结同时生效，反向误差可以解除积累。既有总指令力矩保护和实测力矩保护仍独立生效，补偿不是额外额度。

每个关节回位后根据 `final_mode` 选择 DAMPING 或 HOLD，等待后续关节及全部完成后继续该模式。DAMPING 清空自适应补偿；HOLD 使用逐关节 `hold_kp/hold_kd`，自适应开启时延续补偿、关闭时使用 `hold_torque_ff_nm`。MOVE 切 HOLD 时将增益变化的 PD 差额转入补偿，所需补偿超过上限时，进入 RECOVER 继续用已有回程增益支撑，不因交接退出或强行提高积分上限。刹车阶段不更新积分，也不下发前馈补偿。

按工艺要求，YAML 中 `arrival_tolerance_deg: 10.0` 和 `hold_error_deg: 10.0` 直接使用角度制；脚本内部转换为弧度，PLAN 中的 `_rad` 数值是转换后的有效值。支持逐关节同名覆盖，以及 launch `arrival_tolerance_deg:=10 hold_error_deg:=10`（直接 CLI 用 `--arrival-tolerance-deg` / `--hold-error-deg`）。优先级为 CLI/launch > 逐关节 > defaults，旧 `_rad` 配置仍可使用，但同一层不能同时填写同一参数的 `_deg` 和 `_rad`。仍要求低速持续满足到位时间，回程/保持的反馈跳变和力矩保护不变。此值允许带角度残差进入 DAMPING，不代表编码器角度精确归零。

`RETURN_START` 标记是否启用自适应，`MOTOR_PASS` 记录补偿值，故障诊断及最近命令包含 `adaptive_torque_nm`、`adaptive_limited` 和实际 `tau_ff_nm`。补偿需要偏差和时间才能建立，不能保证刚使能就接住负载；不同惯量、摩擦和姿态仍需现场调参。离线正反负载模型通过不代表实机不振荡。补偿变化速率限制不改变用户选择的直接速度指令。

参数参考来自 2026-11-05 对 `192.168.120.16` 的只读检查：启动进程选择
`l0_v5_amp_11022`，其已安装策略文件为
`/userdata/forlinx/zy_workspace/l0_deploy_v2_ws/install/legged_deploy/share/legged_deploy/policy/l0_v5/l0_v5_amp_11022/rl_config.yaml`。
运动映射的 reduction 均为 1，增益不再经减速比缩放。以下历史回零 `kp`（现为 `bck_kp`） 沿用策略值；
`kd` 为兼容现有接触门槛和力矩预算做了调整，**不是原样复制行走策略增益**。

| 关节（右肘、右肩 Roll 覆盖见下文） | sim2real kp / kd | 回零 kp / kd（寻限位 kp 固定为 0） | 等待保持 kp / kd |
|---|---|---|---|
| HipPitch、HipRoll、HipYaw、Knee | 11.8435 / 1.508 | 11.8435 / 0.75 | 11.8435 / 0.75 |
| AnklePitch | 21.4763 / 2.7344 | 21.4763 / 0.75 | 21.4763 / 0.75 |
| AnkleRoll | 5.4490 / 0.6938 | 5.4490 / 0.6938 | 5.4490 / 0.6938 |
| ShoulderPitch、ShoulderRoll、ElbowRoll | 5 / 0.04 | 5 / 0.5 | 5 / 0.5 |
| NeckYaw、NeckPitch、NeckRoll | 5 / 1 | 5 / 0.75 | 5 / 0.75 |

历史调参记录：右肘 index 17 和右肩 Roll index 16 曾使用配置 `kp=5、kd=0.3、hold_kp=1、hold_kd=0.3`；其余关节保持表中值。
右肘寻位直接发送 `-0.47 rad/s`，回程速度为 `0.47 rad/s`，前馈力矩为零。用户已确认该配置下右肘寻位过程正常并确实碰到机械挡块；完整写零、回程及其他关节尚未完成实机验收。

按脚本模型，静止且位置误差为零时，寻限位转矩估算为
`seek_torque_ff_nm + kd × seek_direction × seek_speed`；前馈为零时右肘转矩幅值为 `0.3 × 0.47 = 0.141 N·m`。软件力矩保护阈值由 `max_torque_nm` 指定，这不是固定输出力矩或硬件限矩。它不能证明足以克服摩擦或重力；起步不动也按相同的停滞判据处理，未满足判据则受寻位总超时限制，不自动加力。预检按前馈与速度项的合计检查接触门槛和最大力矩预算。
力矩阈值不再读取或受 URDF `effort` 限制，只检查电机协议量程。PA35 当前协议配置为 ±3 N·m，因此 `max_torque_nm: 3.0` 可以通过该检查；运行中的反馈力矩和指令估算力矩超限中止仍保留。零位偏移由 YAML 的 `home_angle_deg` 指定；URDF 的关节信息和速度检查继续使用。
不要把图片的最大速度/转矩用作标定速度/转矩。

| 参数 | 起步值 | 用途 |
|---|---|---|
| `contact_torque_nm` | 0.05 N·m | 与位置稳定、低速和持续时间一起判断接触 |
| `max_torque_nm` | 2.0 N·m | 实测/估算转矩的软件中止阈值，不是硬件限矩 |
| `max_temperature_c` | `null` | 取消上位机温度上限；温度仍记录，驱动故障码仍中止。填正数可恢复上限 |
| `max_seek_time_s` | 30 s | 每台电机寻限位超时 |
| `max_seek_travel_rad` | 5.4 rad；双臂肘和肩 Roll 为 3.3 rad | 相应关节全行程外留少量裕量 |
| `max_return_time_s` | 25 s | 覆盖当前 90°/150° 回零轨迹及稳定等待 |
| `max_feedback_step_rad` | 0.02 rad | 相邻新反馈位置跳变中止阈值，约 1.15° |

当前匀速位置轨迹以 `0.47 rad/s` 回转 90°/150°约需 3.34/5.57 秒，实际到位还取决于负载和控制误差。
腿和颈仍为未确认方向，逐关节增益已填写也不能视为其验收参数。
共享的检测时间/容差是软件初始值，也需要在实际反馈频率、量化和噪声下验证。
程序检查正常寻限位在窗口内的位移明显大于判停位置阈值。

`motor_config_file` 默认复用 hpm_motor 包的 `motors.yaml`；实际服务返回的路由/型号
必须与它一致。`calibration_config`、`urdf_file` 可指定外部文件。默认 URDF 是用户
提供文件的原样副本，仅解析关节/限位，不需要其外部 mesh 文件。
图片的膝关节上限 135° 与 URDF 113° 不一致，URDF 限位不自动证明真实机械挡块位置。
标定接触角度以经过现场确认的 YAML `home_angle_deg` 为准，不再因其与 URDF 端点不一致而拒绝标定。
方向确认、角度有限数值、回零目标的电机协议位置范围及运行中的速度/力矩/行程保护仍保留，不自动修改 YAML 或 URDF。

## 控制流程和边界

先读所有目标 ID（每个目标/LS 总线 3 次），RS/MR 定向查询，LS 广播查库存。底层只读 ID 查询收集窗口上限为 200 ms（若 `motor_id_calibration_timeout_ms` 更短则沿用较短值）；三次通常约 0.6 秒，另有 ROS 通信开销。写 ID、保存 ID 的应答超时仍使用原配置。此窗口属于 `hpm_motor`，更新底层库后需要重新启动 `hpm_system` 才会生效。
已配置的 LS 旁路电机可以回应，未知 ID/目标缺失/超时即停止。该检查仅证明
这些查询成功，不等于长期零丢包，也不能保证所有协议均能识别同 ID 冲突。
标定要求正常模式 `motor_id_calibration_mode=false` 和 `launch_motor=true`。

当前电机逐台使能，寻限位使用 MIT `kp=0` 和该关节的 `kd`，发送有方向的速度目标；
位置字段随反馈更新，但不参与寻限位控制。回零时使用 `bck_kp/bck_kd`，位置和速度目标
来自规划轨迹；到位等待其他关节时持续所选最终模式。前馈力矩按下文逐阶段字段发送。
无需先发生位移：启动时已贴限位或中途卡住，均使用相同的停滞判据，不自动反向退让/加力。已删除 `NO_INITIAL_MOTION` 及 `startup_motion_timeout_s`；`minimum_motion_rad` 仅保留为反方向位移保护的容差。位置窗口、低速、转矩阈值持续
满足后，只能得到候选接触点；`write_zero=false` 时停止并退出。`write_zero=true`（默认）时，再检查参考位置没有回弹，才请求 SETZERO。
当前条件为：至少 5 个新反馈覆盖 0.5 秒，位置极差不超过 0.002 rad、窗口平均位置速度绝对值不超过 0.02 rad/s、窗口最低力矩绝对值不小于 0.05 N·m，再持续满足 0.3 秒。理想稳定反馈下约 0.8 秒进入停转阶段；不能据此区分真实挡块和摩擦卡住。
判停和首次写零准备期间维持原寻位输出。对非 LS 电机，流程随后为：只发送写零 → 连续使能及零输出 MIT → 零坐标与使能状态持续满足交接条件后回程 → 必要时自动纯阻尼刹车、稳定后重新回程 → 到位后持续所选最终模式。

非 LS 标定使用 `/actuator/motor/set_zero_send`（仍为 `MotorSetZero` 类型）。服务只检查请求合法性及 `send_motorcmd` 是否成功交给传输层，不等待普通 MIT、专用 ACK 或零坐标验证；`result=0` 在这个入口仅表示已发送，不表示设备已写入或已持久化。原 `/actuator/motor/set_zero` 保留原有等待反馈语义，其他调用者不变。新增入口需要重启更新后的 `hpm_system`，旧节点不提供该入口，脚本将预检失败，不会默默回退。
MR/RS/DM 按 `zero_max_attempts` 固定发送（默认 3 次），ENC 发一次；LS 采用下文专用确认流程。对其余协议，每次传输提交返回后立即连续补发 3 次使能，并紧接 MIT 控制帧，写零提交等待和 BRAKE 阶段继续按 `rate_hz` 补发使能/MIT。MOVE/HOLD/DAMPING 阶段不再周期补发使能。传输发送失败/服务超时仍中止；回程前独立检查实际零坐标及使能状态。首次写零前的接触参考检查保留。

所有关节自动选择回程或刹车，不再配置 `catch_enabled`。旧 `catch_enabled`、`catch_torque_nm`、`zero_damping_kd` 已从本包 YAML 移除；外部旧 YAML 中保留时也不参与控制。

```yaml
defaults:
  brake_kd: 0.2
  brake_max_torque_nm: 0.5
  brake_speed_ratio: 1.5
  brake_speed_margin_rad_s: 0.2
  brake_position_error_deg: 10.0
  brake_confirm_s: 0.1
  brake_overspeed_confirm_s: 0.05
  brake_emergency_speed_ratio: 2.0
  motion_settle_position_deg: 0.1
  brake_velocity_rad_s: 0.02
  brake_settle_s: 0.2
  brake_timeout_s: 2.0
  max_brake_attempts: 3
```

以上参数支持在各 `joints.<关节名>` 已有条目内覆盖。`brake_kd` 只用于临时刹车，`damping_kd` 只用于最终纯阻尼，两者独立。

流程先正常回程；速度绝对值超过 `return_speed_rad_s × brake_speed_ratio + brake_speed_margin_rad_s`，且在时间递增的新反馈上持续 `brake_overspeed_confirm_s` 后进入 BRAKE；低于该阈值的新反馈会重置计时。默认门槛为 0.905 rad/s、持续 0.05 秒。超过该门槛的 `brake_emergency_speed_ratio` 倍则立即刹车，默认 2 倍即 1.81 rad/s；此分支不等待确认。相对当前轨迹的位置误差超过 `brake_position_error_deg` 并持续 `brake_confirm_s` 后也进入 BRAKE。位置误差比的是当前时刻的轨迹点，不是最终 URDF 零位，所以正常尚未回到终点不会触发。有效反馈越出原回程走廊、或 MOVE 候选指令的合计估算力矩超出总预算时，也切为 BRAKE；不会把超预算的普通回程指令发出去。


正常回程中，如果位置反馈确认关节正沿目标方向前进、仍落后轨迹且滞后不超过 `return_max_lead_deg`、离终点超过 `arrival_tolerance_deg`，则允许受限的轨迹追赶。软速度门槛临时取 `max(原软门槛, min(原紧急门槛, return_speed_rad_s + 正向跟踪差 / brake_confirm_s))`，超过它仍需持续 `brake_overspeed_confirm_s` 才刹车。明显高速的紧急门槛始终按原软门槛乘 `brake_emergency_speed_ratio` 计算，不随追赶放宽；反向运动、位置无前进证据、领先轨迹、接近终点或反馈跳变均不放宽。目标速度、增益、前馈、位置/力矩保护均不改变。`RETURN_STATUS` 新增 `catchup_active`、`speed_limit_rad_s` 和 `emergency_speed_rad_s`，分别表示当前是否允许追赶、实际软门槛和固定紧急门槛。


左踝 Pitch（index 4）先启用回程超速附加阻尼：在该关节 YAML 内配置 `return_overspeed_kd: 1.0`、`return_overspeed_torque_limit_nm: 0.5`。仅 MOVE 阶段、反馈沿指令方向且比当前速度目标更快时，在 `bck_kd` 上增加阻尼；附加项估算力矩幅值不超过 0.5 N·m，始终抵抗当前运动。总 kd 受协议范围约束，总指令力矩仍经过原预算检查。阻尼生效时，禁止积分继续累加助推运动的补偿，允许卸掉已有补偿；静止克服摩擦时仍沿用原积分规则。寻位、HOLD/RECOVER、纯 BRAKE 参数及超速触发阈值不变。

`return_overspeed_kd: 0.0` 可关闭附加阻尼；未配置的其他关节默认关闭。两项均可在各关节 YAML 配置，增益须逐关节指定。`RETURN_STATUS.speed_damping_nm` 显示附加项估算力矩，`kd` 显示实际总增益；这不是额外固定前馈，也不是把力矩提高到总上限。固定 `move_torque_ff_nm` 在自适应启用时仍由积分项替代。

BRAKE 发送纯 MIT 阻尼 `q=0、dq=0、kp=0、tau_ff=0`。力矩方向由驱动内部实际速度决定，估算为 `-kd × dq_feedback`；速度换向后阻尼方向随之换向，不锁定方向、不发固定正负前馈。实际 kd 为 `min(brake_kd, brake_max_torque_nm / abs(dq_feedback))`，静止时使用配置 kd。该限值独立于 `max_torque_nm` 且不能超过总预算或协议量程；是软件估算限值，不能代替硬件限矩。

BRAKE 期间积分不更新；解除刹车要求至少 5 个时间递增的新反馈覆盖 `brake_settle_s`，窗口位置极差不超过 `motion_settle_position_deg`（默认 0.1°），且窗口首尾位移 / 时间的平均速度绝对值不超过 `brake_velocity_rad_s`。单帧速度小幅跳动不再重置静止计时；明显高速反馈会清空该窗口。满足后从实际位置重新规划到 URDF 零位，继承刹车前的有界补偿；越过终点则反向回程。

回程 MOVE、保持恢复 RECOVER 的到位使用同一窗口判据：位置须在整个 `arrival_settle_s` 窗口内处于 `arrival_tolerance_deg` 范围，位置极差不超过 `motion_settle_position_deg`，窗口平均速度不超过 `arrival_velocity_rad_s`。即使回程开始时已接近终点，也必须等到窗口完整。低频控制时窗口至少覆盖 4 个控制周期，以容纳 5 个反馈；反馈时间倒退、长于 `feedback_timeout_s` 的间隔会清空窗口，重复序号不能推动判定。大幅往复运动不能靠首尾抵消通过位置极差检查。这里使用主机接收时间，不能替代驱动采样新鲜度验证。

这三个新增参数可放在 defaults 或对应关节条目内；旧 YAML 缺少时自动采用上述默认值。最近命令新增 `motion_span_rad`、`motion_mean_velocity_rad_s`，只有完整窗口时才有值，单位仍为弧度及弧度/秒。每次刹车最多 `brake_timeout_s`，总次数最多 `max_brake_attempts`；恢复不会重置总回程 `max_return_time_s`。速度稳定不代表重力得到静态支撑，纯阻尼无法悬停重力负载；接不稳仍报 BRAKE_TIMEOUT。

`BRAKE_START` 记录触发原因、次数、配置/实际 kd 和刹车估算力矩上限；`BRAKE_RESUME` 记录重规划起点和目标。已完成后的 HOLD 偏差超 `hold_error_deg` 会进入 RECOVER，按下文继续支撑；这段不采用回程 BRAKE 的超时/重试策略。实测力矩超限、无效反馈和通讯超时仍先检查并直接中止。MOVE 中先判断超速、持续轨迹误差、越出走廊及回程指令预算，满足时优先输出纯阻尼；未满足这些条件时，位置跳变仍报 POSITION_JUMP。其他阶段（含已进入的 BRAKE 和 RECOVER）的跳变硬保护保留。POSITION_JUMP 诊断和 BRAKE_START 日志记录新旧位置、位置差及反馈间隔，便于区分实际运动与反馈异常。驱动故障和控制循环卡顿仍中止并清理。

除 LS 外，写零服务返回传输提交结果；后续回程检查使用实际位置和使能反馈。驱动内部写零期间的失能也无法靠主机命令保证消失。使能和接住命令连续发布不代表物理零间隔或实际已恢复输出。所有未确认方向的关节仍受运动预检限制。

MIT 的 tau 是前馈转矩，并非独立的驱动器限矩参数。软件检查实测及估算的转矩
超限只属于辅助保护；必须另行验证驱动器电流/力矩限制、实际零位写入条件、
底层逐电机命令超时及急停。上位机崩溃/断网后不能依靠 Python 清理保证停止。
ROS stamp/sequence 是发布端信息，不能证明设备采样新鲜；上线前验证底层快照
没有重放/复用陈旧采样。机械障碍物也可能满足堵转判据，治具须保证运动路径。

每批使用一个本机进程锁，并检查竞争的运动/使能发布者。这是误操作防护，
不是跨主机的硬件控制权仲裁；所有其他控制器/标零操作须在工艺上停用。

默认 `final_mode: damping` 时，全部完成后切入 `DAMPING`，输出 `CALIBRATION_PASS`，继续按 `rate_hz` 发布：
`q=0, dq=0, kp=0, kd=0.2, tau=0`，按 Ctrl+C 结束。这里除 kd 外为零；
kd 也为零只会卸力，没有阻尼。`damping_kd` 可调整，默认值参考上述 sim2real 启动
命令中的 `emergency_damping_kd:=0.2` 及硬件适配器的阻尼分支；它不是实测报文证据。
原 sim2real 阻尼位置字段跟随反馈，本脚本用零，因为 kp=0 时它不产生位置力矩。
启动/标定时的 lifecycle 零增益分支可以优先于阻尼，所以不能把所有启动状态都称为强阻尼。
阻尼抵抗运动，不锁住 URDF 零位；治具须继续支撑重力负载。
此时仍检查反馈新鲜度、位置跳变、温度、实测和估算转矩，异常仍退出失能。
Python/ROS/Linux 不是硬实时系统；`rate_hz` 是发送调度目标，不保证网络断开、系统卡顿
或底层发送失败时驱动仍不超时，驱动端实际收帧间隔须另行测量。
人工结束或故障时，先停止正常控制发布，再对本次所有已尝试使能的电机发送
零输出和失能（有限次数重发），不操作未选电机。治具须能承受统一失能。
现有 enable topic 没有物理失能应答，日志明确记录 `disable_confirmed:false`。
先确认部件被支撑且停止，再下件。没有自动续跑、回滚零位或运动测试；非 LS 写零失败后不自动重试。

## 结果和离线验证

终端和 JSONL 由独立日志线程按顺序输出，文件默认位于当前目录的 `set_zero_logs/`，可用
`log_dir:=...` 修改。记录配置路径及实际参数、ID 查询、逐电机结果、停止请求和
整批结果。`MOTOR_PASS` 包含接触位置、写零发送记录、目标/实际角度、误差、峰值转矩
和耗时。非 LS 置零超时结果未知，不自动重发；中途失败记录剩余 `not_started`。
日志写盘/终端阻塞不会阻塞控制循环；有界队列满或写盘失败会触发停止清理，
退出时限时等待日志排空。写零成功不等于断电后持久化已经验证。

触发 `COMMAND_TORQUE_BUDGET` 时，`MOTOR_FAIL` / `RUN_FAIL` 的 `diagnostics`
记录被拦截的目标位置/速度及增益、反馈位置/速度/实测力矩/温度/序号/年龄，
以及位置项、速度项、合计估算力矩与阈值。`command.published:false` 表示这帧未发布。
另保留前一帧新反馈，输出两帧的位置差、ROS 接收时间间隔和位置差分速度；
该速度使用 ROS 接收时间，不是板端采样时间，只用于诊断。没有前一帧时相应字段为 `null`。
单电机状态机报错时还输出 `recent_ros_commands`：最近 20 条已完成 ROS 发布的指令，
包含相对使能时间、阶段、目标、增益及计算时使用的反馈。被拦截或发布报错的指令不加入记录。
这是主机 ROS 发布记录，不代表 CAN 已送达或电机已执行；不能用未发布的候选指令解释此前的运动。
这些记录仅缓存在内存，故障时输出，不增加每周期打印。

`stall_velocity_rad_s` 默认改为 `0.02 rad/s`，覆盖 PA35-10-8 的量化静止反馈约
`±0.01588 rad/s`。寻限位、停止、写零准备、到位与刹车解除均采用窗口位置平均速度，不再受 MIT 瞬时速度量化下限约束。
位置阈值和接触力矩条件保持不变；总力矩保护采用每关节解析后的 `max_torque_nm`。

退出码：正常检查/完成后人工结束 0，运行失败 1，配置错误 2，未完成时人工
中断 130。持续位置保持中尚未退出，以上屏结果为准；之后发生故障会追加失败记录。

```bash
cd /home/a/HPM/my_project/99_09_motor_adc_imu/deploy_ws/src/hpm_transport_ros2/scripts/set_zero
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

离线测试使用模拟反馈/服务，不访问真实电机。构建、测试通过不代表实机标定
已经验收。实机按读 ID、左肘单台、左臂三台的顺序另行验证。
测试关闭全局 pytest 插件自动加载，避免机器上其他项目的插件版本影响本包；
加载 ROS 工作区环境后，还会用真实消息类型和假 Node 验证接口，不创建 ROS 图。

### 寻限位窗口判停与诊断

SEEK、STOP 和 ZERO_PREPARE 统一使用位置窗口判静止，不再要求每帧 MIT 瞬时速度都低于阈值。SEEK 使用 `stall_window_s`、`position_epsilon_rad`、`stall_velocity_rad_s`：至少 5 个时间递增的新反馈覆盖窗口，位置极差不超过位置阈值，窗口首尾位移除以时间的平均速度绝对值不超过速度阈值；窗口内每帧力矩绝对值仍须达到 `contact_torque_nm`，上述条件持续 `stall_confirm_s` 才确认接触。STOP 与写零准备使用同一位置阈值和速度阈值，稳定窗口改用 `stop_settle_s`，接触参考位置保护继续保留。重复读取同一反馈不计时，也不会清空已收集窗口。

每秒最多输出一条 `SEEK_STATUS`：`window_full` 表示窗口完整，`position_stable` 表示位置极差和平均速度通过，`torque_contact` 表示窗口最低力矩通过。`position_span_rad`、`mean_velocity_rad_s`、`reported_velocity_peak_rad_s`、`min_abs_torque_nm` 给出实际值；`contact_condition` 表示接触判据同时满足。`CONTACT_DETECTED` 表示从寻位进入停止准备，故障中的 `seek_window` 保留最后的窗口诊断。不能仅凭位置不动区分机械挡块和中途卡住，不能把这些事件当成独立机械限位证明。

未提高或自动修改力矩、增益、速度和位移阈值。判停前后继续原寻位预载，按已有流程写零和回程；反向位移、接触参考移动、累计行程、超时、真实力矩和通讯故障保护保留。

### 慢跟踪时等待，不反复误刹车

回程位置参考按 `return_speed_rad_s` 前进，但沿回程方向最多超前实际位置 `return_max_lead_deg`（默认 8°）。跟踪较慢时参考等待电机追上，继续发送本关节原有速度目标、bck_kp/bck_kd 和有界补偿，不因此切 BRAKE 或清空积分。实际发生反向偏离时，参考不会跟着退回，原有持续位置误差保护仍可触发；超速、越出走廊、力矩、反馈和总回程超时保护保留。

`return_max_lead_deg: 8.0` 可放在 `defaults` 或具体关节中，也可通过 launch 同名参数覆盖。必须大于 0 且小于该关节的 `brake_position_error_deg`（默认 10°）；旧 YAML 缺少时使用 8°。这个参数限制位置参考超前量，不是增加允许的最终零位误差，也不会自动增加力矩。跟不动仍可能触发原有总回程超时。

每秒 `RETURN_STATUS` 记录参考/实际/目标位置、速度、实际下发增益与前馈、力矩估算以及 `reference_limited`。`BRAKE_START` 增加参考误差，便于区别慢跟踪和实际失控。

各协议在写零后进入下文的回程检查。日志分别记录写零服务响应、实测坐标和使能确认结果。

### 腿模组标定 HipPitch 时的膝关节联动

当前通过 YAML 顶层 `hip_pitch_assist_enabled: false` 关闭双腿的膝联动；膝按普通逐台标定流程回零并保持，不再额外去 135°，HipPitch 直接执行自身标定。目标角度保留，恢复联动时设为 `true`；旧配置省略此字段时保持原来的开启行为。开关须为 YAML 布尔值。

开启后仅 `left_leg` / `right_leg` 模组 Action 生效：同侧膝在本批次完成标零后，先到 URDF 135°并保持，再开始 HipPitch 的使能、寻位和写零。HipPitch 写零结束、取得新反馈后，HipPitch 与 Knee 从同一批 MIT 控制指令开始回 URDF 0；各自使用自身的回程速度和到位判据，不承诺同时到达。先回位的 Knee 继续临时位置保持，直到两关节均到位后才恢复 Knee 原来的 `final_mode` 和控制参数。左侧配对为 knee index 3 / HipPitch index 0，右侧为 9 / 6；右腿仍跳过 7。单独指定 `motor_index:=0` 或 `6` 不联动，也不额外查询或使能膝关节。

在已有两个膝关节条目内调整目标角度，不要创建重复关节键：

```yaml
joints:
  left_knee_joint:
    hip_pitch_assist_urdf_deg: 135.0
  right_knee_joint:
    hip_pitch_assist_urdf_deg: 135.0
```

字段只用于模组联动，按确认的膝工艺范围 0..135°及转换后的电机协议位置范围检查。目标按 `motor_rad = radians(hip_pitch_assist_urdf_deg - home_angle_deg)` 换算，原 `home_angle_deg`、标零方向和 URDF 文件不变。135°使用用户明确确认的工艺值，旧 URDF 中约 113°的膝上限保持原样，记录于 `KNEE_ASSIST_PLAN.urdf_model_upper_deg`；实机行程仍需治具确认。

历史方案（已被纯阻尼联动替代，不再执行）：Knee 联动的去 135°、停留和同步回 0，曾统一使用它自己的 **kp、kd、seek_torque_ff_nm**。位置目标仍按回程轨迹生成，临时保持用同一 kp/kd；seek 前馈按原有正负号直接下发，不额外乘寻位方向。该段暂不使用 move/hold 前馈或自适应替换；实现只修改运行时参数副本，不改 YAML 数值。异常刹车仍使用原有纯阻尼及刹车限幅，不叠加 seek 前馈。两关节到位后恢复 Knee 原配置；恢复自适应位置保持时，在既有积分上限内衔接补偿，交接受限仍沿用 RECOVER。

各自的回程超时、速度、力矩、反馈保护和统一退出失能保留；不额外写零或重新寻限位。所有已使能关节持续发指令；准备阶段膝故障会阻止标髋，同时回程时任一关节故障都会中止并清理。`KNEE_ASSIST_START` 的 prepare/restore 区分去 135°和开始同步回 0，记录实际采用的增益和前馈；restore 的 `KNEE_ASSIST_DONE` 在两关节回位及膝参数恢复后输出。批次成功仍在全部动作完成后报告；`check_only` 不执行联动。

### 右腿参数镜像到左腿

左腿六个关节按当前远端右腿的对应关节参数同步，增益、速度、力矩上限、刹车和自适应参数按原值复制。按本模型的左右镜像关系：HipPitch、Knee、AnklePitch 的 `seek_direction`、`home_angle_deg` 及三项固定前馈保持同号；HipRoll、HipYaw、AnkleRoll 的这些有符号项取反。膝联动的 URDF 135°目标保持同号。右侧 `direction_confirmed` 随配置同步，不代表新增了左腿实机验收记录。

左腿索引、CAN 总线、ID 及 URDF 均不改，左腿模组顺序仍为 5→4→3→2→1→0；右腿仍为 11→10→9→8→6，跳过 7 的设置不自动删除左腿的 1。原始配置在同步前备份，镜像不是运行时自动覆盖，后续仍可逐关节单独调参。

### 分阶段增益配置

每个关节明确配置 `kd`（寻限位及写零前预紧）、`bck_kp/bck_kd`（回程、膝联动结束恢复、HOLD 偏离后的 RECOVER）、`hold_kp/hold_kd`（位置保持）。寻限位的 MIT kp 始终为 0。回程基础增益不再跟随 `kd`；可选 `return_overspeed_kd` 仍只叠加于运动中的回程，刹车与联动纯阻尼使用各自原有参数。日志中的 `kp/kd` 是实际 MIT 指令字段名，保持不变。

迁移时 `bck_kp = 原 kp`，`bck_kd = 原 move_kd（若未配置则原 kd）`。所有关节均已显式填写，原 `kp/move_kd` 配置键删除；旧键或缺少新键会在运动预检报错，避免调参被静默忽略。`bck_kp` 必须为有限正数，`bck_kd` 为有限非负数，均受协议量程和原力矩保护约束。此迁移不改变增益数值、力矩、速度或到位阈值。

### 整组完成后冻结 Ki

通过模组标零 Action 运行时，所有选中关节及膝联动恢复完成、输出 `CALIBRATION_PASS` 后，全部关节停止 `adaptive_ki` 的积分累加及卸载，保留已学到的补偿。标定过程中（包括逐关节完成后的等待）仍正常自适应；单独 `--motor-index` 保持原流程。HOLD/RECOVER 的 P/D 控制、增益切换时已有的补偿衔接、力矩限幅和反馈保护继续生效。`CALIBRATION_PASS.adaptive_ki_frozen_indices` 列出已冻结 Ki 的自适应关节。无需新增 YAML 参数，也不改写配置的 `adaptive_ki` 数值。

### 写零后回程检查

写零后继续发送 MIT。默认启用 `post_zero_static_check`，进入回程要求本关节不同序号、接收时间递增的新反馈持续 100 ms 落在写零后容差 `post_zero_tolerance_deg`（默认 ±1.15°）内，且窗口内位置极差和平均速度满足 `position_epsilon_rad` 和 `stall_velocity_rad_s`。写零前的挡块位置检查使用 `zero_reference_tolerance_rad`（0.01 rad）。MR、新版 LS 还须上报已使能，RS 须上报运行模式。失能或坐标超限会重新计时；明确失能时可间隔至少 50 ms 补使能。交接最长等待 `feedback_start_timeout_s`，反馈中断仍受 `feedback_timeout_s` 保护。通过后记录 `RETURN_FEEDBACK_READY`。回程、运动测试或保持时再收到失能反馈会中止清理。

左右 HipPitch 和左右 ShoulderPitch 在各自 YAML 条目中设 `post_zero_static_check: false`。本轮写零请求结束后，连续收到两帧新鲜且已使能的反馈就进入回程判断，日志记录 `handoff=TWO_FRESH_FRAMES` 和 `zero_verified=false`。第二帧超过紧急速度阈值时，首条回程控制命令即为限矩纯阻尼刹车；速度稳定后从实际位置重新规划回程。

### LS 写零确认与可选有界重试

LS 使用专用写零服务等待本电机 ACK（0x7FF，状态 0x0103）。等 ACK 时持续发送寻位预紧 MIT；收到 ACK 后发送使能，继续原有预紧 MIT 并检查新反馈。版本取自正在运行的 hpm_system 参数 `ls_firmware_version`（launch 默认 `old`）。`new` 要求 type‑1 反馈提供真实使能位；`old` 在 ACK 后共发 3 次使能，相邻请求间隔至少 50 ms，并在最后一次后检查新零坐标。旧版日志的 `enable_verified=false` 表示物理使能状态未知。版本读取失败会终止预检。

```yaml
defaults:
  zero_max_attempts: 3
  ls_zero_attempt_timeout_s: 2.0
  ls_zero_total_timeout_s: 10.0
```

设 `ls_zero_retry_until_success: false` 时，LS 缺少应答或已确认应答但零坐标未更新，会在上述总时限内重试，最多 `zero_max_attempts` 次（默认 3，允许配置为 5 等正整数）；成功即停止重试。底层每次等待应答仍受 `hpm_system` 的 `motor_set_zero_timeout_ms` 限制（默认 1000 ms），脚本单次时限应大于它，给 ROS 应答和新位置反馈留余量。ROS 服务本身不返回、传输发送失败、设备明确报错或运动反馈中断均立即退出，不重叠发送未完成的事务。耗尽次数/总时限同样退出清理，不进入回程和下一个关节。确认只针对本次 ACK 和零坐标，不证明断电持久化。

MR 和 RS 按 `zero_max_attempts` 发送写零请求，再检查零坐标和使能状态。更新后需要配套构建和加载 luvbot_hardware_interfaces、hpm_motor、hpm_transport 与 set_zero。

`zero_max_attempts` 必须为正整数。在有界模式下 LS 同时受 `ls_zero_total_timeout_s` 限制，次数与总时限先达到者生效；例如设为 5 次、单次 2 秒时，如需预留完整 5 次窗口，可将总时限设为 10 秒。默认次数仍为 3、总时限为 10 秒。

### 模块标定后的逐电机运动测试

标零完成后通过 `/set_zero/motion_test` 启动逐电机测试，等待和测试期间继续控制已使能的关节。

```yaml
defaults:
  motion_test_amplitude_deg: 15.0  # Half amplitude; nominal total span 30 degrees.
  motion_test_position_margin_deg: 2.0
  motion_test_arrival_tolerance_deg: 2.0
  motion_test_torque_source: seek
  motion_test_speed_rad_s: 0.5
  motion_test_torque_rate_nm_s: 0.25
  motion_test_torque_ff_nm: 0.0  # Only used with torque_source: fixed.
  motion_test_use_seek_feedforward: true
# 单关节覆盖示例：在 joints 下对应关节中添加 motion_test_amplitude_deg: 3.0
```

测试默认请求零位两侧各 15°，总摆幅 30°；按本次标定挡块、URDF 总行程和协议范围裁剪。零位在挡块上的关节裁剪为向外 0～15°，不跨挡块。测试前用现有回程控制缓慢移到摆动端点，再进行完整 5 秒的余弦位置参考 / 正弦速度指令。选择整数周期，保证起止速度为零、峰值速度不超过该关节 `motion_test_speed_rad_s`；速度太低、5 秒不足以完成所设幅度的一整周期时，自动减小实际幅度，实际区间和峰值速度记录在 `MOTION_TEST_START`。

5 秒测试使用 `kp=0`、该关节 `kd`。前馈保持 `abs(seek_torque_ff_nm)` 的固定绝对值，仅随目标运动方向变号；`motion_test_use_seek_feedforward: false` 时为 0。这是速度控制测试，实际位置不由 P 项锁定；超出测试区间再加 `motion_test_position_margin_deg`、持续超速、真实/估算力矩超限、反馈中断或位置跳变均退出清理。只测试已选择并完成标定的关节，不再使能或写零；其他关节继续原最终模式。模块完成后 Ki 冻结规则保留，测试回程沿用已学到的支撑量。

每台测试后立即复用 `bck_kp/bck_kd` 回到原 URDF 零位，再恢复配置的 `final_mode`。测试准备和回位使用原回程超时/刹车保护，到位容差取原值、测试到位容差和测试位置余量的最小值。逐台输出 `MOTION_TEST_MOTOR_PASS`，全部完成输出 `MOTION_TEST_PASS` 并继续保持原最终模式，不退出失能。测试过程 Ctrl+C 记录未完成退出。

裁剪使用本次 SETZERO 挡块为电机坐标 0，以 seek_direction 确定向内方向，另一端距离取 URDF upper-lower，并与协议范围取交集。不会据此修改 home_angle_deg 或重新启用 URDF 力矩限制；URDF 行程须与实物相符，治具和自碰撞仍需现场确认。


### 模块标定后的人工确认与运动测试

标零完成后通过 `/set_zero/motion_test` 启动逐电机测试，等待和测试期间继续控制已使能的关节。

按模块标定顺序逐电机测试：不再预定位到正弦转折点，也不等待测试前到位；从当前实际位置直接以 `kp=0`、该关节的 seek `kd` 运行速度正弦，完成配置数量的换向周期（默认 3 个周期，即 6 次目标方向切换）。测试范围仍按标定零位确定，实际位置端点换向及反馈、超速、力矩、越界保护保持有效。正弦速度峰值不超过 `motion_test_speed_rad_s`（默认 0.5 rad/s），测试后回零也临时采用此速度上限；普通寻限位和标定回程速度保持原值。正弦阶段默认使用该关节 `abs(seek_torque_ff_nm)` 的固定前馈绝对值，仅随目标方向变号；测试后回零仍将其作为可调助力上限。位置默认请求 URDF 0° ±15°（总摆幅 30°），按已标定挡块及 URDF 总行程裁剪；零位在挡块处时只保留向外 0～15°。波形角频率按测试速度上限和裁剪后半幅计算，不再为凑够 5 秒而缩小幅度或改变周期数。实际往返速度仍受负载和前馈影响。

每个电机测试后使用原 `bck_kp/bck_kd`、回程速度与刹车逻辑回到 URDF 0°，到位后恢复该关节原 `final_mode`，然后测试下一个。其他关节始终受控，不重新写零或使能。模块完成后的 Ki 冻结保持不变，回程复用之前已学到的支撑补偿。全部结束输出 `MOTION_TEST_PASS`，继续原最终模式，等待人工退出。单电机 `motor_index:=...`、`check_only` 和寻位诊断不进入这项测试。

```yaml
defaults:
  motion_test_amplitude_deg: 15.0  # Half amplitude; nominal total span 30 degrees.
  motion_test_position_margin_deg: 2.0
  motion_test_arrival_tolerance_deg: 2.0
  motion_test_torque_source: seek
  motion_test_speed_rad_s: 0.5
  motion_test_torque_rate_nm_s: 0.25
  motion_test_torque_ff_nm: 0.0  # Only used with torque_source: fixed.
  motion_test_use_seek_feedforward: true
# 如需单独覆盖，将同名字段写到 joints 下对应关节中。
```

测试使用速度控制，kp 为 0，因此实际位置不保证精确跟随正弦位置参考。实际位置超出测试区间加 `motion_test_position_margin_deg` 时退出清理；反馈新鲜度、位置跳变、超速、实测及估算力矩和温度保护均保留。设置 `motion_test_use_seek_feedforward: false` 可以关闭测试前馈。测试功能尚需在治具上由操作员验证。

### LS 持续写零模式

```yaml
defaults:
  ls_zero_retry_until_success: true
```

`ls_zero_retry_until_success: true`（默认）时，LS 未收到成功专用应答、专用应答表示写零失败，或应答后坐标尚未更新到零，就继续重发 SETZERO，直到成功、总超时或人工 Ctrl+C。每次仍按 `ls_zero_attempt_timeout_s` 等待，避免无间隔重复写零；等待 ACK 时持续寻位预紧 MIT，收到 ACK 后按固件版本补发使能。成功后停止重写，进入反馈交接与回程。

此模式下，LS 不受 `zero_max_attempts` 的次数限制，但仍受 `ls_zero_total_timeout_s` 总时限限制（默认 10 秒，包含所有重试及坐标确认等待）。设 `ls_zero_retry_until_success: false` 则同时受次数与总时限限制；其他协议的逻辑保持原样。服务接口不返回、发送失败、实际过载或运动反馈中断等真实故障仍中止并清理。长时间顶限位有发热风险，持续模式不能证明硬件一定可以写零，应结合日志和治具现场监护。

ID 预检保持原逻辑，本次没有将 ID 缺失改为无限等待。

测试后回零的到位容差由 `motion_test_arrival_tolerance_deg` 配置，当前 YAML 默认 8°，支持逐关节覆盖。实际容差取它与原关节到位容差中的较小值；位置越界保护独立保留。测试结束恢复原关节容差；模块 Ki 冻结不变。`MOTION_TEST_START` 记录 `preposition: false`、`actual_start_rad` 与回零容差；不再产生 `MOTION_TEST_PREPARE` 或执行测试前定位。每个电机测试后仍须完成回零，才会测试下一个；回零超时仍会退出。

LS 的 `ls_zero_total_timeout_s` 在两种重试模式下始终生效。计时从开始 SETZERO 确认流程起算，每次等待会截断到剩余总时限；期间持续发送原控制帧。10 秒内成功就正常回程，否则报告 `LS_ZERO_TOTAL_TIMEOUT` 并执行停止/失能清理，不进入回程。该超时不影响 ID 查询、寻限位或运动测试的原时限。`ZERO_REQUEST` 中的 `total_timeout_s` / `remaining_timeout_s` 记录总时限及剩余时间。

测试力矩源由 `motion_test_torque_source` 配置：`seek`（默认）使用对应电机 `abs(seek_torque_ff_nm)` 作为助力上限，`fixed` 使用 `motion_test_torque_ff_nm` 指定非负峰值。旧字段 `motion_test_use_seek_feedforward: false` 可关闭两段测试助力。源模式、上限、实际测试速度会记录在测试计划中。普通寻位、标零、回程和最终保持参数不随这项测试设置改变。

测试后回零的助力按当前目标位置误差决定正负，只有实际位置落后于运动参考且速度未到上限时才增加；到位、超前或速度已到上限时撤去。测试后回零换向先撤去旧方向输出，再按 `motion_test_torque_rate_nm_s` 逐步增加新方向助力；默认上升速率 0.25 N·m/s，可逐关节覆盖。正弦阶段不使用这段斜坡或按跟踪误差缩小前馈，始终以所选 seek/fixed 幅值变号；零速转折点取即将运动的半周期方向。若 D 项与固定前馈合成的估算力矩超过 max_torque_nm，则报 MOTION_TEST_COMMAND_TORQUE_BUDGET 并退出清理，不暗中缩小前馈。实际总力矩仍含 D 项，不保证绝对值恒定。

模块 Ki 冻结保持不变，测试回程沿用已学到的承重补偿，方向助力单独计算，不把承重补偿整体翻号。HOLD、RECOVER、BRAKE 和测试结束后不叠加方向助力。`RETURN_STATUS.motion_test_torque_nm` 及历史指令记录实际助力；原反馈、超速、位置和力矩保护保留，超速判据按测试速度上限及原保护比率计算。软件离线验证不能代替治具上的运动验证。

测试回程参考速度已经降为 0 但位置尚未到位时，只在低于现有到位/停滞速度阈值的状态下逐步补力脱离静摩擦；开始移动后即撤去该助力，继续原位置/速度控制，避免末端仍按全程速度方向推力。

测试后回零末段参考速度为 0 时，方向助力使用 arrival_settle_s 长度的位置窗口估计实际速度，避免静止位置下的 MIT 瞬时速度噪声反复清零助力。窗口建立前仍保守采用瞬时速度；真实位置开始移动后逐渐撤力，瞬时速度达到测试回程速度上限则立即撤去额外助力。Ki 冻结、到位容差和既有超速/力矩/超时保护不变。实际裁剪区间及 actual_span_deg 会记录在测试计划中。

正弦测试的公共超速配置（所有关节生效，支持逐关节覆盖）：

```yaml
defaults:
  motion_test_speed_limit_rad_s: 1.6
  motion_test_emergency_speed_rad_s: 2.5
```

超过 1.6 rad/s 持续 brake_overspeed_confirm_s（当前 0.05 s）退出，超过 2.5 rad/s 立即退出。两项仅控制 MOTION_TEST，不改变正弦目标速度、前馈、定位/回零或普通回程刹车阈值。计划和故障诊断记录实际阈值。省略配置时沿用原测试速度上限及 brake 比率公式。两项必须为有限正数，且测试目标速度上限 ≤ 持续阈值 ≤ 立即阈值。

正弦测试增加实际位置优先的端点换向：`motion_test_reverse_margin_deg: 2.0` 为公共默认，支持每关节覆盖。到达裁剪后的测试端点前 2° 且目标指令仍向外时，当个控制周期即将目标速度和固定幅值前馈翻向内侧，并重置正弦相位。向内指令保持至新位置反馈确认退回边界内，防止旧正弦相位或位置噪声造成反复外推。换向不会重置已完成周期数或故障超时计时。窄行程会将换向余量限制在实际测试总行程的四分之一；0 表示到测试端点再换向。

这不是机械接触传感器：仅凭位置不能证明已经撞块，算法优先在测试边界前反向。颈部 Roll 参考 ±15°、默认提前 2°，实际约 ±13°触发向内指令，给 ±17°机械边界留下减速余量；能否及时停住仍取决于惯性和负载。超过硬位置范围、超速、位置跳变、力矩超限及反馈丢失仍报错退出，不用换向掩盖故障。`MOTION_TEST_REVERSE` 记录触发位置、边界、方向、速度和前馈。寻限位及写零逻辑不变。



测试周期由 `motion_test_cycles: 3` 指定，支持逐关节覆盖，必须为正整数。首次目标方向只初始化；此后目标方向每改变一次计半个周期，两次换向算 1 个周期。按边界覆盖后的最终指令方向计数，正弦自然换向和边界提前换向都纳入，不要求实际位置触及两侧，也不会把同一方向连续指令重复计数。3 个周期对应 6 次换向，之后回零，再测试下一个。计数表示控制指令已换向，不证明关节已完成整幅运动。

`MOTION_TEST_DIRECTION_CHANGE` 记录累计换向次数和目标次数；`MOTION_TEST_CYCLE` 记录完成周期。`MOTION_TEST_REVERSE.count` 仍仅表示边界保护触发次数，与周期数不同。`MOTION_TEST_START.cycle_count_mode` 为 `command_direction_changes`。

`motion_test_timeout_s` 仍是每电机测试故障超时，不是正常结束时长。反馈丢失、越界、超速及力矩保护保留；测试后的回零仍使用 `max_return_time_s`。
