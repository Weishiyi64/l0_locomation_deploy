# AMP ↔ WBC 交接初始化（2026-11-10）

交接修复来自分支 `fix/amp-wbc-handoff-20261110`。修改前代码保存为
`backup/pre-amp-wbc-handoff-20261110`，基线提交 `55c7ba9`。

2026-11-10，源码已更新到 `192.168.120.23` 的
`/userdata/forlinx/wbc_text_workspace1104`，并在板端使用 RKNN 后端完成 9 包编译。
启动时发现顺序脚本参数数量检查遗漏了两个新增参数，随后已修复源码与安装副本，
并对参数解析做了独立检查。**板端编译和参数解析通过不等于交接动作已完成运行验证**：
目前没有本次新交接逻辑的完整 MuJoCo 或真机动作验收记录。2026-11-09 的实体手柄验证
属于修改前版本，不能作为新交接逻辑的验收结果。

## 行为变化

| 项目 | 修改前 | 本分支 |
| --- | --- | --- |
| WBC → AMP 的初始 last_actions | 全零 | 将 WBC 最后有效关节目标换算成 AMP 动作坐标 |
| AMP 观测历史初始化 | reset 在检查是否跳过推理之前被消费 | 初始化保持待完成状态，成功取得有效观测并填满全部历史后才清除 |
| AMP → WBC 的三帧动作历史 | 全零 | 将 AMP 的有效关节目标换算成 WBC 动作坐标，复制填满三帧 |
| AMP → WBC 目标与 Kp/Kd 插值 | 0.35 秒 | 默认 1.0 秒 |
| WBC → AMP 目标与 Kp/Kd 插值 | 0.35 秒 | 默认 0.35 秒 |
| 接管条件 | 收到比原命令更新的有效命令 | 还要求命令携带本次初始化的 handoff_id |

按关节名对齐后使用：

```text
initial_action[j] = (source_target[j] - destination_default[j]) / destination_action_scale[j]
```

零缩放关节填零，不做除法。拒绝缺失、重复或未知关节，以及非有限值、过期源命令；
AMP 还拒绝超出其 `clip_actions` 的初始化动作。AMP 的 MJ 顺序动作随后转换到 Isaac
策略顺序。两个模型的原始 action 不直接互拷。关节目标来自 PolicyMux 中已经验证的源策略
命令，不来自关节实际测量或 reference；实际测量仍用于观测历史。

这次没有改变运行模型、reference、默认关节角、PD 配置或动作缩放。零缩放关节的目标
仍由原有默认姿态/辅助控制决定，不能用动作历史反解来保持任意姿态。

### 1. WBC → AMP：按目标初始化上一动作

原实现重置 `rbt_output.action` 后，把包含零 `last_actions` 的单帧观测复制到 AMP
历史中。零动作对应 AMP 的默认关节角，在 WBC 结束目标与该默认角不同的时候，输入
历史中的上一动作不能反映交接前的策略目标。

新实现由控制器取得最后一条通过校验且未过期的 WBC 关节目标，调用 AMP 的
`/control/policy/rl/seed_history` 服务。AMP 用自己的默认关节角和 `action_scale` 做
上述反算，将 MJ 顺序结果转换到 Isaac 策略顺序后放入 `rbt_output.action`。再用
当前有效观测组装单帧观测，并将包含该 `last_actions` 的观测复制填满 AMP 历史。
当前 V5 AMP 配置使用 5 帧；其他配置按其 `frame_stack` 和索引处理。

反算对象是源策略的关节**目标**，不是关节实测角，也不是 WBC 原始网络输出；
位置、速度、IMU 等状态观测仍来自当前传感器。`action_scale=0` 时填零，表示该
动作分量不能反算出任意关节目标，不承诺零缩放关节一定保持交接前目标。

### 2. AMP：初始化请求保持待完成

`RL::update()` 把 reset 请求或 AMP 推理重新启用事件记录到
`history_init_pending_`。以下情况发生时不会把“待完成”清掉：本周期因频率超限按
配置跳过推理、当前观测无效/过期、观测更新未成功或历史填充未完成。

只有获取有效观测、按需应用 seed、完成单帧组装并复制填满历史后，才清除该标志。
首帧输出还需满足观测时效、推理时效和有限值检查，才发布接管命令。seed 等待超过
0.5 秒时不退回到零动作推理；本次交接无法得到匹配的有效命令时，控制器按超时处理。

### 3. AMP → WBC：初始化三帧动作历史

控制器在准备 WBC 时，将有效 AMP 关节目标和新的 `handoff_id` 放入 START 请求。
WBC 按目标动作自己的默认角和缩放反算出初始 action，复制到三帧动作历史。
状态历史则用当前采集的实际机器人状态初始化 17 帧；预推理时继续读取有效状态。
因此动作历史与状态历史来源不同，两者分别代表源策略目标和机器人实际观测。

WBC 的首帧预推理成功后才返回准备成功。准备失败、加载失败、命令过期或控制器
状态改变时，沿用原有取消或拒绝流程，不以未准备好的 WBC 命令接管。

### 4. `handoff_id`：确认属于本次初始化的命令

控制器为每次交接分配非零编号。`ControlWbcMotion.START` 带入入场编号和源 AMP
命令；WBC 首帧准备并发布的 `PolicyCommand` 带相同编号。回 AMP 时，控制器先通过
`SeedPolicyHistory` 提交本次编号和源 WBC 命令；服务成功仅表示 seed 已接收，
AMP 完成初始化及推理后才发布带该编号的命令。

控制器同时检查命令编号、更新序号和有效期，之后才安排接管。不是所有“比旧命令
更新的消息”都可以代替本次交接的确认。回切时源命令失效、seed 服务失败或有效
AMP 命令超时，会走已有的急停阻尼处理。手柄/终端仍使用原有动作请求及反馈 topic。

## 交接顺序

AMP → WBC：控制器将有效 AMP 命令及新的 handoff_id 放入 START 请求；WBC 按当前实际
状态初始化状态历史、按换算动作初始化三帧动作历史，再进行首帧预推理。控制器只接受
同一 handoff_id 的有效 WBC 命令，随后在控制循环中开始 1 秒的目标与增益插值。

WBC → AMP：控制器首先调用 `/control/policy/rl/seed_history`，在 AMP 仍被 WBC 暂停时
提交初始动作；服务确认接收后才发布 WBC inactive。AMP 在下一次允许推理且观测有效的
周期填满历史、推理并发布带本次 handoff_id 的命令。控制器等待匹配命令后执行 0.35 秒
插值。普通回切不再额外发送一个无数据的 Bool reset，避免再次把刚初始化的动作清零。

初始化状态跨越超频跳帧或观测无效周期保留；初始化不完整时不发布接管命令。未完成
首帧发布的 seed 超过 0.5 秒后不降级为零动作推理，控制器按已有超时/急停路径处理。
重复 WBC inactive 通知不会再次清空初始化；离开允许推理状态、重新进入 WBC或显式旧式
reset 会使旧 seed/命令编号失效。取消、急停和 MuJoCo 的 crouch-off 完成急停仍保留。

目标与 Kp/Kd 沿用 `JointCommandTransition` 的 smoothstep 插值。动作配置中的
`start_hold_sec` 独立控制 reference 停留第 0 帧的时间，不是交接插值时间；它仍从 WBC
激活时计时。入场插值起点为随后控制循环的接管时刻，两者不是额外串行等待。

## 参数入口

`sim2real.launch.py`、`wbc_lora.launch.py`、`sim2mujoco.launch.py` 支持：

```text
amp_to_wbc_transition_sec:=1.0
wbc_to_amp_transition_sec:=0.35
```

参数经 sequential launch 脚本传给真机/MuJoCo 的硬件适配节点中的共享控制器。
旧的共用 `wbc_transition_sec` 被两个方向参数替代；若外部启动文件曾覆盖旧参数，需一并迁移。
终端动作 ID、请求 topic 与八个手柄组合键保持原样。

### 启动参数数量修复

两项过渡时间是顺序脚本末尾新增的位置参数。真机 `sim2real.launch.py` 传入 38 项，
MuJoCo `sim2mujoco.launch.py` 传入 31 项。此前脚本的数量检查仍按 36/29 项处理，
会在启动前拒绝新调用；现已分别接受 38 项和最多 31 项，并保留原有数量的兼容。
`--ros-args` 后的 ROS 参数会先被剔除，再检查位置参数数量。此修复已同步到 `.23`
的源码和 install 脚本，并在不启动控制程序的情况下检查过参数解析。

## 主要实现位置

| 文件 | 作用 |
| --- | --- |
| [policy_handoff.h](../legged_deploy/include/base/policy_handoff.h) | 校验源命令、按关节名对齐并反算目标策略初始 action |
| [rl_controller.cpp](../legged_deploy/src/rl_controller.cpp) | AMP seed 服务、待完成的历史初始化及首帧命令发布 |
| [rl_controller_wbc.cpp](../legged_deploy/src/rl_controller_wbc.cpp) | WBC 的源目标换算、状态/动作历史填充和预推理 |
| [PolicyMux.cpp](../legged_controllers/src/PolicyMux.cpp) | 源命令快照、seed 服务请求与 handoff 编号匹配 |
| [LeggedControllerWbc.cpp](../legged_controllers/src/LeggedControllerWbc.cpp) | 双向切换顺序、失败处理与交接插值启动 |
| [LeggedControllerInit.cpp](../legged_controllers/src/LeggedControllerInit.cpp) | 两个方向的默认时间和参数校验 |
| [JointCommandTransition.h](../legged_controllers/include/legged_controllers/JointCommandTransition.h) | 目标位置与 Kp/Kd 等命令的 smoothstep 插值 |
| [PolicyCommand.msg](../../luvbot_ros_interfaces/luvbot_control_interfaces/msg/PolicyCommand.msg)、[ControlWbcMotion.srv](../../luvbot_ros_interfaces/luvbot_control_interfaces/srv/ControlWbcMotion.srv)、[SeedPolicyHistory.srv](../../luvbot_ros_interfaces/luvbot_control_interfaces/srv/SeedPolicyHistory.srv) | 交接命令编号、源命令及 AMP seed 服务接口 |

## 接口与构建要求

本次修改了 `luvbot_control_interfaces`：

- `PolicyCommand.msg` 新增 `handoff_id`。
- `ControlWbcMotion.srv` 的 START 新增 `handoff_id` 和 `source_command`；CANCEL 不需要 seed。
- 新增 `SeedPolicyHistory.srv`，由控制器调用 AMP 的 `control/policy/rl/seed_history`。

必须一起更新匹配的 `src/luvbot_ros_interfaces` 和 `src/l0_ros2_deploy_v2`，并重新构建
接口、策略节点及控制器/硬件适配节点；不要把新消息定义与旧二进制混用。
HPM 的 `robot_interfaces` 和 `luvbot_hardware_interfaces` 消息定义未修改。
后续编译应从 ROS 工作区根目录执行，使用对应平台后端及 `--packages-up-to`；
目标至少包含 `legged_deploy legged_controllers legged_real`，仿真还需匹配的仿真包。
回滚时回到备份分支，并对同一批接口及依赖重新构建，不能只回滚一个 launch 文件。

`.23` 的一次构建使用 `--base-paths src`，避免工作区 `updates/` 备份里的
`package.xml` 被当作重复包；对应构建日志位于：

```text
/userdata/forlinx/wbc_text_workspace1104/updates/amp_wbc_joystick_20261110_viGp69k9/build_20261110_src.log
```

该机器人另外保留了 `l0_v5_amp_1109` 并已将其设为真机默认 AMP。该模型和机器人上的
默认值调整属于板端更新；本仓库这次提交的重点是交接代码和配套接口，仓库中的
AMP 默认设置及资产以仓库实际文件为准，不因这段板端记录而自动改变。

## 后续验收

静态检查不证明物理交接已稳定。后续先在 MuJoCo 验证：

1. 记录零速度 AMP → WBC → AMP 的实际关节角、源目标、初始 last_actions、首帧输出及插值后目标。
2. 逐一验证现有八个手柄动作，正常完成返回 AMP；终端入口也应通过。
3. 验证初始化周期跳过、观测过期、服务超时、重复 inactive 通知及过期 handoff_id，不允许部分零历史或旧命令接管。
4. 验证切换期间急停/取消，以及 `L0_crouch_off` 在仿真完成后进入 Emergency 的既有行为。
5. 检查两个插值时间与 reference 开始推进的相对时刻，观察膝角、机身姿态及 Kp/Kd 连续性。

本分支仿真包没有 `L0_Hugme` reference；不得把图片的 Hugme 数字当作本分支测试结果。
需要复现该动作时先补齐并核对匹配的仿真资产；也可先用当前已交付的八个动作验证链路。
