# l0_locomotion_deploy：L0 V3 AMP / Mimic / WBC / 文本动作部署说明

这是一套已经完成真机实验的 L0 V3 一体化部署代码，包含：

- PD 与 AMP 站立/行走；
- 手柄从 AMP 切换到 Mimic；
- 手柄从 AMP 切换到预置 WBC；
- 电脑输入中文文本，生成动作 JSON，并通过 ROS 2 发送给 Jetson；
- Jetson 使用固定的 stg25 WBC TensorRT 引擎执行文本动作；
- 文本 JSON 在后台解析和准备，不阻塞实时控制循环。

当前 GitHub 仓库名为 `l0_locomotion_deploy`，本文统一使用以下部署路径：

- Jetson 真机：`/home/nvidia/l0_locomotion_deploy`；
- x86_64 本地电脑：`/home/luvbotic/l0_locomotion_deploy`。

> 真机首次运行新动作时必须使用吊架，并安排人员随时准备急停。触发 Mimic、WBC 或文本动作前，机器人必须已经稳定处于 AMP，摇杆回中。

## 1. 整体功能和运行方式

系统同时保留了 AMP、Mimic、预置 WBC 和文本动作功能：

| 功能 | 触发方式 | 动作结束后的状态 |
|---|---|---|
| AMP | 手柄 `B + Y` | 持续运行 |
| LieDown Mimic | `LT + 十字键方向` | **EmergencyStop** |
| 预置 WBC | `RT + 十字键方向` | 返回 AMP |
| 文本生成动作 | 电脑运行 `infer.py` | 返回 AMP |

Mimic、WBC 和文本动作共用 imitation/WBC 控制通道，同一时刻只能执行一个动作。机器人不在 AMP、处于急停或已有动作运行时，新动作会被拒绝。

## 2. 真机启动并进入 AMP

### 2.1 Jetson 启动控制器

在机器人 Jetson 上执行：

```bash
cd /home/nvidia/l0_locomotion_deploy

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0

ros2 launch legged_controllers sim2real.launch.py
```

如果部署目录名称不同，只修改第一行路径。手柄默认使用 `/dev/input/js0`。

### 2.2 手柄进入 PD 和 AMP

控制器启动后默认处于 Standby：

1. 将机器人放在吊架保护范围内，确认双脚着地、姿态正常。
2. 逐次按 `B`，每次将 Standby PD 的 `kp` 增加 2，机器人逐渐进入稳定 PD。
3. 如果刚度过大，可以逐次按 `A`，每次将 `kp` 减少 2。
4. 机器人稳定后，摇杆回中，同时按 `B + Y` 进入 AMP/Walking。
5. 进入 AMP 后才能触发 Mimic、WBC 或文本动作。

常用安全按键：

| 按键 | 功能 |
|---|---|
| `B` | Standby 下增加 PD `kp`，每次 +2，最大 80 |
| `A` | Standby 下减少 PD `kp`，每次 -2，最小 0 |
| `B + Y` | Standby → AMP/Walking |
| `RB` | 立即进入 EmergencyStop |
| `LB + RB` | EmergencyStop → Standby，同时将 `kp` 清零 |

急停后的恢复顺序：

```text
确认机器人安全并扶正
    ↓
LB + RB：EmergencyStop → Standby
    ↓
逐次按 B：重新进入稳定 PD
    ↓
B + Y：重新进入 AMP
```

## 3. 手柄触发 Mimic

机器人必须已经稳定处于 AMP。当前启用的 Mimic 按键为：

| 手柄操作 | Mimic 策略 | 结束状态 |
|---|---|---|
| `LT + 十字键下` | `l0_v2_mimic_dunxia_dunzhe_dunqi` | EmergencyStop |
| `LT + 十字键左` | `l0_v3_mimic_LieDown9_24_10_9_flat1010` | EmergencyStop |
| `LT + 十字键右` | `l0_v3_mimic_LieDown_2_liedown2_1015_2` | EmergencyStop |
| `LT + 十字键上` | 当前未绑定 | 无动作 |

这些都是 LieDown 类动作，配置为：

```yaml
on_finish: emergency_stop
```

因此 LieDown 结束后进入 EmergencyStop，不会自动回 AMP。这是当前代码的安全设计。动作结束后，确认机器人安全并扶正，再按 `LB + RB` 回到 Standby，重新进入 PD 和 AMP。

## 4. 手柄触发预置 WBC

机器人必须已经稳定处于 AMP。当前启用的 WBC 按键为：

| 手柄操作 | WBC 策略 | 结束状态 |
|---|---|---|
| `RT + 十字键上` | `wbc_actions/l0_v3_wbc_A-T01-00_mod_1009` | 返回 AMP |
| `RT + 十字键右` | `wbc_actions/l0_v3_wbc_A-T01-01_mod_1009` | 返回 AMP |
| `RT + 十字键下` | `wbc_actions/l0_v3_wbc_A-A02_mod_1009` | 返回 AMP |
| `RT + 十字键左` | `wbc_actions/l0_v3_wbc_A-C03_mod_1009` | 返回 AMP |

这些动作配置为：

```yaml
on_finish: return_rl
```

默认 RL 策略是 AMP，所以预置 WBC 或其他站立类动作结束后会自动回到 AMP，不进入 EmergencyStop。

## 5. 文本生成动作操作流程

文本动作采用双机运行：

```text
本地电脑：中文文本 → text_to_motion_cpu_fixed.onnx → 完整 JSON
                                      ↓ ROS 2 /text_motion/json
Jetson：后台解析 JSON → 动态参考轨迹 → 固定 stg25 policy.trt → 机器人
```

本地电脑使用 `text_to_motion_cpu_fixed.onnx` 生成 JSON；Jetson 不运行文本模型，也不需要安装 PyTorch。

### 5.1 两台机器设置相同的 ROS 2 通信环境

Jetson 和本地电脑的每个相关终端都要执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0
```

注意：

- 两边的 `ROS_DOMAIN_ID` 必须完全相同；
- `ROS_LOCALHOST_ONLY=0` 才允许跨机器发现；
- 环境变量只对当前终端生效；
- 修改变量后，已经启动的 ROS 2 节点不会自动切换 domain，必须关闭相关节点并重新启动；
- 如果之前用其他 domain 启动过节点，最稳妥的方法是关闭旧终端，重新打开终端，再设置环境并启动。

### 5.2 Jetson 开启文本动作状态监测

机器人控制器启动并进入 AMP 后，在 Jetson 新终端执行：

```bash
cd /home/nvidia/l0_locomotion_deploy

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /text_motion/status std_msgs/msg/String
```

这个终端保持打开，用于观察 JSON 是否收到、轨迹是否准备完成以及动作是否被拒绝。

还可以再开一个终端检查关键通道：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/l0_locomotion_deploy/install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0

ros2 topic info /text_motion/json
ros2 topic info /text_motion/status
ros2 topic info /text_motion/ready
ros2 topic hz /rlCmd_imitation
```

发送动作前，`/text_motion/json` 应至少显示：

```text
Subscription count: 1
```

### 5.3 本地电脑验证能否发现 Jetson

在本地电脑新终端执行：

```bash
cd /home/luvbotic/l0_locomotion_deploy

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0

ping -c 3 192.168.50.103
ros2 topic list | grep text_motion
ros2 topic info /text_motion/json
```

正常情况下应能看到：

```text
/text_motion/json
/text_motion/ready
/text_motion/status
```

并且 `/text_motion/json` 有 Jetson 订阅者。

### 5.4 本地电脑生成并直接发送文本动作

下面是之前真机实验使用的完整命令：

```bash
cd /home/luvbotic/l0_locomotion_deploy

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=0

# ===== 本次测试动作 =====
ACTION="左手以大幅度快速抱肚子；右手腹部摆动两次。"

# ===== 重新推理并直接发送 =====
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python infer.py \
  "$ACTION" \
  --model text_to_motion_cpu_fixed.onnx \
  --device cpu \
  --output motion_test4.json \
  --overwrite
```

正常输出类似：

```text
已保存完整 JSON：.../motion_test4.json
160 帧 / 3.20 秒 / 50 FPS
已一次性提交全部 160 帧到 /text_motion/json
执行后端：CPUExecutionProvider
```

只想生成 JSON、不发送机器人时，在命令末尾增加：

```bash
--no-publish
```

必须显式指定：

```bash
--model text_to_motion_cpu_fixed.onnx --device cpu
```

因为当前 `infer.py` 默认仍然查找 `text_to_motion_fp16.onnx`，不能依赖默认模型名。

### 5.5 状态监测结果

Jetson 的 `/text_motion/status` 正常会依次出现：

```text
received: complete JSON queued for background preparation
ready: ... dynamic-reference frames at 50 Hz, fixed_policy=text_motion_wbc_stg25_dynamic
```

含义是：

1. Jetson 已一次性收到完整 JSON；
2. C++ 后台线程完成轨迹准备；
3. 控制器切换到固定 stg25 WBC；
4. 文本动作执行完后自动回 AMP。

## 6. 通信不通时的处理

### 6.1 先确认两边环境一致

本地电脑和 Jetson 都执行：

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_LOCALHOST_ONLY"
```

应输出：

```text
88
0
```

然后确认网络：

```bash
ping -c 3 192.168.50.103
```

### 6.2 重启 ROS 2 discovery

两边分别执行：

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

如果仍然看不到通道：

1. `Ctrl+C` 关闭控制器、监测和推理相关终端；
2. 关闭这些旧终端窗口；
3. 重新打开终端；
4. 重新 `source` ROS 2 和工作区；
5. 重新设置 `ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=0`；
6. 先重启 Jetson 控制器，再开启监测，最后从电脑发送动作。

不要只在旧终端修改环境变量后继续使用已经启动的节点，因为旧节点仍然留在原来的 ROS domain 中。

### 6.3 通信正常但机器人没有动作

检查：

```bash
ros2 topic info /text_motion/json
ros2 topic echo /text_motion/status std_msgs/msg/String
ros2 topic echo /text_motion/ready std_msgs/msg/String
```

同时确认：

- 机器人已经处于 AMP/Walking；
- 当前没有 Mimic、WBC 或其他文本动作正在执行；
- 没有处于 EmergencyStop；
- Jetson 已加载 `text_motion_wbc_stg25_dynamic`；
- Jetson 的固定 `policy.trt` 可用。

如果状态出现：

```text
rejected: another imitation action is active; submit text motion again after it finishes
```

说明已有动作占用 imitation 通道。等待当前 WBC/文本动作回到 AMP 后，再重新运行 `infer.py` 发送。

## 7. 常见问题

### LieDown 完成后为什么不回 AMP？

当前启用的 LieDown Mimic 都配置为 `on_finish: emergency_stop`，结束后进入 EmergencyStop 是预期行为。确认机器人安全后按 `LB + RB` 回 Standby，然后重新进 PD、AMP。

### WBC 或文本动作完成后去哪里？

预置 WBC 使用 `return_rl`，文本 WBC 使用直接返回 RL。当前默认 RL 是 AMP，所以两者完成后都返回 AMP。

### 为什么推理完成但提示没有订阅者？

通常是 Jetson 控制器未启动、两边 ROS domain 不一致、`ROS_LOCALHOST_ONLY` 设置错误，或者旧节点仍运行在之前的 domain。按照第 6 节关闭旧节点并重新开终端。

### 为什么不能连续提交多条文本？

同一时刻只运行一个 imitation 动作。必须等待上一条动作结束并返回 AMP，再发送下一条。

### 每条文本动作都要重新生成 TRT 吗？

不需要。文本每次只生成 JSON。Jetson 上的固定 stg25 `policy.trt` 只需针对目标 Jetson/TensorRT 版本生成一次。

### Jetson 是否需要 PyTorch 或文本 ONNX Runtime？

不需要。文本模型在本地电脑运行，Jetson 只接收 JSON，并运行固定的 WBC TensorRT engine。

## 8. 两个文本动作 ONNX 的区别

目录中有：

- `text_to_motion_cpu_fixed.onnx`
- `text_to_motion_fp16.onnx`

两者的网络权重、输入输出接口和元数据相同，都是 FP16 神经网络，并不是一个 FP32、一个 FP16。区别主要在 CPU 数值兼容处理：

| 项目 | `text_to_motion_cpu_fixed.onnx` | `text_to_motion_fp16.onnx` |
|---|---|---|
| 网络权重 | FP16，与另一模型完全相同 | FP16 |
| 归一化处理 | 范数转 FP32、设置下限，再转回 FP16 | 直接使用 FP16 `Clip` |
| 图节点数 | 1987 | 1984 |
| Cast 节点数 | 64 | 62 |
| 推荐用途 | 本地电脑 CPU 推理，真机已验证 | GPU 或兼容 FP16 Clip 的环境 |

两个模型接口相同：

```text
输入：
  text          STRING[1]
  current_pose  FLOAT16[1, 9]
  speed         FLOAT32 scalar

输出：
  motion          FLOAT16[frames, 9]
  segment_lengths INT64[2]
  embedding       FLOAT16[1, 1024]
  code_indices    INT64[1, 32]
```

当前电脑实测“右手腹部摆动两次。”时，两者都输出 160 帧、3.20 秒、50 FPS，JSON 完全一致。但真机实验使用并验证的是 `text_to_motion_cpu_fixed.onnx`，所以部署时固定使用它。

文件 SHA-256：

```text
text_to_motion_cpu_fixed.onnx
78b985ae1811a3efde40abd09d1d37438e99738f3ed09ce8405f7db84d717721

text_to_motion_fp16.onnx
fa4a397088dc38d7091c550976713fe009f94729c85f07263a96679314c8e3dd
```

## 9. 已验证的运行链路

```text
x86_64 本地电脑
  文本
    ↓
  infer.py + text_to_motion_cpu_fixed.onnx
    ↓
  完整上半身动作 JSON（9 个关节，50 Hz）
    ↓  ROS 2 /text_motion/json

Jetson ARM64
  C++ 后台接收完整 JSON
    ↓
  左右镜像、重采样、低通、AMP 下半身参考填充
    ↓
  固定 stg25 policy.trt + 动态全身参考轨迹
    ↓
  /rlCmd_imitation
    ↓
  机器人执行文本动作
    ↓
  动作结束后返回 AMP
```

固定 stg25 WBC 接口为：

```text
policy[399] + command_short[240] -> action[21]
```

ROS 2 回调只接收并入队一条完整 JSON。解析、镜像、轨迹准备和 AMP 下半身填充都在后台完成，不阻塞实时控制循环，也不会为每条文本动作重新生成 ONNX 或 TRT。

## 10. 关键文件

| 文件 | 用途 |
|---|---|
| `infer.py` | 本地电脑文本推理和 JSON 发布入口 |
| `text_to_motion_cpu_fixed.onnx` | 真机实验使用的文本动作模型 |
| `src/legged_controllers/config/joystick_imitation_actions.yaml` | Mimic/WBC 手柄映射及结束状态 |
| `src/legged_controllers/config/joy_ros2.yaml` | 手柄基础按键映射 |
| `src/legged_controllers/launch/sim2real.launch.py` | 真机启动入口 |
| `src/legged_controllers/launch/sim2mujoco.launch.py` | MuJoCo 启动入口 |
| `src/legged_deploy/src/rl_controller_text_motion.cpp` | C++ 文本 JSON 与动态参考处理 |
| `src/legged_deploy/policy/l0_v3/text_motion_wbc_stg25_dynamic/` | 固定文本 WBC 策略 |

## 11. 真机验证结论

这份代码已经验证：

- 可以进入 PD 和 AMP；
- 可以从 AMP 使用手柄进入 Mimic；
- 可以从 AMP 使用手柄进入预置 WBC；
- LieDown Mimic 完成后进入 EmergencyStop；
- 预置 WBC 和文本动作完成后返回 AMP；
- 本地电脑可以使用 `text_to_motion_cpu_fixed.onnx` 生成动作 JSON；
- ROS 2 可以将完整 JSON 发送到 Jetson；
- Jetson 可以使用固定 stg25 WBC TensorRT 引擎执行文本动作；
- JSON 轨迹准备过程不会阻塞实时控制循环。
