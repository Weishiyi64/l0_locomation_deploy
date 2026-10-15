# head_script.yaml 使用说明

## 作用
头部动作时间线配置：按时间轴控制机器人的 pitch（俯仰）、yaw（左右转）、roll（左右倾），单位均为**度（°）**。

## 加载方式
由 **`LeggedController` 节点**的 ROS 参数 `head_script_config` 指定本文件路径。真机常见入口：`sim2real.launch.py` → `sequential_launch_real.sh`，或 `legged_real.launch.py` 等。**`legged_deploy` 的 `rl_controller` 不读取该参数。** 总时长由所有 segment 的 **最大 t_end** 决定。

---

## 角度符号约定
| 轴 | 正方向 | 负方向 |
|----|--------|--------|
| **pitch** | 低头 + | 抬头 - |
| **yaw**   | 左转 + | 右转 - |
| **roll**  | 右偏 + | 左偏 - |

---

## 每个 segment 的写法

每个 segment 必须包含：`t_start`、`t_end`（秒）。同一时间段内，pitch / yaw / roll 可任选下面一种方式，未写的轴保持上一段结束值或 0。

### 1. 恒定角度
```yaml
- t_start: 0.0
  t_end: 2.0
  pitch: 30.0
  yaw: 0.0
  roll: 0.0
```
该段时间内角度保持不变。

### 2. 线性插值（lerp）
```yaml
- t_start: 2.0
  t_end: 3.0
  pitch_from: 30.0
  pitch_to: -15.0
```
从 `pitch_from` 线性过渡到 `pitch_to`。同理可用 `yaw_from`/`yaw_to`、`roll_from`/`roll_to`。

### 3. 正弦摆动（sin）
```yaml
- t_start: 6.0
  t_end: 9.0
  pitch_sin_amplitude_deg: 4.58
  pitch_sin_frequency_hz: 1.2
```
该段时间内该轴按正弦波摆动。振幅单位：度；频率单位：Hz。同理可用 `yaw_sin_amplitude_deg`/`yaw_sin_frequency_hz`、`roll_sin_amplitude_deg`/`roll_sin_frequency_hz`。

---

## 组合与注意
- 同一 segment 里可以混用：某轴恒定、某轴 lerp、某轴 sin。
- 时间段按 `t_start` 排序，且不要重叠同一轴的控制（同一轴只在一个 segment 里用一种方式）。
- 修改后保存，重启或重新加载使用该参数的节点后生效。
