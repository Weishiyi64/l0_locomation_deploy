# L0 V1 vs V3 URDF 对比

---

## 一、结构差异

| 项目 | V1 | V3 |
|------|----|----|
| robot name | `l0_v1` | `l0_v3_description` |
| battery_link | ✓ 有（固定关节，mass=1.5384 kg） | ✗ 无 |
| neck 关节数 | 3（yaw/pitch/roll） | 3（yaw/pitch/roll）相同 |
| 总 link 数 | 20（含 battery） | 19 |

---

## 二、总质量对比

> V1 总质量含 battery_link（1.5384 kg）。

| 版本 | 总质量 (kg) |
|------|------------|
| V1 | ≈ 11.80 |
| V3 | ≈ 12.76 |

---

## 三、各 Link 质量对比

| Link | V1 mass (kg) | V3 mass (kg) | 差值 (kg) |
|------|-------------|-------------|---------|
| base_link | 3.2327 | 5.7890 | +2.5563 |
| left_hip_pitch_link | 0.3862 | 0.4020 | +0.0158 |
| left_hip_roll_link | 0.2175* | 0.1424 | −0.0751 |
| left_hip_yaw_link | 1.1998 | 1.3030 | +0.1032 |
| left_knee_link | 0.4979 | 0.5340 | +0.0361 |
| left_ankle_pitch_link | 0.0956 | 0.1025 | +0.0069 |
| left_ankle_roll_link | 0.4650 | 0.5038 | +0.0388 |
| right_hip_pitch_link | 0.3862 | 0.4020 | +0.0158 |
| right_hip_roll_link | 0.2169* | 0.1424 | −0.0745 |
| right_hip_yaw_link | 1.1998 | 1.3030 | +0.1032 |
| right_knee_link | 0.4953 | 0.5340 | +0.0387 |
| right_ankle_pitch_link | 0.0948 | 0.1025 | +0.0077 |
| right_ankle_roll_link | 0.4652 | 0.5038 | +0.0386 |
| left_shoulder_pitch_link | 0.2767 | 0.0823 | −0.1944 |
| left_shoulder_roll_link | 0.0268 | 0.4025 | +0.3757 |
| left_elbow_roll_link | 0.2183 | 0.0787 | −0.1396 |
| right_shoulder_pitch_link | 0.2767 | 0.0823 | −0.1944 |
| right_shoulder_roll_link | 0.0268 | 0.4025 | +0.3757 |
| right_elbow_roll_link | 0.2183 | 0.0787 | −0.1396 |
| neck_yaw_link | 0.2893 | 0.3676 | +0.0783 |
| neck_pitch_link | 0.2225 | 0.3053 | +0.0828 |
| neck_roll_link | 0.9809 | 0.9472 | −0.0337 |
| battery_link | 1.5384 | — | — |

*V1 hip_roll 质量在注释中注明为人工修改值（原始 CAD 值更小）。

---

## 四、关节原点（Joint Origin xyz）对比

> 仅列出有差异的关节。

| 关节 | V1 xyz | V3 xyz | 差异说明 |
|------|--------|--------|---------|
| left_shoulder_pitch_joint | `0 0.087694 0.13177` | `0 0.093516 0.13032` | y+0.0058, z−0.0015 |
| right_shoulder_pitch_joint | `0 −0.087694 0.13177` | `0 −0.093516 0.13032` | 对应镜像 |
| left_shoulder_roll_joint | `0 0.0545 0` | `0 0.0499 0` | y−0.0046 |
| right_shoulder_roll_joint | `0 −0.0545 0` | `0 −0.0499 0` | 对应镜像 |
| left_elbow_roll_joint | `0 0.054 0` | `0 0.065 0` | y+0.011 |
| right_elbow_roll_joint | `0 −0.054 0` | `0 −0.065 0` | 对应镜像 |
| left_knee_joint | `−0.0115 0 −0.098` | `−0.0115 0.00039955 −0.098` | y+0.0004 |
| right_knee_joint | `−0.0115 0 −0.098` | `−0.0115 −0.00039955 −0.098` | 对应镜像 |
| left_ankle_pitch_joint | `0 0.0004 −0.113` | `0 0 −0.113` | y−0.0004 |
| right_ankle_pitch_joint | `0 −0.0004 −0.113` | `0 0 −0.113` | 对应镜像 |
| left_ankle_roll_joint | `0.037 0 −0.029451` | `0.035 0 −0.029451` | x−0.002 |
| right_ankle_roll_joint | `0.037 0 −0.029451` | `0.035 0 −0.029451` | x−0.002 |
| neck_yaw_joint | `0 0 0.17167` | `0 0 0.17767` | z+0.006 |
| neck_pitch_joint | `0 0 0.031` | `0 0 0.025` | z−0.006 |

---

## 五、关节限位对比

> 仅列出有差异的关节。

| 关节 | 参数 | V1 | V3 | 差异 |
|------|------|----|----|------|
| left/right_hip_yaw_joint | lower | −1.571 | −1.047 | +0.524 |
| | upper | 1.571 | 1.047 | −0.524 |
| | effort | 14 | 12.5 | −1.5 |
| | velocity | 27 | 12.56 | −14.44 |
| left_ankle_pitch_joint | lower | −0.907 | −0.960 | −0.053 |
| | upper | 1.047 | 1.222 | +0.175 |
| right_ankle_pitch_joint | lower | −0.907 | −0.960 | −0.053 |
| | upper | 1.047 | 1.222 | +0.175 |
| left/right_ankle_roll_joint | velocity | 21.666 | 21 | −0.666 |
| left/right_elbow_roll_joint | lower | −1.745 | −1.571 | +0.174 |
| | upper | 1.745 | 1.571 | −0.174 |
| neck_yaw_joint | effort | 5.5 | 12 | +6.5 |
| | velocity | 10.46 | 21 | +10.54 |
| neck_pitch_joint | effort | 5.5 | 12 | +6.5 |
| | velocity | 10.46 | 21 | +10.54 |
| neck_roll_joint | effort | 5.5 | 12 | +6.5 |
| | velocity | 10.46 | 21 | +10.54 |

---

## 六、关节原点 RPY 对比

所有关节的 rpy 在 V1 与 V3 中完全一致，无差异。

---

## 七、主要变化总结

| 类别 | 变化描述 |
|------|---------|
| 机体重量 | base_link 质量大幅增加（+2.56 kg），整体总质量约增加 0.96 kg |
| 手臂结构 | shoulder_pitch/roll 与 elbow_roll 质量分布大幅重新分配；elbow 关节偏移 y 轴增加 11 mm |
| 手臂限位 | elbow_roll 活动范围收窄（±1.745→±1.571） |
| 髋关节 | hip_yaw 范围收窄（±1.571→±1.047），力矩与速度均下降 |
| 踝关节 | ankle_pitch 限位重新标定（下限更负，上限更正）；ankle_roll 偏移 x 减小 2 mm |
| 膝关节 | knee joint 新增微小 y 轴偏移（约 0.4 mm），对称设置 |
| 颈部 | neck_yaw 安装位置升高 6 mm；neck 三关节力矩与速度大幅提升（effort×2.2, velocity×2） |
| battery_link | V3 已移除，质量可能合并进 base_link |
