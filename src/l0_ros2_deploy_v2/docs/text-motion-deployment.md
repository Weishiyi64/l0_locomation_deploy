# L0 v5 文本动作与新 WBC：MuJoCo 完整测试流程

**2026-10-29 资产更新：** `wbc/` 下五条 `A-*` 动作已从 `/home/luvbotic/json1009/新json` 重新打包，现与 `TEXT01`、`TEXT02` 一起使用 `checkpoint_2000.onnx`。五条动作原有的 RKNN 文件已移除；下文关于 `checkpoint_10000` 和旧 RKNN 校验结果属于历史记录。当前 MuJoCo 测试命令以 [text-wbc-mujoco-test.md](text-wbc-mujoco-test.md) 为准。

**操作文档入口：** [当前 MuJoCo 测试流程](text-wbc-mujoco-test.md) · [真机历史流程](text-wbc-real-test.md)。当前 MuJoCo 文档包含已打包的两条文本动作、五条重新打包的 `A-*` 动作，以及追加其他文本动作的方法；下文保留技术说明与历史验证记录。

本文为 2026-10-28 新版模型适配后的流程，替代旧 `--init_pose` 命令。机器人使用 `l0_v5`，AMP 使用 `l0_v5_amp_07090`。以下命令只启动 MuJoCo。

## 1. 编译（终端一）

```bash
source /opt/ros/humble/setup.bash
cd /home/luvbotic/l0_deploy_v2_ws
colcon build --symlink-install --packages-up-to legged_controllers legged_real legged_mujoco mujoco --packages-skip luvbot_cloud_bridge
source install/setup.bash
```

云桥不是本测试依赖，不必为这轮测试构建 `luvbot_cloud_bridge`。

## 2. 启动 AMP + 五条新 WBC（终端一，保持运行）

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 \
  wbc_enabled:=true \
  'wbc_policies:=[A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' \
  joystick_sequence_enabled:=false \
  joystick_back_emergency_sequence_enabled:=false rviz:=false
```

启动后应看到 `WBC worker ready: 50 Hz, 5 motions`，并等待 MuJoCo 窗口和全部电机就绪。x86 使用 OpenVINO 执行 ONNX；RK3588 构建使用 RKNN。启动检查按实际编译后端选择资产，不把 ONNX 假装成 RKNN。

## 3. 进入 Walking，手动开始动力学（终端二）

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2

# B → 松开 → B+Y → 松开；不需要反复按 B 增加 Kp。
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'

ros2 service list | grep -Fx /sim/mujoco/start_dynamics
ros2 service call /sim/mujoco/start_dynamics std_srvs/srv/Trigger '{}'
```

确认 `FSM: Standby -> Walking`、服务 `success=True`，以及启动终端 `Starting MuJoCo dynamics on manual request`。等机器人站稳再发动作。若服务列表没有输出，先核对启动终端仍在运行、各终端 `ROS_DOMAIN_ID` 一致，不要反复等待不存在的服务。窗口还未完成初始化时需要等待。

## 4. 打开常驻文本参考窗口（终端三，只打开一次）

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
bash texttomotion/run_offline.sh --preview
```

把参考窗口放到实时 MuJoCo 旁边，等待 `ROS 轨迹和状态监听已启动`。它自动接收每次发布的完整 JSON，收到 `started` 后播放，下一条自动替换，无需重新打开。参考窗口固定底座和腿部；实时窗口是动力学反馈，所以二者不保证逐帧完全一致。

## 5. 十条文本动作（终端二，逐条执行）

每条命令会：采集当前九关节姿态 → 同时传入 `--start-pose` 和 `--end-pose` → 调用新 ONNX → 限位裁剪和运动学校验 → 一次发送完整 JSON → 等待 `completed`。不需要另开监听端口，也不用再手工运行发布器。每条结束、站稳后再运行下一条。

1. 头以大幅度慢速点头两次。

```bash
bash texttomotion/run_offline.sh --live --text '头以大幅度慢速点头两次。' --output /tmp/l0_text_01.json
```

2. 头以小幅度快速左右倾斜三次。

```bash
bash texttomotion/run_offline.sh --live --text '头以小幅度快速左右倾斜三次。' --output /tmp/l0_text_02.json
```

3. 头以中幅度慢速向左转。

```bash
bash texttomotion/run_offline.sh --live --text '头以中幅度慢速向左转。' --output /tmp/l0_text_03.json
```

4. 左手以大幅度抬起。

```bash
bash texttomotion/run_offline.sh --live --text '左手以大幅度抬起。' --output /tmp/l0_text_04.json
```

5. 右手以小幅度慢速前后摆动两次。

```bash
bash texttomotion/run_offline.sh --live --text '右手以小幅度慢速前后摆动两次。' --output /tmp/l0_text_05.json
```

6. 左手摸肚子两次。

```bash
bash texttomotion/run_offline.sh --live --text '左手摸肚子两次。' --output /tmp/l0_text_06.json
```

7. 头点头；右手放后面。

```bash
bash texttomotion/run_offline.sh --live --text '头点头；右手放后面。' --output /tmp/l0_text_07.json
```

8. 头以小幅度慢速点头；左手以大幅度抬起。

```bash
bash texttomotion/run_offline.sh --live --text '头以小幅度慢速点头；左手以大幅度抬起。' --output /tmp/l0_text_08.json
```

9. 左手以小幅度前后摆动两次；右手放后面。

```bash
bash texttomotion/run_offline.sh --live --text '左手以小幅度前后摆动两次；右手放后面。' --output /tmp/l0_text_09.json
```

10. 头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。

```bash
bash texttomotion/run_offline.sh --live --text '头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。' --output /tmp/l0_text_10.json
```

正常输出包含 `received`、`ready`、`started`、`completed`。`received` 仅表示收到了完整 JSON，`ready` 才表示逐帧校验完成；接收/校验不阻塞控制线程。失败时命令返回非零，不应直接发下一条。

## 6. 五条新 WBC 动作（终端二，文本播放结束后）

WBC 文件在 `legged_deploy/policy/l0_v5/wbc/`。五条动作共用新 `base_o47xcbco/checkpoint_10000.onnx` 文件，每条有独立的 `reference_motion.onnx` 和配置。这里是完整 WBC 策略执行，不是九关节文本叠加。执行期间 AMP 暂停，结束后返回 AMP；不要同时发送文本动作。

先在另一个终端查看 WBC 回执（ROS 环境和域同上）：

```bash
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

以下每条等对应 `action_id` 的 `completed` 后再运行下一条。`timestamp_ms` 必须使用发送时的当前时间，不能照抄固定时间戳。

1. `A-C04_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C04_mod\"}'}"
```

2. `A-C07_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C07_mod\"}'}"
```

3. `A-S04_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-S04_mod\"}'}"
```

4. `A-C08_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C08_mod\"}'}"
```

5. `A-T08-02_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-T08-02_mod\"}'}"
```

下半身**参考角度**填充实测 AMP 站立值并全程固定；WBC 仍会输出腿部平衡修正，不能理解为把腿锁死。每条增加 0.2 秒站姿保持、0.8 秒进入、0.8 秒返回和 0.2 秒收尾；WBC 自身另有 1 秒开始保持。源 JSON 原样保存在每条动作目录的 `source.json`。

## 7. 本次修改与验证记录

- `texttomotion/text_motion_onnx.py`：新 CLI `--start-pose` 对接模型实际输入 `init_pose`，`--end-pose` 对接 `end_pose`。删除旧版手工返回段重写和特定摆手轨迹替换，以新模型输出为准；保留左右手坐标映射、机械限位裁剪、逐帧有限数检查。
- `texttomotion/run_offline.sh`：唯一操作入口；`--live` 自动采集、生成、发送并等待完成；`--preview` 常驻参考窗口。不修改模型二进制，不缩放手臂到 0.4。
- `TextMotionController.h/.cpp`：有显式、有效 `end_pose` 时，退出阶段保持模型末帧，再交还 AMP；避免用高文本增益额外拉向 AMP 的零位命令。没有新元数据的旧 JSON 保留旧兼容处理。安全抢占、限位、看门狗不变。
- `prepare_wbc_deploy.py`：在既有打包程序增加 `json-batch`，检查新 checkpoint 的 399/240 输入、21 关节顺序，按关节名填充当前 AMP 下半身参考；从训练的 22 刚体追加部署模型的刚性 lidar 为 23 刚体参考。记录 SHA256 和源动作帧数。
- `legged_deploy/include/openvino_wbc_model.h`、`rl_controller_wbc.h/.cpp`、`CMakeLists.txt`：同一 WBC worker 增加 OpenVINO 静态 FP32 后端，复用原观测、时序、预推理和安全退出逻辑；RKNN LoRA 路径保留。
- `legged_controllers/launch/one_start.launch.py`：支持 `policy/l0_v5/wbc/<动作>`，按编译后端检查 ONNX 或 RKNN；旧平铺动作目录仍兼容。
- `legged_deploy/policy/l0_v5/wbc/`：共享新 checkpoint、五份参考、各动作配置、源 JSON、实测 AMP profile 和 manifest。

2026-10-28 本机 x86、ROS 域 202：构建通过；十条文本连续 received→ready→started→completed；五条 WBC 全部 started→completed→返回 AMP。上述十五条动作执行期间未发现 EmergencyStop 或反馈故障；五条 WBC 推理超时均为 0，完整周期平均 2.25–2.44 ms、最大 3.96 ms（不含首次准备）。十五条动作结束后底座高度 0.44790–0.44811 m，仍保持站立。这证明本轮闭环链路可运行，不表示实际轨迹与参考零误差，也不代表板端/真机已验证。

新文本模型默认 `randomize=true`，同文字的帧数/素材可能不同；本入口不覆盖该默认值。未指定 speed/return 等时使用模型内置默认值。当前导出使用本机 CPU ONNX Runtime。`--start-pose` / `--end-pose` 接受九维弧度数组或采集文件；`--live` 每次重新采集，禁止与手工姿态混用。纯离线不传姿态时使用模型 A-pose，不适合直接当成当前 AMP 起止姿态。

结束测试时先关闭参考窗口，再在主 launch 终端按一次 Ctrl+C，等待进程组清理完毕。

补充：十五条测试结束后，用户确认手动关闭主 MuJoCo 窗口，反馈停止后看门狗触发急停并清理进程；这次关窗退出不计为动作运行失败。重新启动后，第 10、5 条文本动作及常驻参考窗口连续切换均通过，随后由测试进程正常关闭仿真。

## 8. RK3588 资产状态与复现打包

新 WBC 已导出 `checkpoint_10000.rknn`，五个动作的 `policy.rknn` 链接到它；每个动作有自己的 `reference_motion.rknn`。每份 `.rknn.json` 记录来源哈希、Toolkit 版本和验证结果。这里共用策略文件，但没有把这五条参考动作转换成 LoRA，也没有宣称只有一个运行时推理 context。

RKNN Toolkit2 2.3.2 模拟器对比：策略 20 组样本最大绝对输出误差 0.00267839；五条参考逐帧和边界校验分别 464、441、579、405、430 次，均通过既有误差阈值。该模拟器延迟不是 RK3588 板端性能。**尚未连接 RK3588 执行 C++ 验证，也未执行真机测试。** 本次 MuJoCo 运行使用 OpenVINO；板端接入仍保留所有命令有效期、传感器超时和急停检查。

复现 ONNX 打包使用现有工具（输出必须是新目录，以免覆盖现有资产）：

```bash
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python -B prepare_wbc_deploy.py \
  /home/luvbotic/json1009/新json /tmp/l0_wbc_repack \
  --mode json-batch \
  --checkpoint-dir /home/luvbotic/l0_mimic-lite/checkpoints/base_o47xcbco \
  --amp-profile legged_deploy/policy/l0_v5/wbc/amp_stand_profile.json \
  --actions A-C04_mod A-C07_mod A-S04_mod A-C08_mod A-T08-02_mod
```

原始 `run.onnx` / `libluvbot_onnx_text.so.1` 二进制未改写；新 WBC checkpoint 也保留原始 ONNX，仅转换部署格式。参考模型的 `Clip` 上下限按 ONNX 规范导出为标量，修改前后输出及边界查询已验证完全一致。

本次工具环境单独放在 `/tmp/l0_rknn_toolkit`（临时目录，重启后可能被清理），未升级系统或原仿真 Python。构建日志 `/tmp/l0_wbc_final_build.log`，十五条闭环测试日志 `/tmp/l0_text_ten_live.log`、`/tmp/l0_wbc_five_live.log`，控制器日志 `/tmp/l0_wbc_cpu_sim.log`；常驻窗口补测日志 `/tmp/l0_new_reference_window.log`。持久验证摘要在 `policy/l0_v5/wbc/manifest.json`。

## 9. 2026-10-28 FP16、限位与手臂跟随复查（以本节为准）

### 精度的实际边界

| 路径 | 当前资产与执行方式 | 已验证 / 未验证 |
| --- | --- | --- |
| 文本生成 | 原始 `texttomotion/run.onnx` 保留；`run_fp16.onnx` 是部署副本，原有网络权重 FP16，加上动作素材表 FP16；分词 `.so` + CPU ONNX Runtime | 十条转换前后素材一致、输出有限、起止姿态一致；两条 FP16 副本闭环完成。没有文本 `.rknn` |
| 五条直接 WBC | `policy/l0_v5/wbc`；编译临时图显式 FP16 参数存储，参考表 FP16，RKNN 非 INT8 量化 | 策略 20 组模拟器对比最大误差 0.00259447；五条参考逐帧/边界校验通过；板端未运行 |
| 80 条 LoRA | `policy/l0_v5/wbc_lora_fp16`；共享 Base、80 份 31 张量参数包、80 份参考均完成 FP16 导出 | 一份共享策略 + 80 份参考 RKNN 编译成功；详见 LoRA 正式部署文档的本次复查节 |

FP16 指网络权重、LoRA 参数和参考素材的存储/编译精度。输入输出 ABI、ROS 消息、关节角计算仍可为 FP32/FP64；字符串、布尔、帧索引必须保持各自类型；物理限位和时间计算常数不做破坏性半精度截断。RKNN 编译器还可能对部分算子采用混合精度，不能声称每个内部算子都是 FP16。

新文本模型实际含 `luvbot.text::StudentCharTokenizer`、字符串输入、动态分支与变长轨迹。Toolkit2 2.3.2 实测 `load_onnx` 返回 0，但 `build` 在常量折叠时报 `KeyError: 'mode'`，没有生成可用 RKNN；此外现有 `.so.1` 的 ELF 架构为 x86-64，不能在 aarch64 的 RK3588 上加载。**因此目前可部署的是“x86 主机生成文本 JSON，RK3588 执行 AMP/WBC 和文本播放”方案；纯板端文本生成尚不具备交付条件。** 如必须全流程只在板端运行，需要模型供应方提供兼容 aarch64 的算子库/运行环境，或重新导出可在 RKNN 上执行的分词与固定接口模型；不能通过改扩展名实现。

现有统一入口默认使用 FP16 参数副本。更新原始 ONNX 后重新导出（不覆盖原始文件）：

```bash
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
bash texttomotion/run_offline.sh --model texttomotion/run.onnx \
  --export-fp16 texttomotion/run_fp16.onnx
```

导出器保留原始 API 和默认控制参数，仅转换参数副本；会检测原始模型 SHA256，拒绝运行过期的 FP16 副本。复核时固定 `randomize=false` 仅用于比较，不改变日常生成的随机默认值。十条对比的最大角度差为 0.006235 rad（约 0.36°），各条帧数和回正末帧一致。限位检查仍在生成之后执行。

### 参考模型与裁剪

参考窗口与实际仿真均使用 `legged_description/l0_v5_description/mjcf/l0_v5.xml`，源码与安装路径解析到同一文件；本轮 SHA256 为 `6fc89b8683e3037b6465581c09da024b10621aa6ed0412a27cb832517ac28c33`。参考窗口播放的是实际发送给控制器的裁剪后 JSON，并固定腿与底座；实际机器人由动力学、PD 与 AMP 平衡共同决定姿态。

下表是本次重新生成的十条动作。模型默认带随机性，因此计数可能与上轮不同。所有非零裁剪都位于 `neck_pitch_joint`，这轮没有手臂关节裁剪。

| 动作编号 | 帧数 | 裁剪值数量 | 颈俯仰原始最大值 rad | 说明 |
| --- | ---: | ---: | ---: | --- |
| 1 | 50 | 28 | 0.267488360 | 超过上限，已裁剪 |
| 2 | 135 | 134 | 0.175809830 | 超过上限，已裁剪 |
| 3 | 41 | 2 | 0.174560547 | 超过上限，已裁剪 |
| 4 | 61 | 0 | 0.174532935 | 无裁剪 |
| 5 | 196 | 11 | 0.174532950 | 浮点边界微小超出 |
| 6 | 98 | 2 | 0.174532950 | 浮点边界微小超出 |
| 7 | 61 | 55 | 0.275471747 | 超过上限，已裁剪 |
| 8 | 79 | 28 | 0.271677345 | 超过上限，已裁剪 |
| 9 | 111 | 0 | 0.174532935 | 无裁剪 |
| 10 | 111 | 28 | 0.277531683 | 超过上限，已裁剪 |

每份 JSON 的 `safety_adjustments.per_joint` 现在保存裁剪次数、原始 min/max 与发送 min/max，终端也逐关节打印；另记录 `max_clip_rad` 和超过 1e-6 rad 的裁剪数量，以区分微小浮点边界与明显超限。不修改机械限位。

### 抬手跟随修复

根因之一是 `JointCommandComposer.cpp` 直接沿用了 AMP 上半身 `Kp=1、Kd=0.2` 的软保持增益。下发轨迹本身覆盖了参考幅度，但实际肩关节没有在较短动作时间内到达参考峰值。

修改：仅在 MuJoCo、Walking、文本动作选中该肩/肘关节时使用 `Kp≥16、Kd≥0.4`。未被文本动作选中的关节、AMP 常态、WBC、真机增益均保持原逻辑；原有仿真文本颈部增益独立保留。没有通过抬高限位或缩小参考轨迹伪造一致性。

使用同一批 JSON（未重新随机生成）比较左肩俯仰峰值：

| 动作 | 参考 | 修复前实际 | 修复后实际 |
| --- | ---: | ---: | ---: |
| 4 | -142.18° | -107.97° | -139.50° |
| 8 | -140.33° | -125.84° | -139.97° |
| 10 | -146.26° | -137.80° | -145.85° |

十条同轨迹回归全部 completed，无动作执行急停。第 4 条仍有约 2.7° 峰值误差，不能宣称动力学与参考逐帧相等。此修正已在 MuJoCo 测量；**不把仿真增益直接推广到真机**，真机跟随质量仍需现场反馈确认。

## 10. 真机部署：板端控制 + 主机文本生成 + 五条 WBC

本节给出可执行流程，未实际启动真机。机器人工作区示例沿用 LoRA 正式文档的 `/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws`。HPM 接线、校准、启动脚本和 ROS 网络环境沿用 [LoRA 正式部署手册](wbc-lora-production-deployment.md) 第 4～6 节；不使用仿真的 domain 203/205，也不调用 `/sim/mujoco/start_dynamics`。

### A. 同步源码及资产（本地主机）

按 LoRA 正式文档同步修改后的源码和匹配接口，再同步模型目录。这里是操作命令，不表示已经向机器人传输过文件：

```bash
local_repo=/home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
robot_target=forlinx@192.168.5.2
robot_repo=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
rsync -av "$local_repo/legged_deploy/policy/l0_v5/wbc/" \
  "$robot_target:$robot_repo/legged_deploy/policy/l0_v5/wbc/"
# 如同时部署 LoRA 80 动作，再同步这份独立包。
rsync -av "$local_repo/legged_deploy/policy/l0_v5/wbc_lora_fp16/" \
  "$robot_target:$robot_repo/legged_deploy/policy/l0_v5/wbc_lora_fp16/"
```

保留相对符号链接，必须整体复制 `wbc`，不能只复制单个动作文件夹。AMP 的现有 RKNN 及其链接目标也必须按正式部署文档保留。文本模型和 x86 算子库留在生成主机，不需要复制到板端才能播放 JSON。

### B. 板端构建与运行依赖检查

```bash
robot_ws=/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws
source /opt/ros/humble/setup.bash
cd "$robot_ws"
colcon build --symlink-install \
  --packages-up-to legged_deploy legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN
source install/setup.bash
cat install/legged_deploy/share/legged_deploy/inference_backend.txt
ldd install/legged_deploy/lib/legged_deploy/rl_controller
```

后端必须显示 RKNN，动态库不得有 `not found`；RKNN_ROOT/librknnrt 的准备采用正式文档既有步骤。x86 本机 build/install 不能复制当作板端二进制。

### C. 板端启动（HPM、控制器分别保持运行）

先按正式文档确认并保存 `deploy_config/wbc-production.env`，其 ROS domain、RMW、网络设置须与 HPM 一致。HPM 终端：

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/install/setup.bash
source /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/deploy_config/wbc-production.env
bash "$HOME/hpm_start.sh"
```

控制器终端：

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/install/setup.bash
source /userdata/forlinx/wbc_workspace/l0_deploy_v2_ws/deploy_config/wbc-production.env
ros2 launch legged_controllers sim2real.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 hpm_mode:=real \
  wbc_enabled:=true wbc_mode:=static \
  'wbc_policies:=[A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' \
  joystick_sequence_enabled:=false \
  joystick_back_emergency_sequence_enabled:=false rviz:=false
```

确认 motors_ready、AMP 持续有效命令和 `WBC worker ready` 后，按现场既有手柄流程 B、B+Y 进入 Walking，零速度站稳。真机不使用仿真服务。出现持续命令过期、硬件故障或失去反馈时，不继续动作。

### D. 文本生成主机（x86，连接同一 ROS 网络）

主机需 source 自己的 ROS 工作区，不能 source 板端安装目录。按实际 HPM 配置输入 domain；跨主机发现要求两侧均允许网络通信，RMW/DDS 设置也按正式文档一致配置。

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
read -r -p '输入已确认的 HPM ROS_DOMAIN_ID: ' hpm_domain
export ROS_DOMAIN_ID="$hpm_domain"
export ROS_LOCALHOST_ONLY=0
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
ros2 topic info /control/policy/observation
ros2 topic info /text_motion/json
ros2 topic info /text_motion/status
bash texttomotion/run_offline.sh --live \
  --text '左手以大幅度抬起。' --output /tmp/l0_real_text_04.json
```

`--live` 现改为读取 `/control/policy/observation` 中按名称排列的校准后九关节角，检查 Walking 观测启用、消息新鲜度与有限数，再传入 start/end pose。仿真和真机共用此接口，不再用 `motor_type` 当真实 HPM 关节索引，也不读取 `/sim/actuator/motor/state`。

生成及完整 JSON 校验在主机完成，板端后台完整接收/校验后才开始，控制线程不等待模型推理。等待 `received → ready → started → completed`。其余九条文本使用本文第 5 节逐条命令；全部在生成主机运行，网络和主机进程保持在线。若只想板端执行预先生成的 JSON，可在板端 source ROS 后调用 `python3 texttomotion/text_motion_publish.py --input 文件`，不需要 ONNX Runtime；但预生成文件不能假定符合当前起止姿态。

### E. 五条 WBC（板端另一终端）

source 同一个 ROS 工作区和 production.env，再监听 `/system/state_machine/wbc_action_fdb`。使用本文第 6 节的五条 `ros2 topic pub` 命令，动作 ID 不变。每条必须等对应 `started → completed` 并回到 AMP；不要与文本同时发送。未加载进 `wbc_policies` 的动作会被拒绝。

停止时遵循正式文档第 9 节：先停止动作并按既有方式退出 Walking，再停策略/硬件桥，保持 HPM 在线以接收 disable，确认之后再停 HPM。不能用直接关闭主机窗口代替机器人停机。

本节验证边界：本机 MuJoCo 与 RKNN Toolkit 模拟器已验证；真实 HPM、RK3588 时延、真机姿态及实际电机跟随尚未执行验收。现有 x86 `.so` 不支持纯板端生成文本，文档没有声称这一项已解决。
