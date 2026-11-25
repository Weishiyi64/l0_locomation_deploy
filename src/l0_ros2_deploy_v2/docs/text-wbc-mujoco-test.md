# MuJoCo 测试：checkpoint_2000 WBC 的五条动作与文本 JSON

更新：2026-10-30。机器人为 `l0_v5`，站立/行走基线为 `l0_v5_amp_07090`。文本动作先由 `texttomotion/run_fp16.onnx` 生成完整 JSON，再按现有 `legged_deploy/policy/l0_v5/wbc/` 动作目录的方式，转成每条独立的 `reference_motion.onnx`。`TEXT01`～`TEXT10` 与五条 `A-*` 动作现均共用由 `/home/luvbotic/下载/稳定下半身，强化上半身/checkpoint_2000.pt` 导出的 `checkpoint_2000.onnx`，由 WBC worker 执行完整 21 关节策略。五条 `A-*` 参考轨迹已从 `/home/luvbotic/json1009/新json` 重新生成并覆盖原有动作目录。

**要看五条重新打包的 WBC 动作，直接执行第 0 节。** 已打包动作的参考姿态保存在 `wbc/amp_stand_profile.json` 中，本次测试不用采集 AMP 站姿，也不用运行 `texttomotion/run_offline.sh` 或 `package_text_wbc.py`。后面的第 1～4 节只用于将新的文本 JSON 添加成 WBC 动作；`--wbc-input` 只生成 JSON，不向 `/text_motion/json` 发布，所以单独运行时机器人不会运动。x86 使用 OpenVINO 执行 ONNX。

## 0. 直接测试已重新打包的五条 WBC 动作

五条动作是 `A-C04_mod`、`A-C07_mod`、`A-S04_mod`、`A-C08_mod`、`A-T08-02_mod`。它们的 `source.json` 与 `/home/luvbotic/json1009/新json` 对应文件一致，`policy.onnx` 均指向 `checkpoint_2000.onnx`。如果 MuJoCo 已在运行，先在主 launch 终端按 Ctrl+C，等待其退出；重新编译并启动后才会加载这次替换的动作文件。

终端一，编译并启动只加载这五条动作的 MuJoCo：

```bash
source /opt/ros/humble/setup.bash
cd /home/luvbotic/l0_deploy_v2_ws
colcon build --symlink-install --packages-up-to legged_controllers legged_real legged_mujoco mujoco --packages-skip luvbot_cloud_bridge --cmake-args -DLEGGED_DEPLOY_BACKEND=OPENVINO
source install/setup.bash
export ROS_DOMAIN_ID=188
ros2 launch legged_controllers sim2mujoco.launch.py robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 wbc_enabled:=true wbc_mode:=static 'wbc_policies:=[A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' joystick_sequence_enabled:=false joystick_back_emergency_sequence_enabled:=false rviz:=false
```

确认启动日志出现 `WBC worker ready: 50 Hz, 5 motions`。终端二进入 Walking，启动 MuJoCo 动力学；有手柄时按 B、松开、B+Y、松开，可跳过四条 `/joy` 命令：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 service call /sim/mujoco/start_dynamics std_srvs/srv/Trigger '{}'
```

等机器人站稳。终端三观察 WBC 回执：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

回到终端二，先定义发送函数，然后**每次只执行一条** `send_wbc`。确认该 `action_id` 的回执为 `started → completed`、机器人回到 AMP 且站稳后，再手动执行下一条；不要一次粘贴五条发送命令：

```bash
send_wbc() {
  ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
    "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"$1\"}'}"
}
```

```bash
send_wbc A-C04_mod
```

```bash
send_wbc A-C07_mod
```

```bash
send_wbc A-S04_mod
```

```bash
send_wbc A-C08_mod
```

```bash
send_wbc A-T08-02_mod
```

若反馈为 `rejected`、`failed`、`stopped` 或发生急停，停止后续动作并检查主 launch 日志。测试完毕在主 launch 终端按一次 Ctrl+C，等待退出。

## 可选：把新文本 JSON 打包为 WBC 动作

以下第 1～4 节只用于生成新的文本 JSON 并追加为 WBC 动作。参考站姿已经保存在仓库，按此流程也无需重新采集 21 关节 AMP 站姿。只测试第 0 节的五条动作时，到这里即可停止。

## 1. 编译并启动 AMP（第一次 MuJoCo）

终端一：

```bash
source /opt/ros/humble/setup.bash
cd /home/luvbotic/l0_deploy_v2_ws
colcon build --symlink-install --packages-up-to legged_controllers legged_real legged_mujoco mujoco \
  --packages-skip luvbot_cloud_bridge --cmake-args -DLEGGED_DEPLOY_BACKEND=OPENVINO
source install/setup.bash
export ROS_DOMAIN_ID=188
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 wbc_enabled:=false \
  joystick_sequence_enabled:=false joystick_back_emergency_sequence_enabled:=false rviz:=false
```

终端二：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2

# 无手柄时依次发送；有手柄则按 B、B+Y，跳过四条 /joy 发布。
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 topic pub --once /joy sensor_msgs/msg/Joy '{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'

ros2 service list | grep -Fx /sim/mujoco/start_dynamics
ros2 service call /sim/mujoco/start_dynamics std_srvs/srv/Trigger '{}'
```

确认 `FSM: Standby -> Walking`、`success=True` 和启动终端的 `Starting MuJoCo dynamics on manual request`。等待机器人站稳。若服务不存在，检查 MuJoCo 窗口已初始化且终端使用同一 `ROS_DOMAIN_ID`。

## 2. 使用已保存的 WBC 参考站姿

仓库中的 `wbc/amp_stand_profile.json` 已存有用于这次五条动作和文本 WBC 的 21 关节参考站姿，`neck_pitch_joint` 的中位数约为 `0.005 rad`。这一步只核对文件存在，不需要运行 `capture_wbc_stand_profile.py`。终端二：

```bash
ls -lh legged_deploy/policy/l0_v5/wbc/amp_stand_profile.json
```

`--wbc-input` 仍会自动读取当时的九关节姿态作为文本模型起点；这与重新采集用于 WBC 打包的 21 关节站姿是两件事。若运行时 `neck_pitch_joint` 又接近 `+0.1745 rad`，先排查 AMP 站姿与控制器反馈，不要改动已保存的参考站姿来掩盖问题。

## 3. 生成文本 JSON，不直接播放

仍在终端二。每条都重新采集当前九关节姿态作为 `--start-pose` 和 `--end-pose`，执行 ONNX、关节限位裁剪和 MuJoCo 运动学校验。`--wbc-input` 不发布文本动作，AMP 继续站立。生成失败就先处理错误，不打包缺失或过期的 JSON。

```bash
bash texttomotion/run_offline.sh --wbc-input --text '头以大幅度慢速点头两次。' --output /tmp/l0_text_wbc/TEXT01.json
bash texttomotion/run_offline.sh --wbc-input --text '头以小幅度快速左右倾斜三次。' --output /tmp/l0_text_wbc/TEXT02.json
bash texttomotion/run_offline.sh --wbc-input --text '头以中幅度慢速向左转。' --output /tmp/l0_text_wbc/TEXT03.json
bash texttomotion/run_offline.sh --wbc-input --text '左手以大幅度抬起。' --output /tmp/l0_text_wbc/TEXT04.json
bash texttomotion/run_offline.sh --wbc-input --text '右手以小幅度慢速前后摆动两次。' --output /tmp/l0_text_wbc/TEXT05.json
bash texttomotion/run_offline.sh --wbc-input --text '左手摸肚子两次。' --output /tmp/l0_text_wbc/TEXT06.json
bash texttomotion/run_offline.sh --wbc-input --text '头点头；右手放后面。' --output /tmp/l0_text_wbc/TEXT07.json
bash texttomotion/run_offline.sh --wbc-input --text '头以小幅度慢速点头；左手以大幅度抬起。' --output /tmp/l0_text_wbc/TEXT08.json
bash texttomotion/run_offline.sh --wbc-input --text '左手以小幅度前后摆动两次；右手放后面。' --output /tmp/l0_text_wbc/TEXT09.json
bash texttomotion/run_offline.sh --wbc-input --text '头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。' --output /tmp/l0_text_wbc/TEXT10.json
```

检查 `safety_adjustments.per_joint` 中裁剪前后的范围。WBC 跟踪的是**已裁剪的 JSON**；如果文本模型本身只生成小角度，WBC 不会把它自动放大。当前十条 JSON 和十个动作目录均已存在。再次生成同名 JSON 不会自动更新动作目录中的 `source.json` 和 `reference_motion.onnx`；当前 `/tmp/l0_text_wbc/TEXT01.json`、`TEXT02.json` 比各自已打包的 `source.json` 更新，测试时播放的是动作目录里的版本。完成所需生成后，在终端一按 Ctrl+C，等待第一次 MuJoCo 完全退出。

## 4. 打包到现有 WBC 目录

仓库已提供 `legged_deploy/policy/l0_v5/wbc/checkpoint_2000.onnx`、`checkpoint_2000.yaml` 和导出记录。ONNX 由指定的 `checkpoint_2000.pt` 与匹配 Base/LoRA 导出；不是把 `.pt` 改名。当前 `TEXT01`～`TEXT10` 已全部打包，**不要再次运行相同动作 ID 的打包命令**；脚本会拒绝覆盖已存在的目录。仅测试这些现有动作时，直接进入第 5 节，重启 MuJoCo 并在 `wbc_policies` 中列出全部需要测试的动作。

```bash
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
ls -lh legged_deploy/policy/l0_v5/wbc/TEXT{01..10}/reference_motion.onnx
```

已有打包脚本复用现有 WBC 的 21 关节参考轨迹构造：保存的 AMP 下半身保持为参考值，文本九关节填入上半身；增加 0.2 秒保持、0.8 秒进入、0.8 秒返回和 0.2 秒收尾。文本 JSON 已按机器人左右手坐标输出，打包时不会再次镜像。每个 `TEXTxx/` 有自己的 `reference_motion.onnx`、`rl_config.yaml`、`source.json`，`policy.onnx` 链接到共享 `../checkpoint_2000.onnx`。新动作请使用新的动作 ID；`package_text_wbc.py` 会拒绝覆盖已存在的目录。

检查全部十条已打包动作：

```bash
ls -lh legged_deploy/policy/l0_v5/wbc/TEXT{01..10}/{policy.onnx,reference_motion.onnx,rl_config.yaml,source.json}
```

## 5. 重新编译并启动 WBC（第二次 MuJoCo）

终端一：

```bash
source /opt/ros/humble/setup.bash
cd /home/luvbotic/l0_deploy_v2_ws
colcon build --symlink-install --packages-up-to legged_controllers legged_real legged_mujoco mujoco \
  --packages-skip luvbot_cloud_bridge --cmake-args -DLEGGED_DEPLOY_BACKEND=OPENVINO
source install/setup.bash
export ROS_DOMAIN_ID=188
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 wbc_enabled:=true wbc_mode:=static \
  'wbc_policies:=[TEXT01,TEXT02,TEXT03,TEXT04,TEXT05,TEXT06,TEXT07,TEXT08,TEXT09,TEXT10,A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' \
  joystick_sequence_enabled:=false joystick_back_emergency_sequence_enabled:=false rviz:=false
```

十条文本动作加五条 `A-*` 动作时，应看到 `WBC worker ready: 50 Hz, 15 motions`。**只在目录中打包动作不会把它加入已运行的 worker；`wbc_policies` 在启动时确定，追加动作后必须重启 MuJoCo。** 如果 `TEXT01`、`TEXT02` 能播放而 `TEXT03` 返回 `rejected`，先检查当前启动命令是否仍是旧的七动作列表。若 WBC 资产缺失，启动会明确报出路径。按第 1 节终端二的 B、B+Y 和 `start_dynamics` 步骤再次进入 Walking 并放行动力学；等机器人站稳。这十五条动作均加载 `checkpoint_2000.onnx`，但各自动作目录使用独立的参考轨迹。

若这次只测试重新打包的五条 `A-*` 动作，保留本节的编译、`source` 和 `ROS_DOMAIN_ID` 步骤，把上面的 `ros2 launch` 换为下面这条。启动日志应显示 `WBC worker ready: 50 Hz, 5 motions`：

```bash
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 wbc_enabled:=true wbc_mode:=static \
  'wbc_policies:=[A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' \
  joystick_sequence_enabled:=false joystick_back_emergency_sequence_enabled:=false rviz:=false
```

## 6. 逐条测试文本 WBC 与原有五条 WBC

另开终端三，source ROS 和工作区、设置同一 `ROS_DOMAIN_ID`，持续观察：

```bash
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

终端二定义发送函数；每调用一次都用当前时间戳。先发 `TEXT01`，等回执中同一 `action_id` 的 `started → completed`，机器人恢复 AMP 并站稳后再发下一条。若回执为 `rejected`、`failed`、`stopped` 或发生急停，先排查，不继续发送。

```bash
send_wbc() {
  ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
    "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"$1\"}'}"
}
send_wbc TEXT01
```

先分别运行 `send_wbc TEXT01`、`send_wbc TEXT02`，等前一条完成并站稳后再发下一条。然后逐条运行 `send_wbc TEXT03` 至 `send_wbc TEXT10`；每条都必须已列入本次启动的 `wbc_policies`。WBC 执行期间 AMP 暂停，结束后返回 AMP；下半身参考角度固定，但 WBC 仍输出腿部平衡修正。

五条重新打包的 `A-*` 动作按下面顺序测试。每次只执行一行；确认反馈中的同一 `action_id` 出现 `started → completed`、机器人回到 AMP 并站稳后，再执行下一行。若反馈 `rejected`、`failed`、`stopped` 或发生急停，停止后续测试并检查启动终端日志。

```bash
send_wbc A-C04_mod
send_wbc A-C07_mod
send_wbc A-S04_mod
send_wbc A-C08_mod
send_wbc A-T08-02_mod
```

这五条使用 `wbc/amp_stand_profile.json` 中已保存的 AMP 站姿；本次测试不需要重新采集。配置、参考轨迹和源 JSON 的身份记录在 `wbc/manifest.json`。五个动作目录中原先的 RKNN 策略链接和参考轨迹已移除；当前文件供 x86 MuJoCo 的 OpenVINO 后端测试，RK3588 真机使用前需要针对 `checkpoint_2000` 重新转换并验证 RKNN。

## 7. 判定与排查

- 每条文本动作：检查 `TEXTxx/source.json` 的文本、关节范围和裁剪记录，与 MuJoCo 动作部位、方向和次数对应；WBC 回执为 `started → completed`，随后 AMP 恢复。
- 若生成 JSON 的参考动作本来就小，先检查 `safety_adjustments.per_joint`、起始姿态和提示词。WBC 不改变 JSON 的机械限位。
- 若打包报模型接口错误，检查 `checkpoint_2000.onnx` 是否为本仓库导出的两输入静态策略，不能把 `.pt` 直接放进运行目录。
- 若 WBC 启动失败，检查 `install/legged_deploy/share/legged_deploy/policy/l0_v5/wbc/TEXTxx/` 是否包含三个资产，并确认重新编译、source 与重启了第二次 MuJoCo。
- 若动作被拒绝，检查 Walking、零速度、有效 AMP 命令、WBC worker 日志和 `/system/state_machine/wbc_action_fdb`。不要绕过姿态、超时或急停校验。

若动作开始或结束时出现额外点头，先在另一个已设置相同 `ROS_DOMAIN_ID` 的终端运行下面的观察命令，再手动发送该动作；将 `TEXT04` 换成实际动作 ID。脚本会同时打印头部目标角度和 MuJoCo 反馈，`started → completed` 区间使用 WBC 回执判定：

```bash
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
/usr/bin/python3 -B texttomotion/inspect_motion.py --wbc-action TEXT04 --seconds 15
```

打包后的 `reference_motion.onnx` 与电机目标角不同：前者是 WBC 的参考输入，后者是 WBC 输出并经控制器切换后的目标。若参考中的 `neck_pitch_joint` 不动而电机目标点头，排查 WBC 输出和切换；若目标不动而反馈点头，排查实际跟随。`--wbc-input` 已将同一次实时九关节姿态传给文本模型的起止输入，不能用“未传初始姿态”解释当前已打包动作的首尾点头。

测试结束后在主 launch 终端按一次 Ctrl+C，等待清理结束。

## 8. checkpoint_2000 导出记录

`wbc/checkpoint_2000_export_manifest.json` 记录输入 checkpoint、adapter、Base 和导出 ONNX 的 SHA256。导出时验证 LoRA 与完整 checkpoint 的 actor 一致、两种归一化统计一致；33 组 CPU 输入的 ONNX 与 checkpoint 最大绝对误差为 `8.35e-7`。这属于离线模型导出验证，实际 MuJoCo 跟踪效果由本篇测试确认。需要重新导出同一模型时，使用仓库的 `export_wbc_static_checkpoint.py` 和原始三个 `.pt`/Base ONNX/YAML；不要手工替换共享策略文件。
