# WBC 手柄与终端并用（2026-11-09）

2026-11-10 本地新增了 [AMP/WBC 交接初始化修复分支](amp-wbc-handoff.md)，八个按键与终端
接口不变。`.23` 工作区已重编译控制接口和依赖；下述 `.41` 板端更新与现场验证记录
对应 2026-11-09 版本，不代表新增交接修复已完成真机动作验收。

普通 WBC 的 MuJoCo 和真机入口共用 `joystick_wbc_request` 节点。节点订阅现有 `/joy`，复用已启动的手柄驱动，将组合键转换为 `/algorithm/brain/wbc_action_cmd` 上的 `{timestamp_ms, action_id}` 请求。终端仍使用同一接口；控制器负责最终的状态、速度、姿态、观测及互斥检查。

## 192.168.120.41 真机启动流程

2026-11-09 已在 `forlinx@192.168.120.41` 的 `/userdata/forlinx/wbc_text_workspace1104` 更新手柄功能并完成 RKNN 后端增量编译，9 个包通过。安装文件、八动作资产及动态库检查通过。上传编译期间没有启动部署控制程序或发送动作指令；随后用户于 2026-11-09 确认已完成实体手柄及真机动作的现场验证。

### 1. 登录及启动前准备

在本机终端登录：

```bash
ssh -o PubkeyAuthentication=no \
    -o PreferredAuthentications=password \
    forlinx@192.168.120.41
```

更新时曾检测到另一个 `planner_workspace` 的控制器和 HPM 正在运行。这是更新时的记录，不代表当前状态。启动本工作区前，按现场停机流程结束旧的部署控制会话，再在其原启动终端退出旧控制程序，避免两套控制器同时向电机发命令。HPM 电机通信服务、校准、使能和站立准备按现场原流程处理；本工作区的 launch 不代替 HPM 启动。

### 2. 加载已编译工作区

以下在机器人终端执行；每个用于控制或查看反馈的新终端都要加载同一环境：

```bash
cd /userdata/forlinx/wbc_text_workspace1104
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_text_workspace1104/install/setup.bash

ros2 pkg prefix legged_controllers
ros2 pkg prefix legged_deploy
```

预期分别输出：

```text
/userdata/forlinx/wbc_text_workspace1104/install/legged_controllers
/userdata/forlinx/wbc_text_workspace1104/install/legged_deploy
```

若指向其他工作区，先修正环境再启动。所有控制终端的 `ROS_DOMAIN_ID` 应与现场 HPM 一致，不套用 MuJoCo 的 localhost/domain 77 配置。本工作区已经编译完成，正常启动无需再次编译。

### 3. 启动普通 WBC 与八动作手柄入口

确认旧部署控制会话已退出、HPM 与电机已准备好后，在终端 A 执行：

```bash
ros2 launch legged_controllers wbc_lora.launch.py \
  hpm_mode:=real \
  joystick_wbc_enabled:=true
```

等待日志出现 `WBC joystick ready`，以及 WBC worker 就绪、86 个动作预加载完成的信息。预加载不会自动播放动作。按现场原流程完成站立准备，使用 **B+Y 进入 Walking/AMP**；摇杆回中、确认机器人站稳后，先松开所有按钮及十字键，再使用下表中的组合键。

八个绑定动作正常完成后返回 AMP；每次完成后松开再按，长按不会连发。原有 **RB 急停、LB+RB 复位**逻辑保留。关闭这次控制会话时，先按现场停机流程处理机器人，再在启动终端按 Ctrl+C，等待子进程退出。

### 4. 查看动作反馈

终端 B 登录同一机器人、加载同一工作区后执行：

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_text_workspace1104/install/setup.bash

ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

正常反馈为同一请求的 `started → completed`。如果出现 `rejected` 或 `failed`，查看终端 A 的原因，不连续重复请求。

### 5. 可选：继续用终端请求动作

终端 C 同样登录并加载工作区。修改动作 ID 后，每次重新执行整段以生成新时间戳：

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_text_workspace1104/install/setup.bash

action_id='L0_excited_1'
wbc_request=$(/usr/bin/python3 -c \
  'import json,sys,time; print(json.dumps({"timestamp_ms":time.time_ns()//1000000,"action_id":sys.argv[1]}))' \
  "$action_id")

ros2 topic pub --once \
  /algorithm/brain/wbc_action_cmd \
  std_msgs/msg/String \
  "{data: '$wbc_request'}"
```

手柄和终端共用同一请求接口，均需处于控制器允许的状态。`mujoco_control` 固定作用于仿真域，不用于真机。上述直接 launch 不生成运行快照，因此不混用依赖运行快照的 `wbc_action_console.sh`。

若只使用终端，将步骤 3 的启动命令替换为：

```bash
ros2 launch legged_controllers wbc_lora.launch.py \
  hpm_mode:=real \
  joystick_wbc_enabled:=false
```

### 6. 文档更新与程序更新

本流程位于源码文档 `src/l0_ros2_deploy_v2/docs/joystick-wbc-control.md`。只修改或上传该 Markdown 文档不需要重新编译、重新 source 或重启运行程序，直接阅读源码目录中的最新版即可。修改 launch、Python 节点、YAML 映射或 C++ 源码后，则需要重新安装构建，让 `install/` 中的运行文件同步更新。

本次手柄代码更新的备份、编译日志与核对记录位于机器人：

```text
/userdata/forlinx/wbc_text_workspace1104/updates/joystick_20261109_UcO7js/
```

## 八个组合键

| 组合键 | 完整动作 ID |
|---|---|
| LT + 上 | `L0_fear_7_A` |
| LT + 下 | `L0_excited_1` |
| LT + 左 | `L0_excited_new_1` |
| LT + 右 | `L0_excited_new_5` |
| RT + 上 | `L0_satisfied_10` |
| RT + 下 | `L0_SayNo_17` |
| RT + 左 | `L0_angry_9` |
| RT + 右 | `L0_angry_new_1` |

配置文件：`legged_controllers/config/joystick_wbc_actions.yaml`。按当前控制器和参考工程的 Joy 布局，LT/RT 是 `buttons[8]`/`buttons[9]`；十字键是 `axes[6]`（左正、右负）和 `axes[7]`（上正、下负）。这里使用数字扳机按钮，不将 LT 速度倍率轴当作动作触发。更换不同布局的手柄时先核对 `/joy`；现有 B、B+Y、RB 急停、LB+RB 复位等控制器按键不变。

## 使用

普通入口默认启用：

```bash
# 本机 MuJoCo，先 source 仿真 install
ros2 launch legged_controllers sim2mujoco.launch.py

# 真机入口：192.168.120.41 已编译，完整步骤见上文
ros2 launch legged_controllers wbc_lora.launch.py
```

两个命令用于不同环境，不要在同一控制会话同时运行。通过原有 B+Y 或仿真终端 `mujoco_control walk` 进入 AMP，姿态稳定后先松开所有按钮和十字键，再按组合键。8 个动作正常完成后均返回 AMP。

终端动作命令仍然可用。例如仿真中：

```bash
ros2 run legged_controllers mujoco_control action L0_excited_1
```

只使用终端时，可关闭新增入口：

```bash
ros2 launch legged_controllers sim2mujoco.launch.py joystick_wbc_enabled:=false
# 真机同名参数：wbc_lora.launch.py joystick_wbc_enabled:=false
```

可用 `joystick_wbc_config_path:=/绝对路径/配置.yaml` 指定映射文件。8 个映射动作必须全部包含在当前 `wbc_policies` 预加载名单中；修改预加载名单时需同时调整/关闭手柄入口。旧 imitation/sequence 手柄绑定与新 WBC 入口不能同时启用，避免同一组合键触发两套动作。

`text_wbc.launch.py` 和 `sim2mujoco_text.launch.py` 显式关闭这 8 个固定动作绑定，因为文本包只加载 `UpperBody_Enhanced`。文本生成终端流程保持原样。

## 输入与反馈规则

- 启动时按住组合键不会触发；在状态允许时，所有按钮和十字键松开至少 0.15 秒后才能触发下一次。
- 每次按键只发一次请求，长按不重复；两只扳机同时按、十字键斜向、混入其他按钮时不发送 WBC 请求。
- 等待反馈或任何 WBC 动作运行期间不排队；动作完成后仍按住的组合键不会自动执行，需松开再按。
- `/joy` 超过 0.5 秒没有新消息，或消息时间戳过旧/格式无效时取消触发准备；恢复消息后仍需松开再按。控制器观测失效或未允许策略推理时同样阻止触发。
- 节点订阅 WBC 活跃状态与动作反馈，日志显示组合键、动作 ID 和 `started/completed/failed/rejected`。反馈缺失时不会自动重试；会保持等待，避免重复执行不确定的请求。
- 手柄和终端同时请求时，最终由现有控制器互斥及状态检查决定是否接受。手柄节点不直接控制电机，不绕过急停、复位、限位或校准逻辑。

此前 `L0_crouch_off` 完成后进 Emergency 的功能仍仅在 MuJoCo 生效，且未绑定到这 8 个组合键；仍可从终端触发。

## 模型与验证范围

真机继续使用原 `wbc_lora_fp16`，Base、LoRA、reference RKNN 和动作配置未修改。MuJoCo 的 `wbc_lora_sim` 从原 6 个动作扩充为 14 个：新增 8 个动作复制同一真机包的 LoRA 和配置，reference ONNX 与真机转换记录的源 SHA256 一致。`L0_SayNo_17` 使用首尾各保留 50 帧的 272 帧版本。来源记录见仿真包 `SIMULATION.json`。

本机没有可用物理手柄设备，自动验证通过发布等价 Joy 消息测试组合键及真实 MuJoCo 控制链。它不替代现场物理手柄的 USB/蓝牙连接、布局和断连事件验收。具体运行结果见 [MuJoCo 测试记录](sim2mujoco-testing.md)。随后已完成 192.168.120.41 的手柄代码更新、板端编译和安装检查，用户随后已确认实体手柄及真机动作完成现场验证；这次真机更新未上传本机新增的 MuJoCo 资产。
