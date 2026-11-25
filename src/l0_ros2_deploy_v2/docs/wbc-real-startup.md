# 真机 WBC 86 动作启动说明

更新：2026-11-04。当前默认配置已与下载工作区的正式 `deploy_config/wbc-production.env` 对齐：v5、AMP `l0_v5_amp_10281`、WBC `wbc_lora_fp16`、86 个动作，动作列表及顺序相同。

新工作区创建、上传和完整两轮测试见[真机 WBC 与文本生成启动测试](real-wbc-text-startup-testing.md)。目标根目录为 `/userdata/forlinx/wbc_text_workspace1104`，无需再套一层工作区目录。

## 入口和配置

- `ros2 launch legged_controllers wbc_lora.launch.py`：默认预加载 86 个动作。
- `ros2 launch legged_controllers sim2real.launch.py`：默认使用 LoRA 模式并预加载同样 86 个动作。
- `scripts/start_sim2real.sh`：读取工作区根目录 `deploy_config/wbc-production.env`；本次已从下载工作区复制该文件及 `wbc-record-qos.yaml`。
- 预加载不等于执行，启动不会自动轮播这 86 个动作。
- 真机默认倾角设置：roll 30°、pitch 360°、连续违规 10 个周期。pitch 的计算范围为 ±90°，因此 360° 设置不会触发该项 pitch 越限保护；roll 及其他保护逻辑保留。
- `text_wbc.launch.py` 仍显式选择单一 `UpperBody_Enhanced`，没有扩展为 86 个动作。它包含真机入口，因此继承该入口的 pitch 默认值。
- 2026-11-04 已在真机 `/userdata/forlinx/wbc_text_workspace1104` 完成上传、9 个包编译和安装检查，未启动控制链或执行动作。实测范围及日志见[构建记录](real-wbc-text-startup-testing.md#7-2026-11-04-真机部署与编译记录)。ROS launch 从 install 加载，后续改动源码需重新构建安装。

## 在真机更新安装文件

在目标真机的工作区根目录操作。以下 `robot_ws` 取本次新工作区路径，如部署到其他目录，替换为实际路径。应使用该工作区匹配的接口源码和板端依赖。

```bash
robot_ws=/userdata/forlinx/wbc_text_workspace1104
cd "$robot_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-up-to legged_controllers legged_deploy \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN -DLEGGED_DEPLOY_WBC_SIM=OFF
source "$robot_ws/install/setup.bash"
```

部署时除源码外，应携带工作区根目录的 `deploy_config/`。只复制 `src/` 不会自动带上这份配置。硬件/HPM 依照原现场流程维护。

## 启动普通 WBC

以下两种入口选择一种，不能同时启动。

直接 ROS 入口：

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/wbc_text_workspace1104/install/setup.bash
ros2 launch legged_controllers wbc_lora.launch.py
```

下载版的运行记录入口：

```bash
cd /userdata/forlinx/wbc_text_workspace1104/src/l0_ros2_deploy_v2
bash scripts/start_sim2real.sh --dry-run
bash scripts/start_sim2real.sh
```

后者会检查安装资产并创建 `runs/wbc_production/` 记录。脚本会执行 `git rev-parse/status/diff` 保存源码版本，目标源码目录必须属于有效 Git 工作树；仅解压且无 Git 元数据的目录不能保证该记录入口成功。直接 ROS 入口不依赖这项 Git 记录逻辑。

## 使用动作控制台

通过 `start_sim2real.sh` 启动并生成运行记录后，在另一个终端执行：

```bash
cd /userdata/forlinx/wbc_text_workspace1104/src/l0_ros2_deploy_v2
bash scripts/wbc_action_console.sh --dry-run
bash scripts/wbc_action_console.sh
```

输入编号或完整动作 ID 请求执行，`list` 显示列表，`q` 退出。本地控制台使用当前 run 的配置快照，并核对运行 worker。若用直接 ROS 入口启动而未创建 run，不要使用依赖 run 的控制台入口。

## 下载版启动文档原件

[下载版 RK3588 WBC-LoRA 正式部署操作手册](wbc-lora-production-deployment-downloaded-20261103.md) 已原样复制，SHA256 为 `9c0be746b16b663014315dcc4657f8f30f2fe53d62ccd62da7f5c308e344c0f9`。

该原件包含历史四动作包、旧路径及旧配置示例。当前 86 动作配置以工作区 `deploy_config/wbc-production.env` 和本文为准，不要用原件中的历史示例覆盖它。完整资产制作说明继续参见[本地正式部署手册](wbc-lora-production-deployment.md)。
