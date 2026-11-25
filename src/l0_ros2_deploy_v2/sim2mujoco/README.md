# L0 v5 MuJoCo 动作查看

**操作文档入口：** [MuJoCo 十条文本＋五条 WBC](../docs/text-wbc-mujoco-test.md) · [真机完整流程](../docs/text-wbc-real-test.md)。两份均包含完整十五条逐条命令；下文保留技术说明与历史验证记录。

这里保留仿真入口。文本模型、算子库、生成和姿态采集脚本在相邻的 `texttomotion/`；部署策略资产仍在 `legged_deploy/policy/l0_v5/`。

## 文本动作：控制器闭环仿真

先从工作区根目录构建，然后在启动终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 \
  joystick_sequence_enabled:=false \
  joystick_back_emergency_sequence_enabled:=false rviz:=false
```

进入 Walking 后手动调用 `/sim/mujoco/start_dynamics`。完整按键、服务和状态检查见 [文本动作部署说明](../docs/text-motion-deployment.md)。启动终端必须持续运行；若关闭 MuJoCo 窗口或主 launch，服务也会消失。

调用服务前，在**另一个终端**加载同一工作区并设置与启动终端相同的 `ROS_DOMAIN_ID`，先检查服务是否可见：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbotic/l0_deploy_v2_ws/install/setup.bash
export ROS_DOMAIN_ID=188  # 与启动终端一致
ros2 service list | grep -Fx /sim/mujoco/start_dynamics
```

若没有输出，先看启动终端是否已出现 `Starting mujoco_sim.launch` 和 MuJoCo 进程；再分别运行 `echo "${ROS_DOMAIN_ID:-0}"` 核对两个终端。服务出现后，按部署说明先进入 Walking，再调用：

```bash
ros2 service call /sim/mujoco/start_dynamics std_srvs/srv/Trigger '{}'
```

服务返回 `success=True` 只表示手动启动请求已登记；以启动终端出现 `Starting MuJoCo dynamics on manual request` 为动力学实际开始的依据。

文本动作的操作入口已移到 `texttomotion/run_offline.sh`：

```bash
bash texttomotion/run_offline.sh --live --text '左手以大幅度抬起。' --output /tmp/l0_text.json
```

从仓库根目录运行。`--live` 每次自动采集当前 start/end pose、生成、校验、发送并等待完成；`--preview` 打开常驻同步参考窗口。完整十条文本与五条新 WBC 闭环测试见 [测试流程](../docs/text-motion-deployment.md)。

## 已部署动作：MuJoCo 参考轨迹预览

例如查看 `legged_deploy/policy/l0_v5/L0_Approve_8`：

```bash
cd /home/luvbotic/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python \
  sim2mujoco/preview_policy.py L0_Approve_8
```

不打开窗口而逐帧校验：

```bash
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python \
  sim2mujoco/preview_policy.py L0_Approve_8 --headless
```

动作名换成 `l0_v5/` 下其他包含 `rl_config.yaml` 和 `reference_motion.onnx` 的目录名即可。此入口使用部署动作的参考轨迹和 L0 v5 MJCF 做运动学播放；**它不执行 `policy.rknn`、不经过 ROS 控制器，也不提供 `/sim/mujoco/start_dynamics` 服务。** 要测试文本动作，必须运行上面的 `ros2 launch legged_controllers sim2mujoco.launch.py`。x86 现已支持 OpenVINO WBC 闭环推理；新五条动作的完整启动和触发命令见 [测试流程](../docs/text-motion-deployment.md)。RK3588 使用 RKNN，板端运行需要另行验证。

新五条 WBC 参考也可用同一预览器查看，例如：

```bash
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python sim2mujoco/preview_policy.py A-C04_mod
```

程序自动查找 `l0_v5/wbc/A-C04_mod`。此命令仅预览参考，闭环执行仍使用完整测试流程中的 ROS WBC 指令。
