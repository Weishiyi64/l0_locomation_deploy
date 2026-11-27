# L0 V5 真机软 STOP · 2026-11-27

板端 `stop_workspace` 的已实测源码快照。运行配置使用 **Sonic800 73983 WBC**、**AMP 1124** 和 **L0 STOP Decoder v2000**。在 WBC 动作中按 LT＋方向键左，板端异步生成全身回正参考，WBC 追踪后切回 AMP。

[模型与最终训练文档](https://github.com/Weishiyi64/partymimic/blob/train/upright-interpolation/docs/upright-stop-decoder-v2000-20261127.md) · [全身 ONNX](https://github.com/Weishiyi64/partymimic/blob/train/upright-interpolation/assets/upright_stop/decoder-v2000-20261127/stop_full_reference.onnx) · [真机验证记录](handoff/20261127-stop/hardware-summary.json)

## 工作区和运行资产

GitHub 检出根目录是 ROS 2 Humble 工作区，对应真机 `stop_workspace/l0_deploy_v2_ws`。根目录的三个入口脚本已适配此目录布局。底层运行依赖为 RK3588 / aarch64、RKNN Runtime 和机器人已有的 HPM 与设备配置。源码包含 22 个 ROS 包，包括部署、真实硬件桥及配套接口。

在编译前，把配套策略放到 `src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/`：

| 子目录 | 必备文件 |
|---|---|
| `l0_v5_amp_1124/` | `policy.rknn`，以及 [AMP 配置](deploy_config/amp1124/) 中的 `rl_config.yaml`、`joy_config.yaml` |
| `stop_decoder_fp16/` | `stop_decoder.rknn` 和 [站姿标定](deploy_config/stop_decoder_fp16/stand_reference.json) |
| `wbc_base_sonic800_73983_20261124/` | 完整部署动作包，包括 actor、动作参考、`base_manifest.json`、每动作配置和转换校验 |

若从已运行的板端复现，使用同一批次的三个策略目录。把 HPM 设备配置及日志各自留在对应运行环境。ODIN 的 aarch64 SDK 动态库由匹配的设备包提供，放到 `src/odin_ros_driver_rev1/module/sdk_api/lib/aarch64/`，保留 `libodin_sdk.so` 的相对链接链。HPM OTA 固件、第三方 SDK 二进制和实际策略权重单独分发。

```bash
cp deploy_config/wbc-production.example.env deploy_config/wbc-production.env
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
bash ./start.sh --dry-run
```

生产配置默认选择 `l0_v5_amp_1124`，搭配 `wbc_base_sonic800_73983_20261124` 的可用动作。HPM 和运控使用同一 ROS 域。在真机按对应设备流程启动 HPM，确认硬件反馈和电机就绪后，从本工作区启动 `bash ./start.sh`。控制器日志出现 `Soft STOP RKNN FP16 decoder preloaded and warmed` 表示模型就绪。动作选发可用 `bash ./console.sh`。

## 软 STOP 的触发和交接

- **手柄。** WBC 正在执行动作时，按 **LT＋方向键左**。组合键只在按下沿触发，AMP 控制时忽略。
- **命令行。** 在运行所用的同一 ROS 域调用控制器服务。

```bash
source install/setup.bash
ros2 service call /control/wbc/stop robot_interfaces/srv/ControlWbcMotion "{command: 4}"
```

`ControlWbcMotion` 增加 `STOP=4` 并绑定当前动作执行 ID。服务返回表示请求排队，随后由 50 Hz WBC worker 锁定按停帧。原参考的前、当、后一帧交给后台规划器，生成新参考并交回原 WBC 编码器。AMP 用本次执行 ID 预览零速动作，等到新鲜指令后，控制器用 0.35 秒全身混合切换。规划超时或结果不可用时进入同一 AMP 恢复路径。现有姿态保护、看门狗和人工急停继续生效。

| 改动 | 入口 |
|---|---|
| 手柄与 STOP 服务 | [JoystickInput.cpp](src/l0_ros2_deploy_v2/legged_controllers/src/JoystickInput.cpp) · [LeggedControllerRosIo.cpp](src/l0_ros2_deploy_v2/legged_controllers/src/LeggedControllerRosIo.cpp) · [ControlWbcMotion.srv](src/l0_ros2_deploy_v2/robot_interfaces/srv/ControlWbcMotion.srv) |
| 规划、轨迹切换与 C++ 合成 | [rl_controller_wbc_soft_stop.cpp](src/l0_ros2_deploy_v2/legged_deploy/src/rl_controller_wbc_soft_stop.cpp) · [stop_reference_fp16.cpp](src/l0_ros2_deploy_v2/legged_deploy/src/stop_reference_fp16.cpp) |
| AMP 预览和交接 | [rl_controller.cpp](src/l0_ros2_deploy_v2/legged_deploy/src/rl_controller.cpp) · [PolicyMux.cpp](src/l0_ros2_deploy_v2/legged_controllers/src/PolicyMux.cpp) · [LeggedControllerWbc.cpp](src/l0_ros2_deploy_v2/legged_controllers/src/LeggedControllerWbc.cpp) |

## RKNN FP16 模型准备

用 RKNN Toolkit2 **2.3.2**，目标 `rk3588`，`float_dtype=float16`，关闭整数化量化。[转换脚本](deploy_config/convert_stop_rknn_fp16.py)

```bash
python3 deploy_config/convert_stop_rknn_fp16.py stop_decoder.onnx stop_decoder.rknn
python3 deploy_config/convert_stop_rknn_fp16.py amp1124.onnx policy.rknn
```

`stop_decoder.onnx` 是第 2,000 步权重导出的下肢子模型，SHA-256 为 `f0eb0bac77b27425ee379721372eb92c5d496bff4ff0cce4a081f4e4ba720123`。AMP 1124 源模型 SHA-256 为 `d52111f0bf384bc7165bbf4ca5e1bbcafad79ff4d14e7a9dba9fe8268e77d77a`。原验证资产和产生的 RKNN 摘要见 [validation.json](handoff/20261127-stop/validation.json)。

板端用 FP16 Decoder 运行骨盆及腿部补间，C++ 合成上肢与头部。PartyMimic 发布的同版本全身 ONNX 可在 x86 CPU 上一次生成完整轨迹，便于离线验证。

## 验证结果

- 原板端部署构建 22 个包通过，正式启动 dry-run 和手柄状态测试通过。
- FP16 板端对照 111 条轨迹，帧数全部一致，最大关节差 0.00268 rad，推理及合成 P95 11.53 ms。[数值报告](handoff/20261127-stop/validation.json)
- 隔离在线测试验证 WBC 持续发指令时 AMP 能给出匹配的预览，也验证了规划超时的恢复路径。[在线测试](handoff/20261127-stop/online_worker_smoke.json) · [超时测试](handoff/20261127-stop/timeout_worker_smoke.json)
- 2026-11-27 真机日志中 36 次软 STOP 均生成参考并执行到末帧。操作者报告所测动作全部回正、无摔倒。[真机摘要](handoff/20261127-stop/hardware-summary.json)
