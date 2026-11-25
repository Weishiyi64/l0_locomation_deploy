# L0 V5 RK3588 部署源码快照

本分支记录 2026-11-25 本地 ROS 2 Humble / RK3588 部署工作区的源码状态。仓库根目录可作为 ROS 工作区。

| 目录 | 内容 |
| --- | --- |
| `src/l0_ros2_deploy_v2/` | 运控、WBC Base/LoRA/Static、文本动作生成、HPM 启动和机器人描述 |
| `src/luvbot_ros_interfaces/` | 与部署源码配套的 ROS 接口 |
| `src/odin_ros_driver_rev1/` | 雷达驱动及 SDK |

机器人描述和雷达 SDK 的文件已随源码收录，克隆后即可在 `src/` 中查看。
板端构建和运行前，请在 `src/l0_ros2_deploy_v2/legged_deploy/policy/` 下配齐目标设备的 AMP/WBC 资产，按需配置文本 RKNN 模型及运行依赖。MQTT 设备凭据由目标设备单独配置。

- [WBC 模型打包、安装和使用](src/l0_ros2_deploy_v2/docs/wbc-lora-deployment.md)
- [本快照的文本动作生成入口](src/l0_ros2_deploy_v2/legged_controllers/launch/text_motion.launch.py)
- [10 月 29 日历史交付记录](handoff/20261029/README.md)

JSON → WBC 文本动作链和已集成的 AMP/WBC 交接请参阅仓库的 `l0_v5` 交付分支。
