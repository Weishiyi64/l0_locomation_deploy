# L0 V5 RK3588 部署源码

`l0_v5` 分支交付当前 L0 V5 / ROS 2 Humble / RK3588 源码，包含 LoRA 共享 WBC、板端文本生成和 JSON → WBC 接入。

```text
src/
├── l0_ros2_deploy_v2/     控制器、模型工具、硬件桥与使用说明
├── luvbot_ros_interfaces/ 匹配版本的 ROS 接口
└── odin_ros_driver_rev1/  雷达驱动源码
```

使用说明：

- [LoRA 正式部署与使用](src/l0_ros2_deploy_v2/docs/wbc-lora-production-deployment.md)
- [板端文本动作生成与 WBC](src/l0_ros2_deploy_v2/docs/text-motion-deployment.md)

所有 `policy/` 目录均不纳入本分支。构建和运行前，需另行部署匹配的 WBC、AMP 及其他策略资产；文本 RKNN 编译产物和工作区运行依赖也按使用说明单独部署。

本仓库根目录作为 ROS 工作区；文档中的工作区路径应替换为实际克隆位置。`src/` 下源码按交付快照保存，机器人描述文件已包含，无需初始化子模块。构建和正式操作按上述两份说明执行。

软件交付变更说明：

- [更新说明与文件／函数级变更清单](handoff/20261029/README.md)
- [离线文档包](handoff/20261029-documentation.zip)（解压后打开 `20261029/更新说明.html`）
