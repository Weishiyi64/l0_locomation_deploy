# L0 V5 ROS 2 Description

`ros2_description` 分支是 L0 V5 的 ROS 2 / ament_cmake 描述包，提供机器人模型、网格、MuJoCo 场景及 RViz 展示入口。训练环境、Python 执行器、模型转换工具和检查报告保留在上游 `main` 分支。

## 资源

| 路径 | 内容 |
| --- | --- |
| `urdf/l0_v5.urdf` | 基础版 ROS 2 模型 |
| `urdf/l0_v5_auto_charge.urdf` | 自动回充版 ROS 2 模型 |
| `mjcf/l0_v5.xml` | 基础版 MuJoCo 模型及场景 |
| `mjcf/l0_v5_auto_charge.xml` | 自动回充版 MuJoCo 模型及场景 |
| `meshes/` | 两个版本共用的 STL 网格 |
| `launch/display.launch.py` | ROS 2 模型展示入口 |
| `rviz/urdf.rviz` | RViz 配置 |

两个版本的区别是躯干碰撞体：基础版使用网格，自动回充版使用球体和盒体。模型的质量、惯量、关节参数、电机限幅与雷达安装位姿均保留上游数值。

URDF 网格使用 `package://l0_v5_description/meshes/...` 路径；MJCF 使用 `../meshes/...` 相对路径。所有资源安装至 `share/l0_v5_description`，可通过 ament 索引定位。

## 构建与展示

在机器人的 Linux/ROS 2 环境中，从工作空间根目录构建：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --packages-up-to l0_v5_description
source install/setup.bash
ros2 pkg prefix --share l0_v5_description
ros2 launch l0_v5_description display.launch.py
```

展示入口支持：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `gui` | `true` | 启用关节状态 GUI；设为 `false` 时使用无界面的关节状态发布器 |
| `rviz` | `true` | 启动 RViz |
| `use_sim_time` | `false` | 使用仿真时钟，需要外部发布 `/clock` |

该入口只展示基础版模型，不启动硬件或控制器。部署时使用 `urdf/l0_v5.urdf`，MuJoCo 消费者使用 `mjcf/l0_v5.xml`；自动回充版可由消费者按表中的路径单独加载。

URDF 保留上游浮动基座；`robot_state_publisher` 展示时会将不支持的浮动关节视为固定关节，RViz 的固定坐标系为 `base_link`。MuJoCo 模型保留浮动基座和原有碰撞配置；URDF 不表达按连杆对设置的碰撞排除。

## 许可证

上游未提供许可证文件，`package.xml` 的许可证暂标为 `TODO`，需由仓库所有者确认。
