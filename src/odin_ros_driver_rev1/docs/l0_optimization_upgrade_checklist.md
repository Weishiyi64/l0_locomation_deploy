# Odin 优化记录与驱动升级恢复清单

本文件记录当前 L0 单 camera0 硬件的需求和适配结果。更新厂商驱动后，按本文件恢复行为，
不要整包覆盖旧源码，以免覆盖厂商的新协议、标定或修复。本文为恢复目标，不代表机器人运行验收已完成。

## v3.7 迁移记录

厂商驱动 3.7.0，内置 SDK 3.6.0，来源见 [COMMIT_INFO.txt](../COMMIT_INFO.txt)。
ROS2 设备话题与服务现使用 `/manifold/device0/`，独立入口为 `driver.launch.py`。
本次修改已恢复 O01～O11 的 L0 行为。设备型号、算法库加载、整机回归及三种模式 CPU 待板端验证。
下方基线表保留 v3.5 的历史记录。现场命令使用 v3.7 前缀。

## 1. 记录基线与边界

| 项目 | 本次记录 |
| --- | --- |
| 整理日期 | 2026-11-11 |
| 厂商发布包 | ROS Driver / SDK 3.5.0；原始来源保留在 [COMMIT_INFO.txt](../COMMIT_INFO.txt) |
| 厂商驱动来源 | `d632a5695829ada9f8d6f5397bcd43070520f823` |
| 厂商 SDK 来源 | `f2738224fbbe21891d9ddd696ea1b9f255b714c9` |
| 整理时本地 HEAD | `d5873a9c24c9171f9278dc6c2e5728ec836e47cb`；本轮适配仍有未提交改动，HEAD 本身不包含完整结果 |
| 硬件 | 只有 camera0，没有 camera1 |
| 维护范围 | 修改 Odin 仓库；Downloads 中同事的业务代码只用于核对需求，不修改 |
| 启动边界 | Odin 可以独立启动，默认配置由 Odin 自己维护，不依赖运控传入参数 |
| 导航需求变更 | `luvbot_navigation` 已不使用，其 BT、Nav2、点云代价地图和 `/scan` 不再作为保留依据 |
| 本地验证范围 | 源码、launch 语法与默认值、差异检查；未在 macOS 编译或运行 ROS，未测 CPU 降幅 |

升级前先保存完整当前目录，或把适配纳入独立 Git 分支。只记录 HEAD 或执行 `git diff`
不能保存未跟踪文件，必须同时保留本文件等新增内容。每次升级后补充新旧版本和验收结果。

## 2. 最终业务需求：这些数据不能关闭

以下话题／服务除 `/tf`、`/tf_static` 外均省略设备前缀。默认前缀是
`/manifold/device0/`，实际设备编号以启动日志为准。

| 代码包／功能 | 必需数据或服务 | 保留原因／条件 |
| --- | --- | --- |
| 运控部署 `l0_ros2_deploy_v2` | `cloud/raw`、`odometry_lf`、设备 TF、`get_calibration` | 规划深度、基座里程计、URDF 安装转换 |
| 探索 `luvbot_exploration-test_robot`：建图与障碍图 | `cloud/slam`、`/tf`、`/tf_static` | 二维建图、障碍图、加载地图后的实时障碍更新 |
| 探索：玻璃检测 | `camera0/undistort` | 建图时提供去畸变图，整机按建图模式启停 |
| 探索：任务控制 | `device_mode`、`save_map`、`load_map`、`set_sync_mode` | 切模式、保存／加载地图、同步设置 |
| 探索：`sensor_monitor` | `imu`、`odometry_lf` | 启用监控时比较 IMU 与里程计航向；因此 IMU 默认保留 |
| 多相机 `l0_multicam_nodes-main`：像素三维查询 | `cloud/raw` | 依赖原始点的顺序和像素索引 |
| 多相机：查询日志 | `odometry_lf` | 记录运动状态 |
| 全局规划 `humanoid_global_planning-main`：手工定位 | `odometry_hf` | 实际用于定位，不能因 LF 已开启而关闭 HF |
| 全局规划：Odin 人体感知联调 | 标定参数、相机 TF | 已确认不需要 Odin 的去畸变图和点云转深度输出 |
| 标定任务 | `camera0/compressed` | camera0 原始压缩图像 |
| SDK 回环／重定位内部输入 | raw 点云、camera0 图像、HF 里程计 | SDK 算法输入，独立于 ROS 图像后处理开关 |
| 回环／重定位 ROS 结果 | 全部 7 个 `loop_closure/*` 结果话题 | 已确认都需要，不能按“仅可视化”删除 |

回环内部输入和 ROS 结果输出是两条不同的处理链，关闭 ROS 后处理不应断开 SDK 算法输入。
Normal 模式下主机回环为 Idle；按任务通过服务切入建图／重定位，不能用启动后暂时没数据判断功能无用。

| 必须保留的回环输出 | 发布条件／持久性要求 |
| --- | --- |
| `loop_closure/optimization_busy` | 优化状态；Reliable、Transient Local、Keep Last 1 |
| `loop_closure/optimization_status_marker` | 状态文字；同上 |
| `loop_closure/optimized_path` | 优化完成后的轨迹；同上 |
| `loop_closure/optimized_cloud` | 优化完成后的全局点云；同上 |
| `loop_closure/map_cloud` | 加载地图后的点云；同上 |
| `loop_closure/corrected_pose` | 实时矫正位姿；Reliable、Volatile、Keep Last 100 |
| `loop_closure/corrected_cloud` | 有点云的关键帧；Reliable、Volatile、Keep Last 100 |

事件结果不保证固定频率；部分话题必须在优化或加载地图后才有消息。
保留 Transient Local，保证后启动的订阅者仍能收到最近一次已发布结果。
细节见 [话题文档](topics.md#回环和重定位)。

## 3. 升级后先恢复默认配置

默认值主要维护在 [launch/driver.launch.py](../launch/driver.launch.py)。
以下属于当前必须达到的状态，其中一些是保留厂商现有行为，不都是本轮新增修改。

| 配置 | 目标默认值 | 升级核查重点 |
| --- | --- | --- |
| `enable_raw_point`、`enable_slam_point` | `true` | 两路点云分别有消费者 |
| `enable_image0` | `true` | 保留标定图像和 SDK 图像输入 |
| `enable_imu` | `true` | 本轮已从关闭恢复为开启，不要回退到旧文档的 false |
| `enable_odom_lf`、`enable_odom_hf` | `true` | 两路分别保留，不新增别名代替原生话题 |
| `enable_tf` | `true` | 保留设备 TF，包括相机、LiDAR／IMU 和定位关系 |
| `enable_image1` | `false` | 不只是默认关闭，还需保留第 4 节的禁用保护 |
| `image_format`、`image_width`、`image_height`、`image_fps` | `mjpeg`、`1280`、`1088`、`30` | 当前相机配置；新设备／固件仍需核对能力表 |
| `device_mode` | `normal` | 独立启动不依赖运控设置；任务需要时用服务切模式 |
| 手写安装静态 TF | 关闭 | v3.7 主驱动由设备提供 TF，规划桥负责机器人安装变换 |
| `tf_publish_rate` | `0` | 关闭定时重发，仍按设备消息正常发布 TF；不是关闭 TF |
| `post_process` | `false` | 独立启动跳过图像后处理节点；整机启动显式开启 |
| `send_image_raw`、`send_image2_raw` | `false` | 不解码／发布 camera raw 图像；第二路不可用 |
| `send_undistort`、`send_rectify`、`send_cloud_render` | `false` | 手动去畸变、双目校正、点云着色默认关闭 |
| `undistort_mapping_only` | `false` | 独立启动保留 `send_undistort` 手动控制；整机启动显式开启，去畸变随模式状态切换 |
| `send_opencv_undistort` | `false` | OpenCV CameraInfo／去畸变辅助节点不启动 |
| `enable_reprojection` | `false` | 点云重投影节点不启动 |
| `enable_pointcloud_to_depth` | `false` | 厂商点云转深度节点不启动；不影响运控自己的规划深度处理 |
| `enable_depth_to_pointcloud` | `false` | v3.7 仅在显式开启时启动此节点 |
| `start_rviz` | `false` | 默认不运行驱动自带 RViz |
| `enable_row_filter`、`min_row`、`confidence_threshold` | `false`、`0`、`7` | 保留现有滤波语义；不要为省 CPU 擅改标定或过滤阈值 |
| 诊断统计开关 `diag_enabled_` | `false` | 默认不逐包统计；诊断服务保留 |

`tf_publish_rate` 的上述目标指默认 launch 路径；当前 C++ 缺省仍为 50。
直接运行节点可执行文件与通过 launch 启动并不完全等价，独立启动应使用下方 launch 命令。

## 4. 必须重新核对／移植的代码改动

| 编号 | 已完成的适配 | 代码位置／搜索入口 | 升级后如何恢复与检查 |
| --- | --- | --- | --- |
| O01 | 移除 raw 派生 gray 图生成和发布 | `src/odin_ros_driver_node.cpp`：`ProcessRawPointCloud`；launch frame 参数 | 不再调用／保留 `HandleGrayImage`，不创建 gray 发布器，不传 `gray_image_frame_id`；保留 raw intensity 字段 |
| O02 | 移除驱动 camera1 数据处理链 | 同文件：回调注册、`InitAsyncQueues`、`BuildDesiredChannels`、发布器初始化 | 不注册第二相机回调，不创建第二路队列／线程／发布器，期望通道集合不含 Image1 |
| O03 | 禁止配置重新开启 camera1 | 同文件：`enable_image1`、参数声明和 `set_channel_config` | 默认 false，ROS2 参数只读；启动传 true 报错，服务启用请求失败且不部分修改其他通道；兼容字段保留并报告 false |
| O04 | 清理后处理中的 camera1 与双目校正 | `src/post_process_node.cpp`：`DeviceProcessor`、参数／服务处理 | 移除第二路订阅、解码器、标定初始化、输出和双目校正初始化；`send_image2_raw`、`send_rectify` 启用请求拒绝，兼容字段保留 |
| O05 | 清理 OpenCV 辅助节点 camera1 路径 | `src/opencv_undistort_node.cpp`、`.hpp` | 不初始化第二相机模型，不订阅或发布 camera1；camera0 可选实现保留 |
| O06 | 不需要的节点跳过整个启动 | `launch/driver.launch.py`：`Node`、`IfCondition` | 给 postprocess、OpenCV、重投影、点云转深度加各自开关条件；不能只把节点内部发布开关设 false |
| O07 | 恢复 IMU 输出 | launch 的 `enable_imu` | 默认 true；设备 TF 与 IMU 数据开关独立，不要误把 TF 中的 imu 坐标系当作 IMU 话题 |
| O08 | 驱动默认不发布手写安装静态 TF | launch；主驱动 `publish_static_tf_`、`GetParam` | 由规划桥负责安装边；驱动将设备 map 相关 TF 转发到 `/tf_static` |
| O09 | 清理 camera1 诊断及工具残留 | `src/diag_subscriber_node.cpp`；主驱动诊断；`config/rosbag2_qos.yaml`；两份 `rviz/driver_view_ros*.rviz` | 不订阅／显示 camera1，不为它建立诊断统计；服务兼容数值字段可保持 0 |
| O10 | 同步使用说明 | `README.md`、`L0_ADAPTATION.md`、`docs/parameters.md`、`docs/topics.md`、本文件 | 明确 IMU 开启、OpenCV 节点跳过、gray 移除、驱动独立默认配置、URDF TF 发布归属 |
| O11 | `camera0/undistort` 按建图模式启停 | `src/odin_ros_driver_node.cpp` 的 `PublishMappingActive`、`src/post_process_node.cpp` 的 `SetupPubSub` 和 `imageCallback`、`launch/driver.launch.py` | 设备初始化及切换模式成功时发布设备前缀下的 `mapping_active`。ROS2 使用 `std_msgs/msg/Bool`、Transient Local、深度 1；SLAM（3）和 ReMap（5）为 true。后处理启用 `undistort_mapping_only` 时按该状态处理。`send_image_raw=false` 仍可输出去畸变图；两路都关闭时跳过逐帧解码。 |

厂商 SDK 的通用 camera1 枚举／接口可以存在；目标是不打开不存在的硬件通道、
不创建其处理链。不能仅凭搜索到 `Image1` 字符串就继续删除 SDK 公共接口。
RViz 和 rosbag QoS 中保留的 camera0 可选输出配置也不等于对应计算正在运行。
当前未为这些适配修改 SDK 协议或回环核心算法。

## 5. TF 归属：不要在升级时改回旧偏移

| 变换／输出 | 发布者与规则 |
| --- | --- |
| 设备自身 TF | Odin 正常转发；涉及设备 map 帧的变换走 `/tf_static`，其他设备变换走 `/tf` |
| `device0/imu → device0/base_link` | 运控的 `planner_sensor_bridge`，通过 `/tf_static` 发布完整平移和旋转 |
| `/planner/base_odometry`、`device0/odom → base_link` | 运控规划桥；复用同一安装外参计算 |
| 单独运行 Odin | 不依赖运控配置；不提供机器人安装静态变换，其他传感器输出正常工作 |
| 联合启动 | `local_planner.launch.py` include Odin launch，传入 `post_process:=true`、`undistort_mapping_only:=true`，安装变换由规划桥发布 |

安装变换计算必须保留方向和旋转：

```text
base_from_imu = base_from_lidar_URDF × lidar_from_imu_设备
imu_from_base = inverse(base_from_imu)
```

规划桥收到首次有效里程计、取得有效设备外参后发布一次 `imu_from_base`。
外参未就绪时等待和限频提示，不发布零变换或旧偏移。
URDF、设备标定或安装方式改变后重启；后加入订阅者应能收到静态变换。
不能直接照抄 URDF xyz，也不能同时启用驱动的旧安装偏移。

此处运控代码只作为外部对接约定，Odin 更新时不应把 URDF 解析或运控依赖搬入驱动：

- `l0_ros2_deploy_v2/legged_controllers/launch/local_planner.launch.py`
- `l0_ros2_deploy_v2/legged_controllers/src/planner_sensor_bridge.cpp`

## 6. 不可破坏的数据约定

| 项目 | 升级后必须保持 |
| --- | --- |
| raw 像素索引 | 正常完整帧为 49152 点（256×192）；原始顺序、数量和无效点占位保留，不抽稀、不重排、不压紧；异常帧不补点伪装完整 |
| raw ROS 消息 | 当前 `height=1`、`point_step=18`；xyz 为 FLOAT32 米；保留 intensity、confidence、offset_time 字段与布局 |
| 坐标和时间 | frame 名称、单位、时间戳与设备标定语义保持兼容；LF／HF 均为原生输出，不擅加定时器改频率 |
| SLAM 点云处理 | 保留现有 SLAM／LF 同步与坐标转换；不能因它属于“后处理”而删除 |
| SDK 算法输入 | raw、camera0、HF 输入不断链；ROS 图像解码关闭不影响 SDK 内部图像处理 |
| 连接与服务 | 设备发现、心跳、重连、标定服务、模式及地图服务保留；它们不属于可直接删除的图像后处理 |
| 业务规划深度 | Odin 的厂商深度节点关闭；运控独立规划深度生产链仍需要 raw，不随之关闭 |

## 7. 已讨论但尚未实施的 CPU 优化

这些不是当前版本的已完成改动，更新后不要误认为已经有补丁需要恢复。

| 建议 | 当前实际状态 | 实施前验证 |
| --- | --- | --- |
| Idle 时跳过 SDK 回环输入准备 | SDK 仍准备点云并提交图像、HF；接口定义 Idle 丢弃输入 | 验证 Idle／建图／重定位／增量建图切换和线程安全；持续建图时收益有限 |
| 减少 camera0 JPEG 复制 | SDK 补尺寸、驱动入队、ROS 消息封装仍有整包复制 | 明确缓冲区所有权与异步生命周期；不能直接删除必要的保活复制 |
| TF 重发为 0 时跳过缓存维护 | 定时重发关闭，但 `ProcessTf` 仍更新缓存 | 保留正常 TF 发布和 map 静态转发；预期收益较小 |
| 关闭诊断的 1 Hz 空定时器 | 诊断默认关闭，定时器仍创建并快速返回 | 保持服务启用／禁用诊断的能力；收益很小 |

异步队列空闲时使用条件变量等待，没有发现忙轮询；不能按线程数直接推断浪费 CPU。
回环核心为外部算法库，当前源码不足以判断其内部全部 CPU 开销。

## 8. 更新后的执行顺序和验收表

1. 保存当前适配、此记录及版本来源，记录新厂商版本；在独立副本或分支更新。
2. 按第 3 节恢复驱动自己的默认值，不在运控 launch 中补一套覆盖参数。
3. 按 O01～O11 移植适配；厂商若已实现相同行为，检查等效结果，不机械重放旧代码。
4. 静态检查 launch 语法、节点启动条件、camera1 启用拒绝、gray 移除、TF 唯一发布关系；运行 `git diff --check`。
5. 在机器人 Linux／ROS2 环境准备 SDK、构建并 source 新安装结果，再做以下验收。不要同时启动两份驱动连接同一设备。

独立启动方式保持不变，不需要运控传参：

```bash
ros2 launch odin_ros_driver_rev1 driver.launch.py
```

| 验收项 | 通过标准 |
| --- | --- |
| 实际进程 | 默认没有 postprocess、OpenCV、重投影、厂商点云转深度、深度转点云和 RViz 进程；不启动 scan 转换 |
| 通道查询 | `get_channel_config` 返回 raw、slam、image0、imu、LF、HF、TF 为 true，image1 为 false |
| 必需服务 | `get_calibration` 可返回标定，模式、保存／加载地图、同步服务可用 |
| 不需要的输出 | Odin 不创建 gray、camera1 发布器；默认不创建 raw image、去畸变图、CameraInfo、厂商深度输出发布器 |
| raw 内容 | 正常帧检查点数、字段、米单位、原始顺序和无效点占位；约 10 Hz 只是当前预期，需实测 |
| 相机内容 | MJPEG 1280×1088、约 30 FPS，压缩图像可供标定消费；主驱动不解码为彩色图 |
| 整机去畸变 | Normal 启动时 `camera0/undistort` 无新帧；进入 SLAM 或 ReMap 后有帧；切回 Normal 或 Relocation 后停止更新。`send_image_raw=false` 时检验 `camera0/raw` 无图像输出，`camera0/compressed` 仍可消费。晚加入的 `mapping_active` 订阅者能收到当前状态。 |
| IMU／里程计 | 三路数据均实际到达；频率按设备实测，不以话题名或 publisher 存在代替数据验收 |
| 单独运行 Odin 的 TF | 保留设备内部关系，没有 Odin 手写的 imu→base_link 安装边；无运控依赖导致的启动失败 |
| 联合运行的 TF | 规划桥发布 `device0/imu → device0/base_link`；无重复发布该安装关系，无环或重复父节点 |
| 安装外参延迟 | 规划桥等待；外参和有效里程计到齐后恢复发布，不能先发零偏移 |
| 后加入订阅者 | 安装静态 TF 可收到；已发布的持久化回环结果可收到 |
| 基座一致性 | 同一时刻比较 `device0/odom → device0/base_link` 与 `/planner/base_odometry` 中 base_link 位姿，在数值精度范围内一致 |
| 回环／重定位 | 算法库成功加载；按实际任务测试模式切换、优化结果、矫正结果、地图加载与重定位 TF；全部 7 路输出按触发条件验收 |
| camera1 保护 | 启动传 true／参数启用／服务启用均不能打开第二路；拒绝请求不影响已运行的其他通道 |
| CPU 比较 | 相同硬件、模式、订阅者、录包和 RViz 条件，分别记录 Normal、SLAM、重定位的进程／线程 CPU；先预热，采样期间避免额外测试订阅干扰 |

常用只读检查命令（前缀按实际设备调整）：

```bash
ros2 node list
ros2 topic list
ros2 service list -t
ros2 topic info /manifold/device0/cloud/raw --verbose
ros2 topic hz /manifold/device0/cloud/raw
ros2 topic hz /manifold/device0/camera0/compressed
# 整机按模式启停验收，配合现有建图流程切换模式：
ros2 topic echo /manifold/device0/mapping_active --qos-durability transient_local --once
ros2 topic hz /manifold/device0/camera0/undistort
# 运控规划桥运行并取得有效输入后执行：
ros2 run tf2_ros tf2_echo device0/imu device0/base_link
ros2 topic echo /tf_static --qos-durability transient_local
```

`topic list` 中出现名称不证明有有效发布者：订阅者也可能让名称出现。
使用 `topic info --verbose` 看发布者，再检查实际数据。服务类型以 `service list -t`
返回结果为准，避免沿用旧版包名。没有发生算法事件时，不要求所有回环话题已有消息。

每次升级完成后，在此追加：新驱动／SDK 版本、适配提交、静态检查结果、机器人验收结果、
三种模式下的 CPU 数据及未通过项。未测量前不要填写节省比例。
