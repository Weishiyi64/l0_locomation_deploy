# 参数 / Parameters

> [← 返回 README](../README.md)

参数分两类：**启动期参数（launch-time）**只在节点启动时读取一次；**运行期参数（runtime / dynamic）**可以在节点运行中通过 `ros2 param set`（ROS2）或 `rqt_reconfigure`（ROS1 ddynamic_reconfigure）随时改写。

Parameters fall into two groups: **launch-time** parameters are read once at node startup; **runtime / dynamic** parameters can be changed while the node is running via `ros2 param set` (ROS2) or `rqt_reconfigure` (ROS1). All parameters listed below are exposed as launch arguments in the device launch files (`launch/odin1Lite.launch(.py)` and `launch/odin2Special.launch(.py)`).

## 动态回调参数白名单 / Dynamic Callback Whitelist

> **⚡ 支持动态回调的参数（whitelist） / Parameters with a dynamic callback**
>
> 以下参数注册了动态回调，运行期 `ros2 param set` / `rqt_reconfigure` 改完会立即生效：
>
> The following parameters have a dynamic callback. Changing them at runtime via `ros2 param set` / `rqt_reconfigure` takes effect immediately:
>
> | 参数 / Parameter | 节点 / Node | 作用 / Effect |
> | --- | --- | --- |
> | `confidence_threshold` | `odin_ros_driver_node` | `sdk::SetConfidenceThreshold` |
> | `sensor_mode` | `odin_ros_driver_node` | `sdk::SetSensorMode` |
> | `log_level` | `odin_ros_driver_node` | 切换 SDK / ROS 日志级别 |
> | `enable_raw_point` | `odin_ros_driver_node` | 动态开/关原始点云通道（热切换）/ Hot-switch raw point channel |
> | `enable_slam_point` | `odin_ros_driver_node` | 动态开/关 SLAM 点云通道（热切换）/ Hot-switch SLAM point channel |
> | `enable_image0` | `odin_ros_driver_node` | 动态开/关左相机图像通道（热切换）/ Hot-switch camera0 channel |
> | `enable_imu` | `odin_ros_driver_node` | 动态开/关 IMU 数据通道（热切换）/ Hot-switch IMU channel |
> | `enable_odom_lf` | `odin_ros_driver_node` | 动态开/关低频里程计通道（热切换）/ Hot-switch low-freq odometry channel |
> | `enable_odom_hf` | `odin_ros_driver_node` | 动态开/关高频里程计通道（热切换）/ Hot-switch high-freq odometry channel |
> | `enable_tf` | `odin_ros_driver_node` | 动态开/关 TF 变换通道（热切换）/ Hot-switch TF channel |
> | `send_image_raw` | `post_process_node` | 动态开/关原始相机0图像发布（热切换）/ Hot-switch raw camera0 image publish |
> | `send_cloud_render` | `post_process_node` | 动态开/关点云渲染图像发布（热切换）/ Hot-switch cloud render publish |
> | `send_undistort` | `post_process_node` | 动态开/关去畸变图像发布（热切换）/ Hot-switch undistort publish |
>
> **其余未列出的参数均为启动期参数**——`ros2 param set` 会成功写入参数服务器，但**不会回流到节点逻辑**。如果要改它们，请通过 `launch/odin1Lite.launch.py`（ROS2）或 `launch/odin1Lite.launch`（ROS1）的 `key:=value` 透传，或修改 launch 文件的默认值后重启节点。
>
> **All other parameters are launch-time only** — `ros2 param set` will update the parameter server but **will not flow into the node logic**. To change them, override via `launch/odin1Lite.launch.py` (ROS2) or `launch/odin1Lite.launch` (ROS1) using `key:=value`, or edit the launch defaults and restart.

> **命名约定 / Naming convention**：launch arg 名与节点参数名始终保持一致（不加前缀）。绝大多数参数只属于某一个节点，无歧义。**唯一跨节点同名的参数是 `sync_queue_size`**（`pointcloud_reprojector_node` + `pointcloud_to_depth_node`），在 launch 中只声明一次，同时下发给两个节点；如需分别设置，用**节点全路径**覆盖：`ros2 param set /pointcloud_to_depth sync_queue_size 20`。
>
> Launch arg names are kept identical to node parameter names (no prefixes). Almost all parameters belong to a single node and are unambiguous. **The only cross-node duplicate is `sync_queue_size`** (`pointcloud_reprojector_node` + `pointcloud_to_depth_node`); it is declared once in the launch and fans out to both nodes. To set them independently, use the **full node path**: `ros2 param set /pointcloud_to_depth sync_queue_size 20`.

---

## 主驱动节点 odin_ros_driver_node — 启动期参数 / Launch-time parameters

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `topic_prefix` | string | `manifold` | 话题前缀根（最终前缀 `/{topic_prefix}/device{N}/`，留空去掉根段）/ Topic prefix root |
| `device_mode` | string | `normal` | 工作模式 / Device mode: `normal` \| `standby` |
| `discovery_timeout_ms` | int | `2000` | SDK 设备发现超时（ms）/ SDK device-discovery timeout |
| `image_width` | int | `1280` | 图像宽度，`0`=设备默认 / Image width, 0 = device default |
| `image_height` | int | `1088` | 图像高度，`0`=设备默认 / Image height, 0 = device default |
| `image_fps` | int | `30` | 图像帧率，`0`=设备默认 / Image FPS, 0 = device default |
| `image_format` | string | `mjpeg` | 图像格式 / Image format: `mjpeg` \| `h264` |
| `raw_frame_id` | string | `lidar` | 原始点云 frame_id / Frame id for raw cloud |
| `slam_frame_id` | string | `odom` | SLAM 点云 frame_id / Frame id for SLAM cloud |
| `image_frame_id` | string | `camera0` | 左相机图像 frame_id / Frame id for left image |
| `imu_frame_id` | string | `imu` | IMU frame_id |
| `odom_frame_id` | string | `odom` | 里程计 frame_id（odom→base_link 的父帧）/ Odometry parent frame |
| `enable_row_filter` | bool | `false` | 启用行滤波：将行号低于 `min_row` 的点置零（分辨率 256×192，行号 0~191）/ Enable row filter: zero out points whose row index < `min_row` (256×192 resolution, row 0~191) |
| `min_row` | int | `0` | 最小行号（`0..191`），低于此行的点被置零 / Minimum row index, points with row < `min_row` are zeroed |

启动期参数命令行设置示例 / Setting launch-time parameters from CLI:

```bash
# ROS2: 通过 launch 透传 / via launch
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py \
  topic_prefix:=mainifold image_width:=640 image_height:=544 image_fps:=30 \
  raw_frame_id:=lidar imu_frame_id:=imu_link tf_publish_rate:=50

# ROS2: 列出全部 launch arg / list every launch arg
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py --show-args

# ROS2: 直接 ros2 run（不通过 launch）/ direct ros2 run
ros2 run odin_ros_driver_rev1 odin_ros_driver_node \
  --ros-args -p topic_prefix:=mainifold -p image_fps:=30

# ROS1: 通过 launch 透传 / via roslaunch
roslaunch odin_ros_driver_rev1 odin1Lite.launch \
  topic_prefix:=mainifold image_width:=640 image_height:=544 image_fps:=30 \
  raw_frame_id:=lidar imu_frame_id:=imu_link tf_publish_rate:=50
```

## 主驱动节点 odin_ros_driver_node — 运行期参数 / Runtime parameters

下列三个参数注册了**动态回调**，运行期间修改会立即下发到 SDK 或日志系统。

The three parameters below register a **dynamic callback** and take effect immediately when changed at runtime.

| 参数 / Parameter | 类型 / Type | 范围 / Range | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- | --- |
| `confidence_threshold` | int | `0..255` | `7` | 点云置信度阈值（低于该值的点会被丢弃）/ Point-cloud confidence threshold (points below are dropped) |
| `sensor_mode` | int | `0..3` | `0` | dToF 传感器模式 / dToF Lidar sensor mode: `0`=HDR `1`=High Peak `2`=Low Peak `3`=Adaptive |
| `log_level` | int | `0..3` | — | 日志级别 / Log level: `0`=ERROR `1`=INFO `2`=WARN `3`=DEBUG |

ROS2 运行期改参示例 / ROS2 runtime change:

```bash
# 列出所有参数 / List all parameters
ros2 param list /odin_ros_driver_node

# 读取 / Get
ros2 param get /odin_ros_driver_node sensor_mode

# 设置 / Set （node 名字以实际启动名字为准）
ros2 param set /odin_ros_driver_node sensor_mode 3
ros2 param set /odin_ros_driver_node confidence_threshold 15
ros2 param set /odin_ros_driver_node log_level 1
```

ROS1 运行期改参示例 / ROS1 runtime change:

```bash
# ROS1 通过 ddynamic_reconfigure 暴露，可用 rqt_reconfigure 或命令行：
# Exposed through ddynamic_reconfigure; use rqt_reconfigure or the CLI:
rosrun rqt_reconfigure rqt_reconfigure
# 或 / or
rosrun dynamic_reconfigure dynparam set /odin_ros_driver_node sensor_mode 3
rosrun dynamic_reconfigure dynparam set /odin_ros_driver_node confidence_threshold 15
```

## 后处理 / 重投影 / 点云转深度图 节点参数

下表中的参数名即 launch arg 名，也即节点参数名（完全一致，不加前缀）。

In the tables below the parameter name is also the launch-arg name and the node parameter name (no prefixes).

### `pointcloud_reprojector_node` 参数 / parameters

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `topic_prefix` | string | `manifold` | 与主驱动一致 / Same root as the main driver |
| `mode` | string | `raw` | 重投影模式 / Mode: `raw` \| `slam` |
| `min_depth` | double | `0.1` | 可视化最小深度（米）/ Visualization min depth (m) |
| `max_depth` | double | `50.0` | 可视化最大深度（米）/ Visualization max depth (m) |
| `point_size` | int | `2` | 叠加点像素半径 / Overlay point pixel radius |
| `use_depth_color` | bool | `true` | 是否按深度上色（false=单色）/ Color by depth or use single color |
| `sync_queue_size` [1] | int | `10` | 时间同步队列长度（与 `pointcloud_to_depth_node` 共用一个 launch arg）/ Time-sync queue size (shares one launch arg with `pointcloud_to_depth_node`) |
| `sync_slop` | double | `0.1` | 时间同步最大偏差（s）/ Max time-sync slop (s) |

### `pointcloud_to_depth_node` 参数 / parameters

参数取值与说明见 [topics.md — pointcloud_to_depth_node](topics.md#pointcloud_to_depth_node点云转深度图节点--point-cloud--depth) 章节。可调参数：`scale`、`z_min`、`z_max`、`dilate_radius`、`use_z_buffer`、`edge_grad_threshold`、`vis_depth_min`、`vis_depth_max`、`sync_queue_size` [1]、`sync_slop_sec`。

> **[1]** `sync_queue_size` 为 `pointcloud_reprojector_node` 与 `pointcloud_to_depth_node` 共用的 launch arg；从 launch 设置时会同时生效于两个节点。需分别设置请走全路径：`ros2 param set /pointcloud_to_depth sync_queue_size 20`。
>
> **[1]** `sync_queue_size` is shared between `pointcloud_reprojector_node` and `pointcloud_to_depth_node`; setting it via the launch arg applies to both nodes. To set them independently, use the full path: `ros2 param set /pointcloud_to_depth sync_queue_size 20`.

### `depth_to_pointcloud_node` 参数 / parameters

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `step` | int | `4` | 像素采样步长（越小越稠密）/ Pixel sampling stride (smaller = denser) |
| `d_min` | double | `0.1` | 有效深度下限（米）/ Valid depth lower bound (m) |
| `d_max` | double | `100.0` | 有效深度上限（米）/ Valid depth upper bound (m) |

### 命令行示例 / CLI examples

```bash
# 通过 launch 一次性覆盖（推荐）/ Override via launch (recommended)
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py \
  mode:=slam max_depth:=20.0 \
  scale:=5.0 use_z_buffer:=true \
  step:=3

# 单独 ros2 run 子节点 / Run a single sub-node directly
ros2 run odin_ros_driver_rev1 pointcloud_to_depth_node \
  --ros-args -p scale:=5.0 -p z_max:=20.0 -p use_z_buffer:=true

# 运行时查询 / Inspect at runtime
ros2 param list /pointcloud_reprojector
ros2 param get  /pointcloud_to_depth scale

# 共享参数分别设置（全路径）/ Disambiguate a shared param via full path
ros2 param set /pointcloud_reprojector sync_queue_size 30
ros2 param set /pointcloud_to_depth   sync_queue_size 15
```

> **注意 / Note**：`pointcloud_reprojector_node`、`pointcloud_to_depth_node`、`depth_to_pointcloud_node` 三个子节点的参数均**只在节点启动时读取一次**，没有动态回调。运行期 `ros2 param set` 会修改参数服务器中的值，但**不会重新进入计算逻辑**，需要重启节点才会生效。
>
> All three sub-nodes (`pointcloud_reprojector_node`, `pointcloud_to_depth_node`, `depth_to_pointcloud_node`) read their parameters **only once at startup**; there is no dynamic callback. A runtime `ros2 param set` will update the parameter server but **will not flow into the node logic** — restart the node to apply changes.

---

默认值以 ROS2 的 `driver.launch.py` 为准。默认仅启动驱动进程。
样例中的 ROS2 型号入口使用同一组默认值。`tf_publish_rate=0` 关闭定时重发，设备 TF 继续发布。

## 数据流开关（launch 参数）/ Stream toggles (launch params)

所有数据流开关（A/B/C/D/E 层）均已从 `config/control_command.yaml` 迁移为 launch 参数。A/B 层支持通过 `rqt` 动态配置和 ROS service 热切换；C/D/E 层为启动期参数（不支持热切换）。

All data stream toggles (layers A/B/C/D/E) have been migrated from `config/control_command.yaml` to launch parameters. Layers A/B support `rqt` dynamic reconfigure and ROS service hot-switch; layers C/D/E are launch-time only (not hot-switchable).

### A 层：SDK 通道开关 / SDK channel switches (`odin_ros_driver_node`)

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `enable_raw_point` | bool | `true` | 原始点云通道 / Raw point cloud channel |
| `enable_slam_point` | bool | `true` | SLAM 点云通道 / SLAM point cloud channel |
| `enable_image0` | bool | `true` | 左相机图像通道 / Left camera image channel |
| `enable_image1` | bool | `false` | L0 只读兼容字段，启用请求会被拒绝 |
| `enable_imu` | bool | `true` | IMU 数据通道 / IMU data channel |
| `enable_odom_lf` | bool | `true` | 低频里程计通道 / Low-frequency odometry channel |
| `enable_odom_hf` | bool | `true` | 高频里程计通道 / High-frequency odometry channel |
| `enable_tf` | bool | `true` | TF 变换通道 / TF transform channel |

### B 层：后处理开关 / Post-process switches (`post_process_node`)

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `post_process` | bool | `false` | 后处理总开关（静态，非热切换）/ Master switch (static, not hot-switchable) |
| `send_image_raw` | bool | `false` | 发布原始相机0图像 / Publish raw camera0 image |
| `send_image2_raw` | bool | `false` | L0 只读兼容字段，启用请求会被拒绝 |
| `send_cloud_render` | bool | `false` | 发布点云渲染图像 / Publish cloud-rendered image |
| `send_undistort` | bool | `false` | 发布去畸变图像 / Publish undistorted images |
| `send_rectify` | bool | `false` | L0 只读兼容字段，启用请求会被拒绝 |
| `undistort_mapping_only` | bool | `false` | 启动期参数。为 true 时根据 `mapping_active` 启停 camera0 去畸变 |

### OpenCV 去畸变节点 / OpenCV undistortion node (`opencv_undistort_node`)

| 参数 / Parameter | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- |
| `send_opencv_undistort` | bool | `false` | 启用 OpenCV 去畸变节点（静态，非热切换）/ Enable OpenCV undistortion node (static, not hot-switchable) |
| `topic_prefix` | str | `manifold` | 话题前缀 / Topic prefix |

启用后发布以下话题 / When enabled, publishes:

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `camera0/opencv_undistort` | `sensor_msgs/Image` | OpenCV `cv::fisheye` 去畸变图像 / OpenCV fisheye undistorted image |
| `camera0/camera_info` | `sensor_msgs/CameraInfo` | KB equidistant 模型 CameraInfo / KB equidistant model CameraInfo |

### C/D/E 层：启动期参数（非热切换）/ Launch-time parameters (not hot-switchable)

C/D/E 层参数通过 launch 文件配置，启动时一次性读取，不支持热切换。如需修改请通过 launch arg 覆盖或重启节点。

Layers C/D/E are configured via launch args, read once at startup, not hot-switchable. Override via launch args or restart the node to apply changes.

| 参数 / Parameter | 节点 / Node | 类型 / Type | 默认值 / Default | 说明 / Description |
| --- | --- | --- | --- | --- |
| `enable_reprojection` | `pointcloud_reprojector_node` | bool | `false` | 启用重投影节点 / Enable reprojector node |
| `enable_pointcloud_to_depth` | `pointcloud_to_depth_node` | bool | `false` | 启用点云转深度图节点 / Enable pointcloud-to-depth node |
| `enable_depth_to_pointcloud` | `depth_to_pointcloud_node` | bool | `false` | 启用深度图转点云节点 / Enable depth-to-pointcloud node |

> **中文**：设备发现、序列号、网卡选择、UDP 端口等都由 SDK 内部自动处理，无需也无法在 launch 中配置。节点会枚举所有发现到的设备并为每台单独建立 topic 前缀（多设备热插拔）。
>
> **English**: Device discovery, serial number, NIC selection and UDP ports are all handled internally by the SDK — they are neither required nor configurable from the launch file. The node enumerates every discovered device and assigns a per-device topic prefix (multi-device hot-plug).
