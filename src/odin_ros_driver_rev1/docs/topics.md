# 发布话题 / Published Topics

> [← 返回 README](../README.md)

**中文**：所有数据话题均带有逐设备前缀 `{prefix} = /{topic_prefix}/device{N}/`（默认 `/manifold/device0/`），用于多设备热插拔区分。`topic_prefix` 由 launch 参数 `topic_prefix` 配置（默认 `manifold`），`{N}` 为枚举到的设备序号。**设备型号（model）不进入话题路径**——型号与 SN 由 `/{topic_prefix}/driver/device_online` 的 JSON 字段（`sn`/`model`/`prefix`/`ip`）告知。下表中的话题列均省略前缀，只列后缀。

**English**: All data topics carry a per-device prefix `{prefix} = /{topic_prefix}/device{N}/` (default `/manifold/device0/`) so that multiple devices can coexist. `topic_prefix` is configurable via the launch parameter `topic_prefix` (default `manifold`); `{N}` is the enumerated device index. The device **model never appears in topic paths** — the model and SN are announced via the `sn`/`model`/`prefix`/`ip` fields of the JSON on `/{topic_prefix}/driver/device_online`. The tables below list only the suffix.

> **⚠ 提示 / Tip**：话题路径只与 `topic_prefix` 和设备序号 `{N}` 有关，与设备型号无关——换用不同型号设备时话题/服务路径保持不变。型号（大小写取决于设备固件上报，驱动不做转换）通过 `/{topic_prefix}/driver/device_online` 的 JSON `model` 字段获知；启动日志中也会打印 `Topic prefix: ...`。可用 `ros2 topic list | grep cloud` 或 `ros2 service list` 验证。
>
> Topic paths depend only on `topic_prefix` and the device index `{N}` — never on the device model, so switching between models keeps topic/service paths stable. The model (case-sensitive as reported by firmware; the driver does **not** lowercase it) is announced via the `model` field of the `/{topic_prefix}/driver/device_online` JSON; the startup log also prints `Topic prefix: ...`. Verify with `ros2 topic list | grep cloud` or `ros2 service list`.

## odin_ros_driver_node（主驱动节点 / main driver node）

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `cloud/raw` | `sensor_msgs/PointCloud2` | 原始点云（256×192）/ Raw point cloud (256×192) |
| `cloud/slam` | `sensor_msgs/PointCloud2` | SLAM 点云 / SLAM point cloud |
| `camera0/compressed` | `sensor_msgs/CompressedImage` | 左相机 JPEG 压缩图像 / Left camera JPEG compressed image |
| `imu` | `sensor_msgs/Imu` | IMU 数据（加速度、角速度）/ IMU data (acceleration, angular velocity) |
| `odometry_lf` | `nav_msgs/Odometry` | 低频里程计数据（含设备上报 6×6 位姿协方差）/ Low-frequency odometry (with device-reported 6x6 pose covariance) |
| `odometry_hf` | `nav_msgs/Odometry` | 高频里程计（含位姿协方差）/ High-frequency odometry (with pose covariance) |
| `mapping_active` | `std_msgs/Bool` | SLAM/ReMap 为 true。Reliable / Transient Local / Keep Last 1 |

此外，主驱动还会在驱动级命名空间 `/{topic_prefix}/driver/` 下发布以下话题（不带设备前缀，所有设备共享）：

The main driver also publishes the following topics under the driver-level namespace `/{topic_prefix}/driver/` (not per-device, shared across devices):

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `/{topic_prefix}/driver/device_online` | `std_msgs/String` | 设备上线事件（latched，JSON 含 SN/型号/prefix/IP）/ Device-online event (latched, JSON with SN/model/prefix/IP) |
| `/{topic_prefix}/driver/device_offline` | `std_msgs/String` | 设备下线事件（latched，JSON 含 SN/型号）/ Device-offline event (latched, JSON with SN/model) |
| `/{topic_prefix}/driver/resolution_change` | `std_msgs/String` | 分辨率变更事件（latched）/ Resolution change event (latched) |

## 点云传输压缩 / Point Cloud Transport Compression

**中文**：`cloud/raw` 和 `cloud/slam` 话题使用 `point_cloud_transport` 发布。base 话题（`cloud/raw`、`cloud/slam`）始终是标准 `sensor_msgs/PointCloud2`，QoS 为 **RELIABLE + VOLATILE**（与普通发布器一致），**无需任何额外配置即可用 RViz / `ros2 topic echo` 正常订阅**。

**English**: The `cloud/raw` and `cloud/slam` topics are published via `point_cloud_transport`. The base topics (`cloud/raw`, `cloud/slam`) are always plain `sensor_msgs/PointCloud2` with **RELIABLE + VOLATILE** QoS (same as a normal publisher), so they can be subscribed to normally in RViz / `ros2 topic echo` **without any extra setup**.

> **⚠ 重要 / Important**：`point_cloud_transport` **没有 RViz 显示插件**！这一点与 `image_transport` 完全不同。RViz 原生的 PointCloud2 显示**不会**出现 "Transport Hint" 下拉框——图像有该选项是因为 `image_transport` 被原生集成进了 RViz，而点云没有。因此 RViz 只能订阅未压缩的 base 话题；要在 RViz 中查看压缩点云，必须先用 `republish` 节点解压（见下文）。
>
> `point_cloud_transport` has **no RViz display plugin** — unlike `image_transport`. RViz's native PointCloud2 display will **not** show a "Transport Hint" dropdown (images have it only because `image_transport` is natively integrated into RViz). So RViz can only subscribe to the uncompressed base topic; to view compressed clouds in RViz you must decompress them first with a `republish` node (see below).

### 压缩话题命名 / Compressed topic naming

安装压缩插件后，驱动会**额外**发布压缩子话题（base 话题依然是未压缩的 `PointCloud2`）：

After installing compression plugins, the driver **additionally** publishes compressed sub-topics (the base topic remains an uncompressed `PointCloud2`):

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `{prefix}cloud/raw` | `sensor_msgs/PointCloud2` | 未压缩 base 话题（RViz 直接订阅）/ uncompressed base topic (subscribe directly in RViz) |
| `{prefix}cloud/raw/zlib` | `point_cloud_interfaces/CompressedPointCloud2` | zlib 压缩 / zlib compressed |
| `{prefix}cloud/raw/zstd` | `point_cloud_interfaces/CompressedPointCloud2` | zstd 压缩 / zstd compressed |
| `{prefix}cloud/raw/draco` | `point_cloud_interfaces/CompressedPointCloud2` | draco 压缩 / draco compressed |

### 安装压缩插件 / Install compression plugins

```bash
# ROS2 Humble（将 humble 替换为对应版本）
# ROS2 Humble (replace 'humble' with your distro)
sudo apt install ros-humble-point-cloud-transport-plugins        # 元包，包含 zlib + zstd + draco

# ROS1 Noetic
sudo apt install ros-noetic-point-cloud-transport-plugins        # 元包
```

> **说明 / Note**：压缩插件是**驱动端进程**加载的（`point_cloud_transport` 用 pluginlib 在运行时扫描已安装插件）。因此插件必须装在**运行驱动的机器**上，装好后**重启驱动**才会出现压缩子话题。如果只想传输未压缩点云，可以不装任何插件。
>
> The compression plugins are loaded by the **driver process** (`point_cloud_transport` scans installed plugins at runtime via pluginlib). So plugins must be installed on the **machine running the driver**, and the **driver must be restarted** for the compressed sub-topics to appear. If you only need uncompressed clouds, no plugins are required.

### 在 RViz 中查看压缩点云（需要 republish 解压）/ View compressed cloud in RViz (needs republish)

由于 RViz 无法直接解码压缩点云，需在**订阅侧机器**跑一个 `republish` 节点，把压缩话题解压回普通 `PointCloud2`，再让 RViz 订阅解压后的话题：

Since RViz cannot decode compressed clouds directly, run a `republish` node on the **subscriber machine** to decompress back to a plain `PointCloud2`, then point RViz at the decompressed topic:

```bash
# ROS2：把 zstd 压缩话题解压到 /viz/cloud
# ROS2: decompress the zstd topic to /viz/cloud
ros2 run point_cloud_transport republish \
  --ros-args \
  -p in_transport:=zstd \
  -r in/zstd:=/manifold/device0/cloud/raw/zstd \
  -p out_transport:=raw \
  -r out:=/viz/cloud

# 然后在 RViz 中订阅 /viz/cloud（普通 PointCloud2）
# Then subscribe to /viz/cloud in RViz (plain PointCloud2)
```

### 代码中直接订阅压缩点云 / Subscribe to compressed cloud in code (C++)

在自己的节点里可用 `point_cloud_transport` 的 subscriber，通过 transport hint 自动解压，无需 republish：

In your own node you can use a `point_cloud_transport` subscriber that decompresses automatically via the transport hint (no republish needed):

```cpp
// ROS2
#include <point_cloud_transport/point_cloud_transport.hpp>
point_cloud_transport::PointCloudTransport pct(node);
point_cloud_transport::Subscriber sub = pct.subscribe(
    "/manifold/device0/cloud/raw", 10,   // base topic, NOT the /zstd sub-topic
    &MyClass::cloudCallback, this,
    "zstd");  // transport hint: "raw" | "zlib" | "zstd" | "draco"
```

```cpp
// ROS1
#include <point_cloud_transport/point_cloud_transport.h>
point_cloud_transport::PointCloudTransport pct(nh);
point_cloud_transport::Subscriber sub = pct.subscribe(
    "/manifold/device0/cloud/raw", 10,
    &MyClass::cloudCallback, this,
    "zstd");
```

### 用 rosbag 录制压缩点云 / Record compressed cloud with rosbag

直接录制压缩子话题即可显著减小 bag 体积（回放后同样需要 republish 才能在 RViz 里看）：

Record the compressed sub-topic directly to shrink the bag significantly (on playback you still need republish to view it in RViz):

```bash
# 只录 zstd 压缩点云 / Record only the zstd compressed cloud
ros2 bag record /manifold/device0/cloud/raw/zstd

# 查看当前节点已加载的传输格式 / List loaded transports
ros2 run point_cloud_transport list_transports
```

### 可用插件 / Available plugins

| 插件 / Plugin | 包名 / Package | 特点 / Features |
| --- | --- | --- |
| `raw` | 内置 / built-in | 无压缩，base 话题默认 / No compression, base topic default |
| `zlib` | `zlib_point_cloud_transport` | 通用无损压缩，兼容性好 / General-purpose lossless, good compatibility |
| `zstd` | `zstd_point_cloud_transport` | 高压缩比无损压缩，速度较快 / High-ratio lossless, fast |
| `draco` | `draco_point_cloud_transport` | Google Draco 有损压缩，压缩比最高 / Google Draco lossy, highest ratio |

> **⚠ 小结 / Summary**：
> - base 话题 `cloud/raw` / `cloud/slam` 始终是标准 `PointCloud2`，RViz 直接订阅即可（QoS 为 RELIABLE，无需改成 Best Effort）。
> - 压缩子话题仅在**驱动端机器装了插件并重启驱动后**才出现。
> - RViz **无法**直接显示压缩点云（没有点云版 Transport Hint），必须先 `republish` 解压。
> - 真正省带宽/省磁盘的场景：跨网络传输时用自己的 PCT subscriber，或 rosbag 直接录压缩子话题。
>
> - The base topics `cloud/raw` / `cloud/slam` are always plain `PointCloud2`; RViz subscribes directly (QoS is RELIABLE, no need to switch to Best Effort).
> - Compressed sub-topics only appear **after plugins are installed on the driver machine and the driver is restarted**.
> - RViz **cannot** display compressed clouds directly (there is no point-cloud Transport Hint); you must `republish` to decompress first.
> - Real bandwidth/disk savings: use your own PCT subscriber for cross-network transport, or record the compressed sub-topic directly with rosbag.

## post_process_node（后处理节点 / post-processing node）

整机按 `mapping_active` 发布 `camera0/undistort`，其帧输出独立于 `send_image_raw`。

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `camera0/raw` | `sensor_msgs/Image` | 左相机解码后 RGB 图像 / Left camera decoded RGB image |
| `camera0/undistort` | `sensor_msgs/Image` | 左相机去畸变图像 / Left camera undistorted image |
| `cloud/render` | `sensor_msgs/PointCloud2` | 着色点云（点云+图像融合）Lidar坐标系 / Colored point cloud (cloud + image fusion) Lidar Frame|

## 回环和重定位

| 话题后缀 | 发布时机 | ROS2 QoS |
| --- | --- | --- |
| `loop_closure/optimization_busy`、`loop_closure/optimization_status_marker` | 算法状态更新 | Reliable / Transient Local / Keep Last 1 |
| `loop_closure/optimized_path`、`loop_closure/optimized_cloud` | 优化完成 | Reliable / Transient Local / Keep Last 1 |
| `loop_closure/map_cloud` | 加载地图后 | Reliable / Transient Local / Keep Last 1 |
| `loop_closure/corrected_pose`、`loop_closure/corrected_cloud` | 矫正位姿及有点云的关键帧 | Reliable / Volatile / Keep Last 100 |

设备 map 相关 TF 通过 `/tf_static` 发布。其他设备变换走 `/tf`。
整机的 `imu → base_link` 安装变换由规划桥用 URDF 和设备外参计算。
L0 的 raw 点云正常完整帧为 49,152 点，`height=1`、`point_step=18`。
点的顺序及无效点占位保留，xyz 单位为米，并携带 intensity、confidence 和 offset_time。

## pointcloud_reprojector_node（点云重投影节点 / point-cloud reprojector）

**中文**：把稀疏 dToF 点云重投影到 `camera0` 图像平面，输出叠加可视化图，用于标定校验。支持两种模式：`raw`（`cloud/raw` + 静态外参）和 `slam`（`cloud/slam` + 实时 odom，世界坐标 → 相机坐标）。源码位于 `src/pointcloud_reprojector/`。

**English**: Reprojects the sparse dToF point cloud onto `camera0` to produce a visualization overlay, useful for calibration verification. Two modes: `raw` (`cloud/raw` + static extrinsic) and `slam` (`cloud/slam` + live odometry, world → camera). Source under `src/pointcloud_reprojector/`.

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `reprojection/overlay` | `sensor_msgs/Image` (bgr8) | 点云叠加在相机图上 / cloud overlaid on the camera image |

主要参数 / Key parameters: `mode` (`raw`/`slam`), `topic_prefix`, `min_depth`, `max_depth`, `point_size`, `cloud_topic_suffix`, `image_topic_suffix`, `odom_topic_suffix`。

## pointcloud_to_depth_node（点云转深度图节点 / point-cloud → depth）

**中文**：基于纯 C 风格的投影核心 (`pointcloud_to_depth_node.hpp`)，把同步好的 `cloud/raw` 与 `camera0/undistort` 投影到 `1/scale` 分辨率的低分画布上，再经膨胀、Sobel 边缘抑制、最近邻上采样得到与相机同分辨率的稠密深度图。**支持多设备**：每个 SN 一个 `DeviceWorker`，通过 `device_online` / `device_offline` 动态生灭，`resolution_change` 按 SN 分发。源码位于 `src/pointcloud_to_depth/`，详见同目录下 `README.md`。

**English**: A library-free point-cloud → depth projection node. Synchronizes `cloud/raw` with `camera0/undistort`, projects onto a `1/scale` low-resolution canvas, then dilates / Sobel-edge-suppresses / nearest-neighbor-upsamples to a full-resolution dense depth map. **Multi-device aware**: one `DeviceWorker` per SN, spawned / torn down on `device_online` / `device_offline`; `resolution_change` is dispatched by SN. The plain-types projection core (`pointcloud_to_depth_node.hpp`) is PCL/Eigen/ROS-free so it can be lifted into the on-device SDK. Source under `src/pointcloud_to_depth/`; see the in-folder `README.md` for the algorithm details.

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `pointcloud2depth/depth` | `sensor_msgs/Image` (32FC1) | 稠密深度图，米，0 表示无效像素 / dense depth (meters), 0 = invalid |
| `pointcloud2depth/depth_color` | `sensor_msgs/Image` (bgr8) | JET 配色可视化（无效像素为黑）/ JET-colored depth (black where invalid) |
| `pointcloud2depth/depth_overlay` | `sensor_msgs/Image` (bgr8) | JET 叠加在相机图上 / JET overlay on the RGB image |

订阅 / Subscribes: `{prefix}cloud/raw`, `{prefix}camera0/undistort`, `/{topic_prefix}/driver/device_online`, `/{topic_prefix}/driver/device_offline`, `/{topic_prefix}/driver/resolution_change`。
依赖服务 / Service: `{prefix}get_calibration`（首次上线时拉取相机内外参 / fetched once per device on first online event）。

主要参数 / Key parameters:

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | --- | --- |
| `scale` | `7.0` | 投影画布下采样比 / projection canvas downscale factor |
| `z_min` / `z_max` | `0.1` / `50.0` | 有效深度范围（米）/ valid depth range (meters) |
| `dilate_radius` | `1` | 投影写入时的 3×3 膨胀 / dilation radius when writing projected pixels |
| `use_z_buffer` | `false` | 是否启用 Z-buffer 防止远点覆盖近点 / enable z-buffer to prevent far-over-near overwrite |
| `edge_grad_threshold` | `0.75` | Sobel 边缘抑制阈值（米/像素）/ Sobel edge suppression threshold (m/pixel) |
| `vis_depth_min` / `vis_depth_max` | `0.1` / `5.0` | JET 配色固定归一化范围；`<0` 时改为逐帧自动归一化 / fixed JET normalization range; set min `<0` for per-frame auto-norm |
| `sync_queue_size` / `sync_slop_sec` | `10` / `0.1` | message_filters 时间同步参数 / time-sync queue size & slop |
