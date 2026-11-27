# Odin ROS Driver

**中文**：基于 Odin SDK 的 ROS 驱动程序，支持 ROS1 和 ROS2 多版本。

**English**: ROS driver built on top of the Odin SDK, supporting both ROS1 and ROS2.

## 目录 / Table of Contents

| 文档 / Document | 说明 / Description |
| --- | --- |
| [发布话题](docs/topics.md) | 所有节点的话题列表、点云压缩传输说明 / Topic list, point-cloud transport compression |
| [服务](docs/services.md) | ROS 服务接口、诊断监控、数据流开关服务 / ROS services, diagnostics, stream toggle services |
| [地图与设备模式服务](docs/map_and_mode_services.md) | save_map / load_map / device_mode 三个服务专题说明 / save_map / load_map / device_mode service reference |
| [参数](docs/parameters.md) | 完整参数列表、动态回调白名单、数据流开关 / Full parameter list, dynamic callback whitelist, stream toggles |
| [硬解码验证](docs/hardware_decode.md) | Rockchip / Jetson 硬件 JPEG 解码验证方法 / Hardware JPEG decode verification (Rockchip / Jetson) |
| [常见问题](docs/troubleshooting.md) | FAQ、GDB 调试、DHCP、TF、cv_bridge、编译内存排障 / FAQ, GDB debugging, DHCP, TF, cv_bridge, build memory troubleshooting |

## 功能特性 / Features

- **多平台支持 / Multi-platform**: ROS1 Noetic、ROS2 Humble/Iron/Jazzy
- **热插拔检测 / Hot-plug**: 自动发现和连接设备，支持设备断开重连 / Auto-discovery and reconnection of devices
- **心跳监控 / Heartbeat**: 实时监测设备连接状态 / Real-time device connection monitoring
- **多数据流 / Multi-stream**: 点云、图像、IMU、里程计同步输出 / Synchronized point cloud, image, IMU and odometry output
- **后处理 / Post-processing**: 图像去畸变、点云着色 / Image undistortion, cloud colorization

## 功能模块 / Nodes

| 节点 / Node | 说明 / Description |
| --- | --- |
| `odin_ros_driver_node` | 主驱动节点：连接设备、发布原始数据、提供标定服务 / Main driver: device connection, raw data publishing, calibration service |
| `post_process_node` | 后处理节点：图像去畸变、点云着色 / Post-processing: undistortion, cloud colorization |
| `opencv_undistort_node` | OpenCV 去畸变节点：FishPoly→KB 转换 + `cv::fisheye` 去畸变 + 发布标准 CameraInfo / OpenCV undistortion: FishPoly→KB conversion + `cv::fisheye` undistort + standard CameraInfo |
| `pointcloud_reprojector_node` | 点云重投影节点：稀疏点云重投影到 `camera0` 输出叠加可视化图 / Reprojects sparse point cloud onto `camera0` to produce an overlay visualization image |
| `pointcloud_to_depth_node` | 点云转深度图节点（多设备）：投影 + 膖胀 + Sobel 边缘抑制 + 最近邻上采样 / Multi-device point-cloud → depth: projection + dilation + Sobel edge suppression + NN upsample |
| `diag_subscriber_node` | 订阅端诊断节点：C++ 订阅驱动话题，测量订阅端帧率与延迟（比 `ros2 topic hz` 准确）/ Subscriber-side diagnostic: C++ subscriber measuring FPS & latency (more accurate than `ros2 topic hz`) |

## 系统要求 / System Requirements

| Ubuntu 版本 / Version | ROS 版本 / ROS Version |
| --- | --- |
| 20.04 | ROS1 Noetic, ROS2 Foxy |
| 22.04 | ROS2 Humble, ROS2 Iron |
| 24.04 | ROS2 Jazzy |

## 依赖安装 / Dependency Installation

### 编译工具（必须）/ Build tools (required)

```bash
sudo apt install build-essential cmake git
```

### ROS1 (Noetic)

```bash
# ROS1 基础 / ROS1 base
sudo apt install ros-noetic-desktop-full

# ROS1 功能包依赖 / ROS1 package dependencies
sudo apt install ros-noetic-ddynamic-reconfigure
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install ros-noetic-pcl-ros ros-noetic-stereo-msgs
sudo apt install ros-noetic-point-cloud-transport

# 系统库依赖 / System library dependencies
sudo apt install libyaml-cpp-dev libopencv-dev libpcl-dev

# 可选但推荐：libyuv 加速 YUV→BGR 色彩转换（H.264 硬解 / NV12 图像路径提速，见文末说明）
# Optional but recommended: libyuv speeds up YUV→BGR colour conversion (H.264 HW-decode / NV12 path)
sudo apt install libyuv-dev
```

### ROS2 (Humble/Iron/Jazzy)

```bash
# ROS2 基础（选择对应版本）/ ROS2 base (pick one for your Ubuntu version)
sudo apt install ros-humble-desktop  # Ubuntu 22.04
# 或 / or
sudo apt install ros-iron-desktop    # Ubuntu 22.04
# 或 / or
sudo apt install ros-jazzy-desktop   # Ubuntu 24.04

# ROS2 功能包依赖（将 humble 替换为对应版本：humble/iron/jazzy）
# ROS2 package dependencies (replace 'humble' with your distro: humble/iron/jazzy)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-pcl-ros ros-humble-stereo-msgs
sudo apt install ros-humble-point-cloud-transport

# 系统库依赖 / System library dependencies
sudo apt install libyaml-cpp-dev libopencv-dev libpcl-dev

# 可选但推荐：libyuv 加速 YUV→BGR 色彩转换（H.264 硬解 / NV12 图像路径提速）
# Optional but recommended: libyuv speeds up YUV→BGR colour conversion (H.264 HW-decode / NV12 path)
sudo apt install libyuv-dev
```

> **libyuv 说明 / Note on libyuv**：安装 `libyuv-dev` 后 CMake 编译时自动检测并启用快速色彩转换；运行期可用 `ODIN_DISABLE_LIBYUV=1` 强制回退到 libswscale/OpenCV 路径
> **With `libyuv-dev` installed**, CMake enables the fast conversion automatically; set `ODIN_DISABLE_LIBYUV=1` at runtime to fall back to the libswscale/OpenCV path

## SDK 获取 / SDK Acquisition

**中文**：本驱动**强依赖** Odin SDK，源码以 git submodule 形式存放在 `module/sdk_api/`。CMake 在 `@CMakeLists.txt:43-54` 直接引用以下路径：

- `module/sdk_api/include`
- `module/sdk_api/sdk`
- `module/sdk_api/build/sdk/libodin_sdk.a`（编译产物）

如果 `module/sdk_api/` 是空目录，`colcon build` / `catkin build` 会因为找不到头文件或链接库而失败。下面给出**在线**和**离线**两种获取方式。

**English**: This driver has a **hard dependency** on the Odin SDK, kept as a git submodule under `module/sdk_api/`. `CMakeLists.txt` (lines 43–54) references `module/sdk_api/include`, `module/sdk_api/sdk` and `module/sdk_api/build/sdk/libodin_sdk.a` directly. If `module/sdk_api/` is empty, the build will fail with "header not found" or "cannot find -lodin_sdk". Two ways to obtain the SDK:

### 方式 A：在线拉取（git submodule）/ Online (git submodule)

```bash
# 首次 clone 时一并拉取 / Clone with submodules
git clone --recurse-submodules <ros_driver_repo_url>

# 或者已经 clone 过、但 module/sdk_api/ 为空
# Or after a normal clone where module/sdk_api/ is empty
cd src/ros_driver
git submodule update --init --recursive

# 后续更新到 submodule 最新版本 / Update to latest later on
git submodule update --remote --recursive
```

submodule 配置见 `@/home/xxx/work/odin2/ros_fold/src/ros_driver/.gitmodules:1-3`：

```ini
[submodule "module/sdk_api"]
    path = module/sdk_api
    url  = https://git.manifoldtech.cn/manifoldtech-sdk/odin-sdk2.git
```

### 方式 B：离线包 / Offline tarball

如果环境无法访问内网 git 服务器（典型场景：客户现场、隔离网络），从研发同事那里拿到 SDK 离线包后按下面步骤放置：

If the build host cannot reach the internal git server (customer site / isolated network), obtain an SDK tarball from the dev team and unpack it into the expected location:

```bash
# 1. 进入 ros_driver 目录 / Go to the driver package
cd src/ros_driver

# 2. 删除空的 submodule 占位（如果存在）/ Remove the empty submodule placeholder
rm -rf module/sdk_api

# 3. 把离线 SDK 解压到 module/sdk_api/ / Unpack the SDK into module/sdk_api/
#    （tarball 顶层应该就是 sdk_api 目录；如果不是则用 --strip-components 调整）
tar -xzf /path/to/odin_sdk_api_<version>.tar.gz -C module/
# 或者直接拷贝整个文件夹 / or copy the folder directly
# cp -r /path/to/odin_sdk_api module/sdk_api

# 4. 校验关键文件存在 / Verify the critical paths exist
ls module/sdk_api/include/
ls module/sdk_api/sdk/
ls module/sdk_api/CMakeLists.txt
```

> **⚠ 注意 / Note**：离线包替换后，`git status` 会因为 submodule 指针错乱报告 dirty；属正常现象。如果不想被 git 追踪 SDK 源码改动，可在 `git config submodule.module/sdk_api.ignore all` 让 git 忽略它。
>
> After replacing the submodule with an offline drop, `git status` will show the submodule as dirty — this is expected. To silence it, run `git config submodule.module/sdk_api.ignore all`.

### 校验 SDK 是否就位 / Verify the SDK is in place

```bash
# 必须存在的三个路径 / The three paths that must exist:
test -d src/ros_driver/module/sdk_api/include   && echo OK include  || echo MISSING include
test -d src/ros_driver/module/sdk_api/sdk       && echo OK sdk      || echo MISSING sdk
test -f src/ros_driver/module/sdk_api/CMakeLists.txt && echo OK cmake || echo MISSING cmake
```

三项都 OK 即可继续编译；`libodin_sdk.a` 会在首次 `colcon build` / `catkin build` 时由 SDK 自身的 CMake 自动产生在 `module/sdk_api/build/sdk/`。

If all three say OK you can proceed; the static library `libodin_sdk.a` will be produced into `module/sdk_api/build/sdk/` automatically by the SDK's own CMake on the first build.

## 编译 / Build

**中文**：SDK 会在编译时自动构建，无需单独安装。**前提是 `module/sdk_api/` 已通过上一节的方式 A 或方式 B 准备就绪**。

**English**: The SDK is built automatically as part of the package build — no separate install step is required, **provided that `module/sdk_api/` has been populated via method A or B above**.

### ROS1

```bash
source /opt/ros/noetic/setup.bash
cd your_ros_workspace
./src/odin_ros_driver/script/build_ros1.sh
```

### ROS2

```bash
source /opt/ros/humble/setup.bash
cd your_ros_workspace
./src/odin_ros_driver/script/build_ros2.sh
```

## 运行 / Run

### ROS1

```bash
source devel/setup.bash
# Odin1Lite（单相机设备）/ Odin1Lite (single-camera device)
roslaunch odin_ros_driver_rev1 odin1Lite.launch

# Odin2Special（双相机设备）/ Odin2Special (dual-camera device)
roslaunch odin_ros_driver_rev1 odin2Special.launch

# 指定图像分辨率和帧率 / Specify image resolution and frame rate
roslaunch odin_ros_driver_rev1 odin1Lite.launch image_width:=640 image_height:=544 image_fps:=30

# 不启动 RViz / Do not launch RViz
roslaunch odin_ros_driver_rev1 odin1Lite.launch start_rviz:=false
```

### ROS2

```bash
source install/setup.bash
# Odin1Lite（单相机设备）/ Odin1Lite (single-camera device)
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py

# Odin2Special（双相机设备）/ Odin2Special (dual-camera device)
ros2 launch odin_ros_driver_rev1 odin2Special.launch.py

# 指定图像分辨率和帧率 / Specify image resolution and frame rate
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py image_width:=640 image_height:=544 image_fps:=30

# 不启动 RViz / Do not launch RViz
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py start_rviz:=false
```

### L0 单相机启动 / L0 single-camera launch

```bash
ros2 launch odin_ros_driver_rev1 driver.launch.py
```

驱动默认独立运行，启动 Normal、raw/SLAM 点云、camera0 压缩图、IMU、LF/HF 和设备 TF。
图像后处理与 RViz 均可通过启动参数按需启用。整机传入
`post_process:=true undistort_mapping_only:=true`，仅 SLAM/ReMap 发布去畸变图。
`odin1Lite.launch.py` 和 `odin2Special.launch.py` 共享 L0 默认参数，各自选择 RViz 配置。
SDK 根据雷达上报型号选择设备实现。Odin2Special 需要同版本设备插件。
参见 [L0 适配与板端验收](L0_ADAPTATION.md)。

## Launch 参数速查 / Launch Parameters Quick Reference

**中文**：launch 文件已把全部可调参数暴露为 `<arg>` / `DeclareLaunchArgument`，可以通过 `key:=value` 在命令行覆盖。下表只列最常用的几个；**完整列表见 [参数文档](docs/parameters.md)**。

**English**: Every tunable knob is exposed as a launch argument and can be overridden via `key:=value` on the command line. The table below lists only the most common ones; **see [Parameters](docs/parameters.md) for the complete list**.

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | --- | --- |
| `start_rviz` | `false` | 是否启动 RViz / Whether to start RViz |
| `topic_prefix` | `manifold` | 话题前缀根 / Topic prefix root |
| `image_width` | `1280` | 图像宽度，`0` = 使用设备能力表第一项；具体取值需在线查询（见下方说明）/ Image width, `0` = first entry of the device's capability table; query online for actual values (see note below) |
| `image_height` | `1088` | 图像高度，`0` = 使用设备能力表第一项 / Image height, `0` = first capability entry |
| `image_fps` | `30` | 图像帧率，`0` = 使用设备能力表第一项 / Image frame rate, `0` = first capability entry |
| `image_format` | `mjpeg` | 图像格式 / Image format: `mjpeg` \| `h264` \| `nv12` |
| `device_mode` | `normal` | 工作模式（2 选 1）/ device mode (one of 2): `normal` \| `standby` |
| `sensor_mode` | `-1` | dToF 传感器模式 / dToF Lidar sensor mode: `-1`=保持设备设置 (keep device setting) \| `0`=HDR \| `1`=High Peak \| `2`=Low Peak \| `3`=Adaptive |
| `confidence_threshold` | `7` | 点云置信度阈值（`0..255`，运行期可调）/ Point-cloud confidence threshold (`0..255`, runtime-tunable) |
| `enable_row_filter` | `false` | 启用行滤波：置零行号低于 `min_row` 的点 / Enable row filter: zero out points below `min_row` |
| `min_row` | `0` | 最小行号（`0..191`），低于此行的点被置零 / Minimum row index (`0..191`), points below are zeroed |
| `log_level` | `1` | 日志级别（4 选 1，运行期可调）/ Log level (one of 4, runtime-tunable): `0`=ERROR \| `1`=INFO \| `2`=WARN \| `3`=DEBUG |

### 📡 `image_*` 取值说明 / Notes on `image_*` values

设备实际支持的分辨率与帧率组合**因设备型号、固件版本而异**，README 不固化清单；驱动启动时会调用 SDK 接口 `sdk::GetSensorCapability(handle, caps, {channel}, timeout_ms)` 拉取能力表，并通过 `QueryAndPrintSensorCapabilities()` 打印到终端。请以**当次启动日志**为准选择 `image_width / image_height / image_fps`，否则会被设备拒绝、回落到默认能力。

The exact resolution/FPS combinations a device supports **vary by model and firmware**, so this README does not freeze the list. On startup the driver calls `sdk::GetSensorCapability(handle, caps, {channel}, timeout_ms)` and prints the capability table via `QueryAndPrintSensorCapabilities()`. Always pick `image_width / image_height / image_fps` from **the current run's log**; otherwise the device will reject the combination and fall back to its default mode.

**典型 Odin2 能力（仅供参考，以实际查询为准）/ Typical Odin2 capabilities (reference only, see runtime query):**

| Channel | 分辨率 / Resolution | 帧率 / FPS | 默认格式 / Default format |
| --- | --- | --- | --- |
| `Image0` (左 / left) | `1280×1088`, `640×544` | `30` | `mjpeg` (`OdinDataFormat::kImageMjpeg = 0x00`) |
| `Image1` (右 / right) | `1280×1088`, `640×544` | `30` | `mjpeg` |

来源 / Source: `@/home/xxx/work/odin2/ros_fold/src/ros_driver/module/sdk_api/docs/protocol/PROTOCOL.md:198-213`

**`image_format` 完整枚举 / Full `image_format` enumeration** (`OdinDataFormat` in `@/home/xxx/work/odin2/ros_fold/src/ros_driver/module/sdk_api/include/odin_lidar_def.h:105-111`):

| launch 字符串 / launch string | SDK 枚举 / SDK enum | 编码值 / value | 说明 / Note |
| --- | --- | --- | --- |
| `mjpeg` (默认 / default) | `kImageMjpeg` | `0x00` | MJPEG 压缩，带宽最低 / lowest bandwidth |
| `h264` | `kImageH264` | `0x01` | H.264 编码视频流 / H.264 encoded video stream |

字符串解析见 / String parsing: `@/home/xxx/work/odin2/ros_fold/src/ros_driver/src/odin_ros_driver_node.cpp:933-943`；未识别字符串一律回落到 `mjpeg` / unknown strings fall back to `mjpeg`.

**运行期获取设备能力的两种方法 / Two ways to inspect device capabilities at runtime:**

1. **看驱动启动日志 / Read driver startup log** —— 搜索 `Sensor capabilities query complete` 或 `╔══...╗` ASCII 框；驱动每次启动都会自动打印当前设备的全部 channel 能力表。
   Search for `Sensor capabilities query complete` or the ASCII box header `╔══...╗` in the console. The driver prints every channel's capability table on every startup.

2. **在自己的代码里直接调用 SDK / Call the SDK directly:**

```cpp
std::vector<sdk::OdinSensorCapability> caps;
sdk::GetSensorCapability(device_handle, caps,
    {sdk::OdinDataChannel::kImage0}, /*timeout_ms=*/1000);
for (const auto& m : caps[0].modes) {
  printf("%ux%u@%u fmt=%d\n", m.width, m.height, m.fps, (int)m.format);
}
```

> ⚠️ 不要把 README 里"典型 Odin2 能力"硬编码到客户脚本中 —— 不同设备/固件可能差异很大，唯一可信来源是 `sdk::GetSensorCapability` 的运行期返回。
>
> ⚠️ Do not hard-code the "Typical Odin2 capabilities" table into your scripts — values may differ across devices/firmware. The only authoritative source is the runtime return of `sdk::GetSensorCapability`.

## 话题与服务速查 / Topics & Services Quick Reference

> 完整话题列表（含点云压缩传输）见 **[话题文档](docs/topics.md)**；完整服务详情（含诊断监控）见 **[服务文档](docs/services.md)**。
>
> Full topic list (including point-cloud transport compression) in **[Topics](docs/topics.md)**; full service details (including diagnostics) in **[Services](docs/services.md)**.

所有话题/服务均带有逐设备前缀 `{prefix} = /{topic_prefix}/device{N}/`（默认 `/manifold/device0/`）。设备型号不进入话题/服务路径，型号（model）与 SN 通过 `/{topic_prefix}/driver/device_online` 的 JSON 字段获知。

All topics/services carry a per-device prefix `{prefix} = /{topic_prefix}/device{N}/` (default `/manifold/device0/`). The device model never appears in topic/service paths; the model and SN are announced via the JSON fields of `/{topic_prefix}/driver/device_online`.

### 主要话题 / Main Topics

| 话题后缀 / Topic suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `cloud/raw` | `sensor_msgs/PointCloud2` | 原始点云 / Raw point cloud |
| `cloud/slam` | `sensor_msgs/PointCloud2` | SLAM 点云 / SLAM point cloud |
| `camera0/compressed` | `sensor_msgs/CompressedImage` | 左相机压缩图像 / Left camera compressed image |
| `imu` | `sensor_msgs/Imu` | IMU 数据 / IMU data |
| `odometry_lf` | `nav_msgs/Odometry` | 低频里程计（含位姿协方差）/ Low-frequency odometry (with pose covariance) |
| `odometry_hf` | `nav_msgs/Odometry` | 高频里程计（含位姿协方差）/ High-frequency odometry (with pose covariance) |

### 主要服务 / Main Services

| 服务后缀 / Service suffix | 说明 / Description |
| --- | --- |
| `get_calibration` | 获取相机标定 / Fetch camera calibration |
| `set_sensor_mode` | 设置传感器模式 / Set sensor mode |
| `reset_initial_pose` | 重置 SLAM 初始位姿 / Reset SLAM initial pose |
| `device_mode` | 切换设备工作模式 / Set device operating mode |
| `save_map` | 保存闭环地图 / Save loop-closure map |
| `load_map` | 加载地图（重定位）/ Load map (relocalization) |
| `set_diag_monitor` | 启用/关闭诊断监控 / Toggle diagnostics |

## 配置文件 / Configuration Files

| 文件 / File | 说明 / Description |
| --- | --- |
| `config/camera_calib.yaml` | 相机标定文件（从设备自动获取）/ Camera calibration file (auto-fetched from device) |
| `launch/odin1Lite.launch` | ROS1 Odin1Lite 启动配置（单相机）/ ROS1 Odin1Lite launch (single-camera) |
| `launch/odin1Lite.launch.py` | ROS2 Odin1Lite 启动配置（单相机）/ ROS2 Odin1Lite launch (single-camera) |
| `launch/odin2Special.launch` | ROS1 Odin2Special 启动配置（双相机）/ ROS1 Odin2Special launch (dual-camera) |
| `launch/odin2Special.launch.py` | ROS2 Odin2Special 启动配置（双相机）/ ROS2 Odin2Special launch (dual-camera) |
| `rviz/odin1lite_driver_view_ros1.rviz` / `rviz/odin1lite_driver_view_ros2.rviz` | Odin1Lite RViz 配置（单相机）/ Odin1Lite RViz config (single-camera) |
| `rviz/odin2special_driver_view_ros1.rviz` / `rviz/odin2special_driver_view_ros2.rviz` | Odin2Special RViz 配置（双相机）/ Odin2Special RViz config (dual-camera) |

## 启动说明 / Startup Notes

1. `device_ip` 留空时自动广播发现设备 / When `device_ip` is empty, devices are auto-discovered via broadcast.
2. 连接后自动设置为 `normal` 模式开始数据推送 / Upon connection the device is set to `normal` mode and starts streaming data.
3. ROS2 可通过 `start_rviz:=true` 启动 RViz / Start RViz on ROS2 with `start_rviz:=true`.
4. Ctrl+C 退出时自动将设备设为 `standby` 模式 / On Ctrl+C exit the device is set back to `standby` mode.

## 目录结构 / Directory Layout

```
ros_driver/
├── config/                         # 配置文件（标定等）/ Configuration files (calibration, etc.)
├── docs/                           # 详细文档 / Detailed documentation
│   ├── topics.md                   # 发布话题 / Published topics
│   ├── services.md                 # 服务 / Services
│   ├── map_and_mode_services.md    # 地图与设备模式服务专题 / Map & device-mode services
│   ├── parameters.md               # 参数 / Parameters
│   ├── hardware_decode.md          # 硬解码验证 / Hardware decode verification
│   └── troubleshooting.md          # 常见问题 / FAQ
├── include/                        # 公共头文件 / Common headers
│   ├── camera/                     # 内参类型 / Intrinsic types
│   ├── service_types.h
│   └── utility/                    # 工具类（ros_compat, yaml loader 等）/ Utilities
├── launch/                         # 启动文件 / Launch files
├── module/sdk_api/                 # Odin SDK（git submodule，详见上文 SDK 获取章节）/ Odin SDK (git submodule, see "SDK Acquisition" above)
├── rviz/                           # RViz 配置 / RViz configs
├── script/                         # 编译脚本 / Build scripts
├── src/                            # 源代码 / Source code
│   ├── odin_ros_driver_node.cpp    # 主驱动节点（ROS1/ROS2 统一）/ Main driver (ROS1/ROS2 unified)
│   ├── post_process_node.cpp       # 后处理节点 / Post-processing node
│   ├── camera_model/               # 相机投影模型 / Camera projection models
│   ├── cloud_render/               # 点云着色 / Cloud colorization
│   ├── color_undistort/            # 图像去畸变 / Image undistortion
│   ├── decoder/                    # JPEG 解码 / JPEG decoding
│   ├── pointcloud_reprojector/     # 点云重投影节点 / Point-cloud reprojector node
│   ├── pointcloud_to_depth/        # 点云转深度图节点（多设备）/ Point-cloud → depth (multi-device)
│   └── stereo_rectifier/           # 立体校正 / Stereo rectification
└── srv/                            # 自定义服务 / Custom services
```

## 版本历史 / Version History

详见 [CHANGELOG.md](CHANGELOG.md) / See [CHANGELOG.md](CHANGELOG.md) for details.
