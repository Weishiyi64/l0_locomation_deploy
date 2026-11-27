# 服务 / Services

> [← 返回 README](../README.md)

所有服务均带有逐设备前缀 `{prefix} = /{topic_prefix}/device{N}/`（与话题相同；设备型号不进入路径）。下文示例使用默认前缀 `/manifold/device0/`。

All services carry the same per-device prefix `{prefix} = /{topic_prefix}/device{N}/` as the topics (the device model never appears in the path). The examples below use the default prefix `/manifold/device0/`.

> **⚠ 提示 / Tip**：服务路径只与 `topic_prefix` 和设备序号有关，与设备型号无关——换用不同型号设备时服务路径保持不变。请以启动日志 `Topic prefix: ...` 为准，或用 `ros2 service list | grep reset` 查询实际名称。
>
> Service paths depend only on `topic_prefix` and the device index — never on the device model, so switching between models keeps them stable. Always refer to the startup log `Topic prefix: ...` or query with `ros2 service list | grep reset`.

## 服务一览 / Service Overview

| 服务后缀 / Service suffix | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `get_calibration` | `odin_ros_driver_rev1/srv/GetCalibration` | 获取相机标定 YAML 内容 / Fetch the camera calibration YAML content |
| `set_sensor_mode` | `odin_ros_driver_rev1/srv/SetSensorMode` | 设置 dToF 传感器工作模式 / Set the dToF sensor mode |
| `reset_initial_pose` | `odin_ros_driver_rev1/srv/ResetInitialPose` | 重置 SLAM 初始位姿 / Reset SLAM initial pose |
| `get_adaptive_param` | `odin_ros_driver_rev1/srv/GetAdaptiveParam` | 查询自适应模式参数 / Query adaptive mode params |
| `set_adaptive_param` | `odin_ros_driver_rev1/srv/SetAdaptiveParam` | 设置自适应模式参数 / Set adaptive mode params |
| `get_sync_mode` | `odin_ros_driver_rev1/srv/GetSyncMode` | 查询默认时间同步模式 / Query default sync mode |
| `set_sync_mode` | `odin_ros_driver_rev1/srv/SetSyncMode` | 设置默认时间同步模式 / Set default sync mode |
| `get_ptp_sync_status` | `odin_ros_driver_rev1/srv/GetPtpSyncStatus` | 查询 PTP 同步状态 / Query PTP sync status |
| `restore_default_config` | `odin_ros_driver_rev1/srv/RestoreDefaultConfig` | 恢复出厂配置 / Restore factory default config |
| `device_mode` | `odin_ros_driver_rev1/srv/DeviceMode` | 切换设备工作模式 / Set device operating mode |
| `save_map` | `odin_ros_driver_rev1/srv/SaveMap` | 保存闭环地图到文件 / Save loop-closure map to file |
| `load_map` | `odin_ros_driver_rev1/srv/LoadMap` | 加载地图用于重定位 / Load map for relocalization |
| `set_diag_monitor` | `odin_ros_driver_rev1/srv/SetDiagMonitor` | 启用/关闭 1Hz 帧率与时间差诊断 / Toggle 1Hz FPS & latency diagnostic |

---

## `get_calibration`

请求空、响应内容（Request empty; Response）：

```text
# Request: (empty)
---
bool   success
string yaml_content   # 标定文件原始 YAML 内容 / raw YAML calibration content
string message
```

ROS2 调用示例 / ROS2 call:

```bash
ros2 service call /manifold/device0/get_calibration \
  odin_ros_driver_rev1/srv/GetCalibration "{}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/get_calibration "{}"
```

## `set_sensor_mode`

```text
uint8 mode   # Lidar mode: 0=HDR 1=High Peak  2=Low Peak  3=Adaptive
---
bool   success
int32  error_code   # 0=OK, 其它为 SDK OdinResult 枚举 / non-zero from SDK OdinResult enum
string message
```

ROS2 调用示例 / ROS2 call:

```bash
# 切换到 Adaptive 模式 / Switch to Adaptive mode
ros2 service call /manifold/device0/set_sensor_mode \
  odin_ros_driver_rev1/srv/SetSensorMode "{mode: 3}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/set_sensor_mode "mode: 3"
```

> **提示 / Note**：`set_sensor_mode` 与下文的 `sensor_mode` 参数功能等价；二者任一被修改时，驱动都会向设备下发 `sdk::SetSensorMode`。
>
> The `set_sensor_mode` service is equivalent to the `sensor_mode` parameter below; modifying either of them triggers `sdk::SetSensorMode` on the device.

## `reset_initial_pose`

重置 SLAM 算法初始位姿（清空位姿估计、地图、因子图），使其从零开始重新初始化。**不中断数据流**，重置在下一帧边界生效。

Resets the SLAM algorithm state (pose estimate, map, factor graph) so it re-initializes from scratch. **Does not interrupt data streaming**; the reset takes effect at the next frame boundary.

```text
# Request: (empty)
---
bool   success
int32  error_code   # 0=OK, 其它为 SDK OdinResult 枚举 / non-zero from SDK OdinResult enum
string message
```

ROS2 调用示例 / ROS2 call:

```bash
ros2 service call /manifold/device0/reset_initial_pose \
  odin_ros_driver_rev1/srv/ResetInitialPose "{}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/reset_initial_pose "{}"
```

> **使用场景 / Use Cases**：
> - SLAM 跟踪丢失时重新初始化 / Re-initialize when SLAM tracking is lost
> - 需要重新建图时 / When re-mapping is needed
> - 位姿估计发散时恢复 / Recover from pose estimation divergence

## `get_adaptive_param` / `set_adaptive_param`

查询/设置自适应模式参数（CDF 百分位、室内/外阈值）。**仅在 `sensor_mode=3`（Adaptive）时生效**。

Query/set adaptive mode parameters (CDF percentile, indoor/outdoor threshold). **Only active when `sensor_mode=3` (Adaptive)**.

```text
# SetAdaptiveParam Request / GetAdaptiveParam Response
uint16 cdf_percentile     # 1-10000, unit: 0.01%
uint16 outdoor_threshold  # mm, distance >= this classified as outdoor
---
bool success  int32 error_code  string message
```

ROS2 调用示例 / ROS2 call:

```bash
# 查询 / Query
ros2 service call /manifold/device0/get_adaptive_param \
  odin_ros_driver_rev1/srv/GetAdaptiveParam "{}"

# 设置 / Set
ros2 service call /manifold/device0/set_adaptive_param \
  odin_ros_driver_rev1/srv/SetAdaptiveParam "{cdf_percentile: 7000, outdoor_threshold: 3000}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/get_adaptive_param "{}"
rosservice call /manifold/device0/set_adaptive_param "{cdf_percentile: 7000, outdoor_threshold: 3000}"
```

## `get_sync_mode` / `set_sync_mode`

查询/设置默认时间同步模式（系统时钟或 PTP 硬件时间戳）。设置后**需重启设备**生效。

Query/set default time sync mode (system clock or PTP hardware timestamp). Setting takes effect after **device reboot**.

```text
# SetSyncMode Request / GetSyncMode Response
uint8 sync_mode    # 0=SysTick, 1=PTP
---
bool success  int32 error_code  string message
```

ROS2 调用示例 / ROS2 call:

```bash
# 查询当前同步模式 / Query current sync mode
ros2 service call /manifold/device0/get_sync_mode \
  odin_ros_driver_rev1/srv/GetSyncMode "{}"

# 设置为 PTP 模式 / Set to PTP mode
ros2 service call /manifold/device0/set_sync_mode \
  odin_ros_driver_rev1/srv/SetSyncMode "{sync_mode: 1}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/get_sync_mode "{}"
rosservice call /manifold/device0/set_sync_mode "{sync_mode: 1}"
```

## `get_ptp_sync_status`

查询 PTP 同步状态快照（实时状态，非持久配置）。

Query real-time PTP synchronization status snapshot (not persisted).

```text
# Request: (empty)
---
bool   success
int32  error_code
string message
uint8  sync_state      # 0=searching, 1=locking, 2=locked, 3=error
uint8  servo_state     # ptp4l raw servo state
int64  offset_ns       # Master-Slave offset (ns)
int64  path_delay_ns   # Path delay (ns)
int32  freq_ppb        # Frequency adjustment (ppb)
```

ROS2 调用示例 / ROS2 call:

```bash
ros2 service call /manifold/device0/get_ptp_sync_status \
  odin_ros_driver_rev1/srv/GetPtpSyncStatus "{}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/get_ptp_sync_status "{}"
```

> **提示 / Tip**：`sync_state=2`（locked）表示 PTP 已锁定；`offset_ns` 越接近 0 同步越精准。
>
> `sync_state=2` (locked) means PTP is locked; `offset_ns` closer to 0 means better sync.

## `restore_default_config`

恢复设备出厂默认配置（删除自定义配置文件）。**需重启设备**生效。

Restore factory default configuration (deletes custom config file). Takes effect after **device reboot**.

```text
# Request: (empty)
---
bool success  int32 error_code  string message
```

ROS2 调用示例 / ROS2 call:

```bash
ros2 service call /manifold/device0/restore_default_config \
  odin_ros_driver_rev1/srv/RestoreDefaultConfig "{}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/restore_default_config "{}"
```

## `device_mode`

切换设备工作模式。成功后驱动会更新内部期望模式（`desired_mode_`），用于重连后恢复。

Set the device operating mode. On success the driver updates its internal desired mode (`desired_mode_`) used for re-connect recovery.

```text
uint8 mode   # 0=Standby 1=Normal 2=Upgrade 3=SLAM 4=Relocation 5=ReMap
---
bool   success
int32  error_code   # 0=OK, 其它为 SDK OdinResult 枚举 / non-zero from SDK OdinResult enum
string message      # 成功时为模式名（如 "SLAM"）/ mode name on success (e.g. "SLAM")
```

模式取值 / Mode values:

| 值 / Value | 模式 / Mode | 说明 / Description |
| --- | --- | --- |
| 0 | Standby | 待机，停止数据流 / Standby, stops data streaming |
| 1 | Normal | 正常工作，开始数据流 / Normal, starts data streaming |
| 2 | Upgrade | 升级模式 / Upgrade mode |
| 3 | SLAM | SLAM 建图模式 / SLAM mapping mode |
| 4 | Relocation | 重定位模式 / Relocalization mode |
| 5 | ReMap | 重定位 + 增量建图模式 / Relocalization + incremental mapping mode |

ROS2 调用示例 / ROS2 call:

```bash
# 切换到 SLAM 模式 / Switch to SLAM mode
ros2 service call /manifold/device0/device_mode \
  odin_ros_driver_rev1/srv/DeviceMode "{mode: 3}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/device_mode "{mode: 3}"
```

> **提示 / Tip**：`device_mode` 与启动参数 `device_mode`（`normal`\`standby`）等价；服务调用成功后 `desired_mode_` 会被更新，Ctrl+C 退出时驱动仍会将设备切回 `standby` 模式。
>
> The `device_mode` service is equivalent to the launch parameter `device_mode` (`normal`\`standby`); a successful call updates `desired_mode_`, and the driver still switches the device back to `standby` on Ctrl+C exit.

## `save_map` / `load_map`

保存/加载设备端闭环（loop-closure）地图，用于 SLAM 建图结果持久化、重定位和 ReMap。通常在先切换到 SLAM 建图模式后保存地图，切换到 Relocation 或 ReMap 模式后加载地图；ReMap 结束后也可以保存增量地图。

Save/load the on-device loop-closure map for SLAM map persistence, relocalization, and ReMap. Typically save the map after switching to SLAM mode, and load the map after switching to Relocation or ReMap mode; ReMap can also save the incrementally extended map.

```text
# Request（两者相同 / same for both）
string path   # 主机侧地图文件路径（save_map 为输出路径，load_map 为输入路径）
              # host-side map file path (output for save_map, input for load_map)
---
bool   success
int32  error_code   # 0=OK, 其它为 SDK OdinResult 枚举 / non-zero from SDK OdinResult enum
string message
```

行为说明 / Behavior:

| 场景 / Case | success | error_code | message |
| --- | --- | --- | --- |
| 路径为空 / Empty path | false | `2` (kInvalidArgument) | `Empty map path` |
| 设备未连接 / Not connected | false | `1` (kNotInitialized) | `Device not connected` |
| SDK 成功 / SDK OK | true | `0` | `Map saved to: <path>` / `Map loaded from: <path>` |
| SDK 失败 / SDK error | false | SDK 错误码 / SDK code | `SDK error: <code>` |

典型调用流程 / Typical flow:

```bash
# 1. 切换到 SLAM 模式建图 / Switch to SLAM mode
ros2 service call /manifold/device0/device_mode \
  odin_ros_driver_rev1/srv/DeviceMode "{mode: 3}"
# 2. 保存地图 / Save map
ros2 service call /manifold/device0/save_map \
  odin_ros_driver_rev1/srv/SaveMap "{path: '/home/user/maps/loop_map.bin'}"

# 3. 切换到重定位模式 / Switch to Relocation mode
ros2 service call /manifold/device0/device_mode \
  odin_ros_driver_rev1/srv/DeviceMode "{mode: 4}"
# 4. 加载地图 / Load map
ros2 service call /manifold/device0/load_map \
  odin_ros_driver_rev1/srv/LoadMap "{path: '/home/user/maps/loop_map.bin'}"
```

ROS2 调用示例 / ROS2 call:

```bash
# 保存地图 / Save map
ros2 service call /manifold/device0/save_map \
  odin_ros_driver_rev1/srv/SaveMap "{path: '/home/user/maps/loop_map.bin'}"

# 加载地图用于重定位 / Load map for relocalization
ros2 service call /manifold/device0/load_map \
  odin_ros_driver_rev1/srv/LoadMap "{path: '/home/user/maps/loop_map.bin'}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/save_map "{path: '/home/user/maps/loop_map.bin'}"
rosservice call /manifold/device0/load_map "{path: '/home/user/maps/loop_map.bin'}"
```

> **提示 / Tip**：地图读写由设备端算法会话执行，路径为主机侧文件路径。若算法会话未就绪（如设备不在 SLAM/重定位/ReMap 模式），SDK 会返回 `error_code=1`（kNotInitialized）。
>
> The map is read/written by the device-side algorithm session; the path is on the host. If the algorithm session is not ready (e.g. device not in SLAM/Relocation/ReMap mode), the SDK returns `error_code=1` (kNotInitialized).

## `exploy_map`

将当前 loop-closure 地图导出到主机侧文件。具体导出格式由 loop-closure 插件的 `ExployMap(path)` 实现决定；该接口不替代 `save_map` 的重定位地图存档。

Export the current loop-closure map to a host-side file. The loop-closure plugin's `ExployMap(path)` implementation defines the output format; this does not replace the relocatable map archive created by `save_map`.

```text
string path
---
bool   success
int32  error_code
string message
```

```bash
# ROS2
ros2 service call /manifold/device0/exploy_map \
  odin_ros_driver_rev1/srv/ExployMap "{path: '/home/user/maps/loop_map.ply'}"

# ROS1
rosservice call /manifold/device0/exploy_map "{path: '/home/user/maps/loop_map.ply'}"
```

## `set_diag_monitor`

### 驱动端（odin_ros_driver_node）/ Driver-side

启用/关闭诊断监控模式。启用后每秒在终端打印各数据通道的帧率（FPS）和本机时间与数据时间戳的差值（`host_time - device_timestamp`，单位 ns）。同时响应中也会返回当前各通道的 FPS 和 latency。

Toggle diagnostic monitor mode. When enabled, prints per-channel FPS and latency (`host_time - device_timestamp`, in ns) to the terminal every second. The response also returns the current FPS and latency for all channels.

```text
# Request
bool enable     # true=启用 1Hz 打印 / enable 1Hz print; false=关闭 / disable
---
# Response
bool   success
string message
bool   monitor_enabled       # 当前状态 / current state after this call
float64 raw_point_fps         # RawPoint 帧率 / FPS
float64 slam_point_fps
float64 image0_fps
float64 image1_fps
float64 imu_fps
float64 odom_lf_fps
float64 odom_hf_fps
float64 tf_fps
int64 raw_point_latency_ns   # 最新一帧 latency(ns) / latest latency
int64 slam_point_latency_ns
int64 image0_latency_ns
int64 image1_latency_ns
int64 imu_latency_ns
int64 odom_lf_latency_ns
int64 odom_hf_latency_ns
int64 tf_latency_ns
```

ROS2 调用示例 / ROS2 call:

```bash
# 启用诊断打印 / Enable diagnostic print
ros2 service call /manifold/device0/set_diag_monitor \
  odin_ros_driver_rev1/srv/SetDiagMonitor "{enable: true}"

# 关闭并查询当前状态 / Disable and query current stats
ros2 service call /manifold/device0/set_diag_monitor \
  odin_ros_driver_rev1/srv/SetDiagMonitor "{enable: false}"
```

ROS1 调用示例 / ROS1 call:

```bash
rosservice call /manifold/device0/set_diag_monitor "{enable: true}"
rosservice call /manifold/device0/set_diag_monitor "{enable: false}"
```

终端打印示例 / Terminal output example:

```text
[DiagMonitor] FPS / Latency(ns) — host_time - device_timestamp
  Channel      FPS     Latency(ns)
  ---------  -------  -------------
  RawPoint     10.0      12345678
  SlamPoint    10.0       9876543
  Image0       30.0       5678901
  Image1       30.0       5678123
  IMU         200.0      1234567
  OdomLF       10.0      1111222
  OdomHF      200.0      1111333
  Tf           10.0      2222333
```

> **说明 / Note**：latency 为本机系统时钟（`system_clock`）减去设备时间戳（hardware monotonic ns）的差值。正值表示本机时间领先设备，负值表示设备时间领先本机。可用于评估传输延迟和时钟同步状态。
>
> The latency is `host system_clock` minus `device timestamp` (hardware monotonic ns). Positive means host clock is ahead of device; negative means device is ahead. Useful for evaluating transport delay and clock sync.

### 标准 ROS 诊断话题 / Standard ROS Diagnostics Topic

启用诊断监控后，驱动还会向标准 `/diagnostics` 话题发布 `diagnostic_msgs/DiagnosticArray` 消息，每个数据通道（RawPoint、SlamPoint、Image0、Image1、IMU、OdomLF、OdomHF、Tf）对应一个 `DiagnosticStatus`。

When the diagnostic monitor is enabled, the driver also publishes `diagnostic_msgs/DiagnosticArray` to the standard `/diagnostics` topic. Each data channel (RawPoint, SlamPoint, Image0, Image1, IMU, OdomLF, OdomHF, Tf) maps to one `DiagnosticStatus`.

**状态级别 / Status levels:**

| 级别 / Level | 值 / Value | 含义 / Meaning |
| --- | --- | --- |
| OK | `0` | 数据流正常（FPS > 0）/ Stream active (FPS > 0) |
| WARN | `1` | FPS 降为 0（曾经有数据）/ FPS dropped to 0 (had data before) |
| ERROR | `2` | 从未收到数据 / No data ever received |

**KeyValue 字段 / KeyValue fields:**

| Key | 说明 / Description |
| --- | --- |
| `fps` | 当前帧率 / Current FPS |
| `latency_avg_ns` | 平均延迟（ns）/ Average latency (ns) |
| `latency_min_ns` | 最小延迟（ns）/ Minimum latency (ns) |
| `latency_max_ns` | 最大延迟（ns）/ Maximum latency (ns) |

**查看方式 / How to view:**

```bash
# 命令行查看 / Command-line
ros2 topic echo /diagnostics

# ROS1
rostopic echo /diagnostics

# 使用 GUI 工具 / Using GUI tools
# ROS2
ros2 run rqt_robot_monitor rqt_robot_monitor
# ROS1
rosrun rqt_robot_monitor rqt_robot_monitor
```

> **提示 / Tip**：`hardware_id` 字段为设备序列号（SN），可用于多设备场景下区分不同设备的诊断信息。
>
> The `hardware_id` field is the device serial number (SN), useful for distinguishing diagnostics from multiple devices.

### 订阅端（diag_subscriber_node）/ Subscriber-side

`diag_subscriber_node` 是一个独立 C++ 节点，订阅驱动发布的所有数据话题，在**订阅端**计数帧率和计算延迟。由于使用 C++ 反序列化，比 `ros2 topic hz`（Python）更准确，尤其对 PointCloud2 等大消息。

`diag_subscriber_node` is a standalone C++ node that subscribes to all driver data topics and counts FPS/latency at the **subscriber side**. Uses C++ deserialization — more accurate than `ros2 topic hz` (Python), especially for large PointCloud2 messages.

**启动 / Launch:**

```bash
# ROS2
ros2 run odin_ros_driver_rev1 diag_subscriber_node \
  --ros-args -p topic_prefix:=/manifold/device0/

# ROS1
rosrun odin_ros_driver_rev1 diag_subscriber_node \
  _topic_prefix:=/manifold/device0/
```

**控制 / Control:**

`diag_subscriber_node` 没有独立的 service。启动后默认启用诊断打印，并通过监听驱动端发布的 `diag_status` 话题自动同步启停状态。只需调用驱动端的 `set_diag_monitor` 即可同时控制驱动端与订阅端。

```bash
# ROS2: 同时启用驱动端和订阅端诊断
ros2 service call /manifold/device0/set_diag_monitor \
  odin_ros_driver_rev1/srv/SetDiagMonitor "{enable: true}"

# ROS1: 同时启用驱动端和订阅端诊断
rosservice call /manifold/device0/set_diag_monitor "{enable: true}"
```

终端输出示例 / Terminal output:

```text
[DiagSubscriber] FPS / Latency(ns) — subscriber-side (after ROS transport)
  Channel      FPS     Latency(ns)
  ---------  -------  -------------
  RawPoint     9.8      12345678
  SlamPoint    9.8       9876543
  Image0       29.5       5678901
  Image1       29.5       5678123
  IMU         198.0      1234567
  OdomLF        9.8      1111222
  OdomHF      198.0      1111333
```

> **对比使用 / Comparison usage**：同时启用驱动端 `set_diag_monitor` 和订阅端 `set_diag_monitor`，对比两侧 FPS。驱动端 = 设备实际产生帧率；订阅端 = 经过 ROS 发布 + DDS 传输 + C++ 反序列化后到达订阅端的帧率。两者差值即为 ROS 链路丢帧率。
>
> Enable both driver-side and subscriber-side `set_diag_monitor` to compare. Driver-side = frames produced by device; subscriber-side = frames arriving at subscriber after full ROS transport. The difference reveals ROS-level frame loss.

## 数据流开关服务 / Stream Toggle Services

除了 `rqt` / `ros2 param set` 之外，还提供 ROS service 进行批量热切换：

In addition to `rqt` / `ros2 param set`, ROS services are provided for batch hot-switching:

| Service | 节点 / Node | 说明 / Description |
| --- | --- | --- |
| `{prefix}set_channel_config` | `odin_ros_driver_node` | 批量设置 A 层通道开关 / Batch-set layer A channel switches |
| `{prefix}get_channel_config` | `odin_ros_driver_node` | 查询 A 层通道当前开关状态 / Query current layer A channel states |
| `{prefix}/postprocess/set_post_process_config` | `post_process_node` | 批量设置 B 层后处理开关 / Batch-set layer B post-process switches |
| `{prefix}/postprocess/get_post_process_config` | `post_process_node` | 查询 B 层后处理当前开关状态 / Query current layer B post-process states |

```bash
# ROS2: 通过 service 热切换 / Hot-switch via service
ros2 service call /manifold/device0/set_channel_config \
  odin_ros_driver_rev1/srv/SetChannelConfig \
  "{enable_raw_point: false, enable_image0: true}"

ros2 service call /manifold/postprocess/set_post_process_config \
  odin_ros_driver_rev1/srv/SetPostProcessConfig \
  "{send_image_raw: 1, send_image2_raw: -1, send_cloud_render: -1, send_undistort: -1, send_rectify: -1, send_opencv_undistort: -1}"

# ROS2: 查询当前通道状态 / Query current channel states
ros2 service call /manifold/device0/get_channel_config \
  odin_ros_driver_rev1/srv/GetChannelConfig

ros2 service call /manifold/postprocess/get_post_process_config \
  odin_ros_driver_rev1/srv/GetPostProcessConfig

# ROS2: 通过 rqt 动态配置 / Dynamic config via rqt
ros2 run rqt_reconfigure rqt_reconfigure

# ROS2: 通过 param set 单个切换 / Single switch via param set
ros2 param set /odin_ros_driver_node enable_raw_point false
ros2 param set /postprocess send_undistort true
```
