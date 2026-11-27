# 常见问题 / FAQ

> [← 返回 README](../README.md)

### 1. 多台电脑话题数据干扰 / Topic collisions across multiple machines

**问题 / Problem**：多台电脑通过 WiFi/局域网连接，启动 ROS Driver 时话题相同导致数据干扰。

When multiple machines run the driver on the same WiFi/LAN, identical topic names cause cross-machine interference.

**ROS2 解决方法 / ROS2 solution**：
```bash
# 查看当前 DOMAIN_ID（空白或 0 为共享模式）
# Check current DOMAIN_ID (empty or 0 means shared)
echo $ROS_DOMAIN_ID

# 设置独立的 DOMAIN_ID（0-100，0 为共享）
# Set an isolated DOMAIN_ID (0-100; 0 = shared)
export ROS_DOMAIN_ID=1
```

**ROS1 解决方法 / ROS1 solution**：
```bash
# 方法一（推荐）/ Option 1 (recommended)
export ROS_LOCALHOST_ONLY=1

# 方法二：使用不同端口 / Option 2: use a different master port
roscore -p 11312
export ROS_MASTER_URI=http://localhost:11312
```

> 以上命令只对当前终端生效。永久设置请添加到 `~/.bashrc` 或 `/etc/profile`。
> The commands above only affect the current shell. Append them to `~/.bashrc` or `/etc/profile` to make them permanent.

### 2. 设备连接失败 / Device connection failure

**问题 / Problem**：无法发现或连接设备。 / Cannot discover or connect to the device.

**解决方法 / Solution**：
1. 确保设备与电脑在同一网段 / Make sure the device and host are on the same subnet.
2. 检查防火墙是否阻止 UDP 广播（端口 60000-60010）/ Check whether the firewall blocks UDP broadcast (ports 60000-60010).
3. 尝试指定设备 IP / Try specifying the device IP explicitly: `roslaunch odin_ros_driver_rev1 odin1Lite.launch device_ip:=192.168.1.251`

### 3. 图像分辨率不生效（ROS1）/ Image resolution not applied (ROS1)

**问题 / Problem**：ROS1 下设置 `image_width`/`image_height` 参数无效。 / The `image_width`/`image_height` parameters have no effect on ROS1.

**解决方法 / Solution**：升级到 v0.9.0 或更高版本，已修复此问题。 / Upgrade to v0.9.0 or later — the issue has been fixed.

### 4. rosbag 录制丢包 / rosbag recording packet loss

**问题 / Problem**：使用 `rosbag record` 或 `ros2 bag record` 录制数据时，高频话题（如 IMU 400Hz）出现丢包或时间戳间隔异常。

When using `rosbag record` or `ros2 bag record`, high-frequency topics (e.g., IMU at 400Hz) experience packet loss or abnormal timestamp intervals.

#### ROS2 解决方法 / ROS2 Solution

ROS2 默认使用 DDS 协议，QoS 策略为 `BEST_EFFORT`，高频数据容易丢包。需要配置 QoS 覆盖文件：

ROS2 uses DDS protocol with default `BEST_EFFORT` QoS, which can cause packet loss for high-frequency data. You need to configure a QoS override file:

```bash
# 使用提供的 QoS 配置文件录制（配置文件位于 config/rosbag2_qos.yaml）
# Use the provided QoS configuration file for recording (located at config/rosbag2_qos.yaml)
ros2 bag record -a --qos-profile-overrides-path <path_to_ros_driver>/config/rosbag2_qos.yaml

# 示例 / Example:
ros2 bag record -a --qos-profile-overrides-path ~/catkin_ws/src/odin_ros_driver/config/rosbag2_qos.yaml
```

**QoS 配置说明 / QoS Configuration**：
- `reliability: reliable` - 确保数据可靠传输 / Ensures reliable data transmission
- `depth: 4000` - 队列深度，建议为频率的 10 倍 / Queue depth, recommended 10x the frequency

#### ROS1 解决方法 / ROS1 Solution

ROS1 使用 TCP 协议（TCPROS），天然保证数据可靠传输，**无需额外配置**。

ROS1 uses TCP protocol (TCPROS), which inherently guarantees reliable data transmission. **No additional configuration is needed.**

```bash
# ROS1 直接录制即可
# ROS1 can record directly
rosbag record -a

# 如果磁盘 IO 较慢，可增加缓冲区
# If disk IO is slow, increase buffer size
rosbag record -a --buffsize=256
```

#### 为什么 ROS1 和 ROS2 操作不同？ / Why are ROS1 and ROS2 different?

| 特性 / Feature | ROS1 | ROS2 |
| --- | --- | --- |
| 传输协议 / Protocol | TCP (TCPROS) | DDS (UDP-based) |
| 默认可靠性 / Default Reliability | 可靠 / Reliable | 尽力而为 / Best Effort |
| 丢包可能性 / Packet Loss | 低 / Low | 高频数据可能丢包 / Possible for high-freq data |
| 配置方式 / Configuration | 无需配置 / No config needed | 需要 QoS YAML 文件 / Requires QoS YAML |

**原理说明 / Technical Explanation**：
- **ROS1**：基于 TCP，协议层保证数据按序到达，不会丢包
- **ROS2**：基于 DDS/UDP，默认 `BEST_EFFORT` 策略不保证送达，需要显式配置 `RELIABLE` 策略

- **ROS1**: Based on TCP, the protocol layer guarantees in-order delivery without packet loss
- **ROS2**: Based on DDS/UDP, default `BEST_EFFORT` policy does not guarantee delivery, explicit `RELIABLE` configuration is required

### 5. 程序崩溃使用 GDB 调试 / Debugging crashes with GDB

**问题 / Problem**：节点运行中崩溃（segfault / abort），需要定位具体代码行和调用栈。

A node crashes at runtime (segfault / abort) and you need the call stack to locate the offending code line.

**前置条件 / Prerequisites**：必须使用 Debug 模式编译，否则栈信息没有行号。`build_ros1.sh` 和 `build_ros2.sh` 都会把命令行参数透传给底层构建工具，**直接在脚本后面追加 Debug 参数即可，无需绕开脚本**。

The package must be compiled in Debug mode, otherwise stack frames will not contain line numbers. Both `build_ros1.sh` and `build_ros2.sh` forward extra arguments to the underlying build tool, so **just append the Debug flag to the script — no need to bypass it**.

```bash
# ROS2（colcon 用 --cmake-args）/ ROS2 (colcon takes --cmake-args)
./src/ros_driver/script/build_ros2.sh --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash

# ROS1（catkin_make 直接吃 -D）/ ROS1 (catkin_make accepts -D directly)
./src/ros_driver/script/build_ros1.sh -DCMAKE_BUILD_TYPE=Debug
source devel/setup.bash
```

> 如果之前已经用 Release 编过，建议先清掉旧产物再编 Debug，避免新旧 `.o` 混用。
> If a Release build already exists, clean it first to avoid mixing Release/Debug objects.
> ROS2: `rm -rf build/odin_ros_driver_rev1 install/odin_ros_driver_rev1`；ROS1: `rm -rf build devel`。

#### 方法一：交互式 GDB（当前终端）/ Interactive GDB (current terminal)

```bash
# ROS2
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py --launch-prefix "gdb -ex run --args"

# ROS1（需要 launch 文件中节点已透传 launch-prefix；否则改 launch 文件）
# ROS1 (only works if the launch file forwards launch-prefix; otherwise edit the launch file)
roslaunch odin_ros_driver_rev1 odin1Lite.launch launch-prefix:="gdb -ex run --args"
```

崩溃后在 `(gdb)` 提示符里执行 / After the crash, at the `(gdb)` prompt run:

```
bt full              # 带局部变量的调用栈 / call stack with locals
thread apply all bt  # 全部线程调用栈 / all-threads stack
frame N              # 切到第 N 帧 / switch to frame N
p <var>              # 打印变量 / print variable
```

#### 方法二：非交互式（自动打印调用栈，推荐 SSH / 嵌入式）/ Non-interactive (auto-prints stack, recommended for SSH / embedded)

```bash
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py \
  --launch-prefix "gdb -batch -ex run -ex 'bt full' --args"
```

崩溃后调用栈会直接打印到终端，无需手动输入。

The stack is printed directly to the terminal on crash — no manual input required.

#### 方法三：独立窗口 GDB（多节点场景）/ GDB in a separate window (multi-node case)

```bash
# 需要先安装 xterm 且有图形界面 / requires xterm + GUI
sudo apt install xterm

ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py --launch-prefix "xterm -e gdb -ex run --args"
```

每个被调试的节点会弹出独立 xterm 窗口运行 GDB，输出与其它节点日志互不干扰。

Each debugged node spawns its own xterm with GDB, isolating its I/O from other nodes' logs.

#### 方法四：仅调试单个节点 / Debug a single node only

`launch-prefix` 默认会作用到 launch 中所有节点。只想调试某一个节点时，使用 `--launch-prefix-filter` 指定可执行文件名（如 `odin_ros_driver_node` / `post_process_node` / `pointcloud_to_depth_node`）：

By default `launch-prefix` is applied to every node in the launch file. To attach GDB to only one node, filter by executable name (e.g. `odin_ros_driver_node` / `post_process_node` / `pointcloud_to_depth_node`):

```bash
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py \
  --launch-prefix "gdb -ex run --args" \
  --launch-prefix-filter post_process_node
```

#### 方法五：通过 core dump 事后分析 / Post-mortem via core dump

适用于偶发崩溃、不便保持 GDB 在线的场景。

Useful for rare/intermittent crashes when keeping GDB attached is impractical.

```bash
# 1. 启用 core 文件 / enable cores
ulimit -c unlimited
echo "core.%e.%p" | sudo tee /proc/sys/kernel/core_pattern

# 2. 正常 launch，崩溃后会在工作目录生成 core 文件
# 2. launch normally; a core file appears in the working directory after the crash
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py

# 3. 用 GDB 加载 core 分析 / load the core in GDB
gdb /home/xxx/work/odin2/ros_fold/install/odin_ros_driver_rev1/lib/odin_ros_driver_rev1/<node_executable> core.<exe>.<pid>
(gdb) bt full
(gdb) thread apply all bt
```

#### 常见问题 / Common pitfalls

| 现象 / Symptom | 原因 / Cause | 处理 / Fix |
| --- | --- | --- |
| 进入不了 `(gdb)` 提示符 / Cannot reach the `(gdb)` prompt | `output="log"` 让 stdout 进了文件 / `output="log"` redirects stdout to file | 改 launch 文件 `output="screen"` 或用方法三 xterm / set `output="screen"` or use method 3 (xterm) |
| 调用栈全是 `??` / Stack is all `??` | 编译产物没有调试符号 / Build has no debug symbols | 用 `-DCMAKE_BUILD_TYPE=Debug` 重新编译 / Rebuild with `-DCMAKE_BUILD_TYPE=Debug` |
| `Unknown substitution: find` | ROS1 launch 文件被 `ros2 launch` 加载 / Loading a ROS1 XML launch with `ros2 launch` | 改用 `odin1Lite.launch.py` / Use `odin1Lite.launch.py` instead |
| `FileNotFoundError: 'xterm'` | 系统未安装 xterm / xterm is not installed | `sudo apt install xterm` 或改用方法一/二 / Install xterm or switch to method 1/2 |

### 6. 网络相机如何通过 DHCP 分配 IP？/ How to assign an IP to the network camera via DHCP?

当相机通过以太网直连或接入局域网、且没有固定 IP 时，可以用本仓库提供的脚本快速启动一台 DHCP 服务器，为相机自动分配 IP。

When the camera is connected via Ethernet directly or to a LAN without a static IP, you can use the bundled scripts to start a DHCP server and assign an IP to the camera automatically.

**1) 准备环境 / Prerequisites**

```bash
# 安装 udhcpd（BusyBox DHCP 服务端）
sudo apt install udhcpd

# 确认相机与主机在同一网段 / Ensure the camera and host are on the same network segment
```

**2) 修改 DHCP 配置 / Edit the DHCP config**

参考 `script/my_dhcp.conf`：

See `script/my_dhcp.conf`:

```text
interface enp129s0        # 监听网卡 / Network interface to listen on
start 192.168.1.3         # IP 池起始 / Start of IP pool
end 192.168.1.254         # IP 池结束 / End of IP pool
opt router 192.168.1.1    # 网关（注意是 router，不是 rounter）/ Gateway
option subnet 255.255.255.0
option lease 86400        # 租约时间（秒）；86400 = 1 天 / Lease time in seconds
```

> **注意 / Note**：
> - 把 `interface` 改成相机所接的网卡名（如 `eth0`、`enp129s0`）。
> - 把本机对应网卡的 IP 设为 `opt router` 里的网关地址（示例 `192.168.1.1`），否则相机拿不到网关。
> - Set `interface` to the actual NIC connected to the camera (e.g. `eth0`, `enp129s0`).
> - Assign the gateway IP in `opt router` to the host's corresponding interface (e.g. `192.168.1.1`).

**3) 启动 DHCP 服务器 / Start the DHCP server**

使用 `script/start_dhcp.sh`，必须传入配置文件路径并以 `sudo` 运行：

Run `script/start_dhcp.sh` with the config path, using `sudo`:

```bash
sudo ./script/start_dhcp.sh /path/to/my_dhcp.conf
```

脚本会自动创建租约目录 `/var/lib/misc` 和租约文件 `/var/lib/misc/udhcpd.leases`，然后前台启动 `udhcpd`。

The script automatically creates the lease directory `/var/lib/misc` and lease file `/var/lib/misc/udhcpd.leases`, then starts `udhcpd` in the foreground.

**4) 验证相机是否拿到 IP / Verify the camera got an IP**

```bash
# 查看租约文件
sudo cat /var/lib/misc/udhcpd.leases

# 或用 ping/arp 扫描网段
ping 192.168.1.3
```

如果看到租约文件里有相机的 MAC 地址和分配到的 IP，说明 DHCP 工作正常；随后即可用该 IP 连接相机。

If the lease file shows the camera's MAC address and assigned IP, DHCP is working; use that IP to connect to the camera.

**5) 后台运行 / Run in background**

`start_dhcp.sh` 默认前台运行，方便看日志。如需后台运行，把最后一行的 `sudo udhcpd -f "$CONFIG_FILE"` 改为：

The script runs in the foreground by default. To run it in the background, change the last line from `sudo udhcpd -f "$CONFIG_FILE"` to:

```bash
sudo udhcpd "$CONFIG_FILE" &
```

### 7. RViz 中 TF 变换失败导致点云无法显示 / TF transform failure prevents point cloud display in RViz

**问题 / Problem**：在 RViz 中将 Fixed Frame 设为 `device0/odom` 时，`cloud_raw` 显示 "Could not transform from [device0/lidar] to [device0/odom]"，但切换到 `device0/lidar` 时一切正常。

When setting Fixed Frame to `device0/odom` in RViz, `cloud_raw` shows "Could not transform from [device0/lidar] to [device0/odom]", but everything works fine when switching to `device0/lidar`.

**原因 / Cause**：设备发送 TF 的频率（~1Hz）与点云发布的时间戳存在微小相位差（点云时间戳比 TF 晚约 0.06s）。tf2 不支持外推（extrapolation），当点云时间戳落在最新 TF 之后，且下一帧 TF 还未到达时，变换查询失败。

The device publishes TF at a low rate (~1Hz), and a small phase offset exists between the TF and point-cloud timestamps (cloud stamp ~0.06s later than the latest TF). tf2 does not support extrapolation — when a point cloud timestamp falls after the latest TF and the next TF has not yet arrived, the transform lookup fails.

**解决方案 / Solution**：启用 `tf_publish_rate` 参数（launch 参数），设为一个大于 0 的值（推荐 50~200Hz）。这会启动一个定时器，以设定频率重复发布最新的 TF 值，确保 tf2 buffer 在任意时间点都有可插值的数据。

Enable the `tf_publish_rate` launch parameter with a value greater than 0 (recommended 50–200Hz). This starts a timer that re-publishes the latest TF values at the specified rate, ensuring the tf2 buffer always has interpolatable data at any point in time.

```bash
# 启用 TF 高频重发（推荐 50~200Hz）/ Enable high-rate TF re-publishing (recommended 50–200Hz)
ros2 launch ros_driver odin1Lite.launch.py tf_publish_rate:=50.0

# 禁用（使用设备原始 TF 频率）/ Disable (use device's native TF rate)
ros2 launch ros_driver odin1Lite.launch.py tf_publish_rate:=0
```

**注意事项 / Notes**：
- `tf_publish_rate=0` 时定时器不启用，保持设备原始行为 / When `tf_publish_rate=0`, the timer is not started and the device's native behavior is preserved.
- 重发不改变 TF 值本身，只增加时间轴上的采样密度 / Re-publishing does not alter the TF values — it only increases the sampling density along the time axis.
- 该参数对 CPU 开销极小（仅为定时器回调 + 缓存值重发）/ CPU overhead is negligible (just a timer callback + cached value re-publish).

### 8. cv_bridge 与 OpenCV 版本不匹配导致 segfault / cv_bridge / OpenCV version mismatch segfault

**问题 / Problem**：`post_process_node` 或下游节点在发布第一帧 RGB 图像时崩溃（segfault / heap corruption），而主驱动节点 `odin_ros_driver_node` 正常运行。常见于 Jetson 或其他预装了不同 OpenCV 的嵌入式平台。

`post_process_node` or downstream nodes crash (segfault / heap corruption) on the first RGB frame, while the main driver node `odin_ros_driver_node` runs fine. Common on Jetson or other embedded platforms with a pre-installed OpenCV that differs from the ROS apt package.

**原因 / Cause**：系统安装了**两个 OpenCV**：

- **cv_bridge**（来自 `/opt/ros/$ROS_DISTRO`，apt 安装）编译时链接的 OpenCV 版本与 ROS 发行版构建农场上的版本一致
- **驱动**通过 `find_package(OpenCV)` 找到的是系统最高优先级的 OpenCV（CMake 优先选 `/usr/local` > `/usr`），可能与 cv_bridge 的版本不同

运行时同一个进程加载了**两份 `libopencv_core`**：`cv::Mat` 由一个版本分配、另一个版本释放，跨越 ABI 边界导致堆损坏。

The host has **two OpenCV versions**: cv_bridge (from apt) was compiled against whatever OpenCV was on the ROS build farm, while the driver's `find_package(OpenCV)` picks the highest-priority installation (CMake ranks `/usr/local` above `/usr`). At runtime the process loads **two copies of `libopencv_core`** — a `cv::Mat` allocated by one version is freed by the other, crossing an ABI boundary and corrupting the heap.

> **为什么主驱动节点不崩溃 / Why the main driver node doesn't crash**：`odin_ros_driver_node` 直接发布 `CompressedImage`（原始 JPEG 字节流），不调用 `cv::imdecode` 或 cv_bridge，因此不加载 `libopencv_core`。崩溃只发生在 `post_process_node` 及下游使用 cv_bridge 做图像转换的节点中。
>
> `odin_ros_driver_node` publishes raw `CompressedImage` bytes without `cv::imdecode` or cv_bridge, so it never loads `libopencv_core`. The crash only affects `post_process_node` and downstream nodes that use cv_bridge for image conversion.

**解决方案 / Solution**：将 `vision_opencv`（cv_bridge）重新编译到当前 workspace，使其链接与系统相同的 OpenCV。

Rebuild `vision_opencv` (cv_bridge) into the current workspace so it links the same OpenCV as the system.

> **⚠️ 如果此前已经编译过本工作空间，必须删除构建产物后重新编译，否则本节操作不会生效 / If this workspace has been built before, you MUST delete the build artifacts and rebuild — otherwise the steps below will have no effect**
>
> `cv_bridge_DIR`（以及 `image_geometry_DIR` 等同类变量）是 CMake 的 **CACHE 变量**。首次 configure 时如果 workspace 里还没有 `vision_opencv`，它会被固化为 `/opt/ros/$ROS_DISTRO/share/cv_bridge/cmake`，而那份 config 导出的是**旧版 OpenCV 的绝对路径**。此后即使把 `vision_opencv` 源码放进 `src` 并编译出新的 config，主工程再次 configure 时 `find_package(cv_bridge)` 也会因为 CACHE 中已有值而**不再重新搜索**，链接行依旧指向旧版 OpenCV，崩溃现象完全不变 —— 这会让人误以为本节方案无效。
>
> `cv_bridge_DIR` (and siblings such as `image_geometry_DIR`) are CMake **CACHE** variables. If `vision_opencv` was absent from the workspace during the first configure, the variable gets pinned to `/opt/ros/$ROS_DISTRO/share/cv_bridge/cmake`, whose config exports **absolute paths to the old OpenCV**. Adding `vision_opencv` to `src` and building it afterwards does not help: on the next configure, `find_package(cv_bridge)` sees a cached value and **skips the search entirely**, so the link line still points at the old OpenCV and the crash reproduces unchanged — easily mistaken for this workaround not working.

**步骤 / Steps**（ROS1 和 ROS2 通用 / common for both ROS1 and ROS2）：

```bash
# 0. 确认当前 ROS 发行版 / Identify your ROS distro
echo $ROS_DISTRO

# 1. 下载 vision_opencv，分支名与 ROS 发行版一致 / Clone vision_opencv (branch = ROS distro)
cd <your_workspace>/src
git clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git

# 2. 【关键】删除已有构建产物，清掉被固化的 CACHE
#    [CRITICAL] Remove existing build artifacts to drop the pinned CACHE
cd <your_workspace>
rm -rf build devel install     # ROS1
rm -rf build install log       # ROS2

# 3. 先单独编译 cv_bridge / Build cv_bridge first
source /opt/ros/$ROS_DISTRO/setup.bash
cd <your_workspace>

# ROS1:
catkin_make -DCATKIN_WHITELIST_PACKAGES="cv_bridge" -j$(nproc)
# ROS2:
colcon build --packages-select cv_bridge --allow-overriding cv_bridge
source install/setup.bash   # ROS2 only

# 4. 再编译驱动 / Then build the driver
# ROS1 注意：CATKIN_WHITELIST_PACKAGES 同样是 CACHE 变量，不清空会导致驱动根本不被编译
# ROS1 note: CATKIN_WHITELIST_PACKAGES is also a CACHE variable — leaving it set silently skips the driver
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j$(nproc)   # ROS1 only
./src/ros_driver/script/build_ros1.sh   # or build_ros2.sh
```

> **不想全量重编时的替代方案 / Alternative if a full rebuild is undesirable**：显式用 `-D` 覆盖被固化的 CACHE 变量，效果等同于清缓存。
>
> Override the pinned CACHE variables explicitly with `-D`; equivalent to clearing the cache.
>
> ```bash
> cd <your_workspace>
> catkin_make -Dcv_bridge_DIR=$PWD/devel/share/cv_bridge/cmake \
>             -Dimage_geometry_DIR=$PWD/devel/share/image_geometry/cmake \
>             -DCATKIN_WHITELIST_PACKAGES="" -j$(nproc)
> ```

> **注意 / Note**：如果 `git clone -b $ROS_DISTRO` 失败，分支名可能不同（如 Jazzy 对应 `rolling`）。请到 [ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv) 查看可用分支。
>
> If `git clone -b $ROS_DISTRO` fails, the branch name may differ (e.g. Jazzy → `rolling`). Check available branches at [ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv).

> **ROS2 覆盖警告 / ROS2 override warning**：ROS 2 下编译 `cv_bridge` 时 colcon 会提示 `'cv_bridge' is in: /opt/ros/$ROS_DISTRO`，这只是 `colcon-override-check` 的告警，覆盖 apt 版本正是本方案的目的，加上 `--allow-overriding cv_bridge` 即可。因为用的是同一发行版分支的源码，API/ABI 兼容，唯一变化是链接的 OpenCV。
>
> On ROS 2, colcon warns that `'cv_bridge' is in: /opt/ros/$ROS_DISTRO`. This is just `colcon-override-check`; overriding the apt package is exactly the intent here, so pass `--allow-overriding cv_bridge`. The source comes from the matching distro branch, so it stays API/ABI compatible — only the linked OpenCV changes.

> **必须 source / `source` is mandatory**：`build_ros2.sh` 只编译驱动包，不会重编 `cv_bridge`；驱动的 `find_package(cv_bridge)` 命中哪一份取决于 `CMAKE_PREFIX_PATH` 顺序，所以第 3 步之前必须先 source workspace 的 `install/setup.bash`（顺序：先 `/opt/ros/$ROS_DISTRO`，后 workspace）。
>
> `build_ros2.sh` builds only the driver package and never rebuilds `cv_bridge`; which copy `find_package(cv_bridge)` resolves to depends on `CMAKE_PREFIX_PATH` order, so the workspace `install/setup.bash` must be sourced before step 3 (order: `/opt/ros/$ROS_DISTRO` first, workspace second).

**验证 / Verify**：

```bash
# 1. 先确认 cv_bridge_DIR 没有被固化到 /opt/ros —— 若指向 /opt/ros 则必须按上面第 2 步删除重编
#    Check cv_bridge_DIR is not pinned to /opt/ros; if it is, redo step 2 (delete and rebuild)
grep -i cv_bridge_DIR <your_workspace>/build/CMakeCache.txt   # ROS1
# 期望 / Expected: cv_bridge_DIR:PATH=<your_workspace>/devel/share/cv_bridge/cmake

# 2. 再检查最终产物的链接情况 / Then inspect the resulting binary
source /opt/ros/$ROS_DISTRO/setup.bash
source <your_workspace>/devel/setup.bash    # ROS1
# 或 / or
source <your_workspace>/install/setup.bash  # ROS2

# 必须检查全部 libopencv_* 模块，不能只看 opencv_core：
# core 可能只有单一版本而 imgproc/calib3d 却是双版本，只 grep core 会误判为“已修复”
# Check ALL libopencv_* modules, not just opencv_core: core may resolve to a single
# version while imgproc/calib3d are doubled — grepping core alone yields a false "fixed"
ldd <path_to_post_process_node> | grep -iE 'cv_bridge|libopencv_'

# 不同主版号的个数，应为 1 / Number of distinct OpenCV major.minor versions; must be 1
ldd <path_to_post_process_node> | grep -oE 'libopencv_[a-z0-9_]+\.so\.[0-9]+\.[0-9]+' \
  | sed -E 's/.*\.so\.//' | sort -u
```

- **已修复 / Fixed**：所有 `libopencv_*` 为同一版本（上面最后一条命令只输出一行）；`libcv_bridge.so` 指向 `<workspace>/...` 或未出现 / All `libopencv_*` are a single version (the last command prints exactly one line); `libcv_bridge.so` resolves to `<workspace>/...` or is absent.
- **仍有问题 / Still broken**：`libopencv_*` 出现**任何**两个不同版本，即使 `opencv_core` 只有一个 —— `imgproc`、`calib3d` 这些模块的错配同样会崩溃 / **Any** two different `libopencv_*` versions appear, even if `opencv_core` shows only one — a mismatch in `imgproc` or `calib3d` crashes just the same.

**何时需要此操作 / When is this needed**：

| 场景 / Scenario | 是否需要 / Needed? |
| --- | --- |
| 系统只有一个 OpenCV，且与 cv_bridge 匹配 / Single OpenCV matching cv_bridge | 否 / No |
| 系统有多个 OpenCV（如 `/usr/local` + `/usr`），cv_bridge (apt) 与驱动链接版本不同 / Multiple OpenCV installations, cv_bridge (apt) differs from driver | **是 / Yes** |
| Jetson / 嵌入式平台预装了非 ROS 标准 OpenCV / Jetson / embedded with non-standard OpenCV | **是 / Yes** |
| 换到新机器部署 / Deploying to a new machine | 先用 `ldd` 检查 / Check with `ldd` first |

> **症状相似但原因不同 / Similar symptom, different cause**：如果崩溃栈里出现 `jpeg_*` 或 `libnvjpeg.so`，那不是本节的 OpenCV 版本冲突，见 §10。
>
> If the crash stack mentions `jpeg_*` or `libnvjpeg.so`, it is not the OpenCV version conflict described here — see §10.

### 9. 编译时内存占满、GUI 卡顿 / Build exhausts RAM and freezes the GUI

**问题 / Problem**：在 8 GB 或更低内存、且 swap 使用 zram 的设备上编译时，整机严重卡顿、系统 GUI 几乎无响应，进度停在某个百分比长时间不动；用 Ctrl+C 中断后重新编译反而能顺利完成。

On devices with 8 GB of RAM or less that use zram for swap, the build makes the whole system stutter and the GUI nearly unresponsive, with progress stalled at some percentage; interrupting with Ctrl+C and rebuilding then completes fine.

**原因 / Cause**：本包中若干节点的单个编译单元需要 **1.0–1.6 GB** 的编译器内存（`post_process_node`、`pointcloud_reprojector_node` 等，因为同时展开 PCL、OpenCV 与 rclcpp 的模板）。构建工具默认按核心数并发（`-j$(nproc)`），多个重量级编译单元同时进行就会把内存推到上限，触发 zram/swap 换页抖动。swap 使用 zram 的设备上，压缩页仍留在物理内存里（不是真正扩容），压缩/解压还要和编译抢 CPU：编译占满 CPU 使换页变慢，内存压力迟迟降不下来，又触发更多换页，最终陷入互相拖慢的恶性循环。中断后重编能完成，是因为已完成的 `.o` 不再重编，剩余任务凑不出同样高的并发峰值。

Several translation units in this package need **1.0–1.6 GB** of compiler memory each (`post_process_node`, `pointcloud_reprojector_node`, … — they instantiate PCL, OpenCV and rclcpp templates in one go). The build tool defaults to one job per core (`-j$(nproc)`), so when several heavy units compile simultaneously memory runs out and the kernel starts thrashing zram/swap. On devices that use zram for swap the compressed pages still occupy physical RAM (no real extra capacity) and (de)compression competes with the build for CPU: the build saturates the CPU, so paging gets slower, memory pressure never drops, and that triggers even more paging — a vicious circle in which both sides keep slowing each other down. A rebuild after Ctrl+C succeeds because the finished `.o` files are not recompiled, so the remaining work never reaches the same concurrency peak.

#### 默认已限制并发 / Parallelism is capped by default

`build_ros1.sh` 和 `build_ros2.sh` 会按可用内存自动选择并发数（约每个编译任务 2 GB，另留 2 GB 给系统，且不超过核心数），并打印实际取值：

`build_ros1.sh` and `build_ros2.sh` pick the job count from total RAM (about 2 GB per compile job, 2 GB reserved for the system, never above the core count) and print what they chose:

```text
Build parallelism: MAKEFLAGS="-j2 -l3" (override with ODIN_BUILD_JOBS=<n>)
```

| 平台 / Platform | 默认并发 / Default jobs |
| --- | --- |
| Jetson Nano / Orin Nano 4 GB | `-j1` |
| Jetson Xavier NX / Orin Nano 8 GB | `-j2` |
| Jetson Orin NX 16 GB | `-j6` |
| 开发机 32 GB / 16 核 / 32 GB desktop, 16 cores | `-j14` |

手动指定或完全接管 / Override the count, or take full control:

```bash
# 指定并发数 / choose the job count
ODIN_BUILD_JOBS=3 ./src/ros_driver/script/build_ros2.sh

# 自己完全控制（脚本检测到 MAKEFLAGS 已设置就不再插手）
# Full control (the script leaves MAKEFLAGS alone once you set it)
MAKEFLAGS="-j3 -l4" ./src/ros_driver/script/build_ros2.sh
```

> `colcon` 与 `catkin_make` 都只在 `MAKEFLAGS` 不含 `-j`/`-l` 时才追加自己的并发参数，因此设置 `MAKEFLAGS` 对 ROS1 和 ROS2 同样有效。
> Both `colcon` and `catkin_make` add their own `-j`/`-l` only when `MAKEFLAGS` carries neither, so setting `MAKEFLAGS` works for ROS1 and ROS2 alike.

#### 进一步：用 cgroup 保护 GUI / Going further: protect the GUI with cgroups

并发已经限好之后，可以再套一层 cgroup，把换页压力关在编译自己的容器里，避免 GUI 的工作集被挤出内存：

Once the job count is capped, wrapping the build in a cgroup keeps reclaim pressure inside its own container so the GUI's working set is not evicted:

```bash
systemd-run --user --scope -p MemoryHigh=5G -p CPUWeight=20 \
  ./src/ros_driver/script/build_ros2.sh
```

`MemoryHigh` 是软上限（溢出进 swap，只变慢），`MemoryMax` 是硬上限（到顶直接 OOM 杀进程）；`CPUWeight=20` 保证编译抢不走交互进程的 CPU。

`MemoryHigh` is a soft limit (the overflow spills to swap and merely slows down), `MemoryMax` is a hard limit (the compiler gets OOM-killed); `CPUWeight=20` keeps the build from stealing CPU from interactive processes.

> **cgroup 不能代替限制并发。** 溢出量必须小到能被 swap 吸收，否则 cgroup 长期处于高内存压力，`systemd-oomd` 会直接杀掉整个 scope（实测：7 个重量级编译单元并发时峰值 8.4 GB，加 `MemoryHigh=2G` 后整个构建被杀掉 23 个进程；限到 2 并发后同样的 `MemoryHigh=2G` 可以正常完成）。
> **A cgroup is not a substitute for capping parallelism.** The overflow must be small enough for swap to absorb, otherwise the cgroup stays under heavy memory pressure and `systemd-oomd` kills the whole scope (measured: 7 heavy TUs in parallel peak at 8.4 GB, and with `MemoryHigh=2G` the build was killed — 23 processes; capped at 2 jobs the same `MemoryHigh=2G` completes normally).

> 需要 cgroup v2（`stat -fc %T /sys/fs/cgroup/` 输出 `cgroup2fs`）且 `memory` 控制器可用（`cat /sys/fs/cgroup/cgroup.controllers` 含 `memory`）。JetPack 5/6 与 Ubuntu 22.04 默认满足。
> Requires cgroup v2 (`stat -fc %T /sys/fs/cgroup/` prints `cgroup2fs`) with the `memory` controller enabled (`cat /sys/fs/cgroup/cgroup.controllers` lists `memory`). JetPack 5/6 and Ubuntu 22.04 satisfy this by default.

#### 最后手段：让编译器更激进地回收内存 / Last resort: make the compiler collect garbage harder

以上两条都不够时，可以调 GCC 的垃圾回收阈值，用编译时间换内存：

If neither of the above is enough, trade compile time for memory by tuning GCC's garbage-collection thresholds:

```bash
./src/ros_driver/script/build_ros2.sh --cmake-args \
  -DCMAKE_CXX_FLAGS="--param ggc-min-expand=10 --param ggc-min-heapsize=32768"
```

实测效果（`post_process_node.cpp`，基线 1613 MB / 18.8 s）/ Measured on `post_process_node.cpp` (baseline 1613 MB / 18.8 s)：

| 参数 / Parameters | 峰值内存 / Peak RSS | 耗时 / Time |
| --- | --- | --- |
| 无（基线）/ none (baseline) | 1613 MB | 18.8 s |
| `ggc-min-expand=30` | 1512 MB (-6%) | 20.7 s (+10%) |
| `ggc-min-expand=10 ggc-min-heapsize=32768` | 1365 MB (-15%) | 25.9 s (+38%) |

**降低 `-O` 等级没有帮助 / Lowering `-O` does not help**：实测 `-O3`/`-O2`/`-O1`/`-O0` 的峰值内存分别为 1613/1596/1624/1603 MB，几乎没有差别——内存主要花在解析和实例化模板，而不是优化阶段。

Measured peaks for `-O3`/`-O2`/`-O1`/`-O0` are 1613/1596/1624/1603 MB — essentially identical. The memory goes into parsing and template instantiation, not the optimizer.

### 10. Jetson 硬解 fallback 到软解时崩溃（libnvjpeg 与 libjpeg-turbo 符号冲突）/ Crash when HW decode falls back to SW on Jetson (libnvjpeg vs libjpeg-turbo symbol clash)

**问题 / Problem**：Jetson（R36/JetPack 6）上硬件 JPEG 解码正常工作，但当硬件解码失败并 fallback 到软件解码时，先打印一条 `RC_LOG_ERROR` 警告，紧接着在 `free()` 里 SIGSEGV。要安全使用纯软件解码，需重新编译为 `JPEG_PREFER_SOFTWARE_DECODER=ON`。

On Jetson (R36/JetPack 6), hardware JPEG decoding works normally, but when HW decode fails and falls back to software, an `RC_LOG_ERROR` warning is printed, followed by a segfault inside `free()`. To use pure software decoding safely, recompile with `JPEG_PREFER_SOFTWARE_DECODER=ON`.

典型调用栈 / Typical stack:

```text
#0  __GI___libc_free (mem=0xe13424160ad17262)
#1  jpeg_destroy_decompress ()   at /usr/lib/aarch64-linux-gnu/nvidia/libnvjpeg.so
#2  cv::JpegDecoder::~JpegDecoder ()  at /lib/libopencv_imgcodecs.so.408
#3  cv::imdecode_ ()
#5  odin_decoder::SoftwareDecoder::Decode (...)
```

**原因 / Cause**：这是 **libjpeg 符号冲突**，不是 §8 的两份 `libopencv_core`。

- `cv::JpegDecoder` 是 OpenCV 按 **libjpeg-turbo** 头文件编译的，`jpeg_decompress_struct` 是 turbo 布局（776 字节）
- NVIDIA 的 `libnvjpeg.so` 导出**同名** `jpeg_*` 符号，但它是按 **libjpeg-8b** 编译的（656 字节，字段偏移完全不同，见 `/usr/src/jetson_multimedia_api/include/libjpeg-8b`）
- R36 上 `USING_JETSON_DECODER=ON` 时，若 `libnvjpeg.so` 被直接链接到可执行文件，它就成为**可执行文件自身的 DT_NEEDED**。ELF 全局符号解析从可执行文件开始广度优先，`libnvjpeg` 排在 `libopencv_imgcodecs` 依赖的 `libjpeg.so.8` 之前，于是 OpenCV 内部所有 `jpeg_*` 调用被 `libnvjpeg` **符号插入（interposition）**

结果：OpenCV 按 turbo 布局分配结构体，nvjpeg 按 8b 偏移读写 → 解码先失败（`Failed to decode compressed image`），析构时 `jpeg_destroy_decompress` 取到的内存池指针是垃圾 → `free()` 野指针崩溃。

This is a **libjpeg symbol clash**, not the dual-`libopencv_core` issue of §8. OpenCV's `cv::JpegDecoder` is built against **libjpeg-turbo** (776-byte `jpeg_decompress_struct`), while NVIDIA's `libnvjpeg.so` exports the **same** `jpeg_*` names built against **libjpeg-8b** (656 bytes, different field offsets). When `libnvjpeg.so` is linked directly into the executable, ELF global symbol lookup (breadth-first from the executable) makes `libnvjpeg` precede the `libjpeg.so.8` that `libopencv_imgcodecs` needs, **interposing** every `jpeg_*` call OpenCV makes. OpenCV allocates a turbo-layout struct, nvjpeg reads/writes it at 8b offsets — decoding fails first, then `jpeg_destroy_decompress` frees a garbage pointer.

> **为什么软解 fallback 会崩 / Why the SW fallback crashes**：硬解走 `NvJpegDecoder`，它本身就是按 libjpeg-8b 头编译的，与 `libnvjpeg` 自洽。但 fallback 到 `cv::imdecode` 时，OpenCV 按 turbo 布局操作结构体，而 `libnvjpeg` 已在全局符号表中 interpose 了 `jpeg_*` → 崩溃。HybridDecoder 在首次 fallback 前打印一次 `RC_LOG_ERROR` 警告，提示用户重新编译。
>
> The hardware path uses `NvJpegDecoder`, compiled against libjpeg-8b headers and consistent with `libnvjpeg`. But the SW fallback calls `cv::imdecode`, where OpenCV uses turbo-layout structs while `libnvjpeg` interposes `jpeg_*` → crash. HybridDecoder prints a one-time `RC_LOG_ERROR` warning before the first fallback, advising the user to recompile.

**确认 / Confirm**：

```bash
NODE=<install>/lib/odin_ros_driver_rev1/post_process_node

# 硬件解码模式（JPEG_PREFER_SOFTWARE_DECODER=OFF，默认）：
# libnvjpeg 是可执行文件的直接依赖
readelf -d $NODE | grep NEEDED | grep -iE 'nvjpeg'
#    预期：输出包含 libnvjpeg.so

# 软件解码模式（-DJPEG_PREFER_SOFTWARE_DECODER=ON）：
# libnvjpeg 不应是可执行文件的直接依赖（关键）
readelf -d $NODE | grep NEEDED | grep -iE 'nvjpeg'
#    预期：无输出

# 顺带检查是否存在多份 OpenCV / libjpeg（见 §8）
ldd $NODE | grep -iE 'opencv_(core|imgcodecs)|jpeg'
ldconfig -p | grep -E 'libopencv_imgcodecs|libjpeg'
```

**解决方案 / Solution**：通过 CMake 选项 `JPEG_PREFER_SOFTWARE_DECODER` 在编译时选择解码模式。

The solution is a CMake compile-time option `JPEG_PREFER_SOFTWARE_DECODER` that selects the decoding mode.

- **`JPEG_PREFER_SOFTWARE_DECODER=OFF`（默认）**：编译 `NvJpegDecoder` 硬件解码器，链接 `libnvjpeg`，使用 `HybridDecoder` 包裹（HW + SW fallback）。硬解失败时 fallback 到 `cv::imdecode`，首次 fallback 前打印一次 `RC_LOG_ERROR` 警告。由于符号冲突，fallback 可能崩溃——这是已知行为，日志中会提示用户重新编译。
- **`JPEG_PREFER_SOFTWARE_DECODER=ON`**：不编译硬件解码器源码，不链接 `libnvjpeg`，`cv::imdecode` 的 `jpeg_*` 符号只解析到 `libjpeg.so.8`（libjpeg-turbo），纯软解安全运行。

- **`JPEG_PREFER_SOFTWARE_DECODER=OFF` (default)**: Compiles `NvJpegDecoder`, links `libnvjpeg`, uses `HybridDecoder` (HW + SW fallback). On HW failure, falls back to `cv::imdecode`; a one-time `RC_LOG_ERROR` warning is printed before the first fallback. Due to the symbol clash, the fallback may crash — this is known behaviour, and the log advises recompiling.
- **`JPEG_PREFER_SOFTWARE_DECODER=ON`**: No hardware decoder sources, no `libnvjpeg` linkage; `cv::imdecode`'s `jpeg_*` symbols resolve only to `libjpeg.so.8` (libjpeg-turbo). Pure SW decode runs safely.

```bash
# 硬件解码（默认编译）
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py

# 切换到软件解码：需重新编译
cmake -DJPEG_PREFER_SOFTWARE_DECODER=ON ...
# 或在 colcon 中：
colcon build --cmake-args -DJPEG_PREFER_SOFTWARE_DECODER=ON
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py
```

CMake 配置日志中可见：

The CMake configure log shows:

```text
-- Jetson platform detected
--   JPEG Decoder: NvJpegDecoder Hardware Decoder
```

| 场景 / Scenario | CMake 选项 / CMake Option | 行为 / Behaviour |
| --- | --- | --- |
| Jetson R36+，硬件解码 / HW decode | `OFF`（默认 / default） | 硬解，失败时 fallback 软解（可能崩，有警告）/ HW, SW fallback on failure (may crash, warned) |
| Jetson R36+，软件解码 / SW decode | `ON` | 纯软解，安全 / Pure SW, safe |
| Jetson R35（硬解自动关闭）/ R35 (HW auto-disabled) | 任意 / any | 纯软解，安全 / Pure SW, safe |
| x86 / 非 Jetson / non-Jetson | 任意 / any | 纯软解，安全 / Pure SW, safe |
