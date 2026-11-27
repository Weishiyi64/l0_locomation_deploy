# Odin Foxglove Bridge

将 Odin LiDAR SDK 的传感器数据实时发布到 [Foxglove Studio](https://foxglove.dev/studio) 进行可视化。

## 功能

通过 WebSocket 协议将以下数据通道发布到 Foxglove Studio：

| Topic | Schema | 数据源 | 说明 |
|---|---|---|---|
| `/lidar/points` | `foxglove.PointCloud` | `OdinRawPoint` (x,y,z,intensity) | 原始点云 |
| `/slam/points` | `foxglove.PointCloud` | `OdinSlamPoint` (x,y,z,r,g,b,a) | SLAM 彩色点云 |
| `/camera/left` | `foxglove.CompressedImage` | MJPEG payload | 左摄像头 |
| `/camera/right` | `foxglove.CompressedImage` | MJPEG payload | 右摄像头 |
| `/odom` | `foxglove.PoseInFrame` | 低频里程计 | Odom LF |
| `/odom_hf` | `foxglove.PoseInFrame` | 高频里程计 | Odom HF |
| `/tf` | `foxglove.FrameTransform` | TF 坐标变换 | 刚体变换 |
| `/telemetry` | `odin.Telemetry` | CPU/内存/负载 | 设备遥测 ~1Hz |

## 前置依赖：Foxglove SDK

本示例依赖 [Foxglove SDK](https://github.com/foxglove/foxglove-sdk) v0.27.0 预编译库（约 100 MB）。  
**该依赖不入库**（已在 `.gitignore` 中忽略），需要手动下载到本地。

### 方法一：自动下载（推荐）

在有网络的机器上执行：

```powershell
# Windows（自动检测平台）
.\sample\foxglove_bridge\download_foxglove_sdk.ps1

# Linux x86_64
./sample/foxglove_bridge/download_foxglove_sdk.ps1 x86_64

# Linux aarch64
./sample/foxglove_bridge/download_foxglove_sdk.ps1 aarch64
```

脚本会将 SDK 下载并解压到 `3rdparty/foxglove-sdk/<platform>/`。

### 方法二：手动下载

1. 打开 Foxglove SDK Release 页面：  
   <https://github.com/foxglove/foxglove-sdk/releases/tag/sdk/v0.27.0>

2. 下载对应平台的 zip 文件：

   | 平台 | 文件名 |
   |---|---|
   | Windows x64 | `foxglove-v0.27.0-cpp-x86_64-pc-windows-msvc.zip` |
   | Linux x86_64 | `foxglove-v0.27.0-cpp-x86_64-unknown-linux-gnu.zip` |
   | Linux aarch64 | `foxglove-v0.27.0-cpp-aarch64-unknown-linux-gnu.zip` |

3. 解压到项目目录：

   ```
   3rdparty/foxglove-sdk/<platform>/
   ├── include/
   │   ├── foxglove/
   │   └── foxglove-c/
   ├── lib/
   │   ├── cmake/foxglove-sdk/   ← CMake 检查此路径判断 SDK 是否存在
   │   └── foxglove.lib
   └── src/
   ```

   > **关键路径**：CMake 通过 `lib/cmake/foxglove-sdk/` 目录判断 SDK 是否已安装。  
   > 解压后务必确认此路径存在，例如：`3rdparty/foxglove-sdk/win64/lib/cmake/foxglove-sdk/`

### 方法三：离线机器中转

```
有网机器:                          离线机器:
.\sample\foxglove_bridge\          (接收 3rdparty/foxglove-sdk/ 目录)
  download_foxglove_sdk.ps1   →    .\build.ps1
                                   ↑ 自动检测本地 SDK，无需网络
```

将 `3rdparty/foxglove-sdk/` 整个目录通过 U 盘或内网共享拷贝到目标机器即可。

## 编译

SDK 就位后，正常执行项目构建即可：

```powershell
.\build.ps1
```

如果 SDK 不存在，构建输出会显示：

```
foxglove_bridge: Foxglove SDK not found — sample SKIPPED.
  To enable, run:  .\sample\foxglove_bridge\download_foxglove_sdk.ps1
  Or see:          sample/foxglove_bridge/README.md
```

其他 sample 和 SDK 本体**不受影响**，正常编译。

## 运行

```powershell
.\build\install\bin\odin_foxglove_bridge.exe [port]
```

- 默认端口 `8765`（与 Foxglove Studio 默认端口一致）
- 程序启动后自动发现并连接 Odin 设备
- 设备连接后自动启动所有数据流

## 在 Foxglove Studio 中查看

### 快速开始：导入预配置布局（推荐）

项目附带了预配置的布局文件 `odin_layout.json`，一次导入后所有面板自动就位：

1. 打开 [Foxglove Studio](https://studio.foxglove.dev)
2. 连接 WebSocket → 输入 `ws://localhost:8765`
3. 点击左上角 **Layouts** 菜单 → **Import from file**
4. 选择 `sample/foxglove_bridge/odin_layout.json`

导入后自动获得：

```
┌──────────────────────────────────────────────┐
│  3D 面板（点云 + TF）    │  左图像 / 右图像  │
│  /lidar/points           │  /camera/left      │
│  /slam/points            │  /camera/right      │
│  follow: odom            ├────────────────────┤
│                          │  /odom  │ /telemetry│
│                          │  原始消息│ 原始消息  │
└──────────────────────────────────────────────┘
```

### 手动添加面板

如果不想导入布局，可以手动配置：

1. 打开 [Foxglove Studio](https://studio.foxglove.dev)（浏览器访问，无需安装）
2. 点击 **Open connection** → 输入 `ws://localhost:8765`
3. 添加面板：
   - **3D** 面板 → 订阅 `/lidar/points` 或 `/slam/points` 查看点云
   - **Image** 面板 → 订阅 `/camera/left` 或 `/camera/right` 查看图像
   - **Raw Message** 面板 → 订阅 `/odom`、`/odom_hf`、`/telemetry`
4. 配置完成后可通过 Layouts → **Export** 导出自己的布局供团队复用

## 文件结构

```
sample/foxglove_bridge/
├── CMakeLists.txt              # 构建配置（SDK 不存在时自动跳过）
├── README.md                   # 本文件
├── odin_layout.json            # Foxglove Studio 预配置布局（导入即用）
├── main.cpp                    # 入口：hotplug 自动连接 + 生命周期管理
├── foxglove_publisher.h        # Publisher 类声明（前置声明 foxglove 类型）
├── foxglove_publisher.cpp      # 数据转换：Odin packet → Foxglove protobuf
└── download_foxglove_sdk.ps1   # SDK 下载脚本
```

## 注意事项

- **SDK 体积**：Foxglove SDK 约 100 MB/平台（Rust 核心 + 静态库），不入版本库
- **宏冲突修复**：`logger.h` 的 `DEBUG/INFO/ERROR/WARN` 宏与 Foxglove SDK 枚举冲突，CMake 通过 `cmake/foxglove_patch_messages.cmake` 自动修复
- **H.264/H.265 图像**：Foxglove CompressedImage 面板仅支持 JPEG，H.264/H.265 帧会带对应 format 标签但无法直接渲染
