# Odin SDK

与 Odin 雷达通信的 C++ 动态库（Linux 下 `libodin_sdk.so`，Windows 下 `odin_sdk.dll`），提供设备发现、连接管理、数据接收、指令发送和安全文件传输功能。支持 Linux（x86_64 / aarch64）与 Windows（x64）双平台编译。

## 功能特性

- **设备发现**：广播发现局域网内的 Odin 设备
- **多设备连接**：支持同时连接多台雷达
- **数据接收**：点云、图像、IMU、里程计等多种数据类型
- **指令发送**：模式切换、固件升级、文件传输等
- **SLAM/Odom 同步**：支持按帧号同步 SLAM 点云和里程计数据
- **HTTPS 安全传输**：基于 mTLS 的加密文件传输（固件升级、标定文件等）
- **热插拔监听**：支持设备热插拔事件监听
- **设备插件**：支持通过外部 `.so` 插件扩展定制设备类型

## 系统要求

| 平台 | 要求 |
|------|------|
| Linux | Ubuntu 20.04 / 22.04（x86_64 / aarch64），GCC + CMake |
| Windows | Windows 10 / 11 x64，Visual Studio 2022 或更新版本（含「使用 C++ 的桌面开发」工作负载）+ CMake |

两个平台都要求 C++17 编译器。SDK 所有三方依赖 (OpenSSL, mongoose, cJSON, tinyxml2, Eigen3, FastCRC) 已内置于 `3rdparty/` 目录，编译时自动构建，无需系统安装。

## 依赖安装

Linux：
```bash
sudo apt install build-essential cmake
```

Windows：
- 安装 [Visual Studio 2022+](https://visualstudio.microsoft.com/)，勾选「使用 C++ 的桌面开发」工作负载（自带 MSVC、CMake、Ninja）
- 若单独安装 CMake，需 3.20+ 并加入 PATH

## 编译与安装

### Linux

```bash
./build.sh
```

默认安装到 `build/install/` 目录：
- `build/install/lib/<arch>/libodin_sdk.so` - 动态库（<arch> 为 x86_64 或 aarch64）
- `build/install/bin/` - 示例程序
- `build/install/include/` - API 头文件（`odin_lidar_api.h` / `odin_lidar_api_ext.h` / `odin_lidar_def.h` / `odin_lidar_def_ext.h`）

自定义安装路径：
```bash
./build.sh /your/custom/path
# 或
INSTALL_PREFIX=/your/custom/path ./build.sh
```

详细说明（环境要求、OpenSSL 预编译机制、Foxglove Bridge、交叉编译、常见问题）见 [docs/build_linux.md](docs/build_linux.md)。

### Windows

```powershell
# 首次使用需放开脚本执行策略（仅需一次，安全且无需管理员权限）
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# 编译 + 安装（自动定位 VS、导入 MSVC 环境、CMake 构建）
.\build.ps1
```

产物：
- `lib\win64\odin_sdk.dll` + `odin_sdk.lib` - 运行库与导入库
- `build\install\bin\` - DLL 与示例程序 exe
- `build\install\lib\x86_64\odin_sdk.lib` - 安装的导入库
- `build\install\include\` - API 头文件

首次构建时若 `lib\win64\` 无预编译 OpenSSL，会自动从源码编译（需 [Strawberry Perl](https://strawberryperl.com/)，约 20-40 分钟，仅此一次）；产物落回 `lib\win64\` 后，后续构建走预编译快速路径，无需 Perl。

详细说明（参数、手动 CMake 方式、常见问题）见 [docs/build_windows.md](docs/build_windows.md)。

## 目录结构

```
sdk_api/
├── include/                    # 公开头文件
│   ├── odin_lidar_api.h        #   核心 API (连接/回调/流控)
│   ├── odin_lidar_api_ext.h    #   扩展 API (滤波/PTP/网络/算法)
│   ├── odin_lidar_def.h        #   核心类型定义
│   ├── odin_lidar_def_ext.h    #   扩展类型定义
│   ├── internal/               #   内部类型 (协议层/命令层共享)
│   └── logger/                 #   日志 C API
├── sdk/                        # SDK 核心实现
│   ├── src/                    # SDK 主实现
│   ├── device/                 # 设备抽象层
│   ├── discovery/              # 设备发现模块
│   ├── protocol/               # 通信协议
│   ├── transport/              # 传输层（UDP/TCP）
│   ├── hotplugListener/        # 热插拔监听
│   ├── utils/                  # 工具库（网络/HTTP）
│   └── logger/                 # 日志模块
├── algo/                       # 算法插件框架（宿主侧）
│   ├── interface/              # 插件契约头
│   └── plugin_loopclosure/     # 回环插件契约头
├── sample/                     # 示例程序
│   ├── hotplug/                # 热插拔监听示例
│   ├── getCalibration/         # 获取标定文件示例（0x41 通道优先，HTTP 回退）
│   ├── sendCalibration/        # 发送标定文件示例
│   ├── getTelemetry/           # 遥测数据订阅示例（0x42 通道）
│   ├── sendAlgoData/           # 算法下行透传示例（0x40 通道）
│   ├── upgrade/                # 固件升级示例 (HTTPS OTA)
│   └── https_client/           # HTTPS 客户端示例
├── 3rdparty/                   # 第三方库（源码内置，无需预装）
│   ├── openssl/                # OpenSSL（Linux/Windows 源码编译）
│   ├── mongoose/               # HTTP/HTTPS 网络库
│   ├── FastCRC/                # CRC 校验库
│   └── ...                     # tinyxml2 / cJson / eigen3
├── cmake/                      # 跨平台 CMake 公共模块
│   ├── OdinArch.cmake          # 架构归一化（AMD64→win64 等）
│   ├── OdinCompilerFlags.cmake # 编译器分支（MSVC /W4 /utf-8；GCC -Wall...）
│   └── OdinPlatform.cmake      # Windows 系统库链接 + DLL 导出策略
├── toolchains/                 # 交叉编译工具链（linux-aarch64）
├── lib/                        # 按架构分目录的构建产物
│   ├── x86_64/                 # Linux x86_64（libodin_sdk.so）
│   ├── aarch64/                # Linux aarch64（libodin_sdk.so）
│   └── win64/                  # Windows x64（odin_sdk.dll + OpenSSL 预编译库）
├── docs/                       # 文档
│   ├── Architecture.md         #   架构说明 (层级依赖/构建系统/插件开发)
│   └── sdk_api.md              #   API 参考文档
├── build.sh                    # Linux 编译脚本
├── build.ps1                   # Windows 编译脚本
└── CMakeLists.txt              # CMake 配置
```

> 内部模块结构及层级依赖关系详见 [docs/Architecture.md](docs/Architecture.md)。

## SDK 使用流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Application                                 │
└─────────────────────────────────────────────────────────────────────┘
                                  │
        ┌─────────────────────────┴─────────────────────────┐
        ▼                                                   ▼
┌───────────────┐                                   ┌───────────────┐
│ DiscoverDevices│ (主动发现)                        │ HotplugListener│ (热插拔监听)
└───────┬───────┘                                   └───────┬───────┘
        │                                                   │
        └─────────────────────────┬─────────────────────────┘
                                  ▼
                    ┌─────────────────────────┐
                    │    ConnectDevice()      │
                    │  获取设备句柄 (Handle)   │
                    └───────────┬─────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
┌───────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ RegisterXxxCb │     │ SendSetModeCmd  │     │ SendFileToDevice│
│ 注册数据回调   │     │ 设置工作模式     │     │ 文件传输 (HTTPS)│
└───────────────┘     └────────┬────────┘     └─────────────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ kNormal / kStandby  │
                    │   开启/关闭数据流    │
                    └─────────┬───────────┘
                              │
                              ▼
            ┌─────────────────┼─────────────────┐
            ▼                 ▼                 ▼
    ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
    │ PointCloud  │   │ Image/SLAM  │   │  IMU/Odom   │
    │   Callback  │   │  Callback   │   │  Callback   │
    └─────────────┘   └─────────────┘   └─────────────┘
```

## 示例程序

Linux 下构建产物在 `build/sample/<name>/`，Windows 下在 `build\sample\<name>\`（安装后统一在 `build/install/bin/`，Windows 下运行需将 `odin_sdk.dll` 所在目录加入 PATH 或与 exe 同目录）。

### 热插拔监听
```bash
./build/sample/hotplug/odin_hotplug
```
- 自动监听设备上下线事件
- 设备连接后自动开始数据流

### 固件升级 (OTA)
```bash
./build/sample/upgrade/odin_OTA_tool <firmware_path> [--ip <ip>] [--timeout <ms>]
```
- `firmware_path`：固件文件路径 (.bin)
- `--ip`：可选，指定设备 IP（跳过发现阶段）
- `--timeout`：可选，发现超时

### 获取标定文件
```bash
./build/sample/getCalibration/odin_get_calibration <save_path> [--device <ip>] [--timeout <ms>]
```
- `<save_path>`：标定文件保存路径
- 通过 0x41 TCP 通道从设备拉取标定文件，失败时自动回退到 HTTP/HTTPS

### 发送标定文件
```bash
./build/sample/sendCalibration/odin_send_calibration <calibration_file> [--device <ip>]
```

### 遥测数据订阅
```bash
./build/sample/getTelemetry/odin_get_telemetry [--device <ip>]
```
- 启动 0x42 遥测通道，持续接收设备 JSON 遥测数据（约 1 Hz）

### 算法下行透传
```bash
./build/sample/sendAlgoData/odin_send_algo_data [--device <ip>]
```
- 通过 0x40 通道向设备发送算法透传数据

## API 汇总

详见头文件 `include/odin_lidar_api.h` 和 `include/odin_lidar_api_ext.h` 中的注释说明。

### 核心接口 (`odin_lidar_api.h`)

| 函数 | 说明 |
|------|------|
| `DiscoverDevices()` | 广播发现局域网内设备 |
| `ConnectDevice()` / `DisconnectDevice()` | 连接 / 断开设备 |
| `RegisterPointCloudCallback()` | 注册原始点云回调 |
| `RegisterSlamCallback()` | 注册 SLAM 点云回调 |
| `RegisterCameraCallback()` / `RegisterCameraCallback2()` | 注册左/右相机图像回调 |
| `RegisterImuCallback()` | 注册 IMU 回调 |
| `RegisterOdomCallback()` | 注册里程计回调 |
| `RegisterTfCallback()` | 注册 TF 变换回调 |
| `RegisterTelemetryCallback()` | 注册遥测数据回调（0x42 通道）|
| `StartTelemetry()` | 启动遥测通道订阅 |
| `StartStream()` | 启动指定通道数据流 |
| `EnableSlamOdomSyncForDevice()` | 启用/禁用 SLAM-里程计同步 |
| `SetDeviceMode()` / `GetDeviceMode()` | 设置/查询设备工作模式 |
| `SetSensorMode()` / `GetSensorMode()` | 设置/查询传感器工作模式 |
| `GetFirmwareVersion()` | 获取固件版本 |
| `GetDeviceFile()` | 从设备获取文件，**同步阻塞**；标定文件优先走 0x41 TCP 通道，失败回退 HTTP/HTTPS |
| `SendFileToDevice()` | 向设备发送文件（固件升级/标定等，HTTPS 加密）|

### 扩展接口 (`odin_lidar_api_ext.h`)

| 函数 | 说明 |
|------|------|
| `GetChannelConfig()` | 查询当前通道订阅状态 |
| `SetFilterParam()` / `GetFilterParam()` | 设置/查询点云滤波参数 |
| `SendAlgoData()` | 向设备发送算法透传数据（0x40 通道）|

## 数据通道（OdinDataChannel）

数据通道枚举定义在 `odin_lidar_def.h` 的 `OdinDataChannel`：

| 枚举名 | 值 | 方向 | 说明 |
|--------|-----|------|------|
| `kRawPoint` | `0x00` | device→host | 原始点云 |
| `kSlamPoint` | `0x01` | device→host | SLAM 点云 |
| `kImage0` | `0x10` | device→host | 左相机图像（MJPEG / H.264）|
| `kImage1` | `0x11` | device→host | 右相机图像（MJPEG / H.264）|
| `kImu` | `0x20` | device→host | IMU 6轴数据 |
| `kOdomLF` | `0x30` | device→host | 低频里程计 |
| `kOdomHF` | `0x31` | device→host | 高频里程计 |
| `kTf` | `0x32` | device→host | TF 变换 |
| `kAlgoDownlink` | `0x40` | host→device | 算法下行透传 |
| `kCalibFile` | `0x41` | device→host | 标定文件下载 |
| `kTelemetry` | `0x42` | device→host | 遥测 JSON（~1 Hz）|

## 设备插件

SDK 支持通过外部 `.so` 插件扩展定制设备类型。插件按架构分目录存放于 `lib/<arch>/`（与 `libodin_sdk.so` 同目录），运行时由 `DeviceLoader` 自动发现加载。

插件开发指南及搜索路径详情见 [docs/Architecture.md](docs/Architecture.md)。

## 安全特性

- **mTLS 双向认证**：客户端和设备端相互验证证书
- **文件签名**：固件和重要文件使用 Ed25519 签名验证
- **加密传输**：所有文件传输通过 HTTPS 加密
