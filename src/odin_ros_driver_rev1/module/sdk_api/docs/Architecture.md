# Odin SDK Architecture

> SDK Version: 3.4.0 | Build Date: 20260827

## 1. 目录结构

```
sdk_api/
├── include/                      # 公开头文件 (Public Headers)
│   ├── odin_lidar_api.h          #   核心 C++ API (连接/回调/流控)
│   ├── odin_lidar_api_ext.h      #   扩展 C++ API (滤波/PTP/网络/算法)
│   ├── odin_lidar_def.h          #   核心类型定义
│   ├── odin_lidar_def_ext.h      #   扩展类型定义
│   ├── internal/
│   │   └── odin_lidar_inner.h    #   内部类型 (协议层/命令层共享)
│   └── logger/
│       └── logger.h             #   日志 C API
├── sdk/                          # SDK 实现 (Private)
│   ├── CMakeLists.txt            #   顶层 CMake — 合并所有 OBJECT 库为 libodin_sdk.so
│   ├── odin_sdk.version          #   链接器版本脚本 (符号导出白名单)
│   ├── src/                      #   SDK 入口层
│   │   ├── odin_sdk.cpp          #     C API 封装 (odin_* C 入口)
│   │   ├── OdinSdkImpl.h/.cpp    #     核心实现 (单例, 设备管理, 调度)
│   ├── device/                   #   设备层 (IDevice, 每个设备独立 CMakeLists)
│   │   ├── IDevice/IDevice.h     #     设备抽象接口
│   │   ├── DeviceLoader.h/.cpp   #     设备加载器 (内置注册 + dlopen 插件)
│   │   ├── DevicePluginEntry.h   #     外部插件 C 入口契约
│   │   ├── OdinProtocolV2Base/   #     V2 协议设备基类 (内置)
│   │   └── Odin1Lite/            #     Odin1Lite 设备实现 (内置)
│   ├── command/                  #   命令层 (ICommand)
│   │   ├── ICommand.hpp          #     命令抽象接口
│   │   └── odin2Command/         #     Odin2 命令实现
│   │       ├── OdinCommand.h/.cpp      #       命令主类 (TLV + HTTP)
│   │       └── internal/
│   │           ├── odin_command_channel.h/.cpp  # TLV 命令通道
│   │           ├── http_config_backend.h/.cpp  # HTTP 配置后端
│   │           └── http_file_backend.h/.cpp     # HTTP 文件传输后端
│   ├── protocol/                 #  协议层 (IProtocol)
│   │   ├── IProtocol/IProtocol.hpp     #    抽象包接口 (IPacket)
│   │   └── OdinProtocol/               #    Odin 协议实现
│   │       ├── OdinProtocol.h/.cpp          #      协议编解码
│   │       ├── OdinFrameAssembler.h/.cpp    #      帧组装/拆包
│   │       ├── OdinPointDecoder.hpp         #      点云解码
│   │       ├── ChunkReassembler.hpp         #      分块重组
│   │       ├── OdomLayoutRouter.hpp         #      里程计布局路由
│   │       ├── OdomGenerations.def          #      里程计字段定义 (V2/V3)
│   │       ├── V2/OdomEntry.hpp             #      V2 里程计入口
│   │       └── V3/OdomEntry.hpp            #      V3 里程计入口
│   ├── transport/                #  传输层 (ITransport)
│   │   ├── ITransport/ITransport.hpp  #    抽象传输接口
│   │   └── Network/
│   │       ├── TcpTransport.h/.cpp    #      TCP 传输实现
│   │       └── UdpTransport.h/.cpp    #      UDP 传输实现
│   ├── capture/                  #  数据捕获层 (ICapture)
│   │   ├── ICapture.hpp          #    捕获抽象接口
│   │   └── odinCapture/
│   │       └── OdinCaptureBase.h/.cpp  #    捕获基类 (传输→帧组装→回调)
│   ├── discovery/                #  设备发现
│   │   └── OdinUdpDiscovery/     #    UDP 发现实现
│   ├── hotplugListener/         #  热插拔监听
│   │   ├── IHotplugListener/    #    抽象接口
│   │   ├── UdpHotplugListener/  #    UDP 实现
│   │   ├── CompositeHotplugListener/ # 组合实现
│   │   └── HotplugListenerFactory.h/.cpp # 工厂
│   ├── utils/
│   │   ├── network_utils.h/.cpp      #    网络工具 (IP/接口查询)
│   │   ├── http/                     #    HTTP 客户端 (mongoose + OpenSSL)
│   │   ├── payload_builder/          #    TLV 负载构建器
│   │   └── slam_odom_synchronizer/    #    SLAM-里程计时间同步
│   └── logger/
│       └── logger.cpp               #    日志实现
├── 3rdparty/                     # 第三方依赖
│   ├── openssl/                  #   静态 OpenSSL (ExternalProject)
│   ├── mongoose/                 #   嵌入式 HTTP 服务器
│   ├── cJson/                    #   JSON 解析
│   ├── tinyxml2/                 #   XML 解析
│   ├── eigen3/                   #   线性代数 (header-only)
│   └── FastCRC/                  #   CRC 硬件/软件加速
├── lib/                          # 编译输出 (libodin_sdk.so)
├── sample/                       # 示例程序
└── docs/                         # 文档
```

## 2. 层级依赖关系

```
┌─────────────────────────────────────────────────────────────────┐
│                    Public API (include/)                         │
│  odin_lidar_api.h / odin_lidar_api_ext.h                         │
│  odin_lidar_def.h / odin_lidar_def_ext.h                         │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│                   SDK 入口层 (sdk/src/)                          │
│  odin_sdk.cpp ──► OdinSdkImpl (单例)                              │
│  职责: 设备发现/连接/断开, 回调注册, 命令转发, 热插拔管理          │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│                   设备层 (sdk/device/)                           │
│  IDevice (抽象接口)                                              │
│  ├─ OdinProtocolV2BaseDevice (V2 协议基类)                       │
│  │   ├─ Odin1LiteDevice                                         │
│  │   └─ Odin2SpecialDevice                                      │
│  DeviceLoader (内置注册 + dlopen 插件加载)                       │
│  职责: 业务逻辑编排, 设备生命周期管理, 心跳, SLAM-里程计同步     │
└──────┬───────────┬──────────────┬───────────────────────────────┘
       │           │              │
┌──────▼─────┐ ┌──▼───────────┐ ┌▼───────────────────────────────┐
│  命令层    │ │  捕获层      │ │  工具层                        │
│ (command/) │ │ (capture/)   │ │ (utils/)                      │
│            │ │              │ │                                │
│ ICommand   │ │ ICapture     │ │ SlamOdomSynchronizer           │
│ OdinCommand│ │ OdinCapture  │ │ network_utils                  │
│ ├ TLV 通道 │ │ ├ 传输接收  │ │ payload_builder               │
│ ├ HTTP配置 │ │ ├ 帧组装    │ │                                │
│ └ HTTP文件 │ │ └ 回调分发  │ │                                │
└──────┬─────┘ └──────┬───────┘ └────────────────────────────────┘
       │              │
       │    ┌─────────▼──────────────────────────┐
       │    │       协议层 (protocol/)            │
       │    │       IPacket / IProtocol            │
       │    │       OdinProtocol (编解码)           │
       │    │       OdinFrameAssembler (帧组装)    │
       │    │       OdinPointDecoder (点云解码)    │
       │    └─────────┬──────────────────────────┘
       │              │
┌──────▼──────────────▼──────────────────────────┐
│              传输层 (transport/)                │
│              ITransport (抽象接口)              │
│              ├ TcpTransport                    │
│              └ UdpTransport                    │
└────────────────────────────────────────────────┘
```

**依赖方向: 自上而下，单向**

- 上层依赖下层接口，下层不感知上层
- `IDevice` 编排 `ICommand` + `ICapture`，二者独立
- `ICommand` 内部使用 `ITransport` 发送命令、`IProtocol` 编解码、HTTP 后端传输文件
- `ICapture` 内部使用 `ITransport` 接收数据、`OdinFrameAssembler` 组装帧

## 3. 构建系统

### 3.1 OBJECT 库合并模型

所有子模块编译为 CMake OBJECT 库，最终合并为单一共享库 `libodin_sdk.so`:

```
libodin_sdk.so = odin_sdk.cpp
               + OdinSdkImpl.cpp
               + logger.cpp
               + $<TARGET_OBJECTS:odin_protocol_obj>
               + $<TARGET_OBJECTS:odin_transport_obj>
               + $<TARGET_OBJECTS:odin_command_obj>
               + $<TARGET_OBJECTS:odin_device_obj>
               + $<TARGET_OBJECTS:odin_ialgo>
               + $<TARGET_OBJECTS:http_transfer_obj>
               + $<TARGET_OBJECTS:mongoose>
               + SlamOdomSynchronizer.cpp
               + HotplugListener*.cpp
               + OdinUdpDiscovery.cpp
               + network_utils.cpp
               + payload_builder.cpp
               + tinyxml2.cpp + cJSON.c
```

**优点**: 外部消费者只需链接 `libodin_sdk.so`，无需管理多个 `.so`。

### 3.2 符号导出控制

`odin_sdk.version` 链接器版本脚本使用正向白名单:

- `odin::sdk::*` — C++ 公共 API (通过 mangled name 前缀 `_ZN4odin3sdk*` 匹配)
- `Odin*` / `odin_*` — C 入口点
- `log_config` / `logger_*` 等 — 固定 logger C API
- 其余符号 (含 STL 弱符号) 一律 `local` 隐藏

**目的**: 防止 STL 模板实例化弱符号 (如 `std::__future_base::*`) 泄漏到动态符号表，避免消费者绑定到 SDK 内部副本导致崩溃。

### 3.3 静态依赖

- **OpenSSL** — 静态链接 (`3rdparty/openssl/`, ExternalProject 编译为 `libssl.a` + `libcrypto.a`)，运行时无 OpenSSL 依赖
- **mongoose** — 编译为 OBJECT 库合并进 `libodin_sdk.so`
- **Eigen3** — header-only, `-isystem` 抑制警告
- **cJSON / tinyxml2 / FastCRC** — 源码直接编译

## 4. 关键设计

### 4.1 设备插件架构

```
DeviceLoader::Load(name)
  ├─ 1. 查找内置注册表 (ODIN_REGISTER_BUILTIN_DEVICE 宏注册)
  │     └─ Odin1Lite 等
  ├─ 2. dlopen 外部 .so 插件 (按请求型号名)
  │     ├─ odin_device_abi_version() — ABI 版本校验
  │     ├─ odin_device_supported_models() — 支持型号检查
  │     └─ odin_device_create() — 创建 IDevice 实例
  └─ 3. 回退内置 Odin1Lite (默认设备, 单摄像头, 始终可用)
```

- **内置设备**: 编译进 `libodin_sdk.so`，通过静态初始化注册 (如 Odin1Lite)
- **插件设备**: 独立 `.so`，通过 `dlopen` 加载，ABI 版本校验 (如 Odin2Special:
  `libodin-device-Odin2Special.so`，构建输出到 `lib/<arch>/`，
  与 `libodin_sdk.so` 同目录，仅在型号名匹配时加载)
- **默认设备**: 未匹配到型号时回退内置 Odin1Lite（单摄像头，
  `kImage1`/`RegisterCameraCallback2` 不可用）
- **插件契约**: `DevicePluginEntry.h` 定义 C 入口函数

### 4.2 命令层静态编译 (v3.5+)

命令层 (`ICommand` / `OdinCommand`) 及其底层依赖 (protocol, transport, HTTP, mongoose, OpenSSL) 全部以 OBJECT 库形式编译进 `libodin_sdk.so`。

**不再单独构建 `libodin_command.so`**。外部设备插件需将 command 及底层依赖源码直接编译进插件 `.so`，实现自包含部署。

**优点**:
- 部署简化: 只需 `libodin_sdk.so` + 可选的插件 `.so`
- 版本一致: 插件与 SDK 源码同步编译，无 ABI 不匹配风险
- 消除符号导出管理: 不再需要 `odin_command.exports` 版本脚本

### 4.3 ICommand 接口

`ICommand` 是设备控制的统一抽象层，具体实现 (`OdinCommand`) 内部可同时使用多种后端:

- **TLV 命令通道** (`odin_command_channel`): UDP/TCP 上的二进制 TLV 协议，用于运行时控制
- **HTTP 配置后端** (`http_config_backend`): HTTP REST API，用于网络配置管理
- **HTTP 文件后端** (`http_file_backend`): HTTPS 文件传输，用于 OTA / 标定 / 日志

`IDevice` 持有 `std::unique_ptr<ICommand>`，所有控制操作通过此接口下发。

### 4.4 数据流路径

```
设备 ──(UDP/TCP)──► ITransport ──► ICapture ──► OdinFrameAssembler ──► 回调
                                     │
                                     ├─► 点云 (OdinPointDecoder)
                                     ├─► SLAM
                                     ├─► 图像 (MJPEG/H264/H265/NV12)
                                     ├─► IMU
                                     ├─► 里程计 (OdomLayoutRouter → V2/V3 OdomEntry)
                                     ├─► TF
                                     └─► 遥测 (JSON)
```

`ICapture` 负责从 `ITransport` 接收原始数据，经 `OdinFrameAssembler` 组装为完整帧，再按数据通道类型分发到用户注册的回调。

### 4.5 SLAM-里程计同步

`SlamOdomSynchronizer` 在 `OdinProtocolV2BaseDevice` 中可选启用，将高频里程计与低频 SLAM 位姿进行时间对齐，输出同步后的里程计/TF 数据。

### 4.6 热插拔监听

`IHotplugListener` 抽象接口 + `CompositeHotplugListener` (组合多个监听器) + `UdpHotplugListener` (UDP 广播监听)。由 `HotplugListenerFactory` 创建，`OdinSdkImpl` 管理。

## 5. 外部插件开发指南

### 5.1 插件 .so 结构

外部设备插件是一个独立的 `.so`，需导出 `DevicePluginEntry.h` 中定义的 C 函数:

```cpp
// my_device_plugin.cpp
#include "DevicePluginEntry.h"
#include "OdinCommand.hpp"       // 静态编译 command 源码
#include "OdinProtocol.hpp"      // 静态编译 protocol 源码
// ... 其他底层依赖

extern "C" {

odin::sdk::IDevice* odin_device_create(odin::sdk::OdinDeviceHandle handle) {
    return new MyDevice(handle);
}

void odin_device_destroy(odin::sdk::IDevice* device) {
    delete device;
}

uint32_t odin_device_abi_version() {
    return ODIN_DEVICE_ABI_VERSION;
}

const char* odin_device_supported_models() {
    return "MyCustomDevice";
}

}  // extern "C"
```

### 5.2 插件 CMake 模板

```cmake
add_library(my_device_plugin SHARED my_device_plugin.cpp
    # 静态编译 command + 底层依赖源码
    ${SDK_SOURCE_DIR}/sdk/command/odin2Command/OdinCommand.cpp
    ${SDK_SOURCE_DIR}/sdk/command/odin2Command/internal/http_config_backend.cpp
    ${SDK_SOURCE_DIR}/sdk/command/odin2Command/internal/http_file_backend.cpp
    ${SDK_SOURCE_DIR}/sdk/command/odin2Command/internal/odin_command_channel.cpp
    ${SDK_SOURCE_DIR}/sdk/protocol/OdinProtocol/OdinProtocol.cpp
    ${SDK_SOURCE_DIR}/sdk/protocol/OdinProtocol/OdinFrameAssembler.cpp
    ${SDK_SOURCE_DIR}/sdk/transport/Network/TcpTransport.cpp
    ${SDK_SOURCE_DIR}/sdk/transport/Network/UdpTransport.cpp
    ${SDK_SOURCE_DIR}/sdk/utils/http/http_client.cpp
    # ... mongoose, FastCRC 等
)
target_include_directories(my_device_plugin PRIVATE
    ${SDK_SOURCE_DIR}/sdk/command
    ${SDK_SOURCE_DIR}/sdk/command/odin2Command
    ${SDK_SOURCE_DIR}/sdk/protocol/IProtocol
    ${SDK_SOURCE_DIR}/sdk/protocol/OdinProtocol
    ${SDK_SOURCE_DIR}/sdk/transport/ITransport
    ${SDK_SOURCE_DIR}/sdk/transport/Network
    ${SDK_SOURCE_DIR}/sdk/utils/http
    ${SDK_SOURCE_DIR}/include
    # ... 其他 include 路径
)
```

### 5.3 插件部署

插件 `.so` 是架构特定的 ELF 二进制，需按架构分目录存放。`DeviceLoader` 按以下顺序搜索:

1. `DeviceLoadRequest::explicit_path` — 调用方指定完整路径
2. `lib/<arch>/` — 编译时内置默认路径，与 `libodin_sdk.so` 同目录 (推荐)
3. `$ODIN_DEVICE_LIB_PATH/` — 环境变量指定目录
4. `$ODIN_DEVICE_LIB_PATH/<arch>/` — 环境变量 + 架构子目录
5. `/usr/lib/odin-device/` — 系统安装路径
6. `/usr/local/lib/odin-device/` — 本地安装路径
7. `ld.so` 搜索路径 — 使用裸 `.so` 文件名

**推荐方式**: 将插件 `.so` 放入 `lib/<arch>/`（与 `libodin_sdk.so` 同目录），例如:

```
lib/
├── x86_64/
│   ├── libodin_sdk.so
│   └── libOdinMyCustomDevice.so
└── aarch64/
    ├── libodin_sdk.so
    └── libOdinMyCustomDevice.so
```

此路径在编译时通过 `ODIN_DEVICE_PLUGIN_DIR` 宏烧入 `libodin_sdk.so`，无需配置环境变量即可自动发现插件。
