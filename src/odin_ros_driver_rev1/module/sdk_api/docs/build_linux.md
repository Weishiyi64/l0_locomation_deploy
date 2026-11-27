# Linux 编译指南

Odin SDK 在 Linux（x86_64 / aarch64）上的编译、安装与使用说明。Windows 编译见 [build_windows.md](build_windows.md)。

## 1. 环境要求

| 组件 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 18.04+ / Debian 10+ / 其他主流发行版 | x86_64 或 aarch64 |
| GCC | 7+ (支持 C++17) | `sudo apt install build-essential` |
| CMake | 3.10+ | `sudo apt install cmake` |
| OpenSSL | 1.1+ 开发库 | `sudo apt install libssl-dev`（仅编译 SDK 时需要） |
| Git | 任意版本 | `sudo apt install git` |

### 一键安装依赖（Ubuntu / Debian）

```bash
sudo apt update
sudo apt install -y build-essential cmake libssl-dev git
```

### 一键安装依赖（CentOS / RHEL）

```bash
sudo yum groupinstall -y "Development Tools"
sudo yum install -y cmake openssl-devel git
```

## 2. 快速开始

```bash
cd /path/to/sdk_api

# 编译 + 安装
./build.sh
```

`build.sh` 自动完成：CMake configure → build → install（默认安装到 `build/install/`）。

### build.sh 参数

```bash
./build.sh                              # Release 构建，安装到 build/install/
./build.sh /usr/local                    # 自定义安装路径
INSTALL_PREFIX=/opt/odin ./build.sh      # 通过环境变量指定安装路径
BUILD_TYPE=Debug ./build.sh             # Debug 构建
```

## 3. 构建产物

| 路径 | 内容 |
|------|------|
| `lib/<arch>/libodin_sdk.so` | SDK 运行库（与 Windows 的 odin_sdk.dll 对应） |
| `build/install/bin/` | 安装的示例程序 |
| `build/install/lib/` | 安装的 SDK 库 |
| `build/install/include/` | 安装的头文件 |

其中 `<arch>` 为 `x86_64` 或 `aarch64`（由系统架构自动检测）。

### 在自己的工程中使用

1. 头文件：`include/`（或安装后的 `<prefix>/include/`）
2. 链接：`lib/<arch>/libodin_sdk.so`
3. 运行：`lib/<arch>/` 需在 `LD_LIBRARY_PATH` 中，或安装到系统库目录

CMake 示例：

```cmake
# 你的工程 CMakeLists.txt
target_include_directories(my_app PRIVATE "/path/to/sdk_api/include")
target_link_libraries(my_app PRIVATE "/path/to/sdk_api/lib/x86_64/libodin_sdk.so")
# 运行时：export LD_LIBRARY_PATH=/path/to/sdk_api/lib/x86_64:$LD_LIBRARY_PATH
```

## 4. OpenSSL 编译机制

Linux 上 OpenSSL 通过 `3rdparty/openssl/` 源码编译（ExternalProject），流程由 CMake 自动管理：

```
lib/<arch>/ 中已存在 libssl.a + libcrypto.a + include/openssl/ ？
├── 是 → 直接链接（快速路径，几分钟完成整个 SDK 构建）
└── 否 → 源码编译 OpenSSL（./config + make，约 10-20 分钟）
         └── 产物自动拷贝到 lib/<arch>/，下次走快速路径
```

**建议**：首次构建完成后，将 `lib/<arch>/` 下的 `libssl.a`、`libcrypto.a`、`include/` 一并提交到版本库，其他人和 CI 克隆后直接走预编译路径。

## 5. 手动 CMake 构建（可选）

适用于不使用 build.sh 的场景：

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=build/install \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build --parallel
cmake --install build
```

### Debug 构建

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --parallel
```

### 只编译 SDK（跳过示例程序）

```bash
cmake -S . -B build -DODIN_BUILD_SAMPLES=OFF
cmake --build build --parallel
cmake --install build
```

## 6. Foxglove Bridge 示例

foxglove_bridge 示例需要额外下载 Foxglove SDK 预编译库。如不需要此示例，构建会自动跳过，不影响其他组件。

### 下载 Foxglove SDK

```bash
# 方法一：PowerShell 脚本（自动检测平台）
pwsh ./sample/foxglove_bridge/download_foxglove_sdk.ps1

# 方法二：指定平台
pwsh ./sample/foxglove_bridge/download_foxglove_sdk.ps1 x86_64    # Linux x64
pwsh ./sample/foxglove_bridge/download_foxglove_sdk.ps1 aarch64   # Linux ARM64

# 方法三：手动下载（无 PowerShell 时）
# 从 https://github.com/foxglove/foxglove-sdk/releases/tag/sdk/v0.27.0 下载：
#   x86_64:  foxglove-v0.27.0-cpp-x86_64-unknown-linux-gnu.zip
#   aarch64: foxglove-v0.27.0-cpp-aarch64-unknown-linux-gnu.zip
# 解压到 3rdparty/foxglove-sdk/x86_64/ 或 3rdparty/foxglove-sdk/aarch64/
```

SDK 就位后正常执行 `./build.sh` 即可编译 foxglove_bridge。

### 运行

```bash
./build/install/bin/odin_foxglove_bridge [port]
```

- 默认端口 `8765`
- 程序启动后自动发现并连接 Odin 设备（同网段 UDP 广播）
- 设备连接后自动启动所有数据流
- 在 Foxglove Studio 中连接 `ws://localhost:8765` 即可查看

## 7. aarch64 交叉编译

仓库附带交叉编译工具链文件 `toolchains/linux-aarch64.cmake`，在 x86_64 主机上编译 aarch64 目标：

```bash
# 安装 aarch64 交叉编译工具链
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 使用工具链文件编译
cmake -S . -B build-arm \
    -DCMAKE_TOOLCHAIN_FILE=toolchains/linux-aarch64.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build-arm --parallel
cmake --install build-arm
```

## 8. 常见问题

### 编译报 "cannot find -lssl" 或 "cannot find -lcrypto"

OpenSSL 开发库未安装：

```bash
sudo apt install libssl-dev    # Ubuntu / Debian
sudo yum install openssl-devel # CentOS / RHEL
```

或检查 `lib/<arch>/` 下是否有预编译的 `libssl.a` + `libcrypto.a`。

### 运行示例报 "error while loading shared libraries: libodin_sdk.so"

动态链接器找不到 SDK 共享库。三种解决方式：

```bash
# 方法一：设置 LD_LIBRARY_PATH（临时）
export LD_LIBRARY_PATH=/path/to/sdk_api/lib/x86_64:$LD_LIBRARY_PATH

# 方法二：安装到系统库目录
sudo ./build.sh /usr/local
sudo ldconfig

# 方法三：使用安装后的目录（已将 so 和 exe 放在一起）
./build/install/bin/odin_foxglove_bridge
```

### 点云 UDP 丢包（PadZerosForTailLoss 警告）

Linux 默认 UDP 接收缓冲区较小，导致大点云帧丢包。增大内核缓冲区：

```bash
# 临时生效（重启后失效）
sudo sysctl -w net.core.rmem_max=16777216     # 16MB
sudo sysctl -w net.core.rmem_default=8388608  #  8MB
sudo sysctl -w net.core.netdev_max_backlog=50000

# 永久生效：追加到 /etc/sysctl.conf
echo "net.core.rmem_max=16777216"     | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_default=8388608"  | sudo tee -a /etc/sysctl.conf
echo "net.core.netdev_max_backlog=50000" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 设备发现失败（跨网段）

Odin SDK 的 UDP 广播发现机制要求设备与主机在**同一网段**。跨网段场景请直接使用设备 IP 连接（参见 SDK API 文档中的直连方式）。

### configure 报 "generator: Ninja does not match"

`build/` 目录中缓存了其它生成器的配置。删除 `build/` 目录后重试：

```bash
rm -rf build
./build.sh
```

### 编译报 "fatal error: cstdint: No such file or directory"

GCC 版本过低，不支持 C++17 标准库。升级 GCC：

```bash
# Ubuntu: 安装 GCC 9+
sudo apt install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
```
