# Windows 编译指南

Odin SDK 在 Windows（x64）上的编译、安装与使用说明。Linux 编译见 [build_linux.md](build_linux.md)。

## 1. 环境要求

| 组件 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Windows 10 / 11 x64 | |
| Visual Studio | 2022 或更新版本 | 需勾选「使用 C++ 的桌面开发」工作负载（含 MSVC 编译器） |
| CMake | 3.20+ | VS 自带的 CMake 即可；单独安装需加入 PATH |
| Ninja | 可选 | VS 和 CMake 均自带；有则用 Ninja（快），无则回退 VS 生成器 |
| Strawberry Perl | 仅源码重编 OpenSSL 时需要 | 正常情况（仓库已含 `lib\win64\` 预编译库）无需安装；仅当预编译库缺失、需要从源码重建 OpenSSL 时使用，[下载地址](https://strawberryperl.com/) |

OpenSSL 无需单独安装：SDK 内置 `3rdparty\openssl\` 源码，首次构建自动编译为静态库，之后走预编译快速路径（见第 4 节）。

## 2. 快速开始

```powershell
cd C:\work\odin2\sdk_api

# 第一次使用 .ps1 脚本需要放开执行策略（每台机器只需一次，无需管理员权限）
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# 编译 + 安装
.\build.ps1
```

`build.ps1` 自动完成：定位 Visual Studio（vswhere）→ 导入 x64 编译环境（vcvarsall）→ CMake configure / build / install。

**不想改执行策略**也可以一次性绕过：

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1
```

### build.ps1 参数

```powershell
.\build.ps1                              # Release 构建，安装到 build\install\
.\build.ps1 -BuildType Debug             # Debug 构建
.\build.ps1 -InstallPrefix "D:\odin"     # 自定义安装路径
.\build.ps1 -Clean                       # 删除 build\ 后全量重建
```

## 3. 构建产物

| 路径 | 内容 |
|------|------|
| `lib\win64\odin_sdk.dll` | SDK 运行库（与 Linux 的 libodin_sdk.so 对应） |
| `lib\win64\odin_sdk.lib` | 导入库（链接时使用） |
| `build\sample\<name>\*.exe` | 示例程序（Ninja）；VS 生成器在 `build\sample\<name>\Release\` |
| `build\install\bin\` | 安装的 DLL + 示例 exe |
| `build\install\lib\x86_64\odin_sdk.lib` | 安装的导入库 |
| `build\install\include\` | 安装的头文件 |

### 在自己的工程中使用

1. 头文件：`include\`（或安装后的 `<prefix>\include\`）
2. 链接：`odin_sdk.lib`
3. 运行：`odin_sdk.dll` 需与 exe 同目录，或将 `lib\win64\` 加入 `PATH`

CMake 示例：

```cmake
# 你的工程 CMakeLists.txt
target_include_directories(my_app PRIVATE "C:/work/odin2/sdk_api/include")
target_link_libraries(my_app PRIVATE "C:/work/odin2/sdk_api/lib/win64/odin_sdk.lib")
# 运行时把 odin_sdk.dll 拷到 exe 目录，或在 PATH 中包含其所在目录
```

## 4. OpenSSL 预编译机制（首次 vs 日常构建）

构建时按以下顺序决策：

```
lib\win64\ 中已存在 libssl.lib + libcrypto.lib + include\openssl\ ？
├── 是 → 直接链接（快速路径，几分钟完成整个 SDK 构建）
└── 否 → 源码编译 OpenSSL（VC-WIN64A + nmake，需 Perl，约 20-40 分钟）
         └── 产物自动拷贝到 lib\win64\，下次走快速路径
```

**建议**：首次构建完成后，将 `lib\win64\` 下的 `libssl.lib`、`libcrypto.lib`、`include\` 一并提交到版本库。这样其他人（以及 CI）克隆后直接构建，无需安装 Perl。`.gitignore` 已配置为只忽略 `lib/*/libodin_sdk.*` 等 SDK 自身产物，不会忽略这些预编译库。两个静态库合计约 16MB（已剔除调试信息；若自己重编时发现接近 42MB，说明 Makefile 的 `/Zi` 剥离补丁未生效）。

## 5. 手动 CMake 构建（可选）

适用于不使用 build.ps1、或已在「Developer PowerShell for VS」中的场景（该环境下 cl.exe 已在 PATH，无需 vcvarsall）：

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=build\install
cmake --build build --parallel
cmake --install build
```

无 Ninja 时去掉 `-G Ninja`（使用 Visual Studio 多配置生成器），构建和安装时需加 `--config Release`：

```powershell
cmake -S . -B build
cmake --build build --config Release --parallel
cmake --install build --config Release
```

注意：同一 `build\` 目录切换生成器（如 Ninja ↔ VS）会报错，删除 `build\` 重新 configure 即可。

## 6. 常见问题

### 运行 .\build.ps1 报「在此系统上禁止运行脚本」

PowerShell 默认执行策略为 Restricted，禁止运行任何 .ps1 文件。解决：

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

或单次绕过：`powershell -ExecutionPolicy Bypass -File .\build.ps1`。这是 Windows 系统级安全设置，与脚本内容无关。

### 首次构建提示需要 Perl

OpenSSL 的构建脚本（Configure）由 Perl 编写。三种解决方式：

1. 恢复预编译库（推荐，若仓库已提交）：`git checkout -- lib\win64` 后重试，直接走预编译路径
2. 安装 [Strawberry Perl](https://strawberryperl.com/)（装完重开终端重试）
3. 从已构建过的机器拷贝 `lib\win64\`（含 `libssl.lib`、`libcrypto.lib`、`include\`）到本机同样路径，直接走预编译路径

### 构建路径含空格

OpenSSL 源码编译分支要求源码路径不含空格（如 `C:\Users\John Doe\...` 会失败）。建议将仓库放在无空格路径下（如 `C:\work\`）。

### configure 报「generator: Ninja does not match」

`build\` 目录中缓存了其它生成器的配置。删除 `build\` 目录（或使用 `.\build.ps1 -Clean`）后重试。

### MSB 错误 / cl.exe 找不到

通常是 VS 安装不完整。确认已安装「使用 C++ 的桌面开发」工作负载；或在「Developer PowerShell for VS」中直接运行 cmake 命令（见第 5 节）。

### 运行示例 exe 提示找不到 odin_sdk.dll

将 `lib\win64\odin_sdk.dll` 拷贝到 exe 同目录，或将 `lib\win64\` 加入 `PATH`。安装后的 `build\install\bin\` 已将两者放在一起，可直接运行。

## 7. 附：Linux aarch64 交叉编译

仓库另附交叉编译工具链文件 `toolchains\linux-aarch64.cmake`（在 Linux 主机上使用，非 Windows）：

```bash
cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=toolchains/linux-aarch64.cmake
cmake --build build-arm --parallel
```

Windows 平台的构建体系说明（cmake/ 模块分工）见 [README](../README.md) 的目录结构一节。
