# Odin2Special 定制设备插件

Odin2Special 是 Odin SDK 的定制设备实现，以 **dlopen 插件**（`libodin-device-Odin2Special.so`）形式构建，不编译进 `libodin_sdk.so`。本仓库作为 `sdk_api` 的 git submodule 挂载于 `sdk/device/Odin2Special/`。

## 设备特性

- 继承 `OdinProtocolV2BaseDevice`（V2 协议基类）
- 双摄像头设备：`GetCameraCount() = 2`，支持 `kImage0` / `kImage1` 通道与 `RegisterCameraCallback2`

## 构建

由父仓库 `sdk_api` 统一构建，本目录自带 `CMakeLists.txt`：

```bash
# 在 sdk_api 根目录
bash build.sh                                        # 默认构建（含本插件）
cmake -B build -DODIN_BUILD_DEVICE_ODIN2SPECIAL=OFF  # 显式关闭本插件
```

- **编译开关**：`ODIN_BUILD_DEVICE_ODIN2SPECIAL`（默认 `ON`），在本目录 `CMakeLists.txt` 内声明
- **未检出 submodule**：父级 `add_subdirectory` 有 `EXISTS` 守卫，构建自动跳过，不会报错
- **输出**：`lib/<arch>/libodin-device-Odin2Special.so`，与 `libodin_sdk.so` 同目录（编译进 SDK 的默认插件搜索路径 `ODIN_DEVICE_PLUGIN_DIR`）
- **RPATH**：`$ORIGIN`，运行时从同目录解析 `libodin_sdk.so`

## 插件契约

实现 `DevicePluginEntry.h` 定义的 C 入口（见 `Odin2SpecialDevice.cpp`）：

| 函数 | 作用 |
|------|------|
| `odin_device_create(handle)` | 创建 `IDevice` 实例 |
| `odin_device_destroy(device)` | 销毁实例 |
| `odin_device_abi_version()` | 返回 `ODIN_DEVICE_ABI_VERSION`，加载时校验 |
| `odin_device_supported_models()` | 返回 `"Odin2Special"`，型号匹配检查 |

运行时由 `DeviceLoader::LoadFromSo` 按型号名 `dlopen` 加载；`odin::sdk::*` 符号从 `libodin_sdk.so` 解析（`odin_sdk.version` 白名单覆盖）。

## 仓库管理

```bash
# 修改本仓库代码后（在 sdk/device/Odin2Special/ 内）
git checkout master        # submodule 默认处于 detached HEAD
git add -A && git commit -m "..."
git push origin master

# 回到 sdk_api 更新 gitlink 指针
cd <sdk_api 根目录>
git add sdk/device/Odin2Special
git commit -m "chore<device>: bump Odin2Special submodule"
```
