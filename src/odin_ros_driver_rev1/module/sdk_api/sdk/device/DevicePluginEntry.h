#pragma once

#include "IDevice.h"

#define ODIN_DEVICE_ABI_VERSION 1

// Export macro for the plugin entry points below.
//   POSIX        → visibility("default") (export the symbols)
//   Windows/MSVC → plugin side defines ODIN_DEVICE_PLUGIN → dllexport;
//                  host side stays empty (host resolves via GetProcAddress,
//                  never links the plugin's import library)
// Same pattern as ODIN_ALGO_EXPORT in algo/plugin_loopclosure/iloopclosure_algo.h.
#ifndef ODIN_DEVICE_PLUGIN_API
#if defined(_WIN32)
#if defined(ODIN_DEVICE_PLUGIN)
#define ODIN_DEVICE_PLUGIN_API __declspec(dllexport)
#else
#define ODIN_DEVICE_PLUGIN_API
#endif
#else
#define ODIN_DEVICE_PLUGIN_API __attribute__((visibility("default")))
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

ODIN_DEVICE_PLUGIN_API odin::sdk::IDevice* odin_device_create(odin::sdk::OdinDeviceHandle handle);

ODIN_DEVICE_PLUGIN_API void odin_device_destroy(odin::sdk::IDevice* device);

ODIN_DEVICE_PLUGIN_API uint32_t odin_device_abi_version();

ODIN_DEVICE_PLUGIN_API const char* odin_device_supported_models();

ODIN_DEVICE_PLUGIN_API const char* odin_device_version();

#ifdef __cplusplus
}
#endif
