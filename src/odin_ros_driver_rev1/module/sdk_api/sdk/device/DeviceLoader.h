#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "IDevice.h"
#include "DevicePluginEntry.h"

namespace odin {
namespace sdk {

using DeviceCreateFn = std::unique_ptr<IDevice> (*)(OdinDeviceHandle);

class DeviceLoader {
 public:
  struct DeviceLoadRequest {
    std::string name;
    std::string explicit_path;
  };

  static DeviceLoader& Instance();

  std::unique_ptr<IDevice> Load(const DeviceLoadRequest& req,
                                OdinDeviceHandle handle,
                                std::string* error = nullptr);

  // True when a device implementation exists for name: either a builtin
  // registration or a plugin file on the search paths. Plugin candidates are
  // only checked for existence — nothing is dlopen'ed, so the query is free
  // of plugin side effects and safe to call during discovery.
  //
  // Known limitation: this approximates dlopen's search and cannot see a
  // per-call DeviceLoadRequest::explicit_path. A device whose plugin is only
  // reachable via explicit_path reports unknown here.
  bool IsKnownDevice(const std::string& name);

  void RegisterBuiltin(const std::string& name, DeviceCreateFn factory);

 private:
  DeviceLoader() = default;

  std::unique_ptr<IDevice> LoadFromSo(const std::string& name,
                                      const std::string& explicit_path,
                                      OdinDeviceHandle handle,
                                      std::string* error);

  std::mutex mutex_;
  std::map<std::string, DeviceCreateFn> builtin_registry_;
};

}  // namespace sdk
}  // namespace odin

#define ODIN_REGISTER_BUILTIN_DEVICE(ClassName, DeviceName)                          \
  namespace {                                                                        \
  std::unique_ptr<odin::sdk::IDevice>                                                 \
  Create##ClassName(odin::sdk::OdinDeviceHandle h) {                                 \
    return std::make_unique<ClassName>(h);                                           \
  }                                                                                  \
  struct ClassName##Registrar {                                                       \
    ClassName##Registrar() {                                                          \
      odin::sdk::DeviceLoader::Instance().RegisterBuiltin(                           \
          DeviceName, &Create##ClassName);                                           \
    }                                                                                \
  };                                                                                 \
  static ClassName##Registrar g_##ClassName##_registrar;                              \
  }
