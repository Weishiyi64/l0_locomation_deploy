#include "Odin2SpecialDevice.h"
#include "DevicePluginEntry.h"

// Odin2Special is shipped as an external device plugin (.so) loaded via
// dlopen by DeviceLoader::LoadFromSo. This translation unit is compiled into
// libodin-device-Odin2Special.so (see device/CMakeLists.txt), not into
// libodin_sdk.so. Entry-point contract: DevicePluginEntry.h.

extern "C" {

ODIN_DEVICE_PLUGIN_API odin::sdk::IDevice* odin_device_create(odin::sdk::OdinDeviceHandle handle) {
  return new odin::sdk::Odin2SpecialDevice(handle);
}

ODIN_DEVICE_PLUGIN_API void odin_device_destroy(odin::sdk::IDevice* device) {
  delete device;
}

ODIN_DEVICE_PLUGIN_API uint32_t odin_device_abi_version() {
  return ODIN_DEVICE_ABI_VERSION;
}

ODIN_DEVICE_PLUGIN_API const char* odin_device_supported_models() {
  return "Odin2Special";
}

}  // extern "C"
