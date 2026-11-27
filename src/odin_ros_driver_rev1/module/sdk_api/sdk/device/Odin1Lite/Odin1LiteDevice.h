#pragma once

#include "../OdinProtocolV2Base/OdinProtocolV2BaseDevice.h"

namespace odin {
namespace sdk {

class Odin1LiteDevice : public OdinProtocolV2BaseDevice {
 public:
  using OdinProtocolV2BaseDevice::OdinProtocolV2BaseDevice;

 protected:
  uint32_t GetCameraCount() const override { return 1; }
};

}  // namespace sdk
}  // namespace odin
