#pragma once

#include "../IDiscovery/IDiscovery.h"

namespace odin {
namespace sdk {

// =============================================================================
// OdinUdpDiscovery - ODIN2 UDP device discovery implementation
// =============================================================================

class OdinUdpDiscovery : public IDiscovery {
 public:
  OdinUdpDiscovery() = default;
  ~OdinUdpDiscovery() override = default;

  const char* GetName() const override;

  // Probe every command epoch this SDK must stay discovery-compatible
  // with (0x02 legacy probe + the current OdinConst::kVersion probe) so
  // firmware validating the request-header version still answers one probe.
  std::vector<uint8_t> GetProbeVersions() const override;

  bool BuildRequest(uint16_t seq, const Context& context,
                    std::vector<uint8_t>& request_out) override;

  bool ParseResponse(const uint8_t* data, size_t length, DiscoveredDevice& device_out) override;

 private:
  static bool IpStringToBytes(const std::string& ip, uint8_t* bytes);
};

}  // namespace sdk
}  // namespace odin
