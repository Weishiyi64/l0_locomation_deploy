#include "OdinUdpDiscovery.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include "../../protocol/OdinProtocol/OdinProtocol.hpp"
#include "DeviceLoader.h"
#include "logger.h"

namespace odin {
namespace sdk {

namespace {
constexpr const char* kContextKeyHostIp = "host_ip";

std::string BytesToMacString(const uint8_t* bytes) {
  if (bytes == nullptr) return "";
  char buf[18] = {0};
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", bytes[0], bytes[1], bytes[2],
           bytes[3], bytes[4], bytes[5]);
  return std::string(buf);
}
}  // namespace

static constexpr uint16_t kCmdIdDeviceQuery = 0x01;
const char* OdinUdpDiscovery::GetName() const { return "ODIN_UDP"; }

std::vector<uint8_t> OdinUdpDiscovery::GetProbeVersions() const {
  // Epochs to probe, listed here on purpose: which versions this SDK must
  // stay discovery-compatible with is a discovery-layer concern. 0x02 =
  // frozen legacy probe (first epoch); OdinConst::kVersion = current
  // probe - it follows the constant when the SDK upgrades, so old firmware
  // answers the legacy probe and new firmware the current one. Replies
  // are deduplicated upstream by SN/IP.
  return {0x02, OdinConst::kVersion};
}

bool OdinUdpDiscovery::IpStringToBytes(const std::string& ip, uint8_t* bytes) {
  unsigned int b0, b1, b2, b3;
  if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &b0, &b1, &b2, &b3) != 4) return false;
  if (b0 > 255 || b1 > 255 || b2 > 255 || b3 > 255) return false;
  bytes[0] = static_cast<uint8_t>(b0);
  bytes[1] = static_cast<uint8_t>(b1);
  bytes[2] = static_cast<uint8_t>(b2);
  bytes[3] = static_cast<uint8_t>(b3);
  return true;
}

bool OdinUdpDiscovery::BuildRequest(uint16_t seq, const Context& context,
                                    std::vector<uint8_t>& request_out) {
  OdinProtocolV1 protocol;

  // Extract host_ip from context (network-specific, optional)
  uint8_t host_bytes[4] = {0, 0, 0, 0};
  auto it = context.find(kContextKeyHostIp);
  if (it != context.end() && !it->second.empty() && it->second != "0.0.0.0") {
    if (!IpStringToBytes(it->second, host_bytes)) return false;
  }

  OdinPacket pkt(kCmdIdDeviceQuery, seq, OdinConst::kCmdTypeReq, OdinConst::kSendTypeHost,
                 host_bytes, 4);

  // Probe epoch (multi-version probing, see GetProbeVersions): explicit
  // value wins; unset (0) falls back to the protocol dialect in Pack.
  auto ver_it = context.find(IDiscovery::kContextKeyProtocolVersion);
  if (ver_it != context.end() && !ver_it->second.empty()) {
    unsigned long epoch = std::strtoul(ver_it->second.c_str(), nullptr, 0);
    pkt.SetAttribute(PacketAttr::kVersion, static_cast<uint8_t>(epoch));
  }

  return protocol.Pack(pkt, request_out);
}

bool OdinUdpDiscovery::ParseResponse(const uint8_t* data, size_t length,
                                     DiscoveredDevice& device_out) {
  if (data == nullptr || length == 0) return false;

  OdinProtocolV1 protocol;
  std::unique_ptr<IPacket> packet;

  int consumed = protocol.Parse(data, length, packet);
  if (consumed <= 0 || !packet) return false;

  auto* odin_pkt = static_cast<OdinPacket*>(packet.get());

  // Check if this is a discovery response
  // Minimum payload: status(1) + SN(16) + IP(4) = 21 bytes
  // Extended payload: + model(32) = 53 bytes
  // Version 0x01 extension: + device_mac(6) = 59 bytes
  if (odin_pkt->GetCmdId() != kCmdIdDeviceQuery) return false;
  if (odin_pkt->GetCmdType() != OdinConst::kCmdTypeAck) return false;
  if (odin_pkt->GetPayloadSize() < 21) return false;

  const uint8_t* payload = odin_pkt->GetPayload();

  // Check status
  if (payload[0] != 0) return false;

  // Parse SN (bytes 1-16)
  std::string sn(reinterpret_cast<const char*>(payload + 1), 16);
  auto null_pos = sn.find('\0');
  if (null_pos != std::string::npos) sn.resize(null_pos);

  // Extract actual SN: if format is "xx-xxxxx", use part after '-'
  auto dash_pos = sn.find('-');
  if (dash_pos != std::string::npos && dash_pos + 1 < sn.size()) {
    sn = sn.substr(dash_pos + 1);
  }

  // Parse IP (bytes 17-20)
  std::string ip = std::to_string(payload[17]) + "." + std::to_string(payload[18]) + "." +
                   std::to_string(payload[19]) + "." + std::to_string(payload[20]);

  // Parse model (bytes 21-52, 32 characters) if available, otherwise use default
  std::string model = "Odin1Lite";
  if (odin_pkt->GetPayloadSize() >= 53) {
    model = std::string(reinterpret_cast<const char*>(payload + 21), 32);
    auto model_null_pos = model.find('\0');
    if (model_null_pos != std::string::npos) model.resize(model_null_pos);
  }

  // Model normalization: an empty field (all-NUL bytes on old firmware)
  // means "unreported", which is the same case as the short-payload default
  // above. Legacy firmware reports the bare string "Odin"/"ODIN"; map it to
  // Odin1Lite so those devices stay recognizable and connectable.
  if (model.empty()) {
    model = "Odin1Lite";
  } else {
    bool legacy = model.size() == 4;
    if (legacy) {
      static const char kLegacy[] = "odin";
      for (size_t i = 0; i < model.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(model[i])) != kLegacy[i]) {
          legacy = false;
          break;
        }
      }
    }
    if (legacy) {
      LOG_INFO("OdinUdpDiscovery: legacy firmware model '%s' (SN=%s, IP=%s) - "
               "using Odin1Lite\n",
               model.c_str(), sn.c_str(), ip.c_str());
      model = "Odin1Lite";
    }
  }

  // Parse device MAC (bytes 53-58, 6 bytes) if available
  std::string mac;
  if (odin_pkt->GetPayloadSize() >= 59) {
    mac = BytesToMacString(payload + 53);
  }

  // Protocol epoch, in priority order:
  //   1. Explicit payload extension (byte 59, 60B extension) - the device
  //      states its epoch in the payload.
  //   2. Reply frame header version byte - firmware reports its own epoch
  //      there (not an echo of the probe). Guarded to the supported range;
  //      out-of-range values keep the struct default.
  //   3. Struct default (odin_lidar_def.h).
  if (odin_pkt->GetPayloadSize() >= 60) {
    device_out.protocol_version = payload[59];
  } else {
    const uint8_t reply_epoch =
        static_cast<uint8_t>(odin_pkt->GetAttribute(PacketAttr::kVersion));
    if (reply_epoch >= OdinConst::kDataVersion &&
        reply_epoch <= OdinConst::kMaxForwardCompatVersion) {
      device_out.protocol_version = reply_epoch;
    }
  }

  // Support gate: a reported model with no builtin device and no plugin on
  // the search paths can never be connected (DeviceLoader::Load would
  // silently fall back to a mismatched implementation). Emit a WARN so the
  // unsupported device is still diagnosable, then drop the response so it
  // never enters the device list.
  if (!DeviceLoader::Instance().IsKnownDevice(model)) {
    LOG_WARN(
        "OdinUdpDiscovery: unrecognized device model '%s' (SN=%s, IP=%s) - "
        "device ignored\n",
        model.c_str(), sn.c_str(), ip.c_str());
    return false;
  }

  device_out.network.ip_address = ip;
  device_out.network.mac = mac;
  device_out.sn = sn;
  device_out.model = model;
  device_out.connection = MTConnectionType::kEthernet;

  return true;
}

// =============================================================================
// DiscoveryFactory
// =============================================================================

namespace {
std::shared_ptr<IDiscovery> g_default_discovery;
std::once_flag g_init_flag;

void InitDefaultDiscovery() { g_default_discovery = std::make_shared<OdinUdpDiscovery>(); }
}  // namespace

std::shared_ptr<IDiscovery> DiscoveryFactory::GetDefault() {
  std::call_once(g_init_flag, InitDefaultDiscovery);
  return g_default_discovery;
}

std::shared_ptr<IDiscovery> DiscoveryFactory::GetByName(const std::string& name) {
  if (name == "ODIN_UDP" || name.empty()) {
    return GetDefault();
  }
  // Future: Add more discoveries here
  // if (name == "ODIN_USB") return std::make_shared<OdinUsbDiscovery>();
  return nullptr;
}

}  // namespace sdk
}  // namespace odin
