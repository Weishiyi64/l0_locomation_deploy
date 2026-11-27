#include "network_utils.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <random>
#include <set>

#ifdef _WIN32
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#endif

#include "../protocol/OdinProtocol/OdinProtocol.hpp"

namespace odin {
namespace sdk {
namespace network {

// =============================================================================
// Winsock Management
// =============================================================================

#ifdef _WIN32
static std::mutex g_winsock_mutex;
static int g_winsock_refcnt = 0;

bool AcquireWinsock() {
  std::lock_guard<std::mutex> lock(g_winsock_mutex);
  if (g_winsock_refcnt == 0) {
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return false;
  }
  ++g_winsock_refcnt;
  return true;
}

void ReleaseWinsock() {
  std::lock_guard<std::mutex> lock(g_winsock_mutex);
  if (g_winsock_refcnt > 0) {
    --g_winsock_refcnt;
    if (g_winsock_refcnt == 0) WSACleanup();
  }
}
#else
bool AcquireWinsock() { return true; }
void ReleaseWinsock() {}
#endif

// =============================================================================
// Socket Operations
// =============================================================================

void CloseSocket(SocketHandle sock) {
  if (sock == kInvalidSocket) return;
#ifdef _WIN32
  closesocket(sock);
#else
  close(sock);
#endif
}

SocketHandle CreateBroadcastSocket(const std::string& host_ip, uint16_t host_port) {
  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == kInvalidSocket) return kInvalidSocket;

  // Enable broadcast
  int enable = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&enable),
                 sizeof(enable)) != 0) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  // Bind to local address
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(host_port);
  if (host_ip.empty() || host_ip == "0.0.0.0") {
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (inet_pton(AF_INET, host_ip.c_str(), &local_addr.sin_addr) != 1) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) != 0) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  return sock;
}

SocketHandle CreateBroadcastSocketForInterface(const std::string& host_ip, uint16_t host_port) {
  // Bind to INADDR_ANY so the socket can receive limited-broadcast replies
  // (destination 255.255.255.255); a socket bound to a concrete unicast
  // address would not receive them.
  SocketHandle sock = CreateBroadcastSocket("0.0.0.0", host_port);
  if (sock == kInvalidSocket) return kInvalidSocket;

#ifndef _WIN32
  // Pin the egress interface to the NIC that owns host_ip so probes leave that
  // NIC even when it is not on the default route. Requires CAP_NET_RAW.
  std::string ifname = GetInterfaceNameForIp(host_ip);
  if (!ifname.empty()) {
    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, ifname.c_str(),
                   static_cast<socklen_t>(ifname.size())) != 0) {
      // Non-fatal: keep the socket but fall back to default-route egress.
    }
  }
#else
  (void)host_ip;
#endif

  return sock;
}

// =============================================================================
// IP Address Utilities
// =============================================================================

std::vector<std::string> GetLocalIPAddresses() {
  std::vector<std::string> ips;
#ifdef _WIN32
  ULONG bufLen = 15000;
  PIP_ADAPTER_ADDRESSES addresses = (PIP_ADAPTER_ADDRESSES)malloc(bufLen);
  if (addresses && GetAdaptersAddresses(AF_INET, 0, NULL, addresses, &bufLen) == NO_ERROR) {
    for (auto addr = addresses; addr; addr = addr->Next) {
      if (addr->OperStatus != IfOperStatusUp) continue;
      for (auto ua = addr->FirstUnicastAddress; ua; ua = ua->Next) {
        if (ua->Address.lpSockaddr->sa_family != AF_INET) continue;
        sockaddr_in* sa = (sockaddr_in*)ua->Address.lpSockaddr;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
        if (strcmp(ip, "127.0.0.1") != 0) {
          ips.push_back(ip);
        }
      }
    }
  }
  if (addresses) free(addresses);
#else
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == 0) {
    for (auto ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr) continue;
      if (ifa->ifa_addr->sa_family != AF_INET) continue;
      if (!(ifa->ifa_flags & IFF_UP)) continue;
      if (ifa->ifa_flags & IFF_LOOPBACK) continue;

      sockaddr_in* sa = (sockaddr_in*)ifa->ifa_addr;
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
      ips.push_back(ip);
    }
    freeifaddrs(ifaddr);
  }
#endif
  return ips;
}

std::string GetInterfaceNameForIp(const std::string& host_ip) {
#ifndef _WIN32
  struct ifaddrs* ifaddr = nullptr;
  std::string result;
  if (getifaddrs(&ifaddr) == 0) {
    for (auto ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr) continue;
      if (ifa->ifa_addr->sa_family != AF_INET) continue;

      char ip[INET_ADDRSTRLEN];
      auto* sa = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
      inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
      if (host_ip == ip) {
        result = (ifa->ifa_name != nullptr) ? ifa->ifa_name : "";
        break;
      }
    }
    freeifaddrs(ifaddr);
  }
  return result;
#else
  (void)host_ip;
  return "";
#endif
}

std::string GetDirectedBroadcastAddress(const std::string& host_ip) {
  if (host_ip.empty() || host_ip == "0.0.0.0") return "";

  std::string result;
#ifdef _WIN32
  ULONG bufLen = 15000;
  PIP_ADAPTER_ADDRESSES addresses = (PIP_ADAPTER_ADDRESSES)malloc(bufLen);
  if (addresses &&
      GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, addresses, &bufLen) ==
          NO_ERROR) {
    for (auto addr = addresses; addr && result.empty(); addr = addr->Next) {
      if (addr->OperStatus != IfOperStatusUp) continue;
      for (auto ua = addr->FirstUnicastAddress; ua; ua = ua->Next) {
        if (ua->Address.lpSockaddr->sa_family != AF_INET) continue;
        sockaddr_in* sa = (sockaddr_in*)ua->Address.lpSockaddr;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
        if (host_ip != ip) continue;

        // /32 point-to-point routes have no broadcast address; /0 routes
        // degenerate to the limited broadcast (rejected below).
        ULONG prefix = ua->OnLinkPrefixLength;
        if (prefix >= 32) break;
        uint32_t mask = (prefix == 0) ? 0u : (0xFFFFFFFFu << (32 - prefix));
        uint32_t broadcast = ntohl(sa->sin_addr.s_addr) | ~mask;

        in_addr broadcast_in;
        broadcast_in.s_addr = htonl(broadcast);
        char bc[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &broadcast_in, bc, sizeof(bc));
        result = bc;
        break;
      }
    }
  }
  if (addresses) free(addresses);
#else
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == 0) {
    for (auto ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr || ifa->ifa_netmask == nullptr) continue;
      if (ifa->ifa_addr->sa_family != AF_INET) continue;

      auto* sa = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
      if (host_ip != ip) continue;

      auto* netmask = reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask);
      uint32_t broadcast = ntohl(sa->sin_addr.s_addr) | ~ntohl(netmask->sin_addr.s_addr);

      in_addr broadcast_in;
      broadcast_in.s_addr = htonl(broadcast);
      char bc[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &broadcast_in, bc, sizeof(bc));
      result = bc;
      break;
    }
    freeifaddrs(ifaddr);
  }
#endif
  // Skip degenerate results: a /32 route yields the address itself and a /0
  // route yields the limited broadcast; neither adds reach beyond unicast.
  if (result == host_ip || result == "255.255.255.255") return "";
  return result;
}

bool IsIpInSameSubnet(const std::string& ip, const std::string& local_ip) {
  in_addr target_addr{};
  if (inet_pton(AF_INET, ip.c_str(), &target_addr) != 1) {
    return true;  // Fail-open: unparsable input keeps the legacy behaviour.
  }

#ifdef _WIN32
  ULONG buf_len = 15000;
  PIP_ADAPTER_ADDRESSES addresses = (PIP_ADAPTER_ADDRESSES)malloc(buf_len);
  if (addresses == nullptr) return true;  // Fail-open

  if (GetAdaptersAddresses(AF_INET, 0, NULL, addresses, &buf_len) != NO_ERROR) {
    free(addresses);
    return true;  // Fail-open
  }

  bool same_subnet = true;  // Fail-open when local_ip is not found.
  bool found = false;
  for (auto addr = addresses; addr != nullptr && !found; addr = addr->Next) {
    for (auto ua = addr->FirstUnicastAddress; ua != nullptr && !found; ua = ua->Next) {
      if (ua->Address.lpSockaddr->sa_family != AF_INET) continue;

      sockaddr_in* sa = (sockaddr_in*)ua->Address.lpSockaddr;
      char lip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &sa->sin_addr, lip, sizeof(lip));
      if (local_ip != lip) continue;

      uint32_t prefix = ua->OnLinkPrefixLength;
      uint32_t mask = 0;
      if (prefix >= 32) {
        mask = 0xFFFFFFFFu;
      } else if (prefix > 0) {
        mask = 0xFFFFFFFFu << (32 - prefix);
      }
      same_subnet =
          (ntohl(target_addr.s_addr) & mask) == (ntohl(sa->sin_addr.s_addr) & mask);
      found = true;
    }
  }
  free(addresses);
  return same_subnet;
#else
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    return true;  // Fail-open
  }

  bool same_subnet = true;  // Fail-open when local_ip is not found.
  for (auto ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_netmask == nullptr) continue;
    if (ifa->ifa_addr->sa_family != AF_INET) continue;

    auto* sa = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
    char lip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &sa->sin_addr, lip, sizeof(lip));
    if (local_ip != lip) continue;

    uint32_t mask = ntohl(reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask)->sin_addr.s_addr);
    same_subnet = (ntohl(target_addr.s_addr) & mask) == (ntohl(sa->sin_addr.s_addr) & mask);
    break;
  }
  freeifaddrs(ifaddr);
  return same_subnet;
#endif
}

bool IpStringToBytes(const std::string& ip, uint8_t* bytes_out) {
  if (ip.empty() || bytes_out == nullptr) return false;

  in_addr addr;
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) return false;

  uint32_t host_order = ntohl(addr.s_addr);
  bytes_out[0] = static_cast<uint8_t>((host_order >> 24) & 0xFF);
  bytes_out[1] = static_cast<uint8_t>((host_order >> 16) & 0xFF);
  bytes_out[2] = static_cast<uint8_t>((host_order >> 8) & 0xFF);
  bytes_out[3] = static_cast<uint8_t>(host_order & 0xFF);
  return true;
}

std::string BytesToIpString(const uint8_t* bytes) {
  if (bytes == nullptr) return "";
  return std::to_string(bytes[0]) + "." + std::to_string(bytes[1]) + "." +
         std::to_string(bytes[2]) + "." + std::to_string(bytes[3]);
}

std::string BytesToMacString(const uint8_t* bytes) {
  if (bytes == nullptr) return "";
  char buf[18] = {0};
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", bytes[0], bytes[1], bytes[2],
           bytes[3], bytes[4], bytes[5]);
  return std::string(buf);
}

// =============================================================================
// Discovery Protocol Helpers
// =============================================================================

bool BuildDiscoveryPacket(const std::string& host_ip, uint16_t seq,
                          std::vector<uint8_t>& frame_out) {
  OdinProtocolV1 protocol;

  uint8_t host_bytes[4] = {0, 0, 0, 0};
  if (!host_ip.empty() && host_ip != "0.0.0.0") {
    if (!IpStringToBytes(host_ip, host_bytes)) return false;
  }

  return protocol.Pack(kCmdIdDeviceQuery, seq, OdinConst::kCmdTypeReq, OdinConst::kSendTypeHost,
                       host_bytes, 4, frame_out);
}

bool SendDiscoveryBroadcast(SocketHandle sock, uint16_t target_port,
                            const IDiscovery::Context& context,
                            std::shared_ptr<IDiscovery> discovery,
                            const std::string& broadcast_ip) {
  static std::atomic<uint16_t> seq_counter{0};

  if (!discovery) discovery = DiscoveryFactory::GetDefault();

  sockaddr_in broadcast_addr{};
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(target_port);
  if (broadcast_ip.empty() || broadcast_ip == "255.255.255.255") {
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  } else if (inet_pton(AF_INET, broadcast_ip.c_str(), &broadcast_addr.sin_addr) != 1) {
    return false;
  }

  // Multi-version probing: emit one frame per epoch the discovery declares
  // (each with its own seq) so firmware that validates the request-header
  // version still accepts one of the probes. All replies land in the same
  // receive window and are deduplicated upstream by SN/IP. An empty list
  // (or a 0 sentinel) means a single request with the discovery's default
  // epoch.
  std::vector<uint8_t> probe_versions = discovery->GetProbeVersions();
  if (probe_versions.empty()) {
    probe_versions.push_back(0);
  }

  size_t frames_sent = 0;
  for (uint8_t probe_version : probe_versions) {
    IDiscovery::Context ctx = context;
    if (probe_version != 0) {
      ctx[IDiscovery::kContextKeyProtocolVersion] = std::to_string(probe_version);
    }

    std::vector<uint8_t> frame;
    if (!discovery->BuildRequest(seq_counter.fetch_add(1), ctx, frame)) {
      continue;
    }

    if (sendto(sock, reinterpret_cast<const char*>(frame.data()),
               static_cast<int>(frame.size()), 0,
               reinterpret_cast<sockaddr*>(&broadcast_addr), sizeof(broadcast_addr)) < 0) {
      continue;
    }
    ++frames_sent;
  }

  return frames_sent > 0;
}

bool SendDiscoveryBroadcastForInterface(SocketHandle sock, uint16_t target_port,
                                         const IDiscovery::Context& context,
                                         std::shared_ptr<IDiscovery> discovery,
                                         const std::string& host_ip) {
  // A directed subnet broadcast (e.g. 192.168.1.255) routes to the on-link
  // interface that owns host_ip. This is the portable egress selector on
  // Windows, where SO_BINDTODEVICE is unavailable and a 255.255.255.255
  // limited broadcast leaves only through the default-route interface.
  const std::string directed_bc = GetDirectedBroadcastAddress(host_ip);
  bool sent = false;
  if (!directed_bc.empty()) {
    sent = SendDiscoveryBroadcast(sock, target_port, context, discovery, directed_bc);
  }
  // Limited broadcast as a fallback for devices that only answer
  // 255.255.255.255 and for interfaces without a resolvable subnet.
  if (SendDiscoveryBroadcast(sock, target_port, context, discovery)) {
    sent = true;
  }
  return sent;
}

// =============================================================================
// Device Discovery
// =============================================================================

namespace {
constexpr const char* kContextKeyHostIp = "host_ip";
constexpr const char* kContextKeyPort = "port";

uint16_t GenerateRandomPort() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint16_t> dis(10000, 65535);
  return dis(gen);
}
}  // namespace

bool DiscoverDevicesOnInterface(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                                uint16_t host_port, uint16_t target_port,
                                const std::string& host_ip, std::shared_ptr<IDiscovery> discovery) {
  if (!discovery) discovery = DiscoveryFactory::GetDefault();
  if (host_port == 0) host_port = kDefaultCommandPort;
  if (target_port == 0) target_port = kDefaultCommandPort;
  if (timeout_ms == 0) timeout_ms = 2000;
  if (!AcquireWinsock()) return false;

  // Bind to INADDR_ANY (to receive limited-broadcast replies with destination
  // 255.255.255.255) while pinning egress to the NIC that owns host_ip via
  // SO_BINDTODEVICE. Send and receive on this single socket.
  SocketHandle sock = CreateBroadcastSocketForInterface(host_ip, host_port);
  if (sock == kInvalidSocket) {
    ReleaseWinsock();
    return false;
  }

  // Build context for discovery
  IDiscovery::Context context;
  context[kContextKeyHostIp] = host_ip;
  context[kContextKeyPort] = std::to_string(target_port);

  // Send discovery broadcast (reuse shared helper)
  if (!SendDiscoveryBroadcastForInterface(sock, target_port, context, discovery, host_ip)) {
    CloseSocket(sock);
    ReleaseWinsock();
    return false;
  }

  // Receive responses
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::set<std::string> seen;
  std::vector<uint8_t> buffer(512);

  while (std::chrono::steady_clock::now() < deadline) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    timeval tv;
    tv.tv_sec = static_cast<long>(remaining.count() / 1000);
    tv.tv_usec = static_cast<long>((remaining.count() % 1000) * 1000);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);
#ifdef _WIN32
    int ready = select(0, &read_fds, nullptr, nullptr, &tv);
#else
    int ready = select(static_cast<int>(sock + 1), &read_fds, nullptr, nullptr, &tv);
#endif
    if (ready <= 0 || !FD_ISSET(sock, &read_fds)) continue;

    sockaddr_in from_addr;
    socklen_t addr_len = sizeof(from_addr);
    int len =
        recvfrom(sock, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0,
                 reinterpret_cast<sockaddr*>(&from_addr), &addr_len);
    if (len <= 0) continue;

    // Parse response using discovery
    DiscoveredDevice device;
    if (!discovery->ParseResponse(buffer.data(), static_cast<size_t>(len), device)) continue;

    // Use actual socket source IP instead of protocol payload IP
    char socket_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &from_addr.sin_addr, socket_ip, INET_ADDRSTRLEN);
    device.network.ip_address = socket_ip;

    if (!seen.insert(device.network.ip_address).second) continue;

    device.host_ip = host_ip;
    devices.push_back(device);
  }

  CloseSocket(sock);
  ReleaseWinsock();
  return true;
}

bool DiscoverNetworkDevices(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                            std::shared_ptr<IDiscovery> discovery) {
  if (!discovery) discovery = DiscoveryFactory::GetDefault();

  auto local_ips = GetLocalIPAddresses();
  if (local_ips.empty()) {
    local_ips.push_back("0.0.0.0");
  }

  uint32_t per_interface_timeout =
      std::max(500u, timeout_ms / static_cast<uint32_t>(local_ips.size()));

  constexpr int kMaxPortRetries = 3;
  std::set<std::string> seen_sns;
  bool found_any = false;

  for (const auto& ip : local_ips) {
    std::vector<DiscoveredDevice> found;
    bool success = false;
    for (int retry = 0; retry < kMaxPortRetries && !success; ++retry) {
      uint16_t host_port = GenerateRandomPort();
      success = DiscoverDevicesOnInterface(found, per_interface_timeout, host_port,
                                           kDefaultCommandPort, ip, discovery);
    }
    if (success) {
      for (const auto& dev : found) {
        if (seen_sns.find(dev.sn) == seen_sns.end()) {
          seen_sns.insert(dev.sn);
          devices.push_back(dev);
          found_any = true;
        }
      }
    }
  }
  return found_any;
}

}  // namespace network
}  // namespace sdk
}  // namespace odin
