#include "odin_lidar_api.h"
#include "logger/logger.h"
#include "foxglove_publisher.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

using namespace odin::sdk;

// ============================================================================
// Global state
// ============================================================================

static FoxglovePublisher* g_pub = nullptr;

void SignalHandler(int) {
  if (g_pub) g_pub->g_running.store(false);
}

// ============================================================================
// Hotplug handlers — auto-connect / auto-disconnect
// ============================================================================

static void OnDeviceAttach(const DiscoveredDevice& device) {
  std::cout << "\n=== Device arrived: " << device.sn
            << " (" << device.network.ip_address << ") ===\n";

  OdinDeviceHandle h = ConnectDevice(device);
  if (h == kInvalidDeviceHandle) {
    std::cerr << "Failed to connect to " << device.sn << "\n";
    return;
  }

  // Register all Foxglove publisher callbacks
  RegisterPointCloudCallback(h, FoxglovePublisher::OnPointCloud, g_pub);
  RegisterSlamCallback(h, FoxglovePublisher::OnSlam, g_pub);
  RegisterCameraCallback(h, FoxglovePublisher::OnCamera, g_pub);
  RegisterCameraCallback2(h, FoxglovePublisher::OnCamera2, g_pub);
  RegisterOdomCallback(h, FoxglovePublisher::OnOdom, g_pub);
  RegisterTfCallback(h, FoxglovePublisher::OnTf, g_pub);
  RegisterTelemetryCallback(h, FoxglovePublisher::OnTelemetry, g_pub);

  // Get firmware version
  std::string fw;
  if (GetFirmwareVersion(h, fw) == OdinResult::kOk) {
    std::cout << "  Firmware: " << fw << "\n";
  }

  // Standby → configure → start streams → normal
  SetDeviceMode(h, OdinDeviceMode::kStandby);

  auto start = [&](OdinDataChannel ch, const char* name,
                   const OdinStreamCfg* cfg = nullptr) {
    auto r = StartStream(h, ch, OdinTransportMode::kUdp, cfg);
    std::cout << "  [" << (r == OdinResult::kOk ? "OK" : "FAIL") << "] " << name << "\n";
  };

  // Force MJPEG for image channels so Foxglove Studio can decode them.
  OdinStreamCfg mjpeg_cfg;
  mjpeg_cfg.format = OdinDataFormat::kImageMjpeg;

  start(OdinDataChannel::kRawPoint,  "RawPoint");
  start(OdinDataChannel::kSlamPoint, "SlamPoint");
  start(OdinDataChannel::kImage0,    "Image0(left)",  &mjpeg_cfg);
  start(OdinDataChannel::kImage1,    "Image1(right)", &mjpeg_cfg);
  start(OdinDataChannel::kOdomLF,    "OdomLF");
  start(OdinDataChannel::kOdomHF,    "OdomHF");
  start(OdinDataChannel::kTf,        "TF");

  // Start telemetry (brings up 0x42 TCP channel, ~1 Hz)
  StartTelemetry(h);
  std::cout << "  [OK] Telemetry\n";

  if (SetDeviceMode(h, OdinDeviceMode::kNormal) == OdinResult::kOk) {
    std::cout << "  Device " << device.sn << " → NORMAL mode, streaming started\n";
  }
}

static void OnDeviceDetach(const DiscoveredDevice& device) {
  std::cout << "\n=== Device removed: " << device.sn
            << " (" << device.network.ip_address << ") ===\n";
  // Note: we don't call DisconnectDevice here because the SDK handles
  // disconnection internally when the device goes offline.
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
#ifndef _WIN32
  setvbuf(stdout, NULL, _IOLBF, 0);
#endif
  log_config(LOG_LEVEL_DEBUG, printf);
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::cout << "========================================\n";
  std::cout << "  ODIN Foxglove Bridge\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n\n";

  // Parse optional port argument
  uint16_t port = 8765;
  if (argc > 1) {
    int p = std::atoi(argv[1]);
    if (p > 0 && p < 65536) port = static_cast<uint16_t>(p);
  }

  // Create and start Foxglove publisher
  FoxglovePublisher publisher(port);
  g_pub = &publisher;

  if (!publisher.Start()) {
    std::cerr << "Failed to start Foxglove publisher\n";
    return 1;
  }

  std::cout << "Open Foxglove Studio and connect to: ws://localhost:" << port << "\n";
  std::cout << "Press Ctrl+C to exit.\n\n";

  // Start hotplug listener for automatic device discovery & connection
  HotplugCallbacks hotplug;
  hotplug.on_attach = OnDeviceAttach;
  hotplug.on_detach = OnDeviceDetach;
  if (!StartHotplugListener(hotplug, true)) {
    std::cerr << "Failed to start hotplug listener\n";
    return 1;
  }

  // Block until signal
  publisher.Run();

  std::cout << "\nShutting down...\n";
  StopHotplugListener();
  publisher.Stop();
  g_pub = nullptr;

  std::cout << "Goodbye!\n";
  return 0;
}
