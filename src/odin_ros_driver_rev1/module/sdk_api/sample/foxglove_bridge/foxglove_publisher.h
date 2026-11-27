#pragma once

#include "odin_lidar_def.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

// Forward-declare foxglove types to avoid pulling SDK headers into this
// public header (which would trigger the logger.h macro conflict).
namespace foxglove {
class WebSocketServer;
class RawChannel;
struct WebSocketServerOptions;
}  // namespace foxglove

/// @brief Bridges Odin SDK sensor data to Foxglove WebSocket channels.
///
/// Creates one foxglove::RawChannel per data type, converts Odin packets to
/// JSON payloads matching well-known Foxglove schemas, and logs them.
/// Odin callbacks are registered as static functions that forward to the
/// corresponding publish() methods via client_data = this.
class FoxglovePublisher {
 public:
  explicit FoxglovePublisher(uint16_t port = 8765);
  ~FoxglovePublisher();

  FoxglovePublisher(const FoxglovePublisher&) = delete;
  FoxglovePublisher& operator=(const FoxglovePublisher&) = delete;

  /// Create WebSocket server and advertise all channels.
  /// @return true on success.
  bool Start();

  /// Gracefully shut down server.
  void Stop();

  /// Block until g_running_ becomes false, printing periodic status.
  void Run();

  // --- Per-type publish methods (called from Odin SDK callbacks) ---
  void PublishPointCloud(const odin::sdk::OdinPointCloudPacket& pkt);
  void PublishSlam(const odin::sdk::OdinSlamPacket& pkt);
  void PublishCamera(const odin::sdk::OdinCameraPacket& pkt, bool is_right);
  void PublishOdom(const odin::sdk::OdinOdomPacket& pkt,
                   odin::sdk::OdomSourceType odom_type);
  void PublishTf(const odin::sdk::OdinTfPacket& pkt);
  void PublishTelemetry(const odin::sdk::OdinTelemetryPacket& pkt);

  /// Emit static identity TF entries (imu→lidar, imu→camera0, imu→camera1)
  /// so the Foxglove 3D panel can resolve all sensor frames.
  void PublishStaticTf();

  // --- Static Odin SDK callbacks (client_data = FoxglovePublisher*) ---
  static void OnPointCloud(const odin::sdk::OdinPointCloudPacket& p, void* ud);
  static void OnSlam(const odin::sdk::OdinSlamPacket& p, void* ud);
  static void OnCamera(const odin::sdk::OdinCameraPacket& p, void* ud);
  static void OnCamera2(const odin::sdk::OdinCameraPacket& p, void* ud);
  static void OnOdom(const odin::sdk::OdinOdomPacket& p,
                     odin::sdk::OdomSourceType t, void* ud);
  static void OnTf(const odin::sdk::OdinTfPacket& p, void* ud);
  static void OnTelemetry(const odin::sdk::OdinTelemetryPacket& p, void* ud);

  std::atomic<bool> g_running{true};

 private:
  uint16_t port_;

  // Channels (heap-allocated so we can forward-declare RawChannel)
  foxglove::RawChannel* ch_points_ = nullptr;
  foxglove::RawChannel* ch_slam_   = nullptr;
  foxglove::RawChannel* ch_img0_   = nullptr;
  foxglove::RawChannel* ch_img1_   = nullptr;
  foxglove::RawChannel* ch_odom_    = nullptr;
  foxglove::RawChannel* ch_odom_hf_ = nullptr;
  foxglove::RawChannel* ch_tf_      = nullptr;
  foxglove::RawChannel* ch_telem_   = nullptr;

  std::unique_ptr<foxglove::WebSocketServer> server_;

  // Frame counters for status display
  std::atomic<uint64_t> cnt_pts_{0};
  std::atomic<uint64_t> cnt_slam_{0};
  std::atomic<uint64_t> cnt_img0_{0};
  std::atomic<uint64_t> cnt_img1_{0};
  std::atomic<uint64_t> cnt_odom_{0};
  std::atomic<uint64_t> cnt_odom_hf_{0};
  std::atomic<uint64_t> cnt_tf_{0};
  std::atomic<uint64_t> cnt_telem_{0};

  std::chrono::steady_clock::time_point last_static_tf_{};

  // --- Helpers (implemented in .cpp) ---
  static std::string Base64Encode(const uint8_t* data, size_t len);
  static std::string TimestampJson(uint64_t ns);
  static std::string EscapeJson(const std::string& s);
};
