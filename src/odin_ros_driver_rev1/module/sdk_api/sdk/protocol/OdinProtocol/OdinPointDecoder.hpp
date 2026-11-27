#pragma once

/**
 * @file OdinPointDecoder.hpp
 * @brief Wire v2 point cloud payload decoder (uint16 → float + zero-fill).
 * @details Pure transformation from the assembled wire payload to the public
 *          float point types. Knows the v2 encoding (uint16 xyz, -30000 mm
 *          offset on y/z) and the tail-loss padding heuristic; does not depend
 *          on the assembler or any transport type. A future v3 decoder
 *          (int16_t xyz, no offset) would live alongside this file.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "OdinProtocol.hpp"
#include "logger.h"
#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// =============================================================================
// Shared constants (used by both the assembler and the decoder)
// =============================================================================

// Hint: max payload bytes per UDP chunk (MTU 1500 - IP/UDP headers - data header).
// Used only for loss-impact estimation, not as a hard protocol constraint.
constexpr size_t kUdpMaxDataSize = 1472 - OdinConst::kDataHeaderSize;

// =============================================================================
// One-shot hint: print sysctl suggestions when packet loss is first detected.
// Uses a function-local static so the message is emitted at most once per
// process, regardless of how many translation units include this header.
// =============================================================================

inline void LogNetBufferHintOnce() {
  static bool shown = false;
  if (shown) return;
  shown = true;
  LOG_WARN("Packet loss detected — this is often caused by insufficient kernel "
           "UDP receive buffer. Consider increasing it (requires sudo):\n"
           "  sudo sysctl -w net.core.rmem_max=16777216     # 16MB\n"
           "  sudo sysctl -w net.core.rmem_default=8388608  #  8MB\n"
           "  sudo sysctl -w net.core.netdev_max_backlog=50000\n"
           "Persist by adding to /etc/sysctl.conf and running 'sudo sysctl -p'.\n");
}

// =============================================================================
// Decoder
// =============================================================================

/**
 * @brief Frame metadata needed by the decoder for tail-loss padding decisions.
 */
struct PointFrameInfo {
  uint16_t expected_chunks = 0;    ///< chunk_count advertised by the first chunk
  bool last_chunk_received = false; ///< true if the device set the last_chunk flag
  uint32_t frame_count = 0;        ///< frame_seq, for logging only
};

/**
 * @brief Wire v2 point cloud decoder for OdinDataFrameHeader v2.
 *
 * Stateless — all inputs are passed per call. Safe to call from multiple
 * threads concurrently.
 */
class OdinPointDecoderV2 {
 public:
  /**
   * @brief Decode a raw point cloud payload (uint16 → float, -30000 offset on y/z).
   * @param payload      Wire payload bytes (array of OdinRawPoint<uint16_t>)
   * @param payload_size Payload byte count
   * @param info         Frame metadata for tail-loss padding
   * @return Decoded float points (zero-padded if tail chunks were lost)
   */
  static std::vector<OdinRawPoint<float>> DecodeRaw(
      const uint8_t* payload, size_t payload_size,
      const PointFrameInfo& info) {
    return Decode<OdinRawPoint<uint16_t>, OdinRawPoint<float>>(
        payload, payload_size, info, "Raw",
        [](const OdinRawPoint<uint16_t>& src, OdinRawPoint<float>& dst) {
          dst.x = static_cast<float>(src.x);
          dst.y = static_cast<float>(src.y) - 30000.0f;
          dst.z = static_cast<float>(src.z) - 30000.0f;
          dst.intensity = src.intensity;
          dst.confidence = src.confidence;
        });
  }

  /**
   * @brief Decode a SLAM point cloud payload (uint16 → float, -30000 offset on y/z).
   * @param payload      Wire payload bytes (array of OdinSlamPoint<uint16_t>)
   * @param payload_size Payload byte count
   * @param info         Frame metadata for tail-loss padding
   * @return Decoded float SLAM points (zero-padded if tail chunks were lost)
   */
  static std::vector<OdinSlamPoint<float>> DecodeSlam(
      const uint8_t* payload, size_t payload_size,
      const PointFrameInfo& info) {
    return Decode<OdinSlamPoint<uint16_t>, OdinSlamPoint<float>>(
        payload, payload_size, info, "SLAM",
        [](const OdinSlamPoint<uint16_t>& src, OdinSlamPoint<float>& dst) {
          dst.x = static_cast<float>(src.x);
          dst.y = static_cast<float>(src.y) - 30000.0f;
          dst.z = static_cast<float>(src.z) - 30000.0f;
          dst.r = src.r;
          dst.g = src.g;
          dst.b = src.b;
          dst.a = src.a;
        });
  }

 private:
  template <typename SrcPoint, typename DstPoint>
  static std::vector<DstPoint> Decode(
      const uint8_t* payload, size_t payload_size,
      const PointFrameInfo& info, const char* label,
      void (*convert)(const SrcPoint&, DstPoint&)) {
    constexpr size_t src_point_size = sizeof(SrcPoint);
    if (payload == nullptr || src_point_size == 0) return {};

    const size_t point_count = payload_size / src_point_size;
    const auto* src = reinterpret_cast<const SrcPoint*>(payload);

    std::vector<DstPoint> points(point_count);
    for (size_t i = 0; i < point_count; ++i) {
      convert(src[i], points[i]);
    }

    PadZerosForTailLoss(points, info, label, src_point_size);
    return points;
  }

  /**
   * @brief Pad the converted float point vector with zero-valued points so
   *        the emitted frame carries the expected point count even when tail
   *        chunks were lost.
   */
  template <typename DstPoint>
  static void PadZerosForTailLoss(std::vector<DstPoint>& points,
                                  const PointFrameInfo& info,
                                  const char* label,
                                  size_t src_point_size) {
    // If the frame was explicitly closed by the device's last_chunk flag, the
    // trailing point count is authoritative: the last chunk may simply be
    // partial. Do not synthesize zero points in that case.
    if (info.last_chunk_received) return;

    if (info.expected_chunks == 0 || src_point_size == 0) return;

    const size_t pts_per_chunk = kUdpMaxDataSize / src_point_size;
    if (pts_per_chunk == 0) return;
    const size_t expected_points =
        static_cast<size_t>(info.expected_chunks) * pts_per_chunk;

    if (points.size() >= expected_points) return;

    const size_t missing_points = expected_points - points.size();
    LOG_WARN("%s point frame_seq=%u incomplete: %zu/%zu points, padding %zu zero point(s)\n",
             label,
             static_cast<unsigned>(info.frame_count),
             points.size(), expected_points, missing_points);
    LogNetBufferHintOnce();

    // Default-constructed DstPoint has all-zero fields (x/y/z = 0, etc.).
    points.resize(expected_points);
  }
};

}  // namespace sdk
}  // namespace odin
