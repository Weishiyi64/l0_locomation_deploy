#pragma once

/**
 * @file ChunkReassembler.hpp
 * @brief Generic chunk-based frame reassembly state for wire protocol v2.
 * @details Unifies the common state and logic shared by point cloud, camera,
 *          and H264 frame assembly: new-frame detection, fragment init, gap
 *          detection, payload append, and completion check. Channel-specific
 *          behaviour (zero-fill, logging, head-loss detection) is left to the
 *          caller.
 */

#include <cstdint>
#include <vector>

namespace odin {
namespace sdk {

template <typename PacketT>
struct ChunkReassemblyState {
  PacketT packet;
  uint16_t expected_chunks = 0;
  uint16_t received_chunks = 0;
  uint16_t last_chunk_index = 0;
  bool last_chunk_received = false;
  bool active = false;

  void Reset() {
    packet = PacketT();
    expected_chunks = 0;
    received_chunks = 0;
    last_chunk_index = 0;
    last_chunk_received = false;
    active = false;
  }

  /// True when frame_seq changes or chunk_index resets to 0 (new frame boundary).
  bool ShouldStartNewFrame(uint32_t frame_count, uint16_t chunk_index) const {
    return active && (frame_count != packet.frame_count || chunk_index == 0);
  }

  /// Initialise state from the first fragment of a new frame.
  void InitFromFragment(const PacketT& fragment, uint16_t chunk_count) {
    active = true;
    packet = fragment;
    packet.payload.clear();
    expected_chunks = chunk_count;
    received_chunks = 0;
    last_chunk_index = 0;
    last_chunk_received = false;
  }

  /// Returns the number of missing chunks if chunk_index jumped, else 0.
  uint16_t DetectGap(uint16_t chunk_index) const {
    if (received_chunks > 0 && chunk_index > last_chunk_index + 1) {
      return static_cast<uint16_t>(chunk_index - last_chunk_index - 1);
    }
    return 0;
  }

  void AppendPayload(const std::vector<uint8_t>& payload) {
    packet.payload.insert(packet.payload.end(), payload.begin(), payload.end());
  }

  void UpdateChunkInfo(uint16_t chunk_index, bool last_chunk) {
    last_chunk_index = chunk_index;
    received_chunks++;
    if (last_chunk) last_chunk_received = true;
  }

  bool IsComplete(bool last_chunk) const {
    return last_chunk ||
        (expected_chunks > 0 && received_chunks >= expected_chunks);
  }
};

}  // namespace sdk
}  // namespace odin
