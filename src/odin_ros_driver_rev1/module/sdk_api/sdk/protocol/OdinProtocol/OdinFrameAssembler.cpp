#include "OdinFrameAssembler.hpp"

#include "logger.h"
#include <algorithm>

namespace odin {
namespace sdk {

// Constants for Odin2 point cloud data
constexpr size_t kDataHeaderSize = OdinConst::kDataHeaderSize;
constexpr size_t kUdpMaxDataSize = 1472 - kDataHeaderSize;
constexpr uint16_t kPclWidth = 256;
constexpr uint16_t kPclHeight = 192;

OdinFrameAssembler::OdinFrameAssembler(OdinDeviceHandle handle, bool use_tcp)
    : handle_(handle), use_tcp_(use_tcp) {}

void OdinFrameAssembler::Reset() {
  // Match processing lock order: TCP buffer, then frame state.
  std::lock_guard<std::mutex> tcp_lock(tcp_buffer_mutex_);
  std::lock_guard<std::mutex> lock(frame_mutex_);
  raw_point_state_.Reset();
  slam_point_state_.Reset();
  image_state_.Reset();

  tcp_buffer_.Clear();
}

// =============================================================================
// Parsing helpers
// =============================================================================

bool OdinFrameAssembler::ParsePointCloudPacket(const uint8_t* data, size_t length,
                                                OdinPointCloudPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;

  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->point_count = header->dot_or_sample_count;
  packet->udp_count = header->udp_count;
  packet->frame_count = header->frame_count;
  packet->data_type = header->data_type;
  packet->time_type = header->time_type;
  packet->reserved = header->reserved;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseImagePacket(const uint8_t* data, size_t length,
                                           OdinImagePacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->udp_count = header->udp_count;
  packet->frame_count = header->frame_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseImuPacket(const uint8_t* data, size_t length,
                                         OdinImuPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->sample_count = header->dot_or_sample_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseOdomPacket(const uint8_t* data, size_t length,
                                          OdinOdomPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->data_count = header->dot_or_sample_count;
  packet->frame_count = header->frame_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

// =============================================================================
// TCP buffer processing
// =============================================================================

void OdinFrameAssembler::ProcessTcpBuffer(
    std::function<void(const uint8_t*, size_t)> handler) {
  std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);

  while (tcp_buffer_.Size() >= kDataHeaderSize) {
    const uint8_t* buf = tcp_buffer_.Data();

    // Parse header to get frame length
    const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(buf);
    uint16_t frame_len = header->length;

    if (frame_len < kDataHeaderSize) {
      LOG_WARN("Invalid TCP frame length %u; clearing stream buffer\n", frame_len);
      tcp_buffer_.Clear();
      break;
    }

    if (tcp_buffer_.Size() < frame_len) {
      // Incomplete frame, wait for more data
      break;
    }

    // Complete frame, process it
    handler(buf, frame_len);
    tcp_buffer_.Consume(frame_len);
  }
}

// =============================================================================
// Point cloud processing
// =============================================================================

void OdinFrameAssembler::ProcessPointFragment(const OdinPointCloudPacket& fragment,
                                               bool is_slam) {
  // Raw is a fixed raster. SLAM is a variable-length filtered cloud: its
  // short last packet (or the next frame) determines the final point count.
  const size_t point_size = is_slam ? sizeof(OdinSlamPoint<uint16_t>)
                                   : sizeof(OdinRawPoint<uint16_t>);
  const size_t total_points = kPclWidth * kPclHeight;
  const size_t points_per_packet = kUdpMaxDataSize / point_size;
  const size_t expected_packets = (total_points + points_per_packet - 1) / points_per_packet;
  const size_t seq = fragment.udp_count;  // Firmware uses zero-based packet indices.
  if (seq >= (is_slam ? (65535 + points_per_packet - 1) / points_per_packet : expected_packets)) return;
  const size_t offset = seq * points_per_packet;
  const size_t expected_points = is_slam ? fragment.point_count :
      std::min(points_per_packet, total_points - offset);
  if (expected_points == 0 || expected_points > points_per_packet ||
      offset + expected_points > 65535 || fragment.point_count != expected_points ||
      fragment.payload.size() != expected_points * point_size) {
    LOG_WARN("Invalid point fragment: seq=%zu points=%u bytes=%zu\n",
             seq, fragment.point_count, fragment.payload.size());
    return;
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;
  UdpStats& stats = is_slam ? slam_stats_ : raw_stats_;
  ++stats.packets;
  if (state.has_frame && fragment.frame_count != state.packet.frame_count) {
    // Ignore delayed older frames; unsigned subtraction also handles counter wrap.
    if (static_cast<uint32_t>(fragment.frame_count - state.packet.frame_count) >= 0x80000000u)
      return;
    if (state.active) EmitPointFrame(is_slam);
    state.has_frame = false;
  }
  if (!state.has_frame) {
    state.packet = fragment;
    state.packet.payload.assign(is_slam ? 0 : total_points * point_size, 0);
    state.packet.point_count = is_slam ? 0 : static_cast<uint16_t>(total_points);
    state.received.assign(is_slam ? 0 : expected_packets, false);
    state.terminal_received = !is_slam;
    state.udp_segments = 0;
    state.highest_sequence = 0;
    state.highest_sequence_valid = false;
    state.active = true;
    state.has_frame = true;
  }
  // A duplicate after completion must not start a second copy of the same frame.
  if (seq < state.received.size() && state.received[seq])
  {
    ++stats.duplicate_packets;
    return;
  }
  if (!state.active) return;
  if (state.highest_sequence_valid && seq < state.highest_sequence) ++stats.reordered_packets;
  state.highest_sequence = std::max(state.highest_sequence, static_cast<uint16_t>(seq));
  state.highest_sequence_valid = true;
  if (is_slam) {
    if (state.terminal_received && seq >= state.received.size()) return;
    if (fragment.point_count < points_per_packet && seq + 1 < state.received.size()) return;
    if (seq >= state.received.size()) {
      state.received.resize(seq + 1, false);
      state.packet.payload.resize((offset + expected_points) * point_size, 0);
      state.packet.point_count = static_cast<uint16_t>(offset + expected_points);
    }
    if (fragment.point_count < points_per_packet) state.terminal_received = true;
  }
  std::memcpy(state.packet.payload.data() + offset * point_size,
              fragment.payload.data(), fragment.payload.size());
  state.received[seq] = true;
  ++state.udp_segments;
  if (state.terminal_received && state.udp_segments == state.received.size())
    EmitPointFrame(is_slam);
}

void OdinFrameAssembler::EmitPointFrame(bool is_slam) {
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  if (!state.active || state.packet.payload.empty()) {
    state.active = false;
    return;
  }

  if (state.udp_segments != state.received.size()) {
    UdpStats& stats = is_slam ? slam_stats_ : raw_stats_;
    ++stats.incomplete_frames;
    stats.missing_packets += state.received.size() - state.udp_segments;
    LOG_WARN("Incomplete %s cloud frame %u: received %u/%zu packets; %s\n",
             is_slam ? "SLAM" : "raw", state.packet.frame_count,
             state.udp_segments, state.received.size(),
             is_slam ? "dropping frame" : "missing slots remain invalid");
    // SLAM has no per-point validity/confidence field. Zero-filled wire points
    // become nonzero coordinates after the SDK transform, polluting the map.
    if (is_slam) {
      state.active = false;
      state.packet.payload.clear();
      state.received.clear();
      return;
    }
  }
  state.packet.udp_count = state.udp_segments;
  size_t total_length = kDataHeaderSize + state.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  state.packet.length = static_cast<uint16_t>(total_length);
  state.packet.device = handle_;
  UdpStats& stats = is_slam ? slam_stats_ : raw_stats_;
  ++stats.frames;

  if (is_slam && slam_cb_) {
    // Convert uint16 to float for SLAM (with -30000 offset for y,z)
    // Coordinate transformation (rotation/translation) will be done by SlamOdomSynchronizer
    size_t src_point_size = sizeof(OdinSlamPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinSlamPoint<uint16_t>* src =
        reinterpret_cast<const OdinSlamPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<uint8_t> new_payload(point_count * sizeof(OdinSlamPoint<float>));
    OdinSlamPoint<float>* dst = reinterpret_cast<OdinSlamPoint<float>*>(new_payload.data());

    for (size_t i = 0; i < point_count; ++i) {
      // Convert to float with offset (in millimeters)
      dst[i].x = static_cast<float>(src[i].x);
      dst[i].y = static_cast<float>(src[i].y) - 30000.0f;
      dst[i].z = static_cast<float>(src[i].z) - 30000.0f;
      dst[i].r = src[i].r;
      dst[i].g = src[i].g;
      dst[i].b = src[i].b;
      dst[i].a = src[i].a;
    }
    state.packet.payload = std::move(new_payload);
    slam_cb_(state.packet);
  } else if (!is_slam && point_cloud_cb_) {
    // Convert uint16 to float for raw point cloud
    size_t src_point_size = sizeof(OdinRawPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinRawPoint<uint16_t>* src =
        reinterpret_cast<const OdinRawPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<uint8_t> new_payload(point_count * sizeof(OdinRawPoint<float>));
    OdinRawPoint<float>* dst = reinterpret_cast<OdinRawPoint<float>*>(new_payload.data());

    for (size_t i = 0; i < point_count; ++i) {
      dst[i].x = static_cast<float>(src[i].x);
      dst[i].y = static_cast<float>(src[i].y) - 30000.0f;
      dst[i].z = static_cast<float>(src[i].z) - 30000.0f;
      dst[i].intensity = src[i].intensity;
      dst[i].confidence = src[i].confidence;
    }
    state.packet.payload = std::move(new_payload);
    point_cloud_cb_(state.packet);
  }

  state.active = false;
  state.packet.payload.clear();
}

void OdinFrameAssembler::ProcessRawPointData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacket packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        ProcessPointFragment(packet, false);
      }
    });
  } else {
    OdinPointCloudPacket packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      ProcessPointFragment(packet, false);
    }
  }
}

void OdinFrameAssembler::ProcessSlamPointData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacket packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        ProcessPointFragment(packet, true);
      }
    });
  } else {
    OdinPointCloudPacket packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      ProcessPointFragment(packet, true);
    }
  }
}

// =============================================================================
// Image processing
// =============================================================================

bool OdinFrameAssembler::DetectJpegTerminator(const std::vector<uint8_t>& chunk) {
  if (chunk.empty()) return false;

  if (image_state_.last_byte_valid && image_state_.last_byte == 0xFF && chunk[0] == 0xD9) {
    return true;
  }

  for (size_t i = 1; i < chunk.size(); ++i) {
    if (chunk[i - 1] == 0xFF && chunk[i] == 0xD9) return true;
  }

  image_state_.last_byte = chunk.back();
  image_state_.last_byte_valid = true;
  return false;
}

void OdinFrameAssembler::ProcessImageFragment(const OdinImagePacket& fragment) {
  // Bound memory even for malformed streams; normal JPEG frames are much smaller.
  constexpr size_t kMaxImagePackets = 4096;
  if (fragment.udp_count >= kMaxImagePackets || fragment.payload.empty() ||
      fragment.payload.size() > kUdpMaxDataSize) return;
  std::lock_guard<std::mutex> lock(frame_mutex_);
  ++image_stats_.packets;
  if (image_state_.has_frame && fragment.frame_count != image_state_.packet.frame_count) {
    if (static_cast<uint32_t>(fragment.frame_count - image_state_.packet.frame_count) >= 0x80000000u)
      return;
    if (image_state_.active) {
      ++image_stats_.incomplete_frames;
      if (image_state_.highest_sequence_valid &&
          static_cast<uint64_t>(image_state_.highest_sequence) + 1 > image_state_.udp_segments)
        image_stats_.missing_packets +=
            static_cast<uint64_t>(image_state_.highest_sequence) + 1 - image_state_.udp_segments;
      LOG_WARN("Incomplete image frame %u dropped\n", image_state_.packet.frame_count);
    }
    image_state_.Reset();
  }
  if (!image_state_.has_frame) {
    image_state_.packet = fragment;
    image_state_.packet.payload.clear();
    image_state_.active = true;
    image_state_.has_frame = true;
  }
  if (fragment.udp_count < image_state_.udp_segments) {
    ++image_stats_.duplicate_packets;
    return;
  }
  if (!image_state_.active) return;
  if (image_state_.highest_sequence_valid && fragment.udp_count < image_state_.highest_sequence)
    ++image_stats_.reordered_packets;
  image_state_.highest_sequence = std::max(image_state_.highest_sequence, fragment.udp_count);
  image_state_.highest_sequence_valid = true;
  if (!image_state_.pending.emplace(fragment.udp_count, fragment.payload).second) {
    ++image_stats_.duplicate_packets;
    return;
  }
  // JPEG bytes and the split EOI marker must be scanned in sequence order.
  while (true) {
    auto next = image_state_.pending.find(image_state_.udp_segments);
    if (next == image_state_.pending.end()) break;
    const bool complete = DetectJpegTerminator(next->second);
    image_state_.packet.payload.insert(image_state_.packet.payload.end(),
                                      next->second.begin(), next->second.end());
    image_state_.pending.erase(next);
    ++image_state_.udp_segments;
    if (complete) {
      EmitImageFrame();
      break;
    }
  }
}

void OdinFrameAssembler::EmitImageFrame() {
  if (!image_state_.active || image_state_.packet.payload.empty()) {
    image_state_.active = false;
    image_state_.pending.clear();
    image_state_.packet.payload.clear();
    return;
  }

  constexpr size_t kLengthFieldSize = 4;
  constexpr size_t kEOISize = 2;

  if (image_state_.packet.payload.size() < kLengthFieldSize + kEOISize + 2) {
    ++image_stats_.incomplete_frames;
    image_state_.active = false;
    image_state_.pending.clear();
    image_state_.packet.payload.clear();
    return;
  }

  size_t payload_size = image_state_.packet.payload.size();
  const uint8_t* len_ptr = image_state_.packet.payload.data() + payload_size - kLengthFieldSize - kEOISize;
  uint32_t expected_len =
      static_cast<uint32_t>(len_ptr[0]) << 24 | (static_cast<uint32_t>(len_ptr[1]) << 16) |
      (static_cast<uint32_t>(len_ptr[2]) << 8) | (static_cast<uint32_t>(len_ptr[3]));

  uint32_t actual_len = static_cast<uint32_t>(payload_size - kLengthFieldSize);

  if (actual_len != expected_len) {
    ++image_stats_.incomplete_frames;
    LOG_WARN("Image frame dropped: expected %u bytes, got %u\n", expected_len, actual_len);
    image_state_.active = false;
    image_state_.pending.clear();
    image_state_.packet.payload.clear();
    return;
  }

  // Restore JPEG EOI marker
  uint8_t* data = image_state_.packet.payload.data();
  size_t jpeg_data_len = payload_size - kLengthFieldSize - kEOISize;
  data[jpeg_data_len] = 0xFF;
  data[jpeg_data_len + 1] = 0xD9;
  image_state_.packet.payload.resize(actual_len);
  ++image_stats_.frames;

  image_state_.packet.udp_count = image_state_.udp_segments;
  size_t total_length = kDataHeaderSize + image_state_.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  image_state_.packet.length = static_cast<uint16_t>(total_length);
  image_state_.packet.device = handle_;

  if (image_cb_) {
    image_cb_(image_state_.packet);
  }

  image_state_.active = false;
  image_state_.pending.clear();
  image_state_.packet.payload.clear();
}

void OdinFrameAssembler::ProcessImageData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinImagePacket packet;
      if (ParseImagePacket(frame_data, frame_len, &packet)) {
        ProcessImageFragment(packet);
      }
    });
  } else {
    OdinImagePacket packet;
    if (ParseImagePacket(data, length, &packet)) {
      ProcessImageFragment(packet);
    }
  }
}

// =============================================================================
// IMU/Odom processing
// =============================================================================

void OdinFrameAssembler::ProcessImuData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinImuPacket packet;
    if (ParseImuPacket(frame_data, frame_len, &packet)) {
      if (!imu_cb_) {
        return;
      }
      
      // Detect firmware version by per-sample size and parse accordingly.
      //
      // Forward-compatible parsing strategy:
      //   - Legacy firmware: 24 bytes per sample (6 floats: gyro_xyz + acc_xyz only).
      //     Need to synthesize timestamp/sequence from packet header.
      //   - Current/future firmware: sample size >= sizeof(OdinImuSample).
      //     Fields are assumed to be appended at the end of the struct over time,
      //     so we copy min(firmware_size, current_struct_size). Any extra trailing
      //     fields the firmware sends (newer than this SDK knows about) are ignored;
      //     any fields newer than firmware sends remain zero-initialized.
      constexpr size_t kLegacySampleSize = 24;  // 6 * sizeof(float)
      constexpr size_t kCurrentSampleSize = sizeof(OdinImuSample);
      
      const size_t payload_size = packet.payload.size();
      
      // Determine per-sample size: prefer current/larger size if payload divides cleanly,
      // otherwise fall back to legacy 24-byte format.
      size_t sample_size = 0;
      if (payload_size >= kCurrentSampleSize && payload_size % kCurrentSampleSize == 0) {
        sample_size = kCurrentSampleSize;
      } else if (payload_size >= kLegacySampleSize && payload_size % kLegacySampleSize == 0) {
        sample_size = kLegacySampleSize;
      } else {
        return;  // Unknown format
      }
      
      const bool is_legacy = (sample_size == kLegacySampleSize);
      const size_t sample_count = payload_size / sample_size;
      const size_t copy_size = std::min(sample_size, kCurrentSampleSize);
      const uint8_t* raw_data = packet.payload.data();
      
      for (size_t i = 0; i < sample_count; ++i) {
        // Zero-initialize so unknown/missing fields default to 0.
        OdinImuSample sample{};
        const uint8_t* sample_ptr = raw_data + i * sample_size;
        
        // Copy known fields by their layout (struct grows by appending fields).
        std::memcpy(&sample, sample_ptr, copy_size);
        
        if (is_legacy) {
          // Legacy firmware doesn't carry per-sample timestamp/sequence - synthesize them.
          sample.timestamp = packet.timestamp;
          sample.sequence = i;
        }
        
        // Create single-sample packet and callback
        OdinImuPacket single_packet;
        single_packet.device = handle_;
        single_packet.version = packet.version;
        single_packet.sample_count = 1;
        single_packet.timestamp = sample.timestamp;
        single_packet.payload.assign(
            reinterpret_cast<const uint8_t*>(&sample),
            reinterpret_cast<const uint8_t*>(&sample) + sizeof(OdinImuSample));
        imu_cb_(single_packet);
      }
    }
  };

  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

void OdinFrameAssembler::ProcessOdomData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinOdomPacket packet;
    if (ParseOdomPacket(frame_data, frame_len, &packet)) {
      packet.device = handle_;
      
      // Extract odom_type from payload
      OdomSourceType odom_type = OdomSourceType::kOdom2ImuLow;
      if (packet.payload.size() >= sizeof(OdinOdomData)) {
        const OdinOdomData* odom_data =
            reinterpret_cast<const OdinOdomData*>(packet.payload.data());
        odom_type = static_cast<OdomSourceType>(odom_data->type);
      }
      
      if (odom_cb_) {
        odom_cb_(packet, odom_type);
      }
    }
  };

  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

}  // namespace sdk
}  // namespace odin
