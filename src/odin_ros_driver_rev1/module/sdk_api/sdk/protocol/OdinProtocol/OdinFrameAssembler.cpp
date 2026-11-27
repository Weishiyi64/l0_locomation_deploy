#include "OdinFrameAssembler.hpp"
#include "OdomLayoutRouter.hpp"  // layout structs + routing enum/function (from OdomGenerations.def)

#include <atomic>

#include <atomic>

#include "logger.h"

namespace odin {
namespace sdk {

// Constants for Odin2 data assembly (protocol v2)
constexpr size_t kDataHeaderSize = OdinConst::kDataHeaderSize;
constexpr uint8_t kFlagLastChunk = 0x01;                    // header.flags bit0 = last_chunk

// One-shot notice for forward-compatible frames: the version byte is
// newer than the SDK baseline (it has already been validated within the
// forward-compat ceiling by PeekDataFrameHeader, so the frame can be split
// safely). Such frames are parsed with the payload layouts the SDK knows;
// per-type geometry checks (layout tables, cross-checks) drop data types
// whose layout actually changed, so nothing can misparse silently. Logging
// once keeps the hot path quiet while leaving a trail for diagnosis.
static void LogForwardCompatFrameOnce(uint8_t frame_version) {
  if (OdinConst::PayloadGenerationOf(frame_version) <= OdinConst::kDataVersion) return;
  static std::atomic<bool> logged{false};
  bool expected = false;
  if (logged.compare_exchange_strong(expected, true)) {
    LOG_INFO("OdinFrameAssembler: frame payload generation %u is newer than SDK baseline "
             "%u; parsing with known layouts (changed layouts drop per type)\n",
             static_cast<unsigned>(OdinConst::PayloadGenerationOf(frame_version)),
             static_cast<unsigned>(OdinConst::kDataVersion));
  }
}

// Geometry gate for fixed-layout payload types that currently have a single
// registered layout (point cloud, IMU). The frame is usable only when
// payload_size == element_count * element_size; anything else means the
// firmware generation changed the payload layout in a way this SDK has no
// registered layout for, so frames are dropped instead of parsed at wrong
// offsets (no silent misparse). The first mismatch per call site logs once
// (the throttling flag is owned by the caller); the drop itself is silent
// after that so a steady stream of unusable frames cannot flood the log.
//
// Reserved window: when a second layout for a type appears (firmware bumps
// the payload generation and changes the element size), give the layout its
// own file in the generation folder (V2/, V3/, ...) with its own
// ConvertToPublic and route by generation - copy the odometry pattern
// (V2/OdomEntry.hpp + V3/OdomEntry.hpp plus the OdomGenerations.def
// manifest, expanded by OdomLayoutRouter.hpp and the dispatch switch).
static bool CheckPayloadGeometry(const char* type_label, size_t payload_size,
                                 size_t element_count, size_t element_size,
                                 std::atomic<bool>& logged) {
  if (payload_size == element_count * element_size) return true;
  bool expected = false;
  if (logged.compare_exchange_strong(expected, true)) {
    LOG_WARN("%s layout mismatch: payload=%zu != element_count=%zu * %zu B/element; "
             "dropping frames (firmware payload layout newer than this SDK?)\n",
             type_label, payload_size, element_count, element_size);
  }
  return false;
}

// Throttled warn for malformed packets dropped by the parse layer. A single
// shared counter covers all channels; the message identifies the channel.
static void LogMalformedPacket(const char* channel) {
  static std::atomic<uint64_t> total{0};
  const uint64_t n = total.fetch_add(1) + 1;
  if (n == 1 || n % 100 == 0) {
    LOG_WARN("Assembler: malformed packet dropped on %s channel (total %llu)\n",
             channel, static_cast<unsigned long long>(n));
  }
}

OdinFrameAssembler::OdinFrameAssembler(OdinTransportMode transport)
    : transport_mode_(transport) {}

// =============================================================================
// Parsing helpers
// =============================================================================

// Fields shared by every internal *PacketFull type. Extracted so each Parse*
// helper only describes what is actually channel-specific.
template <typename PacketT>
static void FillCommonPacketFields(const OdinDataFrameView& frame, PacketT* packet) {
  packet->version = frame.header->version;
  packet->length = static_cast<uint16_t>(kDataHeaderSize + frame.payload_size);
  packet->frame_count = frame.header->frame_seq;
  packet->timestamp = frame.header->timestamp;
  packet->payload.assign(frame.payload, frame.payload + frame.payload_size);
}

void OdinFrameAssembler::ParsePointCloudPacket(const OdinDataFrameView& frame,
                                                OdinPointCloudPacketFull* packet) {
  FillCommonPacketFields(frame, packet);
  packet->point_count = frame.header->element_count;  // points in THIS chunk
  packet->chunk_count = frame.header->chunk_count;
  packet->data_type = frame.header->data_type;
}

void OdinFrameAssembler::ParseCameraPacket(const OdinDataFrameView& frame,
                                           OdinCameraPacketFull* packet) {
  FillCommonPacketFields(frame, packet);
  packet->chunk_count = frame.header->chunk_count;
}

bool OdinFrameAssembler::ParseTfPacket(const OdinDataFrameView& frame,
                                        OdinTfPacketFull* packet) {
  if (frame.payload_size < sizeof(OdinTfData)) return false;

  FillCommonPacketFields(frame, packet);

  const OdinTfData* tf = reinterpret_cast<const OdinTfData*>(frame.payload);
  std::memcpy(packet->translation, tf->translation, sizeof(tf->translation));
  std::memcpy(packet->rotation, tf->rotation, sizeof(tf->rotation));
  // char[16] frame ids may or may not be NUL-terminated; bound by strnlen
  // to safely construct std::string for direct ROS message assignment.
  packet->frame_id.assign(
      tf->frame_id,
      ::strnlen(tf->frame_id, sizeof(tf->frame_id)));
  packet->child_frame_id.assign(
      tf->child_frame_id,
      ::strnlen(tf->child_frame_id, sizeof(tf->child_frame_id)));
  return true;
}

// =============================================================================
// TCP buffer processing
// =============================================================================

void OdinFrameAssembler::ProcessTcpBuffer(const FrameHandler& handler) {
  std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);

  while (tcp_buffer_.Size() >= kDataHeaderSize) {
    const uint8_t* buf = tcp_buffer_.Data();

    // Stream mode must inspect the header before the payload has arrived, so
    // the header-only check is used here; the full frame is validated below.
    const OdinDataFrameHeader* header = PeekDataFrameHeader(buf, tcp_buffer_.Size());
    if (header == nullptr) {
      // Bad header: discard entire buffer and resync. Read the fields before
      // Clear() -- `buf` points into tcp_buffer_ and is invalidated.
      const auto* raw = reinterpret_cast<const OdinDataFrameHeader*>(buf);
      const unsigned bad_sof = raw->sof;
      const unsigned bad_ver = raw->version;
      const size_t discarded = tcp_buffer_.Size();
      tcp_buffer_.Clear();
      static std::atomic<uint64_t> resync_count{0};
      const uint64_t n = resync_count.fetch_add(1) + 1;
      if (n == 1 || n % 16 == 0) {
        LOG_WARN("Assembler: TCP resync #%llu: bad header (sof=0x%02x ver=%u), "
                 "discarded %zuB buffered data\n",
                 static_cast<unsigned long long>(n), bad_sof, bad_ver,
                 discarded);
      }
      break;
    }

    // v2: total frame length = header + pure payload_len
    const size_t frame_len = kDataHeaderSize + header->payload_len;

    if (tcp_buffer_.Size() < frame_len) {
      // Incomplete frame, wait for more data
      break;
    }

    OdinDataFrameView frame;
    if (ParseDataFrame(buf, frame_len, &frame)) {
      LogForwardCompatFrameOnce(frame.header->version);
      handler(frame);
    } else {
      LogMalformedPacket("tcp");
    }
    tcp_buffer_.Consume(frame_len);
  }
}

void OdinFrameAssembler::DispatchTransportData(const uint8_t* raw_data, size_t raw_len,
                                               const FrameHandler& frame_handler) {
  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(raw_data, raw_len);
    }
    ProcessTcpBuffer(frame_handler);
  } else {
    // Datagram mode: one datagram == one frame. Validated here so UDP and TCP
    // share the exact same acceptance criteria.
    OdinDataFrameView frame;
    if (!ParseDataFrame(raw_data, raw_len, &frame)) {
      LOG_WARN("Dropping malformed data frame (%zuB): bad sof, unknown header "
               "generation, payload generation below baseline, or truncated payload\n",
               raw_len);
      return;
    }
    LogForwardCompatFrameOnce(frame.header->version);
    frame_handler(frame);
  }
}

// =============================================================================
// Point cloud processing
// =============================================================================

uint32_t OdinFrameAssembler::TrackFrameSeq(FrameSeqTracker& tracker,
                                         uint32_t seq) {
  uint32_t lost = 0;
  if (tracker.valid) {
    // Signed difference of two uint32 seqs: correct across the 2^32 wrap.
    const int32_t delta = static_cast<int32_t>(seq - tracker.last);
    if (delta > 1) {
      lost = static_cast<uint32_t>(delta) - 1;
    }
    if (delta > 0) {
      tracker.last = seq;
    } else if (delta < -1000) {
      // frame_seq went backwards by a large margin: the device stream was
      // restarted (seq reset). Resync so gap detection keeps working; small
      // negative deltas are just late/reordered chunks and must not resync.
      tracker.last = seq;
    }
  } else {
    tracker.last = seq;
    tracker.valid = true;
  }
  return lost;
}

void OdinFrameAssembler::ProcessPointFragment(const OdinPointCloudPacketFull& fragment,
                                               bool is_slam, uint16_t chunk_index,
                                               bool last_chunk) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  // Whole-frame loss detection: the per-chunk gap check below cannot see a
  // frame whose every chunk was dropped, so track frame_seq continuity
  // explicitly. Late/reordered chunks (delta <= 0) never count as loss.
  const uint32_t lost_frames = TrackFrameSeq(
      is_slam ? slam_seq_tracker_ : raw_seq_tracker_, fragment.frame_count);
  if (lost_frames > 0) {
    LOG_WARN("%s point cloud: %u frame(s) lost (frame_seq jumped %u -> %u)\n",
             is_slam ? "SLAM" : "Raw",
             static_cast<unsigned>(lost_frames),
             static_cast<unsigned>(fragment.frame_count - lost_frames - 1),
             static_cast<unsigned>(fragment.frame_count));
    LogNetBufferHintOnce();
  }

  if (state.ShouldStartNewFrame(fragment.frame_count, chunk_index)) {
    EmitPointFrame(is_slam);
  }

  if (!state.active) {
    state.InitFromFragment(fragment, fragment.chunk_count);
    state.packet.payload.reserve(
        static_cast<size_t>(fragment.chunk_count) * kUdpMaxDataSize);
    state.packet.point_count = 0;
    // Head loss detection: the first fragment of a new frame must be chunk 0.
    // A late/reordered chunk of an already-emitted frame can resurrect the
    // state and trip this too; the warning is still accurate (data lost).
    if (chunk_index != 0) {
      LOG_WARN("%s point frame_seq=%u head lost: first received chunk is %u "
               "(chunks 0..%u missing)\n",
               is_slam ? "SLAM" : "Raw",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(chunk_index),
               static_cast<unsigned>(chunk_index - 1));
      LogNetBufferHintOnce();
    }
  }

  uint16_t lost = state.DetectGap(chunk_index);
  if (lost > 0) {
    LOG_WARN("Point cloud chunk gap: expected %u got %u, filling %u chunk(s) with zeros\n",
             static_cast<unsigned>(state.last_chunk_index + 1),
             static_cast<unsigned>(chunk_index),
             static_cast<unsigned>(lost));
    LogNetBufferHintOnce();
    PadZerosAtGapPosition(lost, is_slam);
  } else if (state.received_chunks > 0 && chunk_index <= state.last_chunk_index) {
    // Reordered/duplicate chunk: payload is appended in arrival order, so the
    // emitted frame is silently corrupt -- make it visible (mirrors H264).
    LOG_WARN("%s point chunk reorder in frame_seq=%u: got chunk %u after %u "
             "(points assembled out of order)\n",
             is_slam ? "SLAM" : "Raw",
             static_cast<unsigned>(state.packet.frame_count),
             static_cast<unsigned>(chunk_index),
             static_cast<unsigned>(state.last_chunk_index));
  }

  state.AppendPayload(fragment.payload);
  state.packet.point_count += fragment.point_count;
  state.UpdateChunkInfo(chunk_index, last_chunk);

  if (state.IsComplete(last_chunk)) {
    EmitPointFrame(is_slam);
  }
}

void OdinFrameAssembler::EmitPointFrame(bool is_slam) {
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  if (!state.active) {
    state.Reset();
    return;
  }
  if (state.packet.payload.empty()) {
    // Frame was closed (last_chunk / flush) without a single byte of payload --
    // the emit below would be a silent no-op, so report the discarded frame.
    LOG_WARN("%s point frame_seq=%u dropped: frame closed with empty payload "
             "(%u chunk(s) received)\n",
             is_slam ? "SLAM" : "Raw",
             static_cast<unsigned>(state.packet.frame_count),
             static_cast<unsigned>(state.received_chunks));
    state.Reset();
    return;
  }

  // Tail loss / truncation: DetectGap only sees mid-frame holes (a later
  // chunk in the same frame reveals the jump). Chunks missing at the tail --
  // or a premature last_chunk flag completing the frame early -- would
  // otherwise emit a silently-short frame. Pad like mid-frame gaps so the
  // emitted point count stays consistent. Throttled like LogMalformedPacket:
  // under sustained loss every frame would trip this.
  static std::atomic<uint64_t> tail_lost_total{0};
  if (state.expected_chunks > 0 &&
      state.received_chunks < state.expected_chunks) {
    const uint16_t lost = static_cast<uint16_t>(
        state.expected_chunks - state.received_chunks);
    const uint64_t n = tail_lost_total.fetch_add(1) + 1;
    if (n == 1 || n % 100 == 0) {
      LOG_WARN("%s point frame_seq=%u incomplete: %u/%u chunks received, "
               "padding %u missing chunk(s) with zeros (total %llu)\n",
               is_slam ? "SLAM" : "Raw",
               static_cast<unsigned>(state.packet.frame_count),
               static_cast<unsigned>(state.received_chunks),
               static_cast<unsigned>(state.expected_chunks),
               static_cast<unsigned>(lost),
               static_cast<unsigned long long>(n));
    }
    PadZerosAtGapPosition(lost, is_slam);
    LogNetBufferHintOnce();
  } else if (state.expected_chunks == 0 && !state.last_chunk_received) {
    // chunk_count unset (firmware left it 0): the missing tail cannot be
    // estimated, so no padding -- but the frame is still incomplete.
    const uint64_t n = tail_lost_total.fetch_add(1) + 1;
    if (n == 1 || n % 100 == 0) {
      LOG_WARN("%s point frame_seq=%u incomplete: %u chunk(s) received, "
               "chunk_count unset (total %llu)\n",
               is_slam ? "SLAM" : "Raw",
               static_cast<unsigned>(state.packet.frame_count),
               static_cast<unsigned>(state.received_chunks),
               static_cast<unsigned long long>(n));
    }
    LogNetBufferHintOnce();
  }

  state.packet.chunk_count = state.received_chunks;
  size_t total_length = kDataHeaderSize + state.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  state.packet.length = static_cast<uint16_t>(total_length);

  PointFrameInfo info;
  info.expected_chunks = state.expected_chunks;
  info.last_chunk_received = state.last_chunk_received;
  info.frame_count = state.packet.frame_count;

  if (is_slam && slam_cb_) {
    auto points = OdinPointDecoderV2::DecodeSlam(
        state.packet.payload.data(), state.packet.payload.size(), info);
    OdinSlamPacket out;
    out.frame_count = state.packet.frame_count;
    out.timestamp = state.packet.timestamp;
    out.points = std::move(points);
    slam_cb_(out);
  } else if (!is_slam && point_cloud_cb_) {
    auto points = OdinPointDecoderV2::DecodeRaw(
        state.packet.payload.data(), state.packet.payload.size(), info);
    OdinPointCloudPacket out;
    out.frame_count = state.packet.frame_count;
    out.timestamp = state.packet.timestamp;
    out.points = std::move(points);
    point_cloud_cb_(std::move(out));
  }

  state.Reset();
}

void OdinFrameAssembler::PadZerosAtGapPosition(uint16_t lost_chunks, bool is_slam) {
  if (lost_chunks == 0) return;

  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  // Estimate per-chunk point count. v2 chunks are not strictly fixed-size, so
  // use the same heuristic as the legacy v1 zero-fill: assume each lost chunk
  // carries up to (kUdpMaxDataSize / point_size) points (the typical UDP
  // payload). Downstream consumers tolerate slightly under/over filled frames;
  // the goal is to keep the spatial layout aligned, not bit-exact reconstruction.
  const size_t point_size =
      is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
  if (point_size == 0) return;
  const size_t pts_per_chunk = kUdpMaxDataSize / point_size;
  if (pts_per_chunk == 0) return;

  const size_t lost_points = static_cast<size_t>(lost_chunks) * pts_per_chunk;
  const size_t fill_bytes = lost_points * point_size;

  state.packet.payload.insert(state.packet.payload.end(), fill_bytes, 0);

  uint32_t new_total = static_cast<uint32_t>(state.packet.point_count) + lost_points;
  if (new_total > 65535) new_total = 65535;
  state.packet.point_count = static_cast<uint16_t>(new_total);

  // Account for the missing chunks so chunk_count accounting and completion
  // checks remain consistent with expected_chunks.
  state.received_chunks = static_cast<uint16_t>(state.received_chunks + lost_chunks);
  state.last_chunk_index = static_cast<uint16_t>(state.last_chunk_index + lost_chunks);
}

void OdinFrameAssembler::ProcessPointData(const uint8_t* data, size_t length, bool is_slam) {
  DispatchTransportData(data, length, [this, is_slam](const OdinDataFrameView& frame) {
    OdinPointCloudPacketFull packet;
    ParsePointCloudPacket(frame, &packet);

    // Fixed-layout geometry gate (see CheckPayloadGeometry for the policy and
    // the promotion recipe for a future second layout). Dropped chunks never
    // activate the reassembly state, so there is no gap/zero-fill noise.
    const size_t point_size = is_slam ? sizeof(OdinSlamPoint<uint16_t>)
                                      : sizeof(OdinRawPoint<uint16_t>);
    static std::atomic<bool> point_layout_logged{false};
    if (!CheckPayloadGeometry(is_slam ? "SLAM point" : "Raw point",
                              packet.payload.size(), packet.point_count, point_size,
                              point_layout_logged)) {
      return;
    }

    ProcessPointFragment(packet, is_slam, frame.header->chunk_index,
                         (frame.header->flags & kFlagLastChunk) != 0);
  });
}

// =============================================================================
// Camera processing
// =============================================================================

void OdinFrameAssembler::ProcessCameraFragment(CameraFrameState& state,
                                               OdinDataFormat format,
                                               const OdinCameraPacketFull& fragment,
                                               uint16_t chunk_index, bool last_chunk) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  // Whole-frame loss detection (same frame_seq tracking as point clouds).
  const uint32_t lost_frames =
      TrackFrameSeq(image_seq_tracker_, fragment.frame_count);
  if (lost_frames > 0) {
    LOG_WARN("Camera frame(s) lost: %u (frame_seq jumped %u -> %u)\n",
             static_cast<unsigned>(lost_frames),
             static_cast<unsigned>(fragment.frame_count - lost_frames - 1),
             static_cast<unsigned>(fragment.frame_count));
    LogNetBufferHintOnce();
  }

  if (state.ShouldStartNewFrame(fragment.frame_count, chunk_index)) {
    // The previous frame is being flushed by the NEXT frame's fragment instead
    // of its own last_chunk flag / chunk_count — its tail chunks were lost
    // (silent otherwise when the device leaves chunk_count = 0).
    LOG_WARN("Camera frame_seq=%u tail lost: flushed by next frame, %u chunk(s) received (%zuB)\n",
             static_cast<unsigned>(state.packet.frame_count),
             static_cast<unsigned>(state.received_chunks),
             state.packet.payload.size());
    LogNetBufferHintOnce();
    EmitCameraFrame(state, format);
  }

  if (!state.active) {
    state.InitFromFragment(fragment, fragment.chunk_count);
    // Head loss detection: the first fragment of a new frame must be chunk 0.
    if (chunk_index != 0) {
      LOG_WARN("Camera frame_seq=%u head lost: first received chunk is %u (chunks 0..%u missing)\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(chunk_index),
               static_cast<unsigned>(chunk_index - 1));
      LogNetBufferHintOnce();
    }
  }

  // Chunk continuity detection: unlike point cloud there is no zero-fill for
  // video — the damaged frame is still delivered (decoder conceals), but log
  // both forward gaps (chunk loss) and backward jumps (UDP reordering).
  if (state.received_chunks > 0) {
    if (chunk_index > state.last_chunk_index + 1) {
      uint16_t lost = static_cast<uint16_t>(chunk_index - state.last_chunk_index - 1);
      LOG_WARN("Camera chunk gap in frame_seq=%u: expected chunk %u got %u, %u chunk(s) lost\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(state.last_chunk_index + 1),
               static_cast<unsigned>(chunk_index), static_cast<unsigned>(lost));
      LogNetBufferHintOnce();
    } else if (chunk_index <= state.last_chunk_index) {
      LOG_WARN("Image chunk reorder in frame_seq=%u: got chunk %u after %u "
               "(payload assembled out of order)\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(chunk_index),
               static_cast<unsigned>(state.last_chunk_index));
    }
  }

  state.AppendPayload(fragment.payload);
  state.UpdateChunkInfo(chunk_index, last_chunk);

  if (state.IsComplete(last_chunk)) {
    EmitCameraFrame(state, format);
  }
}

void OdinFrameAssembler::EmitCameraFrame(CameraFrameState& state, OdinDataFormat format) {
  if (!state.active) {
    state.Reset();
    return;
  }
  if (state.packet.payload.empty()) {
    // Frame was closed (last_chunk / flush) without a single byte of payload --
    // the emit below would be a silent no-op, so report the discarded frame.
    LOG_WARN("Camera frame_seq=%u dropped: frame closed with empty payload "
             "(%u chunk(s) received)\n",
             static_cast<unsigned>(state.packet.frame_count),
             static_cast<unsigned>(state.received_chunks));
    state.Reset();
    return;
  }

  // Report truncated frames (missing chunks) before delivery.
  if (state.expected_chunks > 0 &&
      state.received_chunks < state.expected_chunks) {
    LOG_WARN("Camera frame_seq=%u truncated: %u/%u chunks received (%zuB)\n",
             static_cast<unsigned>(state.packet.frame_count),
             static_cast<unsigned>(state.received_chunks),
             static_cast<unsigned>(state.expected_chunks),
             state.packet.payload.size());
  }

  // v2: payload is the raw encoded byte stream delivered as-is (JPEG carries
  // its own SOI/EOI; NV12 is raw planar data; H264 is Annex-B access units).
  // No trailing 4B length field.
  size_t total_length = kDataHeaderSize + state.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  state.packet.chunk_count = state.received_chunks;
  state.packet.length = static_cast<uint16_t>(total_length);

  if (camera_cb_) {
    // Convert the full-member internal OdinCameraPacketFull to the public
    // OdinCameraPacket. format is passed by the caller (kImageMjpeg /
    // kImageNv12 / kImageH264). frame_count is forwarded from the v2
    // frame_seq for drop detection. width/height are left 0 here; the
    // StartStream lambda fills them from the user-provided OdinStreamCfg.
    OdinCameraPacket out;
    out.format      = format;
    out.timestamp   = state.packet.timestamp;
    out.frame_count = state.packet.frame_count;
    out.payload     = std::move(state.packet.payload);
    camera_cb_(std::move(out));
  }

  state.Reset();
}

void OdinFrameAssembler::ProcessCameraData(const uint8_t* data, size_t length) {
  DispatchTransportData(data, length, [this](const OdinDataFrameView& frame) {
    OdinCameraPacketFull packet;
    ParseCameraPacket(frame, &packet);
    ProcessCameraFragment(camera_state_, camera_format_, packet,
                         frame.header->chunk_index,
                         (frame.header->flags & kFlagLastChunk) != 0);
  });
}

// =============================================================================
// IMU/Odom processing
// =============================================================================

void OdinFrameAssembler::ProcessImuData(const uint8_t* data, size_t length) {
  DispatchTransportData(data, length, [this](const OdinDataFrameView& frame) {
    if (!imu_cb_) return;

    // v2: payload is a pure array of OdinImuSample (24B: gyro + acc only).
    // Per-sample timestamps are reconstructed as header.timestamp + j*interval_ns;
    // each chunk is self-contained (no cross-chunk reassembly).
    constexpr size_t kSampleSize = sizeof(OdinImuSample);  // 24
    const size_t payload_size = frame.payload_size;
    const size_t sample_count = frame.header->element_count;
    // Fixed-layout geometry gate (see CheckPayloadGeometry for the policy and
    // the promotion recipe for a future second layout).
    static std::atomic<bool> imu_layout_logged{false};
    if (!CheckPayloadGeometry("IMU", payload_size, sample_count, kSampleSize,
                              imu_layout_logged)) {
      return;
    }
    const uint8_t* raw_data = frame.payload;
    const uint64_t base_ts = frame.header->timestamp;
    const uint64_t interval_ns = frame.header->interval_ns;

    for (size_t j = 0; j < sample_count; ++j) {
      OdinImuSample sample{};
      std::memcpy(&sample, raw_data + j * kSampleSize, kSampleSize);

      // Emit one single-sample packet per sample; the reconstructed per-sample
      // timestamp is delivered via OdinImuPacket.timestamp.
      OdinImuPacket single_packet;
      single_packet.timestamp = base_ts + static_cast<uint64_t>(j) * interval_ns;
      single_packet.samples.resize(1);
      single_packet.samples[0] = sample;
      imu_cb_(single_packet);
    }
  });
}

namespace {

// OdomLayoutGen, OdomLayoutForGeneration(), OdomLayoutTailBudget() and
// kOdomMaxTailExt live in OdomLayoutRouter.hpp - all generated from the
// OdomGenerations.def manifest, so registering a new generation never
// edits this file.

// Convert one frame's payload as an array of EntryT (the layout selected by
// the payload generation). max_tail_bytes = 0 enforces strict geometry
// (payload_size == element_count * sizeof(EntryT)); a larger budget allows
// same-generation tail extensions (see the block comment above) while the
// three guards - exact divisibility, tail cap, valid `type` per entry -
// keep wrong-version stamps and corrupted counts out. Returns false when
// the frame does not fit; the caller drops such frames loudly.
template <typename EntryT>
bool ConvertOdomFrame(const uint8_t* payload, size_t payload_size, uint32_t count,
                      size_t max_tail_bytes, std::vector<OdinOdomData>& out) {
  if (count == 0 || payload_size % count != 0) return false;
  const size_t stride = payload_size / count;
  const size_t base = sizeof(EntryT);
  if (stride < base) return false;
  const size_t tail = stride - base;
  if (tail > max_tail_bytes) return false;
  out.resize(count);
  const uint8_t* src = payload;
  for (uint32_t i = 0; i < count; ++i, src += stride) {
    ConvertToPublic(*reinterpret_cast<const EntryT*>(src), src + base, tail, out[i]);
    if (tail > 0 &&
        static_cast<uint8_t>(out[i].type) >
            static_cast<uint8_t>(OdomSourceType::kOdom2ImuHigh)) {
      return false;  // base fields do not line up: this is not a tail extension
    }
  }
  if (tail > 0) {
    static std::atomic<bool> tail_logged{false};
    bool expected = false;
    if (tail_logged.compare_exchange_strong(expected, true)) {
      LOG_INFO("Odom frame carries a same-generation tail extension (stride=%zu > base "
               "%zu, %zu B/entry appended): tail fields the SDK has learned are parsed, "
               "the rest are ignored\n",
               stride, base, tail);
    }
  }
  return true;
}

// Manifest dispatch: one branch per registered layout, expanded from
// OdomGenerations.def. Each case returns directly (no converted flag, no
// fallthrough); the tail budget comes from the same manifest line. Kept at
// namespace scope beside ConvertOdomFrame so the generated machinery lives
// together and ProcessOdomData stays plain prose.
bool ConvertOdomFrameForGeneration(const uint8_t* payload, size_t payload_size,
                                   uint32_t count, uint8_t payload_gen,
                                   std::vector<OdinOdomData>& out) {
  switch (protocol::OdomLayoutForGeneration(payload_gen)) {
#define X(MIN_GEN, ENTRY_TYPE, TAG, LENIENT)                                     \
    case protocol::OdomLayoutGen::TAG:                                           \
      return ConvertOdomFrame<ENTRY_TYPE>(                                       \
          payload, payload_size, count,                                          \
          protocol::OdomLayoutTailBudget(protocol::OdomLayoutGen::TAG), out);
#include "OdomGenerations.def"
#undef X
    case protocol::OdomLayoutGen::kNone:
      return false;  // unreachable in practice: framing rejects generations < 2
  }
  return false;
}

}  // namespace

void OdinFrameAssembler::ProcessOdomData(const uint8_t* data, size_t length) {
  DispatchTransportData(data, length, [this](const OdinDataFrameView& frame) {
    if (!odom_cb_) return;

    const uint32_t count = frame.header->element_count;
    const size_t payload_size = frame.payload_size;
    const uint8_t payload_gen = OdinConst::PayloadGenerationOf(frame.header->version);

    OdinOdomPacket out;
    out.frame_count = frame.header->frame_seq;
    out.timestamp = frame.header->timestamp;

    // Version-keyed routing: the payload generation selects the single
    // registered layout for it (OdomGenerations.def, newest first) and its
    // tail budget; the guards inside ConvertOdomFrame decide whether the
    // bytes actually match. Historical generations parse strictly, the
    // newest carries the lenient tail window.
    if (!ConvertOdomFrameForGeneration(frame.payload, payload_size, count, payload_gen,
                                       out.data)) {
      LOG_WARN(
          "Odom frame rejected: payload generation %u (version=0x%02X) does not match "
          "its bytes (payload=%zu element_count=%u). Each generation has exactly one "
          "registered base layout (see OdomGenerations.def); base-layout changes "
          "require a generation bump, and tail extensions are accepted only on the "
          "newest generation (up to %zu B/entry)\n",
          payload_gen, frame.header->version, payload_size, count,
          protocol::kOdomMaxTailExt);
      return;
    }

    // Odom source type of the first entry (per-entry in the payload; the
    // SDK exposes it per packet).
    const OdomSourceType odom_type =
        out.data.empty() ? OdomSourceType::kOdom2ImuLow
                         : static_cast<OdomSourceType>(out.data[0].type);
    odom_cb_(out, odom_type);
  });
}

void OdinFrameAssembler::ProcessTfData(const uint8_t* data, size_t length) {
  DispatchTransportData(data, length, [this](const OdinDataFrameView& frame) {
    if (!tf_cb_) return;

    OdinTfPacketFull packet;
    if (!ParseTfPacket(frame, &packet)) return;

    OdinTfPacket out;
    out.frame_count = packet.frame_count;
    out.timestamp = packet.timestamp;
    std::memcpy(out.translation, packet.translation, sizeof(out.translation));
    std::memcpy(out.rotation, packet.rotation, sizeof(out.rotation));
    out.frame_id = std::move(packet.frame_id);
    out.child_frame_id = std::move(packet.child_frame_id);
    tf_cb_(out);
  });
}

}  // namespace sdk
}  // namespace odin
