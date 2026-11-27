#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

#include "ChunkReassembler.hpp"
#include "OdinProtocol.hpp"
#include "OdinPointDecoder.hpp"
#include "odin_lidar_def.h"
#include "internal/odin_lidar_inner.h"

namespace odin {
namespace sdk {

/**
 * @brief TCP stream buffer for frame reassembly
 */
struct TcpStreamBuffer {
  std::vector<uint8_t> buffer;
  
  void Append(const uint8_t* data, size_t len) {
    buffer.insert(buffer.end(), data, data + len);
  }
  
  void Clear() { buffer.clear(); }
  
  size_t Size() const { return buffer.size(); }
  
  const uint8_t* Data() const { return buffer.data(); }
  
  void Consume(size_t len) {
    if (len >= buffer.size()) {
      buffer.clear();
    } else {
      buffer.erase(buffer.begin(), buffer.begin() + len);
    }
  }
};

using PointFrameState = ChunkReassemblyState<OdinPointCloudPacketFull>;
using CameraFrameState = ChunkReassemblyState<OdinCameraPacketFull>;

/**
 * @brief Odin protocol frame assembler
 * 
 * Handles frame assembly from UDP/TCP packets for different data types.
 * Supports point cloud, camera, IMU, and odometry data.
 */
class OdinFrameAssembler {
 public:
  OdinFrameAssembler(OdinTransportMode transport);
  ~OdinFrameAssembler() = default;

  // Callback types. Point cloud / image / H264 packets are delivered BY VALUE
  // so the consumer can take ownership of the point/payload vector via
  // std::move (zero-copy handoff into e.g. the recorder); lambdas taking
  // `const T&` still bind unchanged.
  using PointCloudCallback = std::function<void(OdinPointCloudPacket)>;
  using SlamCallback = std::function<void(const OdinSlamPacket&)>;
  using CameraCallback = std::function<void(OdinCameraPacket)>;
  using ImuCallback = std::function<void(const OdinImuPacket&)>;
  using OdomCallback = std::function<void(const OdinOdomPacket&, OdomSourceType)>;
  using TfCallback = std::function<void(const OdinTfPacket&)>;

  // Set callbacks
  // Format stamped on frames emitted via the camera (non-H264) path, e.g.
  // kImageMjpeg or kImageNv12. Set from the StartStream config so EmitCameraFrame
  // reports what the device was actually asked to send.
  void SetCameraFormat(OdinDataFormat format) { camera_format_ = format; }
  void SetPointCloudCallback(PointCloudCallback cb) { point_cloud_cb_ = cb; }
  void SetSlamCallback(SlamCallback cb) { slam_cb_ = cb; }
  void SetCameraCallback(CameraCallback cb) { camera_cb_ = cb; }
  void SetImuCallback(ImuCallback cb) { imu_cb_ = cb; }
  void SetOdomCallback(OdomCallback cb) { odom_cb_ = cb; }
  void SetTfCallback(TfCallback cb) { tf_cb_ = cb; }

  // Process raw data for different channel types
  void ProcessPointData(const uint8_t* data, size_t length, bool is_slam);
  void ProcessCameraData(const uint8_t* data, size_t length);
  void ProcessImuData(const uint8_t* data, size_t length);
  void ProcessOdomData(const uint8_t* data, size_t length);
  void ProcessTfData(const uint8_t* data, size_t length);

 private:
  // Handler invoked once per validated data frame. The view is only valid for
  // the duration of the call (it borrows the transport / TCP buffer memory).
  using FrameHandler = std::function<void(const OdinDataFrameView&)>;

  // Parsing helpers. The frame header and payload bounds are already validated
  // by ParseDataFrame(), so these only map header fields onto the internal
  // packet types; only ParseTfPacket can still fail (payload too small).
  // Odom does not need one: ProcessOdomData routes by payload generation
  // (V2/ and V3/ OdomEntry.hpp, version is the sole routing key)
  // straight from the frame view.
  void ParsePointCloudPacket(const OdinDataFrameView& frame, OdinPointCloudPacketFull* packet);
  void ParseCameraPacket(const OdinDataFrameView& frame, OdinCameraPacketFull* packet);
  bool ParseTfPacket(const OdinDataFrameView& frame, OdinTfPacketFull* packet);

  // Frame assembly (v2: driven by chunk_index + chunk_count + flags.last_chunk)
  void ProcessPointFragment(const OdinPointCloudPacketFull& fragment, bool is_slam,
                            uint16_t chunk_index, bool last_chunk);
  void EmitPointFrame(bool is_slam);
  // Fill `lost_chunks` worth of zero points into the in-progress point frame
  // when a chunk_index gap is detected, preserving spatial layout downstream.
  void PadZerosAtGapPosition(uint16_t lost_chunks, bool is_slam);
  void ProcessCameraFragment(CameraFrameState& state, OdinDataFormat format,
                            const OdinCameraPacketFull& fragment,
                            uint16_t chunk_index, bool last_chunk);
  void EmitCameraFrame(CameraFrameState& state, OdinDataFormat format);

  // TCP buffer processing
  void ProcessTcpBuffer(const FrameHandler& handler);

  // Dispatch raw transport bytes to a per-frame handler. In TCP mode the
  // data is appended to the TCP buffer and ProcessTcpBuffer extracts
  // complete frames; in UDP mode the datagram is validated directly.
  // Either way the handler only ever sees frames that passed ParseDataFrame().
  void DispatchTransportData(const uint8_t* raw_data, size_t raw_len,
                             const FrameHandler& frame_handler);

  OdinTransportMode transport_mode_;
  // Payload format for the camera (non-H264) assembly path; see SetCameraFormat.
  OdinDataFormat camera_format_ = OdinDataFormat::kImageMjpeg;

  // Whether the underlying transport is stream-oriented (requires TCP-style
  // buffered reassembly) versus datagram-oriented (one packet == one frame
  // chunk). Centralised here so future transports only need updating in one
  // place.
  bool IsStreamTransport() const {
    return transport_mode_ == OdinTransportMode::kTcp;
  }

  // Frame states
  PointFrameState raw_point_state_;
  PointFrameState slam_point_state_;
  CameraFrameState camera_state_;
  std::mutex frame_mutex_;

  // frame_seq continuity tracking for whole-frame loss logging. Per-chunk gap
  // detection cannot see a frame whose every chunk was dropped, so the
  // fragment handlers compare each incoming frame_seq against the highest one
  // observed so far. Guarded by frame_mutex_ (only touched inside
  // Process*Fragment, which holds the lock).
  struct FrameSeqTracker {
    uint32_t last = 0;   ///< highest frame_seq observed
    bool valid = false;  ///< false until the first fragment is seen
  };
  // Returns the number of frames skipped before `seq` (0 when contiguous) and
  // advances the tracker on forward progress. Wrap-safe for uint32_t seqs.
  static uint32_t TrackFrameSeq(FrameSeqTracker& tracker, uint32_t seq);
  FrameSeqTracker raw_seq_tracker_;
  FrameSeqTracker slam_seq_tracker_;
  FrameSeqTracker image_seq_tracker_;

  // TCP buffer
  TcpStreamBuffer tcp_buffer_;
  std::mutex tcp_buffer_mutex_;

  // Callbacks
  PointCloudCallback point_cloud_cb_;
  SlamCallback slam_cb_;
  CameraCallback camera_cb_;
  ImuCallback imu_cb_;
  OdomCallback odom_cb_;
  TfCallback tf_cb_;
};

}  // namespace sdk
}  // namespace odin
