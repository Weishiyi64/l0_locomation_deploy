#pragma once
#include "h264_decoder.h"

// This header is only meaningful when USE_JETSON_NVDEC_DECODER is defined.
// Including it on non-Jetson builds is safe but the class will be absent.
#ifdef USE_JETSON_NVDEC_DECODER

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

// Forward-declare Jetson Multimedia API types to avoid leaking headers
// into translation units that only need the interface.
class NvVideoDecoder;
class NvBuffer;
struct NvBufSurface;

namespace odin_decoder {

// Jetson NVDEC hardware H.264 decoder.
// Uses NvVideoDecoder (Jetson Multimedia API) to decode Annex-B H.264
// access units via the dedicated NVDEC hardware engine.
//
// Architecture (matches the L4T 00_video_decode reference, blocking mode):
//   * Decode() runs on the caller thread and only feeds the V4L2 output
//     plane (compressed input side).
//   * A dedicated capture thread waits for V4L2_EVENT_RESOLUTION_CHANGE,
//     sets up the capture plane and then continuously dequeues decoded
//     NV12 surfaces, converting them to BGR.
//   * Decode() returns the most recent converted frame, if any.
//
// Thread-safety: Decode() is NOT thread-safe. Each camera channel must have
// its own instance, matching the FfmpegH264Decoder contract.
class JetsonNvdecDecoder : public H264Decoder {
 public:
  explicit JetsonNvdecDecoder(const std::string& camera_label = "");
  ~JetsonNvdecDecoder() override;

  // Non-copyable, non-movable (NvVideoDecoder holds V4L2 state).
  JetsonNvdecDecoder(const JetsonNvdecDecoder&)            = delete;
  JetsonNvdecDecoder& operator=(const JetsonNvdecDecoder&) = delete;

  // Decode one Annex-B access unit into a BGR cv::Mat.
  // Returns false while buffering (e.g., before first IDR or while
  // waiting for the capture plane to be set up after resolution change).
  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;

  std::string GetName() const override { return "JetsonNvdecDecoder (NVDEC)"; }
  bool IsInitialized() const override { return initialized_; }

 private:
  // Copy one Annex-B access unit into a free output-plane buffer and queue
  // it for the hardware. Runs on the caller thread.
  void FeedInput(const uint8_t* data, size_t size);

  // Capture-thread entry point: waits for the resolution-change event,
  // configures the capture plane and drains decoded frames.
  void CaptureLoop();

  // Set up the V4L2 capture plane after the decoder detects the stream
  // resolution from SPS/PPS (signalled via V4L2_EVENT_RESOLUTION_CHANGE).
  // Returns true on success; sets capture_plane_set_up_ on success.
  bool SetupCapturePlane();

  // Convert one decoded capture buffer (NvBufSurface-backed NV12) to BGR.
  bool SurfaceToBgr(NvBuffer* buffer, cv::Mat& out);

  // Lazily allocate the pitch-linear RGBA surface used as the VIC transform
  // destination. Returns false if allocation fails.
  bool EnsureTransformSurface();

  // Release the transform destination surface (if allocated).
  void DestroyTransformSurface();

  // Emit the periodic pipeline statistics line (throttled internally).
  // Called from Decode() on the caller thread.
  void ReportStats();

  NvVideoDecoder* decoder_              = nullptr;
  bool            initialized_          = false;
  // Gate that blocks non-IDR frames after stream start or decode error.
  // Prevents garbled frames when reference frames are missing.
  bool            waiting_for_keyframe_ = true;
  // Whether the capture plane (output side) has been set up after the
  // decoder detected the stream resolution.
  std::atomic<bool> capture_plane_set_up_{false};
  // Display resolution reported by the decoder (may be smaller than the
  // aligned surface dimensions).
  int             display_width_        = 0;
  int             display_height_       = 0;
  // Pitch-linear RGBA destination for the VIC transform. NVDEC emits
  // block-linear NV12, which cannot be read directly by the CPU.
  NvBufSurface*   transform_surface_    = nullptr;

  std::thread             capture_thread_;
  std::mutex              frame_mutex_;
  std::condition_variable frame_cv_;
  // Bounded FIFO of decoded frames.  A single "latest frame" slot loses
  // every frame of a burst but the last one; the queue absorbs bursts and
  // keeps the 1 access unit : 1 published frame ratio.
  std::deque<cv::Mat>     frame_queue_;
  // Maximum backlog kept in frame_queue_; older frames are dropped as stale.
  // Initialised from kDefaultMaxLatencyFrames / ODIN_NVDEC_MAX_LATENCY_FRAMES.
  size_t                  max_latency_frames_ = 2;
  std::atomic<bool> stop_{false};
  std::atomic<bool> capture_failed_{false};

  // --- Diagnostics --------------------------------------------------------
  // Opt-in: set ODIN_NVDEC_STATS=1 to emit the periodic pipeline stats line
  // and the frame-gap warnings. Silent otherwise; the counters themselves are
  // always maintained so enabling costs nothing but the log output.
  bool                  stats_enabled_ = false;
  std::atomic<uint64_t> stat_fed_{0};        // access units queued to NVDEC
  std::atomic<uint64_t> stat_decoded_{0};    // frames dequeued + converted
  std::atomic<uint64_t> stat_delivered_{0};  // frames handed to the caller
  std::atomic<uint64_t> stat_overwritten_{0};// frames dropped on queue overflow
  std::atomic<uint64_t> stat_starved_{0};    // Decode() found no new frame
  std::atomic<uint64_t> stat_feed_wait_max_us_{0};  // output-plane dqBuffer
  std::atomic<uint64_t> stat_convert_max_us_{0};    // VIC transform + cvtColor
  // Burst diagnostics: largest interval between two consecutive Decode()
  // calls (input side) and between two decoded frames (decoder side).
  std::atomic<uint64_t> stat_input_gap_max_us_{0};
  std::atomic<uint64_t> stat_decode_gap_max_us_{0};
  std::atomic<uint64_t> stat_queue_max_{0};
  std::chrono::steady_clock::time_point last_input_time_{};
  std::chrono::steady_clock::time_point last_decoded_time_{};
  std::chrono::steady_clock::time_point last_stat_time_{};
  std::chrono::steady_clock::time_point last_delivery_time_{};
};

}  // namespace odin_decoder

#endif  // USE_JETSON_NVDEC_DECODER
