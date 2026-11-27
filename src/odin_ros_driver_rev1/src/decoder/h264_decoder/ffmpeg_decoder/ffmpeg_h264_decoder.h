#pragma once
#include "h264_decoder.h"

#include <string>

// Forward-declare FFmpeg types to avoid polluting headers with C symbols.
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace odin_decoder {

// FFmpeg H.264 decoder with hardware acceleration support.
// Tries NVDEC → CUVID → QSV → VAAPI → VDPAU at runtime, then dynamically
// discovers any other HW decoders compiled into FFmpeg, then generic HW decode
// fallback (Intel/AMD VAAPI), finally falls back to software.
// Probing is silent: per-backend failures (and FFmpeg's own probe logs) are
// collected and reported as ONE consolidated warning only when NO backend is
// available; a successful probe prints exactly one line.
// On runtime HW decode failure, falls back to software after 1 error.
// Wraps libavcodec + libswscale to decode Annex-B access units into BGR cv::Mat.
// Thread-safety: NOT thread-safe. Each camera channel must have its own instance.
class FfmpegH264Decoder : public H264Decoder {
 public:
  explicit FfmpegH264Decoder(const std::string& camera_label = "");
  ~FfmpegH264Decoder() override;

  // Non-copyable, non-movable (AVCodecContext is not safely copyable).
  FfmpegH264Decoder(const FfmpegH264Decoder&)            = delete;
  FfmpegH264Decoder& operator=(const FfmpegH264Decoder&) = delete;

  // Decode one Annex-B access unit into a BGR cv::Mat.
  // Returns false while buffering (e.g., before first IDR) — not an error.
  // Flushes codec context and enters waiting_for_keyframe_ on unrecoverable errors.
  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;

  std::string GetName() const override;
  bool IsInitialized() const override { return initialized_; }

 private:
  // Try to initialise a hardware decoder.
  // Probes all backends silently (FFmpeg log output is muted and per-backend
  // failures are collected); a consolidated warning is emitted only when the
  // entire candidate chain fails.
  // Phase 1: named decoders (NVDEC → CUVID → QSV → VAAPI → VDPAU).
  // Phase 1b: dynamically discovered HW decoders.
  // Phase 2: generic h264 + HW device ctx (Intel/AMD VAAPI).
  // Returns true and sets hw_backend_name_ on success.
  bool TryInitHardware();

  // Initialise the software-only H.264 decoder.
  bool TryInitSoftware();

  // Destroy the current HW decoder context and reinitialise as software.
  // Called when HW decode fails at runtime (e.g. GPU driver crash, OOM).
  bool FallbackToSoftware();

  AVCodecContext* codec_ctx_          = nullptr;
  AVFrame*        frame_              = nullptr;
  AVFrame*        sw_frame_           = nullptr;  // CPU-side frame for hw→sw transfer
  AVPacket*       pkt_                = nullptr;
  SwsContext*     sws_ctx_            = nullptr;
  bool            initialized_        = false;
  bool            is_hardware_        = false;
  std::string     hw_backend_name_;
  // Gate that blocks non-IDR frames after stream start, Reset(), or decode error.
  // Prevents garbled/error-concealed frames from being published when reference
  // frames are missing. Cleared only when a frame containing an IDR NAL arrives.
  bool            waiting_for_keyframe_ = true;
  int             hw_fail_count_       = 0;  // consecutive HW decode failures
};

}  // namespace odin_decoder
