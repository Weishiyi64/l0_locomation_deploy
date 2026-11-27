#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace odin_decoder {

// Abstract H.264 decoder interface. Each instance is stateful and must be
// used for exactly one stream (one camera channel). Accepts Annex-B access
// units (NAL-unit sequences delimited by 0x000001 / 0x00000001 start codes).
class H264Decoder {
 public:
  virtual ~H264Decoder() = default;

  // Decode one Annex-B access unit into a BGR cv::Mat.
  // Returns true when a decoded frame is available in `output`.
  // May return false without an error when the decoder is still buffering
  // (e.g., waiting for the first IDR frame after stream start or seek).
  // Callers must not treat a false return as a fatal failure.
  virtual bool Decode(const uint8_t* data, size_t size, cv::Mat& output) = 0;

  // Human-readable backend name, used in log messages.
  virtual std::string GetName() const = 0;

  // Whether the backend initialised successfully.
  // A NullH264Decoder always returns false here so callers can distinguish
  // "no backend compiled in" from a genuine decode error.
  virtual bool IsInitialized() const { return true; }

 protected:
  std::string camera_label_;  // e.g. "camera0" — used in log prefixes
};

// Factory: returns the best available H.264 decoder for this platform/build.
// Selection order:
//   1. JetsonNvdecDecoder (if USE_JETSON_NVDEC_DECODER compiled in)
//   2. RkH264Decoder      (if USE_RK_H264_DECODER compiled in)
//   3. FfmpegH264Decoder  (if USE_FFMPEG_H264_DECODER compiled in)
//   4. NullH264Decoder    (always available, IsInitialized()==false)
// Returns a NullH264Decoder (IsInitialized() == false) when no backend is
// compiled in so that callers can always call Decode() safely without NULL checks.
std::unique_ptr<H264Decoder> CreateH264Decoder(const std::string& camera_label = "");

}  // namespace odin_decoder
