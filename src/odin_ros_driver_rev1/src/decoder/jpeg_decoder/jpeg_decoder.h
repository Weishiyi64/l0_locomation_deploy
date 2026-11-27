#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace odin_decoder {

// Abstract JPEG decoder interface.
// Backends (software / Jetson NvJpeg / RK MPP / ...) implement this.
class JpegDecoder {
 public:
  virtual ~JpegDecoder() = default;

  // Decode a JPEG buffer into a BGR cv::Mat.
  // Returns true on success.
  virtual bool Decode(const uint8_t* data, size_t size, cv::Mat& output) = 0;

  // Human-readable backend name (used in logs).
  virtual std::string GetName() const = 0;

  // Whether the backend is ready to decode.
  // Hardware backends may fail to initialize; software backends are always ready.
  virtual bool IsInitialized() const { return true; }

  // Set a human-readable label (e.g. "camera0") for log messages.
  virtual void SetCameraLabel(const std::string& label) { camera_label_ = label; }

 protected:
  std::string camera_label_;
};

// Factory: returns the best available JPEG decoder for this platform/build.
// Backend selection is compile-time only (USING_JETSON_DECODER / USE_RK_DECODER).
std::unique_ptr<JpegDecoder> CreateDecoder();

}  // namespace odin_decoder
