#pragma once
#include "jpeg_decoder.h"

// This header is only meaningful when USING_JETSON_DECODER is defined.
// Including it on non-Jetson builds is safe but the class will be empty.
#ifdef USING_JETSON_DECODER

class NvJPEGDecoder;  // forward-declare to avoid leaking NvJpegDecoder.h

namespace odin_decoder {

// Jetson NvJPEG hardware decoder.
// Note: Hardware decoder is only enabled at compile-time for JetPack R36+.
// R36+ uses libjpeg-8b which is ABI compatible with NvJpegDecoder.
// R35 and below are blocked at CMake level due to libjpeg-turbo ABI conflict.
class JetsonHardwareDecoder : public JpegDecoder {
 public:
  JetsonHardwareDecoder();
  ~JetsonHardwareDecoder() override;

  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;
  std::string GetName() const override;
  bool IsInitialized() const override { return initialized_; }

 private:
  NvJPEGDecoder* decoder_ = nullptr;
  bool initialized_ = false;
};

}  // namespace odin_decoder

#endif  // USING_JETSON_DECODER
