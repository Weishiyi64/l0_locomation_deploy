#pragma once
#include "jpeg_decoder.h"

namespace odin_decoder {

// OpenCV-based software JPEG decoder.
// Always available; used as fallback when hardware decoders are unavailable.
class SoftwareDecoder : public JpegDecoder {
 public:
  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;
  std::string GetName() const override;
};

}  // namespace odin_decoder
