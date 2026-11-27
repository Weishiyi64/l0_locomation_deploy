#pragma once
#include "jpeg_decoder.h"

namespace odin_decoder {

// Vendor-agnostic hybrid decoder.
// Wraps an optional hardware backend with a guaranteed software fallback,
// and performs SOI/EOI sanity checks on every frame.
//
// Behaviour:
//   - If hw != nullptr and hw->IsInitialized(): use HW; drop frame on HW failure.
//   - Otherwise: use software decoder.
class HybridDecoder : public JpegDecoder {
 public:
  HybridDecoder(std::unique_ptr<JpegDecoder> hw,
                std::unique_ptr<JpegDecoder> sw);

  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;
  std::string GetName() const override;
  bool IsInitialized() const override { return swDecoder_ != nullptr; }

 private:
  std::unique_ptr<JpegDecoder> hwDecoder_;
  std::unique_ptr<JpegDecoder> swDecoder_;
  bool useHardware_ = false;
  int hwFailCount_ = 0;
  int invalidJpegCount_ = 0;
  bool swFallbackWarned_ = false;
};

}  // namespace odin_decoder
