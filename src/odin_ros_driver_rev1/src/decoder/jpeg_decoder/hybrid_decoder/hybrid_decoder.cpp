#include "hybrid_decoder.h"
#include "utility/ros_compat.hpp"

namespace odin_decoder {

HybridDecoder::HybridDecoder(std::unique_ptr<JpegDecoder> hw,
                             std::unique_ptr<JpegDecoder> sw)
    : hwDecoder_(std::move(hw)), swDecoder_(std::move(sw)) {
  if (hwDecoder_ && hwDecoder_->IsInitialized()) {
    useHardware_ = true;
    RC_LOG_INFO("[HybridDecoder] Mode: Hardware decoder (%s)",
                hwDecoder_->GetName().c_str());
  } else {
    useHardware_ = false;
    RC_LOG_WARN("[HybridDecoder] Mode: Software decoder (HW unavailable or init failed)");
  }
}

bool HybridDecoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  // Validate JPEG structure - MUST have SOI (0xFFD8) at start and EOI (0xFFD9) at end
  if (!data || size < 4) {
    return false;
  }

  bool has_soi = (data[0] == 0xFF && data[1] == 0xD8);
  bool has_eoi = (data[size - 2] == 0xFF && data[size - 1] == 0xD9);

  if (!has_soi || !has_eoi) {
    // Incomplete/corrupt JPEG - skip this frame entirely to avoid crash
    invalidJpegCount_++;
    if (invalidJpegCount_ <= 10 || invalidJpegCount_ % 100 == 0) {
      RC_LOG_WARN("[HybridDecoder] Incomplete JPEG (no SOI/EOI), frame dropped (count: %d)",
                  invalidJpegCount_);
    }
    return false;
  }

  // Use hardware decoder if available
  if (useHardware_ && hwDecoder_) {
    if (hwDecoder_->Decode(data, size, output)) {
      return true;
    }
    // Hardware available but failed on this frame: drop it (do NOT silently fall back to SW,
    // to keep timing predictable and avoid hiding HW regressions).
    hwFailCount_++;
    if (hwFailCount_ <= 10 || hwFailCount_ % 100 == 0) {
      RC_LOG_WARN("[HybridDecoder] HW decode failed on valid JPEG (fail count: %d), frame dropped",
                  hwFailCount_);
    }
    return false;
  }

  // Software decoder path (HW unavailable)
  if (swDecoder_ && swDecoder_->Decode(data, size, output)) {
    return true;
  }
  return false;
}

std::string HybridDecoder::GetName() const {
  if (useHardware_ && hwDecoder_) {
    return std::string("Hybrid(HW=") + hwDecoder_->GetName() + ", SW=" +
           (swDecoder_ ? swDecoder_->GetName() : "none") + ")";
  }
  return std::string("Hybrid(SW only=") +
         (swDecoder_ ? swDecoder_->GetName() : "none") + ")";
}

}  // namespace odin_decoder
