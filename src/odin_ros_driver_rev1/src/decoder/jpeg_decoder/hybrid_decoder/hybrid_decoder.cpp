#include "hybrid_decoder.h"
#include "utility/rc_log.hpp"

namespace odin_decoder {

// Build a log prefix like "[JPEG|camera0|HybridDecoder]" or "[JPEG|HybridDecoder]".
static std::string LogTag(const std::string& label) {
  std::string s = "[JPEG";
  if (!label.empty()) s += "|" + label;
  s += "|HybridDecoder]";
  return s;
}

HybridDecoder::HybridDecoder(std::unique_ptr<JpegDecoder> hw,
                             std::unique_ptr<JpegDecoder> sw)
    : hwDecoder_(std::move(hw)), swDecoder_(std::move(sw)) {
  if (hwDecoder_ && hwDecoder_->IsInitialized()) {
    useHardware_ = true;
    RC_LOG_DEBUG("%s [init] Mode: Hardware decoder (%s)",
                 LogTag(camera_label_).c_str(), hwDecoder_->GetName().c_str());
  } else {
    useHardware_ = false;
    RC_LOG_WARN("%s [init] Mode: Software decoder (HW unavailable or init failed)", LogTag(camera_label_).c_str());
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
      RC_LOG_WARN("%s [decode] Incomplete JPEG (no SOI/EOI), frame dropped (count: %d)",
                  LogTag(camera_label_).c_str(), invalidJpegCount_);
    }
    return false;
  }

  // Use hardware decoder if available
  if (useHardware_ && hwDecoder_) {
    if (hwDecoder_->Decode(data, size, output)) {
      return true;
    }
    // Hardware available but failed on this frame: fall back to SW decoder.
    hwFailCount_++;
    if (hwFailCount_ <= 10 || hwFailCount_ % 100 == 0) {
      RC_LOG_WARN("%s [decode] HW decode failed on valid JPEG (fail count: %d), falling back to SW",
                  LogTag(camera_label_).c_str(), hwFailCount_);
    }
  }

  // Software decoder path (HW unavailable or HW failed on this frame)
#ifdef USING_JETSON_DECODER
  if (!swFallbackWarned_) {
    RC_LOG_ERROR(
        "%s [decode] About to use software decoder (cv::imdecode) while "
        "libnvjpeg is linked. libnvjpeg's jpeg_* symbols may interpose and "
        "cause a segfault. If this crashes, recompile with "
        "-DJPEG_PREFER_SOFTWARE_DECODER=ON to use pure software decoding.",
        LogTag(camera_label_).c_str());
    swFallbackWarned_ = true;
  }
#endif
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
