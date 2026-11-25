#include "software_decoder.h"

namespace odin_decoder {

bool SoftwareDecoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  cv::Mat raw(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t*>(data));
  output = cv::imdecode(raw, cv::IMREAD_COLOR);
  return !output.empty();
}

std::string SoftwareDecoder::GetName() const {
  return "OpenCV Software Decoder";
}

}  // namespace odin_decoder
