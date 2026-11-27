#include "h264_decoder.h"

#include "utility/rc_log.hpp"

// Only include the FFmpeg backend when compiled with FFmpeg support.
// The build system sets USE_FFMPEG_H264_DECODER and adds the .cpp to sources.
#ifdef USE_FFMPEG_H264_DECODER
#include "ffmpeg_decoder/ffmpeg_h264_decoder.h"
#endif

// Only include the Jetson NVDEC backend when compiled on a Jetson platform
// with the Jetson Multimedia API (nvvideo_dec.h) available.
#ifdef USE_JETSON_NVDEC_DECODER
#include "jetson_nvdec_decoder/jetson_nvdec_decoder.h"
#endif

// Only include the Rockchip MPP backend when compiled with Rockchip MPP/RGA
// support. Independent of the JPEG decoder's own USE_RK_DECODER flag.
#ifdef USE_RK_H264_DECODER
#include "rockchip_decoder/rockchip_h264_decoder.h"
#endif

namespace odin_decoder {

namespace {

// Returned when no H.264 backend is compiled in, or when all backends fail
// to initialise at runtime. Decode() always returns false so callers fall
// through to their "no frame available" path without crashing.
class NullH264Decoder : public H264Decoder {
 public:
  bool Decode(const uint8_t*, size_t, cv::Mat&) override { return false; }
  std::string GetName() const override {
    return "NullH264Decoder (no H.264 backend; "
           "install libavcodec-dev libavutil-dev libswscale-dev and rebuild)";
  }
  bool IsInitialized() const override { return false; }
};

}  // namespace

std::unique_ptr<H264Decoder> CreateH264Decoder(const std::string& camera_label) {
  std::string tag = camera_label.empty() ? "[H264]" : ("[H264|" + camera_label + "]");

  // --- 1. Hardware decoder -----------------------------------------------
#ifdef USE_JETSON_NVDEC_DECODER
  auto hw = std::unique_ptr<H264Decoder>(new JetsonNvdecDecoder(camera_label));
  if (hw->IsInitialized()) {
    RC_LOG_DEBUG("%s [init] Using Jetson NVDEC hardware decoder", tag.c_str());
    return hw;
  }
  // Fall through: NVDEC compiled in but failed to initialise at runtime.
  RC_LOG_WARN("%s [init] NVDEC init failed, falling back to software decoder", tag.c_str());
#endif
#ifdef USE_RK_H264_DECODER
  auto rk = std::unique_ptr<H264Decoder>(new RkH264Decoder(camera_label));
  if (rk->IsInitialized()) {
    RC_LOG_DEBUG("%s [init] Using Rockchip MPP hardware decoder", tag.c_str());
    return rk;
  }
  // Fall through: RK MPP compiled in but failed to initialise at runtime.
  RC_LOG_WARN("%s [init] Rockchip MPP init failed, falling back to software decoder", tag.c_str());
#endif

  // --- 2. FFmpeg decoder (hardware-accelerated, software fallback) ---------
#ifdef USE_FFMPEG_H264_DECODER
  auto sw = std::unique_ptr<H264Decoder>(new FfmpegH264Decoder(camera_label));
  if (sw->IsInitialized()) {
    return sw;
  }
  // Fall through: FFmpeg found at build time but failed to open at runtime.
#endif

  // --- 3. Null fallback ----------------------------------------------------
  return std::unique_ptr<H264Decoder>(new NullH264Decoder());
}

}  // namespace odin_decoder
