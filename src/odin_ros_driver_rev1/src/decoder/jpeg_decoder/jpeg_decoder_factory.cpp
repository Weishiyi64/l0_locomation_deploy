// JPEG decoder factory.
// Selects a backend at compile time based on platform feature macros.
//
// Supported backends (mutually exclusive at compile time):
//   - USING_JETSON_DECODER : NVIDIA Jetson NvJpeg (JetPack R36+), HW + SW fallback
//   - USE_RK_DECODER     : Rockchip MPP (HW + SW fallback)
//   - (none)             : OpenCV software decoder only
//
// When USING_JETSON_DECODER is defined the hardware decoder is wrapped in a
// HybridDecoder with a software fallback.  However, because libnvjpeg's
// jpeg_* symbols interpose cv::imdecode, the SW fallback may crash on
// Jetson — HybridDecoder prints a one-time warning before the first SW
// decode attempt (see hybrid_decoder.cpp).
#include "jpeg_decoder.h"

#include "software_decoder/software_decoder.h"
#include "hybrid_decoder/hybrid_decoder.h"
#include "utility/rc_log.hpp"

#ifdef USING_JETSON_DECODER
#include "jetson_decoder/jetson_hardware_decoder.h"
#endif

#ifdef USE_RK_DECODER
#include "rockchip_decoder/rockchip_hardware_decoder.h"
#endif

namespace odin_decoder {

std::unique_ptr<JpegDecoder> CreateDecoder() {
  auto sw = std::make_unique<SoftwareDecoder>();

#if defined(USING_JETSON_DECODER)
  auto hw = std::make_unique<JetsonHardwareDecoder>();
  RC_LOG_INFO("[JPEG] [init] Using Hybrid Decoder (Jetson HW + Software fallback)");
  return std::make_unique<HybridDecoder>(std::move(hw), std::move(sw));
#elif defined(USE_RK_DECODER)
  auto hw = std::make_unique<RkHardwareDecoder>();
  RC_LOG_DEBUG("[JPEG] [init] Using Hybrid Decoder (Rockchip HW + Software fallback)");
  return std::make_unique<HybridDecoder>(std::move(hw), std::move(sw));
#else
  RC_LOG_DEBUG("[JPEG] [init] Using OpenCV Software Decoder");
  return sw;
#endif
}

}  // namespace odin_decoder
