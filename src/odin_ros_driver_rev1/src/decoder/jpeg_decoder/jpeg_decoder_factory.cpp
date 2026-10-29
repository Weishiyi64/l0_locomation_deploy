// JPEG decoder factory.
// Selects a backend at compile time based on platform feature macros, and
// wraps any available hardware backend with HybridDecoder (HW + SW fallback +
// SOI/EOI sanity check).
//
// Supported backends (mutually exclusive at compile time, for now):
//   - USE_JETSON_DECODER : NVIDIA Jetson NvJpeg (JetPack R36+)
//   - USE_RK_DECODER     : Rockchip MPP (planned, step 2)
//   - (none)             : OpenCV software decoder only
#include "jpeg_decoder.h"

#include "software_decoder/software_decoder.h"
#include "hybrid_decoder/hybrid_decoder.h"
#include "utility/ros_compat.hpp"

#ifdef USE_JETSON_DECODER
#include "jetson_decoder/jetson_hardware_decoder.h"
#endif

#ifdef USE_RK_DECODER
#include "rockchip_decoder/rockchip_hardware_decoder.h"
#endif

namespace odin_decoder {

std::unique_ptr<JpegDecoder> CreateDecoder() {
  auto sw = std::make_unique<SoftwareDecoder>();

#if defined(USE_JETSON_DECODER)
  auto hw = std::make_unique<JetsonHardwareDecoder>();
  RC_LOG_INFO("[JPEG Decoder] Using Hybrid Decoder (Jetson HW + Software fallback)");
  return std::make_unique<HybridDecoder>(std::move(hw), std::move(sw));
#elif defined(USE_RK_DECODER)
  auto hw = std::make_unique<RkHardwareDecoder>();
  RC_LOG_INFO("[JPEG Decoder] Using Hybrid Decoder (Rockchip HW + Software fallback)");
  return std::make_unique<HybridDecoder>(std::move(hw), std::move(sw));
#else
  RC_LOG_INFO("[JPEG Decoder] Using OpenCV Software Decoder");
  return sw;
#endif
}

}  // namespace odin_decoder
