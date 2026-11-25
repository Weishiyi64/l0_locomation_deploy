#pragma once
#include "jpeg_decoder.h"

// This header is only meaningful when USE_RK_DECODER is defined.
// Including it on non-Rockchip builds is safe but the class will be empty.
#ifdef USE_RK_DECODER

#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>

namespace odin_decoder {

// Output pixel format the caller can choose at construction time.
//   kBGR888: byte order B,G,R  (default; matches the bgr8 publish path and the
//            Jetson/software backends).
//   kRGB888: byte order R,G,B  (for consumers that want RGB).
// Both map to a packed 24-bit cv::Mat (CV_8UC3). Conversion from the decoder's
// NV12 output is done by the RGA hardware (zero CPU color conversion).
enum class RkOutputFormat { kBGR888, kRGB888 };

// Rockchip MPP MJPEG hardware decoder.
//
// Implementation notes:
//   - Uses the MPP advanced + MppTask path (the MJPEG decoder does not support
//     the simple decode_put_packet path, which fails with BUFFER_FULL -1012).
//   - Output frame + DMA buffer are pre-allocated and reused across frames.
//   - Input packet buffers are allocated per-frame.
//   - Frame size is parsed from the first JPEG SOF segment lazily (resources
//     are built on the first Decode() call). A single instance does not
//     support runtime resolution changes; recreate the decoder if needed.
//   - Not thread-safe; the caller must serialize Decode() invocations.
//   - ZERO-COPY OUTPUT: the cv::Mat returned by Decode() wraps an internal,
//     reused DMA buffer (no copy). It stays valid only until the next Decode()
//     call. A caller that keeps the result beyond that MUST clone() it first
//     (post_process_node already does this).
class RkHardwareDecoder : public JpegDecoder {
 public:
  // fmt selects the output byte order (default BGR for backward compatibility).
  explicit RkHardwareDecoder(RkOutputFormat fmt = RkOutputFormat::kBGR888);
  ~RkHardwareDecoder() override;

  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;
  std::string GetName() const override;
  bool IsInitialized() const override { return initialized_; }

 private:
  // Allocate output frame / DMA buffer / packet buffer group based on first
  // JPEG dimensions. Called once on the first successful Decode().
  bool LazyInitAdvanced(int width, int height);

  // Tear down all MPP resources. Idempotent.
  void Shutdown();

  bool initialized_ = false;

  // ---- MPP context ----
  MppCtx ctx_ = nullptr;
  MppApi* mpi_ = nullptr;

  // ---- Output side: pre-allocated frame + DMA buffer, reused ----
  MppBufferGroup frameGroup_ = nullptr;
  MppBuffer frameBuf_ = nullptr;
  MppFrame outputFrame_ = nullptr;

  // ---- Input side: only the group is kept; per-frame buffers are
  // get/put on the fly inside Decode().
  MppBufferGroup packetGroup_ = nullptr;

  // ---- Color side: RGA converts NV12 -> BGR/RGB into this cacheable DMA
  // buffer; the returned cv::Mat wraps it zero-copy (caller clones).
  MppBufferGroup colorGroup_ = nullptr;
  MppBuffer colorBuf_ = nullptr;
  RkOutputFormat outputFormat_ = RkOutputFormat::kBGR888;

  // ---- Lazy-init state ----
  bool advancedInited_ = false;
  int jpegW_ = 0;
  int jpegH_ = 0;
  int jpegHorStride_ = 0;  // MPP_ALIGN(w, 16)
  int jpegVerStride_ = 0;  // MPP_ALIGN(h, 16)
};

}  // namespace odin_decoder

#endif  // USE_RK_DECODER
