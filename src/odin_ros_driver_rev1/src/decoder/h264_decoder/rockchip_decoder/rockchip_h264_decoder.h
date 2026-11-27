#pragma once
#include "h264_decoder.h"

// This header is only meaningful when USE_RK_H264_DECODER is defined.
// Including it on non-Rockchip builds is safe but the class will be empty.
#ifdef USE_RK_H264_DECODER

#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>

namespace odin_decoder {

// Rockchip MPP H.264 hardware decoder.
//
// Implementation notes:
//   - Uses the MPP "simple" streaming API (decode_put_packet /
//     decode_get_frame). Unlike the MJPEG decoder (which requires the
//     advanced + MppTask path — see RkHardwareDecoder), generic video
//     codecs decode fine through the simple API.
//   - Input Annex-B access units are handed to MPP via a plain
//     mpp_packet_init(ptr, size) packet (no DMA buffer needed for input;
//     decode_put_packet copies/consumes the bitstream synchronously).
//   - Output NV12 frames are DMA buffers owned by MPP's internal frame
//     pool. RGA converts NV12 -> BGR (zero CPU color conversion) into a
//     single reused DMA buffer, reallocated only when the stream
//     resolution changes.
//   - split_parse=1 lets MPP internally re-frame the bitstream, so the
//     caller does not need to guarantee one NAL/frame per Decode() call.
//   - Not thread-safe; the caller must serialize Decode() invocations.
//     Each camera channel must have its own instance, matching the
//     FfmpegH264Decoder / JetsonNvdecDecoder contract.
//   - ZERO-COPY OUTPUT: the cv::Mat returned by Decode() wraps an internal,
//     reused DMA buffer (no copy). It stays valid only until the next
//     Decode() call. A caller that keeps the result beyond that MUST
//     clone() it first (post_process_node already does this).
class RkH264Decoder : public H264Decoder {
 public:
  explicit RkH264Decoder(const std::string& camera_label = "");
  ~RkH264Decoder() override;

  // Non-copyable, non-movable (MppCtx holds hardware decoder state).
  RkH264Decoder(const RkH264Decoder&)            = delete;
  RkH264Decoder& operator=(const RkH264Decoder&) = delete;

  // Decode one Annex-B access unit into a BGR cv::Mat.
  // Returns false while buffering (e.g., before the first IDR/SPS/PPS, or
  // while MPP is still absorbing a resolution-change event). Callers must
  // not treat a false return as a fatal failure.
  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override;

  std::string GetName() const override;
  bool IsInitialized() const override { return initialized_; }

 private:
  // (Re)allocate the RGA BGR output DMA buffer for the given frame size.
  // Called lazily on the first decoded frame and again whenever the
  // decoder reports a resolution change.
  bool EnsureColorBuffer(int width, int height);

  // Convert one decoded NV12 MppFrame to BGR via RGA. Writes into
  // colorBuf_ and wraps it (zero-copy) into `output`.
  bool ConvertFrame(MppFrame frame, cv::Mat& output);

  // Tear down all MPP resources. Idempotent.
  void Shutdown();

  bool initialized_ = false;
  // Gate that blocks non-keyframe access units after stream start or a
  // decode error. Prevents garbled/error-concealed frames from being
  // published when reference frames are missing. Cleared once an AU
  // containing an IDR or SPS NAL arrives (matches FfmpegH264Decoder /
  // JetsonNvdecDecoder).
  bool waiting_for_keyframe_ = true;

  // ---- MPP context ----
  MppCtx ctx_ = nullptr;
  MppApi* mpi_ = nullptr;

  // ---- Color side: RGA converts NV12 -> BGR into this cacheable DMA
  // buffer; the returned cv::Mat wraps it zero-copy (caller clones).
  MppBufferGroup colorGroup_ = nullptr;
  MppBuffer colorBuf_ = nullptr;
  int colorW_ = 0;
  int colorH_ = 0;
};

}  // namespace odin_decoder

#endif  // USE_RK_H264_DECODER
