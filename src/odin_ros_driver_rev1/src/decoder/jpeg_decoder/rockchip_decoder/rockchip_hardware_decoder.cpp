#include "rockchip_decoder/rockchip_hardware_decoder.h"

#ifdef USE_RK_DECODER

#include <cstring>

#include <rockchip/mpp_meta.h>     // mpp_meta_set_frame / KEY_OUTPUT_FRAME / KEY_INPUT_PACKET
#include <rockchip/mpp_task.h>     // MppTask + mpp_task_meta_set_* + MPP_PORT_INPUT/OUTPUT
#include <rockchip/rk_vdec_cfg.h>  // mpp_dec_cfg_init/set_u32/deinit + MppDecCfg

#include <rga/im2d.h>  // imcvtcolor / wrapbuffer_fd / IM_STATUS / imStrError
#include <rga/rga.h>   // RK_FORMAT_YCbCr_420_SP / RK_FORMAT_BGR_888 / RK_FORMAT_RGB_888

#include <fcntl.h>           // ioctl
#include <sys/ioctl.h>       // ioctl
#include <linux/dma-buf.h>   // DMA_BUF_IOCTL_SYNC / dma_buf_sync

#include "utility/rc_log.hpp"

// MPP_ALIGN: round x up to a multiple of a (a must be a power of two).
// The original definition lives in mpp_common.h, which is an internal header
// not shipped in the sysroot, so we duplicate it here.
#ifndef MPP_ALIGN
#define MPP_ALIGN(x, a) (((x) + ((a) - 1)) & ~((a) - 1))
#endif

namespace odin_decoder {

namespace {

// ============================================================================
// DMA buffer cache sync helpers
// ============================================================================
// mpp_buffer_sync_begin/end are only available in newer librk_mpp (Ubuntu
// 22.04+). On older versions (Ubuntu 20.04) they are not declared. But the
// underlying mechanism is the standard Linux DMA_BUF_IOCTL_SYNC ioctl, which
// is available on all kernels >= 3.18. So we fall back to calling the ioctl
// directly when the MPP wrapper is missing.
//
//   sync_end  (after CPU write, before device read):  flush CPU cache → RAM
//   sync_begin (after device write, before CPU read): invalidate CPU cache
// ============================================================================
#if !defined(MPP_HAS_BUFFER_SYNC)
void mpp_buffer_sync_end(MppBuffer buffer) {
  int fd = mpp_buffer_get_fd(buffer);
  struct dma_buf_sync sync = {};
  sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
  int ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  (void)ret;
}
void mpp_buffer_sync_begin(MppBuffer buffer) {
  int fd = mpp_buffer_get_fd(buffer);
  struct dma_buf_sync sync = {};
  sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
  int ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  (void)ret;
}
#endif

// Minimal JPEG header parser: extract width/height from the SOF0/1/2/3 segment.
//
// JPEG segment layout (ITU-T T.81):
//   - No-payload markers: SOI(FFD8) / EOI(FFD9) / TEM(FF01) / RSTm(FFD0..FFD7)
//   - Payload markers   : [FF marker] [length 2B BE, includes self] [data...]
//   - SOF0..3 (FFC0/C1/C2/C3): baseline / extended / progressive / lossless
//       segment body: [precision 1] [height 2B BE] [width 2B BE] [Nf 1] ...
//   - Markers 0xC4 (DHT) / 0xC8 (JPG) / 0xCC (DAC) live in the same range
//     but are NOT SOF; skip them.
bool ParseJpegSize(const uint8_t* data, size_t size, int& width, int& height) {
  if (size < 4 || data[0] != 0xFF || data[1] != 0xD8) return false;  // not SOI
  size_t i = 2;
  while (i + 4 <= size) {
    if (data[i] != 0xFF) return false;
    // Skip 0xFF padding bytes (multiple allowed)
    while (i + 1 < size && data[i + 1] == 0xFF) ++i;
    uint8_t marker = data[i + 1];
    i += 2;

    // No-payload markers: SOI / EOI / TEM / RST0..7
    if (marker == 0xD8 || marker == 0xD9 || marker == 0x01) continue;
    if (marker >= 0xD0 && marker <= 0xD7) continue;

    // Payload marker: read 2-byte big-endian length
    if (i + 2 > size) return false;
    const uint16_t segLen = (uint16_t(data[i]) << 8) | data[i + 1];
    if (segLen < 2 || i + segLen > size) return false;

    // SOF: 0xC0..0xCF excluding DHT / JPG / DAC
    if (marker >= 0xC0 && marker <= 0xCF
        && marker != 0xC4 && marker != 0xC8 && marker != 0xCC) {
      if (segLen < 7) return false;
      // Body offset +2 (length): precision(1) | height(2 BE) | width(2 BE)
      height = (uint16_t(data[i + 3]) << 8) | data[i + 4];
      width  = (uint16_t(data[i + 5]) << 8) | data[i + 6];
      return (width > 0 && height > 0);
    }

    i += segLen;  // jump to next segment
  }
  return false;
}

}  // namespace

// =====================================================================
//  ctor / dtor
// =====================================================================

RkHardwareDecoder::RkHardwareDecoder(RkOutputFormat fmt) : outputFormat_(fmt) {
  MPP_RET ret = mpp_create(&ctx_, &mpi_);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] mpp_create failed ret=%d", static_cast<int>(ret));
    Shutdown();
    return;
  }

  ret = mpp_init(ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] mpp_init(DEC,MJPEG) failed ret=%d", static_cast<int>(ret));
    Shutdown();
    return;
  }

  // mpp_dec_cfg + SET_OUTPUT_FORMAT ordering is sensitive; keep strict order:
  //   cfg_init -> GET_CFG -> set_u32(split_parse) -> SET_OUTPUT_FORMAT
  //            -> SET_CFG -> cfg_deinit
  // SET_OUTPUT_FORMAT must precede SET_CFG, otherwise it is ignored.
  MppDecCfg cfgHandle = nullptr;
  ret = mpp_dec_cfg_init(&cfgHandle);
  if (ret != MPP_OK || !cfgHandle) {
    RC_LOG_ERROR("[RkDecoder] mpp_dec_cfg_init failed ret=%d", static_cast<int>(ret));
    Shutdown();
    return;
  }
  ret = mpi_->control(ctx_, MPP_DEC_GET_CFG, cfgHandle);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] MPP_DEC_GET_CFG failed ret=%d", static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  // split_parse=1: enable internal frame splitting (required for variable-size streams).
  ret = mpp_dec_cfg_set_u32(cfgHandle, "base:split_parse", 1);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] set base:split_parse=1 failed ret=%d", static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  // SET_OUTPUT_FORMAT: NV12 (MPP_FMT_YUV420SP). The decoder always outputs
  // NV12; RGA converts it to the caller-selected BGR/RGB afterwards.
  MppFrameFormat mppOutFmt = MPP_FMT_YUV420SP;
  ret = mpi_->control(ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &mppOutFmt);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] MPP_DEC_SET_OUTPUT_FORMAT(NV12) failed ret=%d",
                 static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  ret = mpi_->control(ctx_, MPP_DEC_SET_CFG, cfgHandle);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] MPP_DEC_SET_CFG failed ret=%d", static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  mpp_dec_cfg_deinit(cfgHandle);

  initialized_ = true;
  RC_LOG_DEBUG("[RkDecoder] init OK (codec=MJPEG, decode=NV12 -> RGA -> %s, advanced+MppTask)",
               (outputFormat_ == RkOutputFormat::kRGB888) ? "RGB888" : "BGR888");
}

RkHardwareDecoder::~RkHardwareDecoder() {
  Shutdown();
}

// =====================================================================
//  LazyInitAdvanced -- build all advanced-path resources from (w, h)
//    1) output: frame_group + DMA buffer + MppFrame, bound together
//    2) input : packet_group only (per-frame buffers are get/put inside Decode)
// =====================================================================

bool RkHardwareDecoder::LazyInitAdvanced(int width, int height) {
  jpegW_ = width;
  jpegH_ = height;
  jpegHorStride_ = MPP_ALIGN(width, 16);
  jpegVerStride_ = MPP_ALIGN(height, 16);
  // *4 is the upper bound for YUV422 (NV12 only needs *1.5); kept consistent
  // with the RK MPP demo.
  const size_t outBufSize = static_cast<size_t>(jpegHorStride_) * jpegVerStride_ * 4;

  // ---- 1) output: frame_group + DMA buffer + MppFrame + bind ----
  MPP_RET ret = mpp_buffer_group_get_internal(&frameGroup_, MPP_BUFFER_TYPE_DRM);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] frame_group get_internal failed ret=%d", static_cast<int>(ret));
    return false;
  }

  ret = mpp_buffer_get(frameGroup_, &frameBuf_, outBufSize);
  if (ret != MPP_OK || !frameBuf_) {
    RC_LOG_ERROR("[RkDecoder] frame_buf get(%zu) failed ret=%d",
                 outBufSize, static_cast<int>(ret));
    return false;
  }

  ret = mpp_frame_init(&outputFrame_);
  if (ret != MPP_OK || !outputFrame_) {
    RC_LOG_ERROR("[RkDecoder] mpp_frame_init failed ret=%d", static_cast<int>(ret));
    return false;
  }
  mpp_frame_set_buffer(outputFrame_, frameBuf_);

  // ---- 2) input: packet_group (buffers are allocated per-frame in Decode()) ----
  ret = mpp_buffer_group_get_internal(&packetGroup_, MPP_BUFFER_TYPE_DRM);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] packet_group get_internal failed ret=%d",
                 static_cast<int>(ret));
    return false;
  }

  // ---- 3) color side: DMA buffer for RGA's BGR/RGB output ----
  // RGA (DMA) writes the packed 24-bit image here; the returned cv::Mat wraps
  // it zero-copy.
  const size_t colorBytes = static_cast<size_t>(width) * height * 3;
  ret = mpp_buffer_group_get_internal(&colorGroup_, MPP_BUFFER_TYPE_DRM);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] color_group get_internal failed ret=%d", static_cast<int>(ret));
    return false;
  }
  ret = mpp_buffer_get(colorGroup_, &colorBuf_, colorBytes);
  if (ret != MPP_OK || !colorBuf_) {
    RC_LOG_ERROR("[RkDecoder] color_buf get(%zu) failed ret=%d",
                 colorBytes, static_cast<int>(ret));
    return false;
  }

  advancedInited_ = true;
  return true;
}

// =====================================================================
//  Shutdown -- idempotent, order sensitive
//    mpp_destroy(ctx) must run first so the decoder releases its internal
//    references to frame/group; only then is it safe to release frame ->
//    buffer -> buffer_group.
// =====================================================================

void RkHardwareDecoder::Shutdown() {
  if (ctx_)         { mpp_destroy(ctx_); ctx_ = nullptr; mpi_ = nullptr; }
  if (outputFrame_) { mpp_frame_deinit(&outputFrame_); outputFrame_ = nullptr; }
  if (frameBuf_)    { mpp_buffer_put(frameBuf_); frameBuf_ = nullptr; }
  if (frameGroup_)  { mpp_buffer_group_put(frameGroup_); frameGroup_ = nullptr; }
  if (packetGroup_) { mpp_buffer_group_put(packetGroup_); packetGroup_ = nullptr; }
  if (colorBuf_)    { mpp_buffer_put(colorBuf_); colorBuf_ = nullptr; }
  if (colorGroup_)  { mpp_buffer_group_put(colorGroup_); colorGroup_ = nullptr; }

  advancedInited_ = false;
  jpegW_ = jpegH_ = jpegHorStride_ = jpegVerStride_ = 0;
  initialized_ = false;
}

// =====================================================================
//  Decode -- single-frame advanced + MppTask path
//
//  Steps: get input_buf -> memcpy -> packet_init ->
//    (input)  poll -> dequeue -> set meta -> enqueue (kicks decoding) ->
//    (output) poll -> dequeue -> consume frame -> enqueue (returns task) -> cleanup
// =====================================================================

bool RkHardwareDecoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  if (!initialized_ || !mpi_) return false;
  if (!data || size < 4)      return false;

  // ---- A) First call: parse JPEG SOF for size, lazy-init resources ----
  if (!advancedInited_) {
    int w = 0, h = 0;
    if (!ParseJpegSize(data, size, w, h)) {
      RC_LOG_ERROR("[RkDecoder] ParseJpegSize failed (not a JPEG?)");
      return false;
    }
    if (!LazyInitAdvanced(w, h)) {
      RC_LOG_ERROR("[RkDecoder] LazyInitAdvanced(%dx%d) failed", w, h);
      return false;
    }
  }

  MppBuffer inputBuf = nullptr;
  MppPacket packet   = nullptr;

  auto cleanup = [&]() {
    if (packet)   { mpp_packet_deinit(&packet); packet = nullptr; }
    if (inputBuf) { mpp_buffer_put(inputBuf);   inputBuf = nullptr; }
  };

  // ---- B) Allocate input_buf, copy JPEG, cache flush ----
  MPP_RET ret = mpp_buffer_get(packetGroup_, &inputBuf, size);
  if (ret != MPP_OK || !inputBuf) {
    RC_LOG_ERROR("[RkDecoder] mpp_buffer_get(packet_group, %zu) failed ret=%d",
                 size, static_cast<int>(ret));
    cleanup();
    return false;
  }
  std::memcpy(mpp_buffer_get_ptr(inputBuf), data, size);
  mpp_buffer_sync_end(inputBuf);

  // ---- C) packet_init_with_buffer ----
  ret = mpp_packet_init_with_buffer(&packet, inputBuf);
  if (ret != MPP_OK || !packet) {
    RC_LOG_ERROR("[RkDecoder] mpp_packet_init_with_buffer failed ret=%d",
                 static_cast<int>(ret));
    cleanup();
    return false;
  }
  mpp_packet_set_pts(packet, 0);

  // ---- D) MppTask: input poll -> dequeue -> set meta -> enqueue ----
  MppTask task = nullptr;
  ret = mpi_->poll(ctx_, MPP_PORT_INPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] poll INPUT failed ret=%d", static_cast<int>(ret));
    cleanup();
    return false;
  }
  ret = mpi_->dequeue(ctx_, MPP_PORT_INPUT, &task);
  if (ret != MPP_OK || !task) {
    RC_LOG_ERROR("[RkDecoder] dequeue INPUT failed ret=%d task=%p",
                 static_cast<int>(ret), static_cast<void*>(task));
    cleanup();
    return false;
  }
  mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
  mpp_task_meta_set_frame (task, KEY_OUTPUT_FRAME, outputFrame_);

  ret = mpi_->enqueue(ctx_, MPP_PORT_INPUT, task);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] enqueue INPUT failed ret=%d", static_cast<int>(ret));
    cleanup();
    return false;
  }

  // ---- E) MppTask: output poll -> dequeue -> consume frame -> enqueue ----
  // poll(BLOCK) puts the calling thread into futex_wait until the jpegd
  // hardware finishes -> IRQ -> MPP worker pushes the frame -> wake_up.
  ret = mpi_->poll(ctx_, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] poll OUTPUT failed ret=%d", static_cast<int>(ret));
    cleanup();
    return false;
  }
  task = nullptr;
  ret = mpi_->dequeue(ctx_, MPP_PORT_OUTPUT, &task);
  if (ret != MPP_OK || !task) {
    RC_LOG_ERROR("[RkDecoder] dequeue OUTPUT failed ret=%d task=%p",
                 static_cast<int>(ret), static_cast<void*>(task));
    cleanup();
    return false;
  }

  bool success = false;

  // ---- F) Validate + convert NV12 to BGR ----
  if (mpp_frame_get_errinfo(outputFrame_) || mpp_frame_get_discard(outputFrame_)) {
    RC_LOG_ERROR("[RkDecoder] frame err=%u discard=%u",
                 mpp_frame_get_errinfo(outputFrame_),
                 mpp_frame_get_discard(outputFrame_));
    // Even on error, we must enqueue the task back, otherwise ctx state breaks.
    mpi_->enqueue(ctx_, MPP_PORT_OUTPUT, task);
    cleanup();
    return false;
  }

  MppBuffer retBuf = mpp_frame_get_buffer(outputFrame_);
  if (!retBuf) {
    RC_LOG_ERROR("[RkDecoder] outputFrame has no buffer");
    mpi_->enqueue(ctx_, MPP_PORT_OUTPUT, task);
    cleanup();
    return false;
  }
  const int w   = mpp_frame_get_width(outputFrame_);
  const int h   = mpp_frame_get_height(outputFrame_);
  const int hor = mpp_frame_get_hor_stride(outputFrame_);
  const int ver = mpp_frame_get_ver_stride(outputFrame_);

  // Note: we do NOT CPU-read the NV12 buffer anymore; RGA reads it via DMA (by
  // fd), so no sync_begin(retBuf) is needed here.
  if (w > 0 && h > 0 && hor >= w && ver >= h) {
    // RGA (DMA) reads the NV12 DMA buffer, converts to BGR/RGB and removes the
    // alignment stride, writing into colorBuf_. This replaces the old
    // "CPU read uncached NV12 + cv::cvtColor" path (slow + 100% CPU).
    // The NV12 source is passed by fd with the valid (w,h) region plus the
    // (hor,ver) stride, so RGA skips the alignment padding automatically.
    const int dstFmt = (outputFormat_ == RkOutputFormat::kRGB888)
                           ? RK_FORMAT_RGB_888
                           : RK_FORMAT_BGR_888;
    rga_buffer_t rsrc = wrapbuffer_fd(mpp_buffer_get_fd(retBuf), w, h,
                                      RK_FORMAT_YCbCr_420_SP, hor, ver);
    rga_buffer_t rdst = wrapbuffer_fd(mpp_buffer_get_fd(colorBuf_), w, h, dstFmt);
    IM_STATUS st = imcvtcolor(rsrc, rdst, RK_FORMAT_YCbCr_420_SP, dstFmt);
    if (st != IM_STATUS_SUCCESS) {
      RC_LOG_ERROR("[RkDecoder] imcvtcolor NV12->%s failed: %s",
                   (outputFormat_ == RkOutputFormat::kRGB888) ? "RGB" : "BGR",
                   imStrError(st));
    } else {
      // RGA wrote colorBuf_ via DMA; invalidate cache before the CPU reads it.
      mpp_buffer_sync_begin(colorBuf_);
      uint8_t* colorPtr = static_cast<uint8_t*>(mpp_buffer_get_ptr(colorBuf_));
      // Zero-copy: cv::Mat wraps the reused DMA buffer (no allocation/copy).
      // Safe here because the caller clones the result before the next Decode()
      // overwrites colorBuf_ (see post_process_node DeviceProcessor callbacks).
      output = cv::Mat(h, w, CV_8UC3, colorPtr);
      success = !output.empty();
    }
  } else {
    RC_LOG_ERROR("[RkDecoder] invalid frame dimensions w=%d h=%d hor=%d ver=%d",
                 w, h, hor, ver);
  }

  // ---- G) Return the output task so the decoder can reuse its slot ----
  ret = mpi_->enqueue(ctx_, MPP_PORT_OUTPUT, task);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("[RkDecoder] enqueue OUTPUT failed ret=%d", static_cast<int>(ret));
    // Data already consumed; do not flip success, but next Decode() may break.
  }

  cleanup();
  return success;
}

std::string RkHardwareDecoder::GetName() const {
  return initialized_ ? "Rockchip MPP Hardware (MJPEG)" : "Rockchip MPP (init failed)";
}

}  // namespace odin_decoder

#endif  // USE_RK_DECODER