#include "rockchip_decoder/rockchip_h264_decoder.h"

#ifdef USE_RK_H264_DECODER

#include <cstring>
#include <unistd.h>  // usleep

#include <rockchip/rk_vdec_cfg.h>  // mpp_dec_cfg_init/set_u32/deinit + MppDecCfg

#include <rga/im2d.h>  // imcvtcolor / wrapbuffer_fd / IM_STATUS / imStrError
#include <rga/rga.h>   // RK_FORMAT_YCbCr_420_SP / RK_FORMAT_BGR_888

#include <fcntl.h>          // ioctl
#include <sys/ioctl.h>      // ioctl
#include <linux/dma-buf.h>  // DMA_BUF_IOCTL_SYNC / dma_buf_sync

#include "utility/rc_log.hpp"

namespace odin_decoder {

// Build a log prefix like "[H264|camera0|RkH264Decoder]" or "[H264|RkH264Decoder]".
static std::string LogTag(const std::string& label) {
  std::string s = "[H264";
  if (!label.empty()) s += "|" + label;
  s += "|RkH264Decoder]";
  return s;
}

namespace {

// ============================================================================
// DMA buffer cache sync helpers (see rockchip_hardware_decoder.cpp for the
// full rationale — duplicated here so this translation unit stays
// self-contained).
// ============================================================================
#if !defined(MPP_HAS_BUFFER_SYNC)
void mpp_buffer_sync_begin(MppBuffer buffer) {
  int fd = mpp_buffer_get_fd(buffer);
  struct dma_buf_sync sync = {};
  sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
  int ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  (void)ret;
}
#endif

// Number of times decode_put_packet is retried when MPP reports its input
// queue is full (MPP_ERR_BUFFER_FULL). The hardware normally drains within
// a few milliseconds.
constexpr int kMaxPutRetries = 20;
// Number of decode_get_frame polls per Decode() call. One AU normally
// yields at most one frame; a couple of extra retries absorb the info-change
// event that precedes real output after a resolution change.
constexpr int kMaxGetRetries = 4;

// Scan an Annex-B access unit for an IDR slice (NAL type 5) or SPS (NAL
// type 7) NAL unit. Matches JetsonNvdecDecoder::ContainsH264Keyframe: SPS is
// accepted as a keyframe boundary because some sources emit the initial
// SPS/PPS as a standalone access unit.
bool ContainsH264Keyframe(const uint8_t* data, size_t size) {
  if (!data || size < 5) return false;
  size_t i = 0;
  while (i + 3 < size) {
    size_t nal_start = 0;
    if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01) {
      nal_start = i + 3;
    } else if (i + 4 < size &&
               data[i] == 0x00 && data[i + 1] == 0x00 &&
               data[i + 2] == 0x00 && data[i + 3] == 0x01) {
      nal_start = i + 4;
    } else {
      ++i;
      continue;
    }
    if (nal_start >= size) break;
    uint8_t nal_unit_type = data[nal_start] & 0x1F;
    if (nal_unit_type == 5 || nal_unit_type == 7) return true;
    i = nal_start + 1;
  }
  return false;
}

}  // namespace

// =====================================================================
//  ctor / dtor
// =====================================================================

RkH264Decoder::RkH264Decoder(const std::string& camera_label) {
  camera_label_ = camera_label;
  MPP_RET ret = mpp_create(&ctx_, &mpi_);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] mpp_create failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    Shutdown();
    return;
  }

  ret = mpp_init(ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingAVC);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] mpp_init(DEC,AVC) failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    Shutdown();
    return;
  }

  MppDecCfg cfgHandle = nullptr;
  ret = mpp_dec_cfg_init(&cfgHandle);
  if (ret != MPP_OK || !cfgHandle) {
    RC_LOG_ERROR("%s [init] mpp_dec_cfg_init failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    Shutdown();
    return;
  }
  ret = mpi_->control(ctx_, MPP_DEC_GET_CFG, cfgHandle);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] MPP_DEC_GET_CFG failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  // split_parse=1: let MPP internally re-frame the Annex-B bitstream, so the
  // caller does not need to guarantee exactly one frame's NAL units per
  // Decode() call.
  ret = mpp_dec_cfg_set_u32(cfgHandle, "base:split_parse", 1);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] set base:split_parse=1 failed ret=%d",
                 LogTag(camera_label_).c_str(), static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  ret = mpi_->control(ctx_, MPP_DEC_SET_CFG, cfgHandle);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] MPP_DEC_SET_CFG failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    mpp_dec_cfg_deinit(cfgHandle);
    Shutdown();
    return;
  }
  mpp_dec_cfg_deinit(cfgHandle);

  // Decoder always outputs NV12; RGA converts it to BGR afterwards.
  MppFrameFormat mppOutFmt = MPP_FMT_YUV420SP;
  ret = mpi_->control(ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &mppOutFmt);
  if (ret != MPP_OK) {
    RC_LOG_ERROR("%s [init] MPP_DEC_SET_OUTPUT_FORMAT(NV12) failed ret=%d",
                 LogTag(camera_label_).c_str(), static_cast<int>(ret));
    Shutdown();
    return;
  }

  initialized_ = true;
  RC_LOG_DEBUG("%s [init] OK (codec=AVC, decode=NV12 -> RGA -> BGR888, simple API)", LogTag(camera_label_).c_str());
}

RkH264Decoder::~RkH264Decoder() {
  Shutdown();
}

// =====================================================================
//  EnsureColorBuffer -- (re)allocate the RGA BGR destination DMA buffer
// =====================================================================

bool RkH264Decoder::EnsureColorBuffer(int width, int height) {
  if (colorBuf_ && width == colorW_ && height == colorH_) {
    return true;  // already sized correctly
  }

  if (colorBuf_) {
    mpp_buffer_put(colorBuf_);
    colorBuf_ = nullptr;
  }
  if (!colorGroup_) {
    MPP_RET ret = mpp_buffer_group_get_internal(&colorGroup_, MPP_BUFFER_TYPE_DRM);
    if (ret != MPP_OK) {
      RC_LOG_ERROR("%s [decode] color_group get_internal failed ret=%d",
                   LogTag(camera_label_).c_str(), static_cast<int>(ret));
      return false;
    }
  }

  const size_t colorBytes = static_cast<size_t>(width) * height * 3;
  MPP_RET ret = mpp_buffer_get(colorGroup_, &colorBuf_, colorBytes);
  if (ret != MPP_OK || !colorBuf_) {
    RC_LOG_ERROR("%s [decode] color_buf get(%zu) failed ret=%d",
                 LogTag(camera_label_).c_str(), colorBytes, static_cast<int>(ret));
    return false;
  }

  colorW_ = width;
  colorH_ = height;
  return true;
}

// =====================================================================
//  ConvertFrame -- NV12 (MPP DMA) -> BGR888 (RGA DMA), zero-copy wrap
// =====================================================================

bool RkH264Decoder::ConvertFrame(MppFrame frame, cv::Mat& output) {
  MppBuffer srcBuf = mpp_frame_get_buffer(frame);
  if (!srcBuf) {
    RC_LOG_ERROR("%s [decode] decoded frame has no buffer", LogTag(camera_label_).c_str());
    return false;
  }

  const int w   = mpp_frame_get_width(frame);
  const int h   = mpp_frame_get_height(frame);
  const int hor = mpp_frame_get_hor_stride(frame);
  const int ver = mpp_frame_get_ver_stride(frame);
  if (w <= 0 || h <= 0 || hor < w || ver < h) {
    RC_LOG_ERROR("%s [decode] invalid frame dimensions w=%d h=%d hor=%d ver=%d",
                 LogTag(camera_label_).c_str(), w, h, hor, ver);
    return false;
  }

  if (!EnsureColorBuffer(w, h)) {
    return false;
  }

  rga_buffer_t rsrc = wrapbuffer_fd(mpp_buffer_get_fd(srcBuf), w, h,
                                    RK_FORMAT_YCbCr_420_SP, hor, ver);
  rga_buffer_t rdst = wrapbuffer_fd(mpp_buffer_get_fd(colorBuf_), w, h, RK_FORMAT_BGR_888);
  IM_STATUS st = imcvtcolor(rsrc, rdst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_BGR_888);
  if (st != IM_STATUS_SUCCESS) {
    RC_LOG_ERROR("%s [decode] imcvtcolor NV12->BGR failed: %s", LogTag(camera_label_).c_str(), imStrError(st));
    return false;
  }

  // RGA wrote colorBuf_ via DMA; invalidate cache before the CPU reads it.
  mpp_buffer_sync_begin(colorBuf_);
  uint8_t* colorPtr = static_cast<uint8_t*>(mpp_buffer_get_ptr(colorBuf_));
  // Zero-copy: cv::Mat wraps the reused DMA buffer (no allocation/copy).
  // Safe here because the caller clones the result before the next Decode()
  // overwrites colorBuf_ (see post_process_node DeviceProcessor callbacks).
  output = cv::Mat(h, w, CV_8UC3, colorPtr);
  return !output.empty();
}

// =====================================================================
//  Shutdown -- idempotent, order sensitive
//    mpp_destroy(ctx) must run first so the decoder releases its internal
//    references to any frames still held; only then is it safe to release
//    the color buffer/group.
// =====================================================================

void RkH264Decoder::Shutdown() {
  if (ctx_)        { mpp_destroy(ctx_); ctx_ = nullptr; mpi_ = nullptr; }
  if (colorBuf_)   { mpp_buffer_put(colorBuf_); colorBuf_ = nullptr; }
  if (colorGroup_) { mpp_buffer_group_put(colorGroup_); colorGroup_ = nullptr; }

  colorW_ = colorH_ = 0;
  initialized_ = false;
}

// =====================================================================
//  Decode -- simple streaming API: put one AU, drain any ready frame(s)
// =====================================================================

bool RkH264Decoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  if (!initialized_ || !mpi_) return false;
  if (!data || size == 0)     return false;

  // ---- waiting_for_keyframe_ gate ----------------------------------------
  // Only feed MPP after a keyframe boundary (IDR or SPS) to avoid garbled
  // error-concealed frames when reference frames are missing (stream start
  // or after a decode error).
  if (waiting_for_keyframe_) {
    if (!ContainsH264Keyframe(data, size)) {
      return false;  // discard non-keyframe data
    }
    waiting_for_keyframe_ = false;
  }

  // ---- A) Wrap the caller's buffer in a plain (non-DMA) MppPacket.
  // decode_put_packet consumes the bitstream synchronously (MPP copies it
  // into its internal ring buffer before returning), so no DMA input buffer
  // is required here — unlike the MJPEG advanced+MppTask path.
  MppPacket packet = nullptr;
  MPP_RET ret = mpp_packet_init(&packet, const_cast<uint8_t*>(data), size);
  if (ret != MPP_OK || !packet) {
    RC_LOG_ERROR("%s [decode] mpp_packet_init failed ret=%d", LogTag(camera_label_).c_str(), static_cast<int>(ret));
    return false;
  }

  bool put_done = false;
  for (int i = 0; i < kMaxPutRetries; ++i) {
    ret = mpi_->decode_put_packet(ctx_, packet);
    if (ret == MPP_OK) {
      put_done = true;
      break;
    }
    usleep(2000);
  }
  mpp_packet_deinit(&packet);
  if (!put_done) {
    RC_LOG_WARN("%s [decode] decode_put_packet busy, dropping AU (%zu bytes)", LogTag(camera_label_).c_str(), size);
    waiting_for_keyframe_ = true;
    return false;
  }

  // ---- B) Drain decoded frame(s). One AU normally yields at most one
  // displayable frame; an info-change event (resolution change) may consume
  // one iteration without producing output.
  bool success = false;
  for (int i = 0; i < kMaxGetRetries; ++i) {
    MppFrame frame = nullptr;
    ret = mpi_->decode_get_frame(ctx_, &frame);
    if (ret != MPP_OK) break;
    if (!frame) break;  // nothing ready yet — decoder still buffering

    if (mpp_frame_get_info_change(frame)) {
      RC_LOG_INFO("%s [decode] stream info change: %dx%d",
                  LogTag(camera_label_).c_str(),
                  mpp_frame_get_width(frame), mpp_frame_get_height(frame));
      mpi_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
      mpp_frame_deinit(&frame);
      continue;
    }

    if (mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame)) {
      RC_LOG_WARN("%s [decode] frame err=%u discard=%u",
                  LogTag(camera_label_).c_str(),
                  mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
      waiting_for_keyframe_ = true;
      mpp_frame_deinit(&frame);
      continue;
    }

    success = ConvertFrame(frame, output);
    mpp_frame_deinit(&frame);
    break;
  }

  return success;
}

std::string RkH264Decoder::GetName() const {
  return initialized_ ? "Rockchip MPP Hardware (H264)" : "Rockchip MPP H264 (init failed)";
}

}  // namespace odin_decoder

#endif  // USE_RK_H264_DECODER
