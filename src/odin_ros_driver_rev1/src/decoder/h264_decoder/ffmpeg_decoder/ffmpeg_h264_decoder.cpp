#include "ffmpeg_h264_decoder.h"

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <vector>

// FFmpeg is a C library; all headers must be in an extern "C" block.
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#ifdef ODIN_HAVE_LIBYUV
// libyuv fast colour conversion (YUV 4:2:0 → BGR). ~7x faster than libswscale
// on the 1280x1088 ODIN stream (0.3 ms vs 2.2 ms per frame), measured with
// h264_bench on Meteor Lake + VAAPI.
#include <libyuv.h>
#endif

#include "utility/rc_log.hpp"

namespace odin_decoder {

// Build a log prefix like "[H264|camera0]" or "[H264]" when no label is set.
static std::string LogTag(const std::string& label) {
  if (label.empty()) return "[H264]";
  return "[H264|" + label + "]";
}

// Scan an H.264 Annex-B access unit for an IDR slice NAL unit
// (nal_unit_type == 5) per ITU-T H.264 §7.3.2.1 / §7.4.2.4.
// Start code detection follows §B.1 (3-byte 0x000001 or 4-byte 0x00000001).
// The scan is O(n) in the worst case but terminates as soon as the IDR is found;
// for typical camera streams the IDR NAL appears within the first 100 bytes.
static bool ContainsIDR(const uint8_t* data, size_t size) {
  if (!data || size < 5) return false;
  size_t i = 0;
  while (i + 3 < size) {
    // Detect 3-byte (0x000001) or 4-byte (0x00000001) start code.
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
    if (nal_unit_type == 5) return true;  // IDR slice
    i = nal_start + 1;  // Skip past the NAL header byte and continue
  }
  return false;
}

// Return the user-specified device path from environment variables, or nullptr
// to let FFmpeg choose the default device.
//   CUDA  → ODIN_CUDA_DEVICE
//   VAAPI → ODIN_VA_DEVICE   (e.g. /dev/dri/renderD128)
//   VDPAU → ODIN_VDPAU_DEVICE (e.g. /dev/dri/card0)
static const char* GetDevicePath(enum AVHWDeviceType type) {
  const char* env = nullptr;
  switch (type) {
    case AV_HWDEVICE_TYPE_CUDA:  env = std::getenv("ODIN_CUDA_DEVICE");   break;
    case AV_HWDEVICE_TYPE_VAAPI: env = std::getenv("ODIN_VA_DEVICE");    break;
    case AV_HWDEVICE_TYPE_VDPAU: env = std::getenv("ODIN_VDPAU_DEVICE"); break;
    default: break;
  }
  return (env && env[0] != '\0') ? env : nullptr;
}

// get_format callback: steer the decoder to output HW surface frames when the
// codec context has been configured for hardware decode (hw_device_ctx set).
// Without this callback the decoder would silently fall back to software
// output even when a hardware device is attached.  Mirrors the canonical
// FFmpeg hw_decode example: once we pick the HW pixel format, the decoder
// allocates hw_frames_ctx internally with correct surface pool sizing.
static enum AVPixelFormat GetHwFormat(AVCodecContext* ctx,
                                      const enum AVPixelFormat* pix_fmts) {
  for (const enum AVPixelFormat* p = pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(*p);
    if (desc && (desc->flags & AV_PIX_FMT_FLAG_HWACCEL)) {
      return *p;
    }
  }
  return pix_fmts[0];
}

// ---------------------------------------------------------------------------
// Silent hardware probing
// ---------------------------------------------------------------------------
// Hardware probing walks EVERY backend (named CUDA/QSV/VAAPI/VDPAU decoders →
// dynamically discovered decoders → generic h264 + hw_device_ctx) and on any
// given machine most of them are EXPECTED to fail (no NVIDIA GPU → the CUDA
// probes fail, and so on).  Two noise sources used to make even a *successful*
// probe look like an error storm:
//   1. our own per-backend "av_hwdevice_ctx_create failed" INFO lines, and
//   2. FFmpeg's internal stderr logs ("Cannot load libcuda.so.1",
//      "libva: iHD_drv_video.so init failed", ...).
// Both are silenced while probing: our per-backend failures are collected into
// a vector and FFmpeg messages are captured by an av_log filter callback.  The
// collected material is emitted exactly once — and ONLY when no backend
// matched — as one consolidated warning.  When a backend succeeds the user
// sees a single success line and no errors at all.
namespace {

// >0 while some thread is probing hardware backends.  FFmpeg log output is
// captured instead of printed during that window.
std::atomic<int> g_probe_depth(0);
std::mutex g_capture_mutex;
std::vector<std::string> g_captured_ffmpeg_logs;

// av_log callback: capture (instead of print) while probing; otherwise forward
// to the default callback, which applies av_log_set_level as usual.
void ProbeLogFilter(void* avcl, int level, const char* fmt, va_list vl) {
  if (g_probe_depth.load(std::memory_order_acquire) > 0) {
    if (level > av_log_get_level()) return;  // below threshold — invisible anyway
    char buf[512];
    if (vsnprintf(buf, sizeof(buf), fmt, vl) <= 0) return;
    buf[sizeof(buf) - 1] = '\0';
    std::string line(buf);
    // Strip trailing newlines — the ROS log macros add their own.
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    if (line.empty()) return;
    std::lock_guard<std::mutex> lock(g_capture_mutex);
    if (g_captured_ffmpeg_logs.size() < 64) {  // hard cap: probing is chatty
      g_captured_ffmpeg_logs.push_back(line);
    }
    return;
  }
  av_log_default_callback(avcl, level, fmt, vl);
}

// Install the filter exactly once.  av_log_set_callback is process-global;
// C++11 guarantees thread-safe initialisation of function-local statics.
void InstallProbeLogFilter() {
  static const bool installed = []() {
    av_log_set_callback(&ProbeLogFilter);
    return true;
  }();
  (void)installed;
}

// RAII guard: mute FFmpeg log output for the duration of a hardware probe.
// Nesting-safe: only the outermost probe clears the capture buffer.
class ScopedFfmpegProbeMute {
 public:
  ScopedFfmpegProbeMute() {
    InstallProbeLogFilter();
    if (g_probe_depth.fetch_add(1, std::memory_order_acq_rel) == 0) {
      std::lock_guard<std::mutex> lock(g_capture_mutex);
      g_captured_ffmpeg_logs.clear();
    }
  }
  ~ScopedFfmpegProbeMute() {
    g_probe_depth.fetch_sub(1, std::memory_order_acq_rel);
  }
};

// Hand out (and clear) the FFmpeg messages captured during probing.
std::vector<std::string> TakeCapturedFfmpegLogs() {
  std::lock_guard<std::mutex> lock(g_capture_mutex);
  std::vector<std::string> out;
  out.swap(g_captured_ffmpeg_logs);
  return out;
}

// Open `codec` with a hardware device context of `device_type`.  Returns the
// opened context (caller takes ownership) or nullptr with a short failure
// reason in *reason.  Never logs on failure — probe failures are reported only
// when the entire candidate chain fails (see TryInitHardware).
AVCodecContext* OpenWithHwDevice(const AVCodec* codec,
                                 enum AVHWDeviceType device_type,
                                 std::string* reason) {
  const char* device = GetDevicePath(device_type);
  const char* device_disp = device ? device : "<default>";

  AVBufferRef* hw_ctx = nullptr;
  if (av_hwdevice_ctx_create(&hw_ctx, device_type, device, nullptr, 0) < 0) {
    *reason = std::string("av_hwdevice_ctx_create failed (device=") + device_disp + ")";
    return nullptr;
  }

  AVCodecContext* ctx = avcodec_alloc_context3(codec);
  if (!ctx) {
    av_buffer_unref(&hw_ctx);
    *reason = "avcodec_alloc_context3 failed";
    return nullptr;
  }

  ctx->flags  |= AV_CODEC_FLAG_LOW_DELAY;
  ctx->flags2 |= AV_CODEC_FLAG2_FAST;
  ctx->hw_device_ctx = av_buffer_ref(hw_ctx);
  ctx->get_format = GetHwFormat;
  av_buffer_unref(&hw_ctx);

  if (avcodec_open2(ctx, codec, nullptr) < 0) {
    avcodec_free_context(&ctx);
    *reason = "avcodec_open2 failed";
    return nullptr;
  }
  return ctx;
}

}  // namespace

FfmpegH264Decoder::FfmpegH264Decoder(const std::string& camera_label) {
  camera_label_ = camera_label;
  // Testing switch: set ODIN_H264_FORCE_SW=1 to skip all hardware backends
  // and force pure software decoding (for HW-vs-SW A/B comparison).
  const char* force_sw = std::getenv("ODIN_H264_FORCE_SW");
  const bool force_software = (force_sw && force_sw[0] == '1');
  if (force_software) {
    RC_LOG_INFO("%s [init] ODIN_H264_FORCE_SW=1 set — forcing SOFTWARE decode",
                LogTag(camera_label_).c_str());
  }
  if (!force_software && TryInitHardware()) {
    // Hardware decoder initialised successfully.
  } else if (TryInitSoftware()) {
    // Software fallback — emit a prominent warning.
    RC_LOG_WARN("%s ============================================================", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  FFmpeg H.264 SOFTWARE decoding in use (no HW backend found)", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  CPU usage will be significantly higher.", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  To enable HW decoding: install GPU drivers and ensure", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  FFmpeg is compiled with --enable-nvdec or --enable-vaapi.", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  For Intel/AMD iGPU: install va-driver-all and", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  intel-media-va-driver or mesa-va-drivers.", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s  Optionally set ODIN_VA_DEVICE=/dev/dri/renderD128.", LogTag(camera_label_).c_str());
    RC_LOG_WARN("%s ============================================================", LogTag(camera_label_).c_str());
  }

  if (initialized_) {
    frame_ = av_frame_alloc();
    pkt_   = av_packet_alloc();
    if (!frame_ || !pkt_) {
      std::cerr << LogTag(camera_label_) << " [init] av_frame/packet alloc failed\n";
      initialized_ = false;
      return;
    }
    if (is_hardware_) {
      sw_frame_ = av_frame_alloc();
      if (!sw_frame_) {
        std::cerr << LogTag(camera_label_) << " [init] sw_frame alloc failed\n";
        initialized_ = false;
        return;
      }
    }
  }
}

bool FfmpegH264Decoder::TryInitHardware() {
  struct HwBackend {
    const char* decoder_name;  // nullptr → use generic h264 decoder with hw_device_ctx
    enum AVHWDeviceType device_type;
    const char* display_name;
  };

  // Priority order:
  //   1. Named HW decoders (h264_nvdec, h264_cuvid, h264_qsv, h264_vaapi, h264_vdpau)
  //   2. Dynamically discovered HW decoders (any other H.264 decoder with HW config)
  //   3. Generic h264 decoder + HW device context (works for Intel/AMD VAAPI)
  static const HwBackend backends[] = {
    {"h264_nvdec", AV_HWDEVICE_TYPE_CUDA,  "NVDEC"},
    {"h264_cuvid", AV_HWDEVICE_TYPE_CUDA,  "CUVID"},
    {"h264_qsv",   AV_HWDEVICE_TYPE_QSV,   "QSV"},
    {"h264_vaapi", AV_HWDEVICE_TYPE_VAAPI, "VAAPI"},
    {"h264_vdpau", AV_HWDEVICE_TYPE_VDPAU, "VDPAU"},
  };

  // Collect names of decoders already covered by the static list so the
  // dynamic discovery phase doesn't retry them.
  std::set<std::string> already_tried;
  for (const auto& b : backends) {
    already_tried.insert(b.decoder_name);
  }

  // Probe silently: nothing is logged per backend while the candidate chain
  // is being walked (FFmpeg log output is muted too — see ScopedFfmpegProbeMute).
  // Failures accumulate in `failures` (ours) and in the captured FFmpeg log
  // buffer (FFmpeg's own diagnostics) and are reported once, at the bottom,
  // ONLY if no backend matched.  A machine with working VAAPI still fails the
  // CUDA probes first — printing those as errors mid-probe was pure noise.
  ScopedFfmpegProbeMute probe_mute;
  std::vector<std::string> failures;
  const std::string tag = LogTag(camera_label_);

  // --- Phase 1: try each named HW decoder ---------------------------------
  for (const auto& b : backends) {
    const AVCodec* codec = avcodec_find_decoder_by_name(b.decoder_name);
    if (!codec) continue;  // not compiled into this FFmpeg build — absent, not failed

    std::string reason;
    AVCodecContext* ctx = OpenWithHwDevice(codec, b.device_type, &reason);
    if (!ctx) {
      failures.push_back(std::string(b.display_name) + ": " + reason);
      continue;
    }

    codec_ctx_       = ctx;
    is_hardware_     = true;
    hw_backend_name_ = b.display_name;
    initialized_     = true;
    RC_LOG_INFO("%s [init] FFmpeg hardware decoder: %s (%s)",
                tag.c_str(), b.display_name, b.decoder_name);
    return true;
  }

  // --- Phase 1b: dynamically discover any remaining HW decoders -----------
  // Scan all registered decoders for H.264 codecs that advertise HW config
  // with AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX. This catches decoders
  // compiled into FFmpeg that we don't know about at compile time (e.g.
  // h264_d3d11va, h264_vulkan, or future HW backends).
  void* iter = nullptr;
  const AVCodec* dyn_codec = nullptr;
  while ((dyn_codec = av_codec_iterate(&iter))) {
    if (dyn_codec->id != AV_CODEC_ID_H264) continue;
    if (!avcodec_find_decoder_by_name(dyn_codec->name)) continue;
    if (already_tried.count(dyn_codec->name)) continue;

    // Check if this decoder supports HW_DEVICE_CTX method.
    bool has_hw_config = false;
    enum AVHWDeviceType dyn_device_type = AV_HWDEVICE_TYPE_NONE;
    for (int i = 0; ; i++) {
      const AVCodecHWConfig* cfg = avcodec_get_hw_config(dyn_codec, i);
      if (!cfg) break;
      if (cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) {
        has_hw_config = true;
        dyn_device_type = cfg->device_type;
        break;
      }
    }
    if (!has_hw_config) continue;

    already_tried.insert(dyn_codec->name);

    std::string reason;
    AVCodecContext* ctx = OpenWithHwDevice(dyn_codec, dyn_device_type, &reason);
    if (!ctx) {
      failures.push_back(std::string(dyn_codec->name) + ": " + reason);
      continue;
    }

    codec_ctx_       = ctx;
    is_hardware_     = true;
    hw_backend_name_ = dyn_codec->name;
    initialized_     = true;
    const char* device = GetDevicePath(dyn_device_type);
    RC_LOG_INFO("%s [init] FFmpeg hardware decoder (dynamic): %s (device=%s)",
                tag.c_str(), dyn_codec->name, device ? device : "<default>");
    return true;
  }

  // --- Phase 2: generic h264 decoder + HW device context ------------------
  // Many VAAPI builds (Intel iHD, AMD radeonsi) expose HW acceleration only
  // through the standard h264 decoder with hw_device_ctx set.
  struct GenericHwPath {
    enum AVHWDeviceType device_type;
    const char* display_name;
  };
  static const GenericHwPath generic_paths[] = {
    {AV_HWDEVICE_TYPE_VAAPI, "VAAPI"},
    {AV_HWDEVICE_TYPE_VDPAU, "VDPAU"},
  };

  const AVCodec* sw_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (sw_codec) {
    for (const auto& g : generic_paths) {
      std::string reason;
      AVCodecContext* ctx = OpenWithHwDevice(sw_codec, g.device_type, &reason);
      if (!ctx) {
        failures.push_back(std::string("generic-") + g.display_name + ": " + reason);
        continue;
      }

      codec_ctx_       = ctx;
      is_hardware_     = true;
      hw_backend_name_ = std::string(g.display_name) + "-generic";
      initialized_     = true;
      const char* device = GetDevicePath(g.device_type);
      RC_LOG_INFO("%s [init] FFmpeg hardware decoder (generic): %s (h264 + hw_device_ctx, device=%s)",
                  tag.c_str(), g.display_name, device ? device : "<default>");
      return true;
    }
  }

  // --- Nothing matched: consolidated report (the ONLY error output) -------
  // Every candidate failed — this is the single place probe failures become
  // visible: our per-backend reasons first, then FFmpeg's captured
  // diagnostics (e.g. "libva: iHD_drv_video.so init failed", the direct
  // evidence for driver-stack problems — see docs/hardware_decode.md).
  if (failures.empty()) {
    RC_LOG_WARN("%s [init] no hardware H.264 backend available "
                "(this FFmpeg build exposes no usable HW decoder)",
                tag.c_str());
    return false;
  }

  RC_LOG_WARN("%s [init] no hardware H.264 backend available — %zu candidate(s) tried, all failed:",
              tag.c_str(), failures.size());
  for (const auto& f : failures) {
    RC_LOG_WARN("%s [init]   %s", tag.c_str(), f.c_str());
  }

  // Replay FFmpeg's own probe output, deduplicated and capped so the report
  // stays readable.  Empty if a concurrent probe already consumed the buffer.
  std::vector<std::string> ffmpeg_lines = TakeCapturedFfmpegLogs();
  std::set<std::string> seen;
  std::vector<std::string> unique_lines;
  for (const auto& line : ffmpeg_lines) {
    if (seen.insert(line).second) unique_lines.push_back(line);
  }
  const size_t shown = std::min(unique_lines.size(), static_cast<size_t>(12));
  for (size_t i = 0; i < shown; ++i) {
    RC_LOG_WARN("%s [init]   ffmpeg: %s", tag.c_str(), unique_lines[i].c_str());
  }
  if (unique_lines.size() > shown) {
    RC_LOG_WARN("%s [init]   ffmpeg: ... %zu more diagnostic line(s) suppressed",
                tag.c_str(), unique_lines.size() - shown);
  }

  return false;
}

bool FfmpegH264Decoder::TryInitSoftware() {
  const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    std::cerr << LogTag(camera_label_) << " [init] avcodec_find_decoder(H264) failed\n";
    return false;
  }

  AVCodecContext* ctx = avcodec_alloc_context3(codec);
  if (!ctx) {
    std::cerr << LogTag(camera_label_) << " [init] avcodec_alloc_context3 failed\n";
    return false;
  }

  ctx->flags  |= AV_CODEC_FLAG_LOW_DELAY;
  ctx->flags2 |= AV_CODEC_FLAG2_FAST;

  if (avcodec_open2(ctx, codec, nullptr) < 0) {
    std::cerr << LogTag(camera_label_) << " [init] avcodec_open2 failed\n";
    avcodec_free_context(&ctx);
    return false;
  }

  codec_ctx_   = ctx;
  is_hardware_ = false;
  initialized_ = true;
  return true;
}

bool FfmpegH264Decoder::FallbackToSoftware() {
  RC_LOG_WARN("%s [fallback] HW decoder '%s' failed at runtime, switching to software",
              LogTag(camera_label_).c_str(), hw_backend_name_.c_str());

  // Tear down the existing HW codec context.
  if (codec_ctx_) avcodec_free_context(&codec_ctx_);
  // sw_frame_ is no longer needed for software decoding.
  if (sw_frame_) av_frame_free(&sw_frame_);
  // Reset sws context since pixel format may change.
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }

  is_hardware_ = false;
  hw_fail_count_ = 0;

  if (!TryInitSoftware()) {
    std::cerr << LogTag(camera_label_) << " [fallback] software fallback failed too!\n";
    initialized_ = false;
    return false;
  }

  // Re-allocate frame_ and pkt_ if they were freed.
  if (!frame_) frame_ = av_frame_alloc();
  if (!pkt_)   pkt_   = av_packet_alloc();
  if (!frame_ || !pkt_) {
    initialized_ = false;
    return false;
  }

  waiting_for_keyframe_ = true;
  RC_LOG_WARN("%s [fallback] software decoder ready, waiting for next IDR",
              LogTag(camera_label_).c_str());
  return true;
}

FfmpegH264Decoder::~FfmpegH264Decoder() {
  if (sws_ctx_)   sws_freeContext(sws_ctx_);
  if (sw_frame_)  av_frame_free(&sw_frame_);
  if (frame_)     av_frame_free(&frame_);
  if (pkt_)       av_packet_free(&pkt_);
  if (codec_ctx_) avcodec_free_context(&codec_ctx_);
}

bool FfmpegH264Decoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  if (!initialized_) return false;

  // --- waiting_for_keyframe_ gate ----------------------------------------
  // When the decoder starts, recovers from an error, or is externally Reset(),
  // it sits in waiting_for_keyframe_ mode.  In this state we inspect every
  // incoming AU for an IDR NAL before touching FFmpeg at all.  Non-IDR frames
  // are silently discarded; feeding them to FFmpeg could produce
  // error-concealed (garbled) output frames that would be published.
  if (waiting_for_keyframe_) {
    if (!ContainsIDR(data, size)) {
      return false;  // Discard: no IDR present, decoder not ready
    }
    // IDR found: flush any stale state and give it a clean DPB.
    avcodec_flush_buffers(codec_ctx_);
    waiting_for_keyframe_ = false;
    RC_LOG_INFO("%s [init] IDR received, leaving keyframe-wait state", LogTag(camera_label_).c_str());
  }

  // Point the AVPacket at the caller's buffer (no copy — avcodec_send_packet
  // processes the data synchronously before returning).
  pkt_->data = const_cast<uint8_t*>(data);
  pkt_->size = static_cast<int>(size);

  int ret = avcodec_send_packet(codec_ctx_, pkt_);
  if (ret < 0) {
    std::cerr << LogTag(camera_label_) << " [decode] avcodec_send_packet error: " << ret << "\n";
    // If the HW decoder fails, fall back to software immediately.
    if (is_hardware_) {
      if (FallbackToSoftware()) {
        // Retry this frame with the new software decoder.
        pkt_->data = const_cast<uint8_t*>(data);
        pkt_->size = static_cast<int>(size);
        ret = avcodec_send_packet(codec_ctx_, pkt_);
        if (ret < 0) {
          avcodec_flush_buffers(codec_ctx_);
          waiting_for_keyframe_ = true;
          return false;
        }
      } else {
        return false;
      }
    } else {
      // Software decoder error — flush and wait for keyframe.
      avcodec_flush_buffers(codec_ctx_);
      waiting_for_keyframe_ = true;
      return false;
    }
  }

  ret = avcodec_receive_frame(codec_ctx_, frame_);
  if (ret == AVERROR(EAGAIN)) {
    // Decoder is buffering; needs more input before it can emit a frame.
    // This is expected until the first IDR + SPS/PPS have been processed.
    return false;
  }
  if (ret == AVERROR_EOF) {
    // Stream ended.
    return false;
  }
  if (ret < 0) {
    std::cerr << LogTag(camera_label_) << " [decode] avcodec_receive_frame error: " << ret << "\n";
    // If the HW decoder fails, fall back to software immediately.
    if (is_hardware_) {
      if (FallbackToSoftware()) {
        // Retry this frame with the new software decoder.
        pkt_->data = const_cast<uint8_t*>(data);
        pkt_->size = static_cast<int>(size);
        ret = avcodec_send_packet(codec_ctx_, pkt_);
        if (ret == 0) {
          ret = avcodec_receive_frame(codec_ctx_, frame_);
        }
        if (ret < 0) {
          avcodec_flush_buffers(codec_ctx_);
          waiting_for_keyframe_ = true;
          return false;
        }
      } else {
        return false;
      }
    } else {
      avcodec_flush_buffers(codec_ctx_);
      waiting_for_keyframe_ = true;
      return false;
    }
  }

  // For hardware decoders, the decoded frame lives in GPU memory.
  // Transfer it to a CPU-accessible frame before colour conversion.
  AVFrame* decode_frame = frame_;
  if (is_hardware_ && frame_->hw_frames_ctx) {
    if (!sw_frame_) {
      av_frame_unref(frame_);
      return false;
    }
    // Reuse the sw_frame_ buffer across frames: av_hwframe_transfer_data only
    // allocates on first use (or after the unref below when geometry changed),
    // and keeping the same ~2 MB buffer hot in cache/TLB roughly halves the
    // GPU→CPU transfer cost (2.8 ms → 1.3 ms per 1280x1088 frame, measured
    // with h264_bench; per-frame re-allocation adds malloc + first-touch page
    // faults). Rebuild the buffer only when the frame geometry changes.
    if (sw_frame_->data[0] &&
        (sw_frame_->width != frame_->width || sw_frame_->height != frame_->height)) {
      av_frame_unref(sw_frame_);
    }
    int hw_ret = av_hwframe_transfer_data(sw_frame_, frame_, 0);
    if (hw_ret < 0) {
      std::cerr << LogTag(camera_label_) << " [decode] av_hwframe_transfer_data error: " << hw_ret << "\n";
      av_frame_unref(frame_);
      av_frame_unref(sw_frame_);
      // GPU→CPU transfer failed: fall back to software immediately.
      if (FallbackToSoftware()) {
        // Retry this frame with the new software decoder.
        pkt_->data = const_cast<uint8_t*>(data);
        pkt_->size = static_cast<int>(size);
        ret = avcodec_send_packet(codec_ctx_, pkt_);
        if (ret == 0) {
          ret = avcodec_receive_frame(codec_ctx_, frame_);
        }
        if (ret < 0) {
          avcodec_flush_buffers(codec_ctx_);
          waiting_for_keyframe_ = true;
          return false;
        }
        // Software decoder produces CPU frames directly; skip hw transfer.
        decode_frame = frame_;
      } else {
        waiting_for_keyframe_ = true;
        return false;
      }
    } else {
      decode_frame = sw_frame_;
    }
  }

  // Convert decoded pixel format → BGR24 for OpenCV.
  int w = decode_frame->width;
  int h = decode_frame->height;
  output.create(h, w, CV_8UC3);

  bool converted = false;
#ifdef ODIN_HAVE_LIBYUV
  // --- libyuv fast path -----------------------------------------------------
  // WARNING: libyuv's naming is counter-intuitive for RGB24 output. The
  // "...ToRGB24" family writes B,G,R byte order (identical to OpenCV BGR24 /
  // cv_bridge "bgr8"), while the "...ToRAW" family writes R,G,B byte order.
  // We need BGR24 here, so the "...ToRGB24" functions are the correct ones;
  // using "...ToRAW" swaps the red and blue channels (skin tones turn blue).
  // libyuv's default matrix is BT.601 limited-range, which matches libswscale's
  // default for this stream (verified on real frames: the luma path agrees to
  // mean 0.42/255; the chroma path differs only in the upsampling filter —
  // libyuv point-samples, sws bilinearly interpolates). Streams explicitly
  // tagged with another matrix (BT.709/2020) or full range (JPEG) fall through
  // to sws so the colour matrix stays correct.
  // Escape hatch: ODIN_DISABLE_LIBYUV=1 forces the sws path.
  static const bool libyuv_forced_off = []() {
    const char* env = std::getenv("ODIN_DISABLE_LIBYUV");
    return env && env[0] == '1';
  }();
  const bool matrix_is_bt601 =
      (decode_frame->colorspace == AVCOL_SPC_UNSPECIFIED ||
       decode_frame->colorspace == AVCOL_SPC_BT470BG ||
       decode_frame->colorspace == AVCOL_SPC_SMPTE170M) &&
      decode_frame->color_range != AVCOL_RANGE_JPEG;
  if (!libyuv_forced_off && matrix_is_bt601 && (w & 1) == 0) {
    int ly_ret = -1;
    const int dst_stride = static_cast<int>(output.step);
    if (decode_frame->format == AV_PIX_FMT_NV12 && decode_frame->data[1]) {
      ly_ret = libyuv::NV12ToRGB24(decode_frame->data[0], decode_frame->linesize[0],
                                   decode_frame->data[1], decode_frame->linesize[1],
                                   output.data, dst_stride, w, h);
    } else if (decode_frame->format == AV_PIX_FMT_YUV420P) {
      ly_ret = libyuv::I420ToRGB24(decode_frame->data[0], decode_frame->linesize[0],
                                   decode_frame->data[1], decode_frame->linesize[1],
                                   decode_frame->data[2], decode_frame->linesize[2],
                                   output.data, dst_stride, w, h);
    } else if (decode_frame->format == AV_PIX_FMT_NV21 && decode_frame->data[1]) {
      ly_ret = libyuv::NV21ToRGB24(decode_frame->data[0], decode_frame->linesize[0],
                                   decode_frame->data[1], decode_frame->linesize[1],
                                   output.data, dst_stride, w, h);
    }
    converted = (ly_ret == 0);
  }
#endif

  if (!converted) {
    // sws_getCachedContext reuses the existing context when dimensions/formats
    // are unchanged, avoiding re-allocation on every frame.
    sws_ctx_ = sws_getCachedContext(
        sws_ctx_,
        w, h, static_cast<AVPixelFormat>(decode_frame->format),
        w, h, AV_PIX_FMT_BGR24,
        SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_ctx_) {
      av_frame_unref(frame_);
      return false;
    }
    uint8_t* dst[1] = {output.data};
    int      lsz[1] = {static_cast<int>(output.step)};
    sws_scale(sws_ctx_, decode_frame->data, decode_frame->linesize, 0, h, dst, lsz);
  }

  // frame_ must return to the hw surface pool every iteration. sw_frame_ is
  // intentionally kept: its CPU buffer is overwritten by the next
  // av_hwframe_transfer_data call (see the transfer comment above).
  av_frame_unref(frame_);

  return true;
}

std::string FfmpegH264Decoder::GetName() const {
  if (is_hardware_) {
    return "FfmpegH264Decoder (" + hw_backend_name_ + " hardware)";
  }
  return "FfmpegH264Decoder (software)";
}

}  // namespace odin_decoder
