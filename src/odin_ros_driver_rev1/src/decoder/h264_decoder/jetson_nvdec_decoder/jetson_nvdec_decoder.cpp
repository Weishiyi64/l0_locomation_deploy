#include "jetson_nvdec_decoder.h"

#ifdef USE_JETSON_NVDEC_DECODER

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

// V4L2 definitions (v4l2_buffer, v4l2_plane, V4L2_PIX_FMT_H264, etc.)
extern "C" {
#include <linux/videodev2.h>
}

// Jetson Multimedia API headers.
// NvVideoDecoder.h declares the V4L2 video decoder helper class.
// NvV4l2Element.h provides the base class with output_plane / capture_plane.
// NvBuffer.h defines NvBuffer, NvBufferPlane, MAX_PLANES, etc.
#include "NvVideoDecoder.h"
#include "NvV4l2Element.h"
#include "NvBuffer.h"

// NvBufSurface is the CPU access path for decoded surfaces on L4T R35/R36.
// On JetPack 5/6 the decoder capture-plane buffers are NvBufSurface-backed
// DMA buffers: plain v4l2 mmap of the capture plane is not supported, the
// surface must be reached through NvBufSurfaceFromFd(buffer->planes[0].fd).
// nvbufsurftransform.h provides the VIC transform used to convert the
// block-linear decoder output into a CPU-readable pitch-linear surface.
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

#include "utility/rc_log.hpp"

namespace odin_decoder {

// Build a log prefix like "[H264|camera0|JetsonNvdecDecoder]" or "[H264|JetsonNvdecDecoder]".
static std::string LogTag(const std::string& label) {
  std::string s = "[H264";
  if (!label.empty()) s += "|" + label;
  s += "|JetsonNvdecDecoder]";
  return s;
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr uint32_t kNumOutputBuffers  = 4;                  // input plane
// Extra capture-plane buffers on top of the driver minimum. Keep this small:
// every additional buffer is a frame the decoder may hold before releasing.
static constexpr uint32_t kNumCaptureBuffers = 4;                  // capture plane
static constexpr uint32_t kInputBufferSize   = 4 * 1024 * 1024;   // 4 MiB
// Upper bound on how long Decode() waits for the capture thread to publish
// the next decoded frame. Must stay below the 33 ms inter-frame interval.
static constexpr int      kFrameWaitMs       = 20;
// Hard cap of the decoded-frame FIFO (memory bound).
static constexpr size_t   kMaxQueuedFrames   = 16;
// Steady-state backlog allowed in the FIFO. The queue absorbs bursts, but a
// backlog that never drains is pure latency (each queued frame = ~33 ms), so
// anything above this is discarded as stale at delivery time.
// Tunable at runtime with ODIN_NVDEC_MAX_LATENCY_FRAMES.
static constexpr size_t   kDefaultMaxLatencyFrames = 2;

// ---------------------------------------------------------------------------
// ContainsH264Keyframe — detects Annex-B keyframe boundaries.
// Accepts IDR slice (NAL type 5) and SPS (NAL type 7), matching the SDK's
// DetectH264Keyframe and preventing the decoder from discarding the initial
// SPS/PPS parameter-set packet that the device may emit as a standalone frame.
// ---------------------------------------------------------------------------
static bool ContainsH264Keyframe(const uint8_t* data, size_t size) {
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
    // IDR slice (5) or SPS (7) indicates a keyframe / parameter-set boundary.
    if (nal_unit_type == 5 || nal_unit_type == 7) return true;
    i = nal_start + 1;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
JetsonNvdecDecoder::JetsonNvdecDecoder(const std::string& camera_label) {
  camera_label_ = camera_label;
  // 1. Create the NvVideoDecoder instance in blocking mode (flags = 0),
  //    exactly like the L4T 00_video_decode reference.
  //    Blocking mode is only safe because the capture plane is drained by
  //    a dedicated thread: the caller thread never waits for a decoded
  //    frame, and the capture thread's blocking VIDIOC_DQBUF is released
  //    by NvV4l2Element::abort() (STREAMOFF on both planes) at shutdown.
  //    The previous non-blocking design polled dqEvent()/dqBuffer() from
  //    the caller thread and called SetPollInterrupt() before every
  //    dqEvent — SetPollInterrupt is the API's *abort* mechanism for a
  //    blocking DevicePoll, not an event-polling prerequisite; using it
  //    that way put the driver into an inconsistent state ("NVMMLITE
  //    NvVideo: This condition is not possible" + "Error while DQing
  //    event: Invalid argument") and the resolution-change event was
  //    never delivered, so no frame was ever produced.
  decoder_ = NvVideoDecoder::createVideoDecoder("h264dec", 0);
  if (!decoder_) {
    RC_LOG_ERROR("%s [init] createVideoDecoder failed", LogTag(camera_label_).c_str());
    return;
  }

  // 2. Subscribe to V4L2_EVENT_RESOLUTION_CHANGE before streaming starts.
  //    The decoder emits this event after parsing SPS/PPS from the stream.
  if (decoder_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0) < 0) {
    RC_LOG_WARN("%s [init] subscribeEvent failed (continuing)", LogTag(camera_label_).c_str());
  }

  // 3. Configure the output plane (input side) for H.264 Annex-B.
  if (decoder_->setOutputPlaneFormat(V4L2_PIX_FMT_H264,
                                     kInputBufferSize) < 0) {
    RC_LOG_ERROR("%s [init] setOutputPlaneFormat failed", LogTag(camera_label_).c_str());
    delete decoder_;
    decoder_ = nullptr;
    return;
  }

  // 4. Complete-frame input mode.
  //    setFrameInputMode() writes V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT:
  //    1 = input buffers may contain partial frames (chunk / NALU feeding),
  //    0 = every input buffer holds one complete access unit.
  //    Each CompressedImage payload from the device is a self-contained
  //    Annex-B access unit, so 0 is correct and avoids the extra parser
  //    latency (and the one-frame hold-back) of chunk mode.
  if (decoder_->setFrameInputMode(0) < 0) {
    RC_LOG_WARN("%s [init] setFrameInputMode failed (continuing)", LogTag(camera_label_).c_str());
  }

  // 5. Disable the Decoded Picture Buffer (DPB).
  //    With the DPB enabled the decoder holds frames back until its
  //    reference buffer is full and then releases them in bursts (measured:
  //    ~400 ms of silence followed by ~12 frames at once), which shows up
  //    downstream as a large, permanent latency.  The ODIN stream contains
  //    no B-frames, so no reordering is needed and the DPB only adds delay.
  //    Escape hatch: ODIN_NVDEC_KEEP_DPB=1 restores the default behaviour.
  const char* keep_dpb = std::getenv("ODIN_NVDEC_KEEP_DPB");
  if (!(keep_dpb && keep_dpb[0] == '1')) {
    if (decoder_->disableDPB() < 0) {
      RC_LOG_WARN("%s [init] disableDPB failed (continuing)", LogTag(camera_label_).c_str());
    }
  }

  // 6. Set maximum performance mode (clocks pinned high).
  if (decoder_->setMaxPerfMode(1) < 0) {
    RC_LOG_WARN("%s [init] setMaxPerfMode failed (continuing)", LogTag(camera_label_).c_str());
  }

  // 7. Allocate and mmap output-plane (input) buffers.
  //    map=true so planes[i].data is directly accessible from CPU.
  //    allocate=false lets the V4L2 driver manage the buffer memory.
  if (decoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP,
                                        kNumOutputBuffers, true, false) < 0) {
    RC_LOG_ERROR("%s [init] output_plane setupPlane failed", LogTag(camera_label_).c_str());
    delete decoder_;
    decoder_ = nullptr;
    return;
  }

  // 8. Start streaming on the output plane.
  if (decoder_->output_plane.setStreamStatus(true) < 0) {
    RC_LOG_ERROR("%s [init] output_plane setStreamStatus failed", LogTag(camera_label_).c_str());
    delete decoder_;
    decoder_ = nullptr;
    return;
  }

  // 9. Start the capture thread: it waits for V4L2_EVENT_RESOLUTION_CHANGE,
  //    configures the capture plane and then drains decoded frames.
  // Pipeline diagnostics are opt-in: ODIN_NVDEC_STATS=1 re-enables the
  // periodic stats line and the frame-gap warnings.
  const char* stats_env = std::getenv("ODIN_NVDEC_STATS");
  stats_enabled_        = (stats_env && stats_env[0] == '1');
  max_latency_frames_     = kDefaultMaxLatencyFrames;
  const char* latency_env = std::getenv("ODIN_NVDEC_MAX_LATENCY_FRAMES");
  if (latency_env) {
    const int v = std::atoi(latency_env);
    if (v >= 1 && static_cast<size_t>(v) <= kMaxQueuedFrames) {
      max_latency_frames_ = static_cast<size_t>(v);
    }
  }
  last_stat_time_       = std::chrono::steady_clock::now();
  last_delivery_time_   = last_stat_time_;

  initialized_ = true;
  capture_thread_ = std::thread(&JetsonNvdecDecoder::CaptureLoop, this);

  RC_LOG_DEBUG("%s [init] NVDEC hardware decoder initialized", LogTag(camera_label_).c_str());
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
JetsonNvdecDecoder::~JetsonNvdecDecoder() {
  stop_.store(true);
  frame_cv_.notify_all();

  // abort() calls VIDIOC_STREAMOFF on both planes, which releases the
  // capture thread from its blocking dqBuffer()/dqEvent() call.
  if (decoder_) {
    decoder_->abort();
  }
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  DestroyTransformSurface();

  if (decoder_) {
    delete decoder_;
    decoder_ = nullptr;
  }
}

// ---------------------------------------------------------------------------
// SetupCapturePlane — called after V4L2_EVENT_RESOLUTION_CHANGE
// ---------------------------------------------------------------------------
bool JetsonNvdecDecoder::SetupCapturePlane() {
  // Deinit old capture plane if reconfiguring for a new resolution.
  if (capture_plane_set_up_) {
    decoder_->capture_plane.setStreamStatus(false);
    decoder_->capture_plane.deinitPlane();
    capture_plane_set_up_ = false;
  }

  // Retrieve the capture format negotiated by the decoder from SPS/PPS.
  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  if (decoder_->capture_plane.getFormat(format) < 0) {
    RC_LOG_ERROR("%s [decode] capture_plane getFormat failed", LogTag(camera_label_).c_str());
    return false;
  }

  // Get the display resolution, which may differ from the buffer format.
  struct v4l2_crop crop;
  memset(&crop, 0, sizeof(crop));
  if (decoder_->capture_plane.getCrop(crop) < 0) {
    RC_LOG_WARN("%s [decode] capture_plane getCrop failed, "
                "falling back to buffer format", LogTag(camera_label_).c_str());
    crop.c.width  = format.fmt.pix_mp.width;
    crop.c.height = format.fmt.pix_mp.height;
  }

  // Explicitly set the capture plane format. This is required by the
  // Jetson Multimedia API before requesting capture buffers.
  if (decoder_->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                       format.fmt.pix_mp.width,
                                       format.fmt.pix_mp.height) < 0) {
    RC_LOG_ERROR("%s [decode] setCapturePlaneFormat failed", LogTag(camera_label_).c_str());
    return false;
  }

  // Ask the driver how many capture buffers it actually needs.
  uint32_t capture_buffer_count = kNumCaptureBuffers;
  int min_dec_capture_buffers = 0;
  if (decoder_->getMinimumCapturePlaneBuffers(min_dec_capture_buffers) == 0 &&
      min_dec_capture_buffers > 0) {
    capture_buffer_count =
        static_cast<uint32_t>(min_dec_capture_buffers) + kNumCaptureBuffers;
  }

  // Allocate new capture-plane buffers.
  // map=false: on L4T R35/R36 the decoder capture buffers are
  // NvBufSurface-backed DMA buffers; v4l2 mmap of the capture plane is not
  // supported.  CPU access goes through NvBufSurfaceFromFd(planes[0].fd)
  // in SurfaceToBgr(), matching the 00_video_decode reference.
  if (decoder_->capture_plane.setupPlane(V4L2_MEMORY_MMAP,
                                         capture_buffer_count, false, false) < 0) {
    RC_LOG_ERROR("%s [decode] capture_plane setupPlane failed", LogTag(camera_label_).c_str());
    return false;
  }

  // Start streaming on the capture plane.
  if (decoder_->capture_plane.setStreamStatus(true) < 0) {
    RC_LOG_ERROR("%s [decode] capture_plane setStreamStatus failed", LogTag(camera_label_).c_str());
    return false;
  }

  // Queue all capture buffers — the decoder fills them with decoded frames.
  for (uint32_t i = 0; i < decoder_->capture_plane.getNumBuffers(); i++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane   planes[MAX_PLANES];
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, sizeof(planes));
    v4l2_buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf.memory  = V4L2_MEMORY_MMAP;
    v4l2_buf.index   = i;
    v4l2_buf.m.planes = planes;
    v4l2_buf.length   = decoder_->capture_plane.getNumPlanes();
    if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0) {
      RC_LOG_WARN("%s [decode] capture_plane qBuffer[%u] failed", LogTag(camera_label_).c_str(), i);
    }
  }

  // Resolution may have changed — drop the old VIC destination surface so
  // EnsureTransformSurface() reallocates it at the new size.
  DestroyTransformSurface();
  display_width_  = static_cast<int>(crop.c.width);
  display_height_ = static_cast<int>(crop.c.height);
  capture_plane_set_up_.store(true);

  RC_LOG_INFO("%s [decode] Capture plane ready: %dx%d (buffer %dx%d)",
              LogTag(camera_label_).c_str(),
              display_width_, display_height_,
              format.fmt.pix_mp.width, format.fmt.pix_mp.height);
  return true;
}

// ---------------------------------------------------------------------------
// EnsureTransformSurface — pitch-linear RGBA destination for the VIC
// ---------------------------------------------------------------------------
bool JetsonNvdecDecoder::EnsureTransformSurface() {
  if (transform_surface_) return true;
  if (display_width_ <= 0 || display_height_ <= 0) return false;

  NvBufSurfaceCreateParams create_params;
  memset(&create_params, 0, sizeof(create_params));
  create_params.gpuId       = 0;
  create_params.width       = static_cast<uint32_t>(display_width_);
  create_params.height      = static_cast<uint32_t>(display_height_);
  create_params.size        = 0;
  create_params.layout      = NVBUF_LAYOUT_PITCH;
  create_params.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  create_params.memType     = NVBUF_MEM_SURFACE_ARRAY;

  if (NvBufSurfaceCreate(&transform_surface_, 1, &create_params) != 0 ||
      !transform_surface_) {
    transform_surface_ = nullptr;
    RC_LOG_ERROR("%s [decode] NvBufSurfaceCreate failed (%dx%d)",
                 LogTag(camera_label_).c_str(), display_width_, display_height_);
    return false;
  }
  transform_surface_->numFilled = 1;
  return true;
}

void JetsonNvdecDecoder::DestroyTransformSurface() {
  if (transform_surface_) {
    NvBufSurfaceDestroy(transform_surface_);
    transform_surface_ = nullptr;
  }
}

// ---------------------------------------------------------------------------
// SurfaceToBgr — decoded NV12 surface → BGR24
//
// NVDEC writes its output as *block-linear* NV12 (a hardware-tiled layout).
// Reading that memory linearly produces horizontal colour banding, which is
// exactly what the previous direct-mmap/NvBufSurfaceMap read produced.
// The supported CPU access path (as in the 00_video_decode reference) is to
// run the surface through the VIC with NvBufSurfTransform into a
// pitch-linear destination first; converting straight to RGBA also offloads
// the YUV→RGB conversion to the VIC.
// ---------------------------------------------------------------------------
bool JetsonNvdecDecoder::SurfaceToBgr(NvBuffer* buffer, cv::Mat& out) {
  if (!buffer || buffer->planes[0].fd < 0) return false;

  NvBufSurface* src = nullptr;
  if (NvBufSurfaceFromFd(buffer->planes[0].fd,
                         reinterpret_cast<void**>(&src)) != 0 || !src) {
    return false;
  }
  src->numFilled = 1;

  if (!EnsureTransformSurface()) return false;

  // NvBufSurfTransform operates on a process-global default session, so the
  // two per-camera decoder instances must not drive the VIC concurrently.
  static std::mutex vic_mutex;
  std::lock_guard<std::mutex> vic_lock(vic_mutex);

  NvBufSurfTransformRect src_rect;
  NvBufSurfTransformRect dst_rect;
  memset(&src_rect, 0, sizeof(src_rect));
  memset(&dst_rect, 0, sizeof(dst_rect));
  src_rect.width  = static_cast<uint32_t>(display_width_);
  src_rect.height = static_cast<uint32_t>(display_height_);
  dst_rect = src_rect;

  NvBufSurfTransformParams transform_params;
  memset(&transform_params, 0, sizeof(transform_params));
  transform_params.transform_flag =
      NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_CROP_SRC |
      NVBUFSURF_TRANSFORM_CROP_DST;
  transform_params.transform_filter = NvBufSurfTransformInter_Nearest;
  transform_params.src_rect = &src_rect;
  transform_params.dst_rect = &dst_rect;

  if (NvBufSurfTransform(src, transform_surface_, &transform_params) !=
      NvBufSurfTransformError_Success) {
    return false;
  }

  if (NvBufSurfaceMap(transform_surface_, 0, 0, NVBUF_MAP_READ) != 0) {
    return false;
  }
  NvBufSurfaceSyncForCpu(transform_surface_, 0, 0);

  NvBufSurfaceParams& params = transform_surface_->surfaceList[0];
  bool success = false;
  if (params.mappedAddr.addr[0]) {
    // Wrap the mapped RGBA surface (no copy) and convert to BGR; cvtColor
    // writes into `out`, so the data is owned by the caller afterwards.
    cv::Mat rgba(display_height_, display_width_, CV_8UC4,
                 params.mappedAddr.addr[0],
                 static_cast<size_t>(params.planeParams.pitch[0]));
    cv::cvtColor(rgba, out, cv::COLOR_RGBA2BGR);
    success = !out.empty();
  }

  NvBufSurfaceUnMap(transform_surface_, 0, 0);
  return success;
}

// ---------------------------------------------------------------------------
// CaptureLoop — dedicated capture-plane thread
// ---------------------------------------------------------------------------
void JetsonNvdecDecoder::CaptureLoop() {
  // --- Phase 1: wait for the decoder to parse SPS/PPS ----------------------
  // The decoder signals the detected stream resolution with
  // V4L2_EVENT_RESOLUTION_CHANGE; the capture plane cannot be configured
  // before it arrives.
  bool got_resolution = false;
  while (!stop_.load() && !decoder_->isInError()) {
    struct v4l2_event event;
    memset(&event, 0, sizeof(event));
    if (decoder_->dqEvent(event, 1000) == 0) {
      if (event.type == V4L2_EVENT_RESOLUTION_CHANGE) {
        got_resolution = true;
        break;
      }
      continue;
    }
    // No event yet (EAGAIN after the 1 s wait) — retry until the first
    // access units have been fed by Decode().
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!got_resolution) return;

  RC_LOG_INFO("%s [decode] Got V4L2_EVENT_RESOLUTION_CHANGE", LogTag(camera_label_).c_str());
  if (!SetupCapturePlane()) {
    capture_failed_.store(true);
    return;
  }

  // --- Phase 2: drain decoded frames ---------------------------------------
  bool convert_error_logged = false;
  while (!stop_.load() && !decoder_->isInError()) {
    struct v4l2_buffer cap_buf;
    struct v4l2_plane  cap_planes[MAX_PLANES];
    NvBuffer*          out_buf = nullptr;

    memset(&cap_buf, 0, sizeof(cap_buf));
    memset(cap_planes, 0, sizeof(cap_planes));
    cap_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    cap_buf.memory   = V4L2_MEMORY_MMAP;
    cap_buf.m.planes = cap_planes;

    // Blocking dequeue; released by abort() (STREAMOFF) from the destructor.
    if (decoder_->capture_plane.dqBuffer(cap_buf, &out_buf, nullptr, 0) < 0) {
      if (errno == EAGAIN && !stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      break;
    }

    const auto convert_start = std::chrono::steady_clock::now();
    cv::Mat bgr;
    const bool converted = SurfaceToBgr(out_buf, bgr);
    const auto convert_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - convert_start).count();

    if (converted) {
      if (static_cast<uint64_t>(convert_us) > stat_convert_max_us_.load()) {
        stat_convert_max_us_.store(static_cast<uint64_t>(convert_us));
      }
      stat_decoded_.fetch_add(1);

      // Interval between two decoded frames: a large value here means the
      // decoder itself emits in bursts (input burst or DPB hold-back).
      const auto now = std::chrono::steady_clock::now();
      if (last_decoded_time_.time_since_epoch().count() != 0) {
        const auto gap_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now - last_decoded_time_).count();
        if (static_cast<uint64_t>(gap_us) > stat_decode_gap_max_us_.load()) {
          stat_decode_gap_max_us_.store(static_cast<uint64_t>(gap_us));
        }
      }
      last_decoded_time_ = now;

      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_queue_.push_back(std::move(bgr));
        if (frame_queue_.size() > kMaxQueuedFrames) {
          frame_queue_.pop_front();
          stat_overwritten_.fetch_add(1);
        }
        if (frame_queue_.size() > stat_queue_max_.load()) {
          stat_queue_max_.store(frame_queue_.size());
        }
      }
      frame_cv_.notify_one();
    } else if (!convert_error_logged) {
      convert_error_logged = true;
      RC_LOG_WARN("%s [decode] NV12 surface conversion failed "
                  "(fd=%d) — no image will be published",
                  LogTag(camera_label_).c_str(),
                  out_buf ? out_buf->planes[0].fd : -1);
    }

    if (decoder_->capture_plane.qBuffer(cap_buf, nullptr) < 0) break;
  }
}

// ---------------------------------------------------------------------------
// FeedInput — push one access unit into the output (input) plane
// ---------------------------------------------------------------------------
void JetsonNvdecDecoder::FeedInput(const uint8_t* data, size_t size) {
  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane  planes[MAX_PLANES];
  NvBuffer*          input_buf = nullptr;

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, sizeof(planes));
  v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  v4l2_buf.memory   = V4L2_MEMORY_MMAP;
  v4l2_buf.m.planes = planes;
  v4l2_buf.length   = 1;  // H.264 compressed data is single-plane

  const uint32_t queued = decoder_->output_plane.getNumQueuedBuffers();
  const uint32_t total  = decoder_->output_plane.getNumBuffers();

  if (queued < total) {
    // Initial fill: no buffer has been consumed yet, so grab an unqueued
    // one directly instead of dequeuing.
    input_buf = decoder_->output_plane.getNthBuffer(queued);
    v4l2_buf.index = queued;
  } else {
    // Blocking dequeue: this is where the caller thread waits when all
    // output-plane buffers are still owned by the hardware.
    const auto wait_start = std::chrono::steady_clock::now();
    const int  ret = decoder_->output_plane.dqBuffer(v4l2_buf, &input_buf,
                                                     nullptr, 10);
    const auto wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - wait_start).count();
    if (static_cast<uint64_t>(wait_us) > stat_feed_wait_max_us_.load()) {
      stat_feed_wait_max_us_.store(static_cast<uint64_t>(wait_us));
    }
    if (ret < 0) {
      RC_LOG_WARN("%s [decode] output_plane dqBuffer failed, "
                  "dropping access unit", LogTag(camera_label_).c_str());
      return;
    }
  }
  if (!input_buf) return;

  const size_t copy_size =
      std::min(size, static_cast<size_t>(kInputBufferSize));
  memcpy(input_buf->planes[0].data, data, copy_size);
  input_buf->planes[0].bytesused = static_cast<uint32_t>(copy_size);
  v4l2_buf.m.planes[0].bytesused = static_cast<uint32_t>(copy_size);

  if (decoder_->output_plane.qBuffer(v4l2_buf, nullptr) < 0) {
    RC_LOG_WARN("%s [decode] output_plane qBuffer failed", LogTag(camera_label_).c_str());
    return;
  }
  stat_fed_.fetch_add(1);
}

// ---------------------------------------------------------------------------
// ReportStats — periodic pipeline diagnostics
//
// Interpretation of a periodic freeze in the published image:
//   * starved ≈ delivered   → the capture thread cannot keep up, or the
//                             decoder holds frames back (DPB / reordering).
//   * overwritten > 0       → frames are decoded but never consumed: the
//                             caller (imageCallback) is running slower than
//                             the decoder, i.e. the drop is downstream.
//   * fed rate << 30/s      → the gap is upstream (network / device), not
//                             in the decoder at all.
//   * feed_wait_max large   → the caller thread is blocked in the
//                             output-plane dqBuffer (input starvation).
// ---------------------------------------------------------------------------
void JetsonNvdecDecoder::ReportStats() {
  if (!stats_enabled_) return;  // opt-in via ODIN_NVDEC_STATS=1

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_stat_time_).count();
  if (elapsed_ms < 5000) return;
  last_stat_time_ = now;

  const double secs = static_cast<double>(elapsed_ms) / 1000.0;
  const uint64_t fed         = stat_fed_.exchange(0);
  const uint64_t decoded     = stat_decoded_.exchange(0);
  const uint64_t delivered   = stat_delivered_.exchange(0);
  const uint64_t overwritten = stat_overwritten_.exchange(0);
  const uint64_t starved     = stat_starved_.exchange(0);
  const uint64_t feed_wait   = stat_feed_wait_max_us_.exchange(0);
  const uint64_t convert_max = stat_convert_max_us_.exchange(0);
  const uint64_t input_gap   = stat_input_gap_max_us_.exchange(0);
  const uint64_t decode_gap  = stat_decode_gap_max_us_.exchange(0);
  const uint64_t queue_max   = stat_queue_max_.exchange(0);

  RC_LOG_INFO("%s [stats] %.0fs: fed=%lu (%.1f/s) "
              "decoded=%lu (%.1f/s) delivered=%lu (%.1f/s) "
              "starved=%lu dropped=%lu queue_max=%lu "
              "input_gap_max=%.0fms decode_gap_max=%.0fms "
              "feed_wait_max=%.1fms convert_max=%.1fms",
              LogTag(camera_label_).c_str(), secs,
              static_cast<unsigned long>(fed),       fed / secs,
              static_cast<unsigned long>(decoded),   decoded / secs,
              static_cast<unsigned long>(delivered), delivered / secs,
              static_cast<unsigned long>(starved),
              static_cast<unsigned long>(overwritten),
              static_cast<unsigned long>(queue_max),
              input_gap / 1000.0, decode_gap / 1000.0,
              feed_wait / 1000.0, convert_max / 1000.0);
}

// ---------------------------------------------------------------------------
// Decode — feed one access unit, return the newest decoded frame if any
// ---------------------------------------------------------------------------
bool JetsonNvdecDecoder::Decode(const uint8_t* data, size_t size,
                                 cv::Mat& output) {
  if (!initialized_ || !decoder_) return false;
  if (capture_failed_.load() || decoder_->isInError()) return false;

  // --- waiting_for_keyframe_ gate ------------------------------------------
  // Same rationale as FfmpegH264Decoder: only feed NVDEC after a keyframe
  // boundary (IDR or SPS) to avoid garbled error-concealed frames when
  // reference frames are missing (stream start, after reset, or after decode
  // error).  The device may emit the initial SPS/PPS as a standalone packet,
  // so SPS must be accepted as a keyframe boundary.
  if (waiting_for_keyframe_) {
    if (!ContainsH264Keyframe(data, size)) {
      return false;  // discard non-keyframe data
    }
    waiting_for_keyframe_ = false;
  }

  // Interval between two Decode() calls: a large value here means the
  // compressed frames arrive in bursts, i.e. the stall is upstream of the
  // decoder (device, network or ROS callback scheduling).
  const auto call_time = std::chrono::steady_clock::now();
  if (last_input_time_.time_since_epoch().count() != 0) {
    const auto gap_us = std::chrono::duration_cast<std::chrono::microseconds>(
        call_time - last_input_time_).count();
    if (static_cast<uint64_t>(gap_us) > stat_input_gap_max_us_.load()) {
      stat_input_gap_max_us_.store(static_cast<uint64_t>(gap_us));
    }
  }
  last_input_time_ = call_time;

  FeedInput(data, size);
  ReportStats();

  // The capture thread decodes asynchronously.  Waiting briefly here keeps
  // the 1 input AU : 1 output frame ratio when the frame is only a few
  // milliseconds late; longer bursts are absorbed by the FIFO instead.
  std::unique_lock<std::mutex> lock(frame_mutex_);
  if (frame_queue_.empty() && capture_plane_set_up_.load()) {
    frame_cv_.wait_for(lock, std::chrono::milliseconds(kFrameWaitMs),
                       [this] { return !frame_queue_.empty() || stop_.load(); });
  }
  if (frame_queue_.empty()) {
    stat_starved_.fetch_add(1);
    return false;
  }

  // Drain stale backlog. Production and consumption both run at the stream
  // frame rate, so a backlog built up by one burst would otherwise persist
  // forever and show up as a constant delay against the other topics.
  while (frame_queue_.size() > max_latency_frames_) {
    frame_queue_.pop_front();
    stat_overwritten_.fetch_add(1);
  }

  output = std::move(frame_queue_.front());
  frame_queue_.pop_front();
  stat_delivered_.fetch_add(1);

  // Freeze detection: log the exact moment a visible gap occurs so the
  // periodic stall can be correlated with the counters above.
  const auto now = std::chrono::steady_clock::now();
  const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_delivery_time_).count();
  last_delivery_time_ = now;
  if (stats_enabled_ && gap_ms > 200) {
    RC_LOG_WARN("%s [decode] frame delivery gap %ld ms "
                "(consecutive starved decodes since last frame)",
                LogTag(camera_label_).c_str(), static_cast<long>(gap_ms));
  }
  return true;
}

}  // namespace odin_decoder

#endif  // USE_JETSON_NVDEC_DECODER
