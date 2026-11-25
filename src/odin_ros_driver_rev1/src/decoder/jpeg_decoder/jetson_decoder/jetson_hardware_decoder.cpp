#include "jetson_hardware_decoder.h"

#ifdef USE_JETSON_DECODER

#include <cstdlib>
#include <cstring>
#include <vector>
#include <memory>

#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <unistd.h>

#include "utility/ros_compat.hpp"

namespace odin_decoder {

JetsonHardwareDecoder::JetsonHardwareDecoder() {
  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
  if (!decoder_) {
    RC_LOG_ERROR("[JetsonDecoder] Failed to create NvJPEGDecoder");
    initialized_ = false;
    return;
  }
  initialized_ = true;
  RC_LOG_INFO("[JetsonDecoder] Hardware decoder initialized (JetPack R36+ verified at compile time)");
}

JetsonHardwareDecoder::~JetsonHardwareDecoder() {
  if (decoder_) {
    delete decoder_;
    decoder_ = nullptr;
  }
}

bool JetsonHardwareDecoder::Decode(const uint8_t* data, size_t size, cv::Mat& output) {
  if (!initialized_ || !decoder_ || !data || size < 100) return false;

  // Step 1: Allocate aligned memory (64-byte aligned, GPU friendly)
  void* aligned_data = nullptr;
  if (posix_memalign(&aligned_data, 64, size) != 0) {
    return false;
  }
  std::unique_ptr<void, decltype(&free)> aligned_guard(aligned_data, &free);

  // Step 2: Copy data
  memcpy(aligned_data, data, size);

  // Step 3: Call decodeToFd (data is now aligned)
  int fd = -1;
  uint32_t width = 0, height = 0;
  uint32_t pixfmt = 0;

  int decode_ret = decoder_->decodeToFd(fd, static_cast<unsigned char*>(aligned_guard.get()),
                                        static_cast<unsigned long>(size), pixfmt, width, height);

  if (decode_ret < 0 || fd < 0 || width == 0 || height == 0) {
    if (fd >= 0) close(fd);
    RC_LOG_ERROR("[JetsonDecoder] decodeToFd failed (ret=%d, fd=%d, w=%u, h=%u, pixfmt=%u)",
            decode_ret, fd, width, height, pixfmt);
    return false;
  }

  NvBufSurface* surf = nullptr;
  if (NvBufSurfaceFromFd(fd, reinterpret_cast<void**>(&surf)) != 0 || !surf) {
    close(fd);
    RC_LOG_ERROR("[JetsonDecoder] NvBufSurfaceFromFd failed");
    return false;
  }

  if (surf->numFilled == 0 || !surf->surfaceList) {
    close(fd);
    RC_LOG_ERROR("[JetsonDecoder] Invalid surface (numFilled=%d, surfaceList=%p)",
            surf->numFilled, (void*)surf->surfaceList);
    return false;
  }

  NvBufSurfaceParams& params = surf->surfaceList[0];

  // Validate surface parameters
  if (params.width == 0 || params.height == 0 || params.planeParams.num_planes == 0) {
    close(fd);
    RC_LOG_ERROR("[JetsonDecoder] Invalid params (w=%u, h=%u, planes=%u)",
            params.width, params.height, params.planeParams.num_planes);
    return false;
  }

  // YUV420/NV12 require even width/height
  if ((params.planeParams.num_planes >= 2) &&
      ((params.width & 1u) || (params.height & 1u))) {
    close(fd);
    RC_LOG_WARN("[JetsonDecoder] Reject odd dimension frame (w=%u, h=%u)",
            params.width, params.height);
    return false;
  }

  // Basic pitch check
  for (uint32_t i = 0; i < params.planeParams.num_planes; ++i) {
    if (params.planeParams.pitch[i] == 0) {
      close(fd);
      RC_LOG_ERROR("[JetsonDecoder] Invalid pitch at plane %u", i);
      return false;
    }
  }

  // Map all planes for YUV420 planar format
  uint32_t mapped_planes = 0;
  for (uint32_t i = 0; i < params.planeParams.num_planes; i++) {
    if (NvBufSurfaceMap(surf, 0, i, NVBUF_MAP_READ) != 0) {
      for (uint32_t j = 0; j < mapped_planes; j++) {
        NvBufSurfaceUnMap(surf, 0, j);
      }
      close(fd);
      return false;
    }
    mapped_planes++;
    // Sync each plane individually to ensure cache coherency
    NvBufSurfaceSyncForCpu(surf, 0, i);
  }

  // Validate mapped addresses
  for (uint32_t i = 0; i < params.planeParams.num_planes; i++) {
    if (!params.mappedAddr.addr[i]) {
      for (uint32_t j = 0; j < mapped_planes; j++) {
        NvBufSurfaceUnMap(surf, 0, j);
      }
      close(fd);
      return false;
    }
  }

  int h = static_cast<int>(params.height);
  int w = static_cast<int>(params.width);

  bool success = false;

  // Handle I420/YUV420 planar format (Y, U, V separate planes)
  // colorFormat=2 = NVBUF_COLOR_FORMAT_YUV420, pixfmt=842091865 = V4L2_PIX_FMT_YUV420
  if (params.planeParams.num_planes >= 3) {
    cv::Mat y_plane(h, w, CV_8UC1, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
    cv::Mat u_plane(h/2, w/2, CV_8UC1, params.mappedAddr.addr[1], params.planeParams.pitch[1]);
    cv::Mat v_plane(h/2, w/2, CV_8UC1, params.mappedAddr.addr[2], params.planeParams.pitch[2]);

    // I420 layout: Y (w*h) + U (w/2 * h/2) + V (w/2 * h/2), row by row, contiguous.
    std::vector<uint8_t> yuv_data(w * h * 3 / 2);

    cv::Mat y_dst(h, w, CV_8UC1, yuv_data.data());
    y_plane.copyTo(y_dst);

    cv::Mat u_dst(h/2, w/2, CV_8UC1, yuv_data.data() + w * h);
    u_plane.copyTo(u_dst);

    cv::Mat v_dst(h/2, w/2, CV_8UC1, yuv_data.data() + w * h + w * h / 4);
    v_plane.copyTo(v_dst);

    cv::Mat yuv_i420(h * 3 / 2, w, CV_8UC1, yuv_data.data());
    cv::cvtColor(yuv_i420, output, cv::COLOR_YUV2BGR_I420);
    success = !output.empty();
  }
  // Handle NV21 format (Y plane + interleaved VU plane - Jetson outputs NV21)
  else if (params.planeParams.num_planes == 2) {
    cv::Mat nv21(h * 3 / 2, w, CV_8UC1);
    cv::Mat y_plane(h, w, CV_8UC1, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
    cv::Mat vu_plane(h/2, w, CV_8UC1, params.mappedAddr.addr[1], params.planeParams.pitch[1]);
    y_plane.copyTo(nv21(cv::Rect(0, 0, w, h)));
    vu_plane.copyTo(nv21(cv::Rect(0, h, w, h/2)));
    cv::cvtColor(nv21, output, cv::COLOR_YUV2BGR_NV21);
    success = !output.empty();
  }
  else {
    cv::Mat raw(h, w, CV_8UC3, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
    raw.copyTo(output);
    success = !output.empty();
  }

  for (uint32_t i = 0; i < mapped_planes; i++) {
    NvBufSurfaceUnMap(surf, 0, i);
  }
  // Note: Do NOT call NvBufSurfaceDestroy(surf)!
  // surf is mapped from fd, close(fd) will automatically release the underlying buffer.
  // Calling Destroy corrupts NvJPEGDecoder internal state, causing subsequent decode failures.
  close(fd);

  return success;
}

std::string JetsonHardwareDecoder::GetName() const {
  return initialized_ ? "NvJpegDecoder Hardware" : "NvJpegDecoder (init failed)";
}

}  // namespace odin_decoder

#endif  // USE_JETSON_DECODER
