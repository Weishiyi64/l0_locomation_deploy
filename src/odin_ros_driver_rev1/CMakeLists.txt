cmake_minimum_required(VERSION 3.10)
project(odin_ros_driver_rev1)
add_definitions(-DODIN_ROS_PACKAGE_NAME=\"${PROJECT_NAME}\")

# Default to Release build with -O3 optimization (preserves precision)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Print build configuration
message(STATUS "========================================")
message(STATUS "Build Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  CXX Flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "  JPEG Decoder: OpenCV cv::imdecode (libjpeg-turbo accelerated)")
message(STATUS "========================================")

# Enable comprehensive warnings
add_compile_options(
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-unused-function
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# ROS Version Detection
# =============================================================================
set(ODIN_ROS_VERSION 1)
if(DEFINED ENV{ROS_VERSION})
  set(ODIN_ROS_VERSION $ENV{ROS_VERSION})
endif()
message(STATUS "Building for ROS${ODIN_ROS_VERSION}")

# =============================================================================
# ODIN SDK Location (直接指定目录，可添加多个)
# =============================================================================
set(ODIN_SDK_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/module/sdk_api")
set(ODIN_SDK_INCLUDE_DIRS
  ${ODIN_SDK_MODULE_DIR}/include
  ${ODIN_SDK_MODULE_DIR}/include/logger
  ${ODIN_SDK_MODULE_DIR}/sdk
  # 添加更多头文件目录...
)

# Per-arch shared library directory (matches `uname -m`: x86_64, aarch64, ...)
if(NOT DEFINED ODIN_TARGET_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR)
    set(ODIN_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
  else()
    execute_process(COMMAND uname -m
      OUTPUT_VARIABLE ODIN_TARGET_ARCH
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()
endif()
message(STATUS "odin_ros_driver: target arch = ${ODIN_TARGET_ARCH}")

set(ODIN_SDK_LIBRARY_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/module/sdk_api/lib/${ODIN_TARGET_ARCH}
  # 添加更多库目录...
)

# Link the shared library by name; CMake / linker will resolve via
# link_directories() above to the per-arch folder.
set(ODIN_SDK_LIBRARIES
  odin_sdk
  # 添加更多库文件...
)

# =============================================================================
# Common Dependencies (shared by ROS1 and ROS2)
# =============================================================================
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
# Vendored Eigen3 header-only library (shared with SDK, module/sdk_api/3rdparty/eigen3/).
# Eliminates the libeigen3-dev build dependency — all 15 source files using Eigen
# (stereo_rectifier, pointcloud_to_depth, cloud_render, camera_model, etc.) get
# their include path from this vendored copy. Eigen is template-only, so no ABI
# or shared-library concern with PCL/OpenCV which bring their own Eigen.
# NOTE: We use the literal path directly in include_directories() instead of
# storing it in EIGEN3_INCLUDE_DIR, because find_package(PCL) internally calls
# find_package(Eigen3) which would overwrite the variable with the system path.
set(ODIN_VENDORED_EIGEN3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/module/sdk_api/3rdparty/eigen3")
find_package(PCL REQUIRED)

# =============================================================================
# ROS-Specific Setup
# =============================================================================
if(ODIN_ROS_VERSION EQUAL 2)
  # --- ROS2 packages ---
  add_definitions(-DODIN_ROS2=1)
  find_package(ament_cmake REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(point_cloud_transport REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  ament_package_xml()

  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetCalibration.srv"
    "srv/SetSensorMode.srv"
    "srv/ResetInitialPose.srv"
    "srv/GetAdaptiveParam.srv"
    "srv/SetAdaptiveParam.srv"
    "srv/GetSyncMode.srv"
    "srv/SetSyncMode.srv"
    "srv/GetPtpSyncStatus.srv"
    "srv/RestoreDefaultConfig.srv"
    "srv/SetDiagMonitor.srv"
    "srv/SetChannelConfig.srv"
    "srv/GetChannelConfig.srv"
    "srv/SetPostProcessConfig.srv"
    "srv/GetPostProcessConfig.srv"
    "srv/SaveMap.srv"
    "srv/LoadMap.srv"
    "srv/ExployMap.srv"
    "srv/DeviceMode.srv"
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  # --- ROS1 packages ---
  find_package(catkin REQUIRED COMPONENTS
    roscpp roslib sensor_msgs nav_msgs std_msgs geometry_msgs visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs diagnostic_msgs point_cloud_transport
    cv_bridge ddynamic_reconfigure message_generation
  )
  add_service_files(FILES
    GetCalibration.srv SetSensorMode.srv ResetInitialPose.srv
    GetAdaptiveParam.srv SetAdaptiveParam.srv
    GetSyncMode.srv SetSyncMode.srv
    GetPtpSyncStatus.srv RestoreDefaultConfig.srv
    SetDiagMonitor.srv
    SetChannelConfig.srv GetChannelConfig.srv
    SetPostProcessConfig.srv GetPostProcessConfig.srv
    SaveMap.srv LoadMap.srv ExployMap.srv DeviceMode.srv
  )
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(
    CATKIN_DEPENDS roscpp roslib sensor_msgs nav_msgs std_msgs
                   geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
                   cv_bridge ddynamic_reconfigure message_runtime
  )
  
  # Suppress ROS1 rosconsole macro warnings and notes on Ubuntu 20.04
  add_compile_options(
    -Wno-gnu-zero-variadic-macro-arguments
    -Wno-format-security
  )
endif()

set(APP_VERSION "${${PROJECT_NAME}_VERSION}")
configure_file(
  include/odin_app_version.hpp.in
  "${CMAKE_CURRENT_BINARY_DIR}/generated/odin_app_version.hpp"
  @ONLY
)

# =============================================================================
# Jetson Platform Detection & Hardware JPEG Decoder (NvJpegDecoder)
# =============================================================================
# JPEG_PREFER_SOFTWARE_DECODER is a CMake cache option that disables hardware JPEG
# decoding at compile time. When OFF (default), the Jetson NvJpegDecoder is
# linked directly into post_process_node. When ON, no hardware decoder sources
# are compiled and libnvjpeg is not linked, so cv::imdecode uses libjpeg-turbo
# exclusively.
#
# Why this is a compile-time choice: libnvjpeg.so exports the same jpeg_*
# symbols as libjpeg-turbo but with an incompatible ABI (libjpeg-8b vs turbo
# struct layout). When both are linked into the same binary, ELF symbol
# interposition makes cv::imdecode call libnvjpeg's implementation with a
# turbo-layout struct, causing a segfault. There is no runtime fallback from
# hardware to software within the same binary — if the hardware decoder fails
# at runtime, the user must recompile with -DJPEG_PREFER_SOFTWARE_DECODER=ON.
# (see docs/troubleshooting.md §10).
set(JPEG_PREFER_SOFTWARE_DECODER OFF CACHE BOOL "Prefer software JPEG decoder (skip hardware, avoid libnvjpeg symbol clash)")
set(USING_JETSON_DECODER OFF)
set(JETSON_DECODER_SOURCES "")

if(EXISTS "/etc/nv_tegra_release")
  message(STATUS "Jetson platform detected")
  
  # Read JetPack version from /etc/nv_tegra_release
  # Format: "# R36 (release), REVISION: 4.4, ..." or "# R35 (release), REVISION: 6.0, ..."
  file(READ "/etc/nv_tegra_release" NV_TEGRA_RELEASE_CONTENT)
  string(REGEX MATCH "R([0-9]+)" JETPACK_MAJOR_MATCH "${NV_TEGRA_RELEASE_CONTENT}")
  string(REGEX REPLACE "R([0-9]+)" "\\1" JETPACK_MAJOR_VERSION "${JETPACK_MAJOR_MATCH}")
  message(STATUS "  JetPack major version: R${JETPACK_MAJOR_VERSION}")
  
  # Check JetPack version compatibility for hardware decoder
  # R36+ (Ubuntu 22.04): Hardware decoder supported
  # R35 (Ubuntu 20.04): Hardware decoder NOT supported (OpenCV libjpeg-turbo ABI conflict)
  set(JETPACK_HW_DECODE_SUPPORTED OFF)
  if(JETPACK_MAJOR_VERSION GREATER_EQUAL 36)
    set(JETPACK_HW_DECODE_SUPPORTED ON)
    message(STATUS "  JetPack R${JETPACK_MAJOR_VERSION}: Hardware JPEG decoder SUPPORTED")
  else()
    message(STATUS "")
    message(STATUS "╔═══════════════════════════════════════════════════════════════════════╗")
    message(STATUS "║  JetPack R${JETPACK_MAJOR_VERSION} detected - Hardware JPEG decoder NOT SUPPORTED        ║")
    message(STATUS "╠═══════════════════════════════════════════════════════════════════════╣")
    message(STATUS "║  Reason: OpenCV in R35 uses libjpeg-turbo (776 bytes struct)          ║")
    message(STATUS "║          NvJpegDecoder requires libjpeg-8b (656 bytes struct)         ║")
    message(STATUS "║          ABI mismatch causes runtime crash                            ║")
    message(STATUS "╠═══════════════════════════════════════════════════════════════════════╣")
    message(STATUS "║  Solution: Upgrade to JetPack 6.x (R36+) with Ubuntu 22.04            ║")
    message(STATUS "║            Or use software decoder (auto-selected below)              ║")
    message(STATUS "╚═══════════════════════════════════════════════════════════════════════╝")
    message(STATUS "")
  endif()
  
  # Check for Jetson Multimedia API
  set(JETSON_MULTIMEDIA_API_DIR /usr/src/jetson_multimedia_api)
  
  if(NOT JPEG_PREFER_SOFTWARE_DECODER AND JETPACK_HW_DECODE_SUPPORTED AND EXISTS "${JETSON_MULTIMEDIA_API_DIR}/include/NvJpegDecoder.h")
    message(STATUS "  Jetson Multimedia API found")
    
    # Find required libraries
    find_library(NVBUF_UTILS NAMES nvbufsurface PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    find_library(NVBUF_TRANSFORM NAMES nvbufsurftransform PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    find_library(NVJPEG_LIB NAMES nvjpeg PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    
    if(NVBUF_UTILS AND NVBUF_TRANSFORM AND NVJPEG_LIB)
      message(STATUS "  nvbufsurface: ${NVBUF_UTILS}")
      message(STATUS "  nvbufsurftransform: ${NVBUF_TRANSFORM}")
      message(STATUS "  nvjpeg: ${NVJPEG_LIB}")
      
      set(USING_JETSON_DECODER ON)
      
      # Include paths for Jetson Multimedia API
      include_directories(
        ${JETSON_MULTIMEDIA_API_DIR}/include
        ${JETSON_MULTIMEDIA_API_DIR}/include/libjpeg-8b
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes
      )
      
      # Source files for the Jetson hardware JPEG decoder (compiled into
      # post_process_node when USING_JETSON_DECODER is ON).
      set(JETSON_DECODER_SOURCES
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvJpegDecoder.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvElement.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvElementProfiler.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvBuffer.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvLogging.cpp)
      message(STATUS "  JPEG Decoder: NvJpegDecoder Hardware Decoder")
    else()
      message(STATUS "  Required libraries not found - using software decoder")
    endif()
  else()
    message(STATUS "  Jetson Multimedia API not found - using software decoder")
  endif()
else()
  message(STATUS "Non-Jetson platform - using software decoder")
endif()

# =============================================================================
# Rockchip Platform Detection & Hardware JPEG Decoder (rockchip_mpp / MJPEG)
# =============================================================================
# Only probed when Jetson HW is not in use.
# Override paths with -DMPP_INCLUDE_DIR / -DMPP_LIBRARY.
set(USE_RK_DECODER OFF)

if(NOT USING_JETSON_DECODER AND NOT JPEG_PREFER_SOFTWARE_DECODER)
  find_path(MPP_INCLUDE_DIR rockchip/rk_mpi.h
            DOC "Rockchip MPP headers directory (containing rockchip/rk_mpi.h)")
  find_library(MPP_LIBRARY rockchip_mpp
               DOC "librockchip_mpp.so")
  # RGA (2D raster graphic acceleration): used by the RK decoder to convert
  # the decoded NV12 to BGR/RGB on hardware (replaces CPU cv::cvtColor).
  find_path(RGA_INCLUDE_DIR rga/im2d.h
            DOC "Rockchip RGA headers directory (containing rga/im2d.h)")
  find_library(RGA_LIBRARY rga
               DOC "librga.so")

  if(MPP_INCLUDE_DIR AND MPP_LIBRARY AND RGA_INCLUDE_DIR AND RGA_LIBRARY)
    message(STATUS "Rockchip MPP + RGA detected")
    message(STATUS "  MPP_INCLUDE_DIR: ${MPP_INCLUDE_DIR}")
    message(STATUS "  MPP_LIBRARY:     ${MPP_LIBRARY}")
    message(STATUS "  RGA_INCLUDE_DIR: ${RGA_INCLUDE_DIR}")
    message(STATUS "  RGA_LIBRARY:     ${RGA_LIBRARY}")
    set(USE_RK_DECODER ON)
    include_directories(${MPP_INCLUDE_DIR} ${RGA_INCLUDE_DIR})

    # Probe whether mpp_buffer_sync_begin/end exist in this MPP version.
    # Ubuntu 20.04 / older librk_mpp lacks them; 22.04 / newer has them.
    # When absent, the .cpp falls back to direct DMA_BUF_IOCTL_SYNC ioctl.
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_INCLUDES ${MPP_INCLUDE_DIR})
    set(CMAKE_REQUIRED_LIBRARIES ${MPP_LIBRARY})
    set(CMAKE_REQUIRED_QUIET TRUE)
    check_cxx_source_compiles("
#include <rockchip/mpp_buffer.h>
int main() {
  MppBuffer buf = nullptr;
  mpp_buffer_sync_begin(buf);
  mpp_buffer_sync_end(buf);
  (void)buf;
  return 0;
}
" MPP_HAS_BUFFER_SYNC)
    unset(CMAKE_REQUIRED_INCLUDES)
    unset(CMAKE_REQUIRED_LIBRARIES)
    unset(CMAKE_REQUIRED_QUIET)
    if(MPP_HAS_BUFFER_SYNC)
      message(STATUS "  MPP: mpp_buffer_sync_begin/end available (using native)")
    else()
      message(STATUS "  MPP: mpp_buffer_sync_begin/end not available (using ioctl fallback)")
    endif()
  else()
    message(STATUS "Rockchip MPP/RGA not found - using software decoder")
    message(STATUS "  (install librockchip-mpp-dev + librga-dev, or pass")
    message(STATUS "   -DMPP_INCLUDE_DIR / -DMPP_LIBRARY / -DRGA_INCLUDE_DIR / -DRGA_LIBRARY)")
  endif()
endif()

if(NOT USING_JETSON_DECODER AND NOT USE_RK_DECODER)
  message(STATUS "  JPEG Decoder: OpenCV cv::imdecode (libjpeg-turbo)")
endif()

# =============================================================================
# H.264 Decoder Detection (FFmpeg libavcodec + libswscale) — optional
# =============================================================================
# When FFmpeg is present the post_process_node gains the ability to decode
# H.264 CompressedImage frames (format="h264") into sensor_msgs/Image so that
# RViz can display them via the camera0/raw and camera1/raw topics.
# If FFmpeg is absent the build still succeeds and H.264 frames are silently
# skipped (NullH264Decoder returned by CreateH264Decoder).
find_package(PkgConfig QUIET)
set(USE_FFMPEG_H264_DECODER OFF)
set(H264_DECODER_SOURCES "")
if(PkgConfig_FOUND)
  pkg_check_modules(FFMPEG QUIET libavcodec libavutil libswscale)
  if(FFMPEG_FOUND)
    message(STATUS "FFmpeg found — H.264 decoder enabled (hardware-accelerated with software fallback)")
    message(STATUS "  avcodec: ${FFMPEG_VERSION}")
    set(USE_FFMPEG_H264_DECODER ON)
    include_directories(${FFMPEG_INCLUDE_DIRS})
    link_directories(${FFMPEG_LIBRARY_DIRS})
    set(H264_DECODER_SOURCES
      src/decoder/h264_decoder/ffmpeg_decoder/ffmpeg_h264_decoder.cpp)
  else()
    message(STATUS "FFmpeg not found — H.264 decoder disabled (RViz will not show H.264 streams)")
    message(STATUS "  To enable: sudo apt-get install libavcodec-dev libavutil-dev libswscale-dev")
  endif()
else()
  message(STATUS "PkgConfig not found — H.264 decoder disabled")
endif()

# =============================================================================
# libyuv Detection (fast YUV 4:2:0 → BGR colour conversion) — optional
# =============================================================================
# libyuv (Google) converts NV12/I420/NV21 → BGR (memory-order, identical to
# OpenCV BGR24) about 7x faster than libswscale / cv::cvtColor (0.3 ms vs
# 2.2 ms per 1280x1088 frame, measured with h264_bench). When found,
# post_process_node defines ODIN_HAVE_LIBYUV and the H.264 decoder plus the
# NV12 composite-image path use the libyuv conversion; without it they keep
# using sws/OpenCV. Runtime escape hatch: ODIN_DISABLE_LIBYUV=1.
# Note: Ubuntu's libyuv-dev ships no pkg-config file, so probe directly.
set(USE_LIBYUV OFF)
set(LIBYUV_LIBRARIES "")
set(LIBYUV_INCLUDE_DIRS "")
find_path(LIBYUV_INCLUDE_DIR NAMES libyuv.h)
find_library(LIBYUV_LIBRARY NAMES yuv libyuv)
if(LIBYUV_INCLUDE_DIR AND LIBYUV_LIBRARY)
  set(USE_LIBYUV ON)
  set(LIBYUV_LIBRARIES ${LIBYUV_LIBRARY})
  set(LIBYUV_INCLUDE_DIRS ${LIBYUV_INCLUDE_DIR})
  message(STATUS "libyuv found — fast YUV→BGR colour conversion enabled (${LIBYUV_LIBRARY})")
else()
  message(STATUS "libyuv not found — YUV→BGR conversion uses libswscale/OpenCV")
  message(STATUS "  To accelerate: sudo apt-get install libyuv-dev")
endif()

# =============================================================================
# H.264 NVDEC Hardware Decoder Detection (Jetson Multimedia API) — optional
# =============================================================================
# When the Jetson Multimedia API is present (Jetson platform with NvVideoDecoder.h),
# the post_process_node gains hardware H264 decoding via the dedicated NVDEC
# engine. This is independent of the JPEG hardware decoder: the user can
# compile with JPEG_PREFER_SOFTWARE_DECODER=ON for JPEG while still using NVDEC for
# H264. At runtime the factory tries NVDEC first and falls back to FFmpeg
# unless the force_software_decode parameter is set.
set(USE_JETSON_NVDEC_DECODER OFF)
set(H264_NVDEC_SOURCES "")
set(H264_NVDEC_LIBS "")

if(EXISTS "/etc/nv_tegra_release")
  set(JETSON_MM_API_DIR /usr/src/jetson_multimedia_api)

  if(EXISTS "${JETSON_MM_API_DIR}/include/NvVideoDecoder.h")
    # libnvvideo provides the NvVideoDecoder implementation.
    find_library(NVVIDEO_LIB NAMES nvvideo
      PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)

    if(NVVIDEO_LIB)
      message(STATUS "Jetson NVDEC found — H.264 hardware decoder enabled")
      message(STATUS "  nvvideo: ${NVVIDEO_LIB}")
      set(USE_JETSON_NVDEC_DECODER ON)

      # Include paths for Jetson Multimedia API.
      # These are idempotent — CMake deduplicates include_directories().
      include_directories(
        ${JETSON_MM_API_DIR}/include
        ${JETSON_MM_API_DIR}/samples/common/classes
      )

      set(H264_NVDEC_SOURCES
        src/decoder/h264_decoder/jetson_nvdec_decoder/jetson_nvdec_decoder.cpp
        # NvVideoDecoder depends on V4L2 helper classes that are provided as
        # source code in the Multimedia API samples directory.
        ${JETSON_MM_API_DIR}/samples/common/classes/NvVideoDecoder.cpp
        ${JETSON_MM_API_DIR}/samples/common/classes/NvV4l2Element.cpp
        ${JETSON_MM_API_DIR}/samples/common/classes/NvV4l2ElementPlane.cpp)

      # libnvv4l2 provides v4l2_open/v4l2_close/v4l2_ioctl used by NvV4l2Element.
      # libnvbufsurftransform and libdrm are needed by NvBuffer.
      find_library(NVV4L2_LIB NAMES nvv4l2
        PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
      find_library(NVBUF_SURF_TRANSFORM NAMES nvbufsurftransform
        PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
      find_library(DRM_LIB NAMES drm)
      # libnvbufsurface provides NvBufSurfaceFromFd/Map/SyncForCpu, used to
      # read the decoded NV12 surfaces from the capture plane.
      find_library(NVBUF_SURFACE NAMES nvbufsurface
        PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
      set(H264_NVDEC_LIBS ${NVVIDEO_LIB})
      if(NVV4L2_LIB)
        list(APPEND H264_NVDEC_LIBS ${NVV4L2_LIB})
      endif()
      if(NVBUF_SURFACE)
        list(APPEND H264_NVDEC_LIBS ${NVBUF_SURFACE})
      endif()
      if(NVBUF_SURF_TRANSFORM)
        list(APPEND H264_NVDEC_LIBS ${NVBUF_SURF_TRANSFORM})
      endif()
      if(DRM_LIB)
        list(APPEND H264_NVDEC_LIBS ${DRM_LIB})
      endif()

      # NvVideoDecoder also depends on common classes (NvElement, NvBuffer, …).
      # The JPEG hardware decoder compiles its own copies of these same classes
      # when USING_JETSON_DECODER is ON, but duplicate object files in the same
      # executable are harmless (same source, same compiler).
      list(APPEND H264_NVDEC_SOURCES
        ${JETSON_MM_API_DIR}/samples/common/classes/NvElement.cpp
        ${JETSON_MM_API_DIR}/samples/common/classes/NvElementProfiler.cpp
        ${JETSON_MM_API_DIR}/samples/common/classes/NvBuffer.cpp
        ${JETSON_MM_API_DIR}/samples/common/classes/NvLogging.cpp)
    else()
      message(STATUS "nvvideo library not found — H.264 NVDEC hardware decoder disabled")
    endif()
  else()
    message(STATUS "NvVideoDecoder.h not found — H.264 NVDEC hardware decoder disabled (non-Jetson or JetPack too old)")
  endif()
else()
  message(STATUS "Non-Jetson platform — H.264 NVDEC hardware decoder disabled")
endif()

# =============================================================================
# H.264 RK Hardware Decoder Detection (Rockchip MPP h264) — optional
# =============================================================================
# Independent of the JPEG decoder selection: probes for the same
# librockchip_mpp + librga libraries used by the MJPEG decoder (find_path /
# find_library are cached, so re-probing here is cheap and idempotent). This
# lets H.264 hardware decoding be enabled even if the JPEG decoder ends up
# using a different backend.
set(USE_RK_H264_DECODER OFF)
set(H264_RK_SOURCES "")

if(NOT USE_JETSON_NVDEC_DECODER)
  find_path(MPP_INCLUDE_DIR rockchip/rk_mpi.h
            DOC "Rockchip MPP headers directory (containing rockchip/rk_mpi.h)")
  find_library(MPP_LIBRARY rockchip_mpp
               DOC "librockchip_mpp.so")
  find_path(RGA_INCLUDE_DIR rga/im2d.h
            DOC "Rockchip RGA headers directory (containing rga/im2d.h)")
  find_library(RGA_LIBRARY rga
               DOC "librga.so")

  if(MPP_INCLUDE_DIR AND MPP_LIBRARY AND RGA_INCLUDE_DIR AND RGA_LIBRARY)
    message(STATUS "Rockchip MPP + RGA detected — H.264 hardware decoder enabled")
    set(USE_RK_H264_DECODER ON)
    include_directories(${MPP_INCLUDE_DIR} ${RGA_INCLUDE_DIR})

    # Probe whether mpp_buffer_sync_begin/end exist in this MPP version
    # (same check as the JPEG RK decoder; skipped if already computed above).
    if(NOT DEFINED MPP_HAS_BUFFER_SYNC)
      include(CheckCXXSourceCompiles)
      set(CMAKE_REQUIRED_INCLUDES ${MPP_INCLUDE_DIR})
      set(CMAKE_REQUIRED_LIBRARIES ${MPP_LIBRARY})
      set(CMAKE_REQUIRED_QUIET TRUE)
      check_cxx_source_compiles("
#include <rockchip/mpp_buffer.h>
int main() {
  MppBuffer buf = nullptr;
  mpp_buffer_sync_begin(buf);
  mpp_buffer_sync_end(buf);
  (void)buf;
  return 0;
}
" MPP_HAS_BUFFER_SYNC)
      unset(CMAKE_REQUIRED_INCLUDES)
      unset(CMAKE_REQUIRED_LIBRARIES)
      unset(CMAKE_REQUIRED_QUIET)
    endif()

    set(H264_RK_SOURCES
      src/decoder/h264_decoder/rockchip_decoder/rockchip_h264_decoder.cpp)
  else()
    message(STATUS "Rockchip MPP/RGA not found — H.264 hardware decoder disabled")
  endif()
else()
  message(STATUS "Jetson NVDEC active — H.264 RK hardware decoder disabled")
endif()

# Summary (use WARNING to ensure visibility in colcon build)
if(USING_JETSON_DECODER)
  message(STATUS "│  JPEG Decoder: Hardware (NvJpegDecoder)                     │")
  message(STATUS "│  Platform: Jetson R${JETPACK_MAJOR_VERSION}                                          │")
elseif(USE_RK_DECODER)
  message(STATUS "│  JPEG Decoder: Hardware (Rockchip MPP MJPEG)                │")
  message(STATUS "│  Platform: Rockchip                                         │")
else()
  message(STATUS "│  JPEG Decoder: Software (OpenCV cv::imdecode)               │")
  if(EXISTS "/etc/nv_tegra_release")
    message(STATUS "│  Platform: Jetson R${JETPACK_MAJOR_VERSION} (HW decode not supported)               │")
  else()
    message(STATUS "│  Platform: Non-Jetson (x86/other)                           │")
  endif()
endif()

# H.264 decoder summary
if(USE_JETSON_NVDEC_DECODER AND USE_FFMPEG_H264_DECODER)
  message(STATUS "│  H.264 Decoder: NVDEC (HW) + FFmpeg (SW fallback)            │")
elseif(USE_JETSON_NVDEC_DECODER)
  message(STATUS "│  H.264 Decoder: NVDEC (HW only, no SW fallback)             │")
elseif(USE_RK_H264_DECODER AND USE_FFMPEG_H264_DECODER)
  message(STATUS "│  H.264 Decoder: Rockchip MPP (HW) + FFmpeg (SW fallback)     │")
elseif(USE_RK_H264_DECODER)
  message(STATUS "│  H.264 Decoder: Rockchip MPP (HW only, no SW fallback)      │")
elseif(USE_FFMPEG_H264_DECODER)
  message(STATUS "│  H.264 Decoder: FFmpeg (software only)                       │")
else()
  message(STATUS "│  H.264 Decoder: Disabled (no backend)                        │")
endif()

# =============================================================================
# Common Include Directories
# =============================================================================
# Source code lives in snake_case feature subfolders under src/. Each header
# is included by its bare filename (e.g. #include "color_undistort.hpp"), so
# every subfolder must be on the include path. Keep this list flat for now
# and revisit once we promote stable interfaces to include/.
include_directories(
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  include
  include/utility
  include/camera
  src/camera_model
  src/cloud_render
  src/color_undistort
  src/decoder/jpeg_decoder
  src/decoder/h264_decoder
  src/depth_to_pointcloud
  src/disparity_to_pointcloud
  src/pointcloud_reprojector
  src/pointcloud_to_depth
  src/stereo_rectifier
  ${ODIN_SDK_INCLUDE_DIRS}
  ${ODIN_VENDORED_EIGEN3_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${ODIN_SDK_LIBRARY_DIRS})

# Resolve the actual SDK shared library path so we can install it next to
# the executables and let the loader find it at runtime.
# 解析 SDK 动态库的绝对路径，便于安装到运行目录并设置 RPATH。
find_library(ODIN_SDK_SO_PATH
  NAMES odin_sdk
  PATHS ${ODIN_SDK_LIBRARY_DIRS}
  NO_DEFAULT_PATH)
if(NOT ODIN_SDK_SO_PATH)
  message(WARNING
    "libodin_sdk.so not found in ${ODIN_SDK_LIBRARY_DIRS}. "
    "Build the SDK first (cmake --build module/sdk_api/build).")
endif()
# The SDK ships versioned symlinks (libodin_sdk.so -> .so.N -> .so.N.M.P).
# Collect the whole chain so the installed copy is not a dangling symlink.
# SDK 库是版本化符号链接链，需整条链一起安装，否则安装后是悬空链接。
set(ODIN_SDK_SO_FILES "")
if(ODIN_SDK_SO_PATH)
  set(ODIN_SDK_SO_FILES "${ODIN_SDK_SO_PATH}")
  set(_odin_sdk_cur "${ODIN_SDK_SO_PATH}")
  while(IS_SYMLINK "${_odin_sdk_cur}")
    file(READ_SYMLINK "${_odin_sdk_cur}" _odin_sdk_tgt)
    if(NOT IS_ABSOLUTE "${_odin_sdk_tgt}")
      get_filename_component(_odin_sdk_dir "${_odin_sdk_cur}" DIRECTORY)
      set(_odin_sdk_tgt "${_odin_sdk_dir}/${_odin_sdk_tgt}")
    endif()
    list(APPEND ODIN_SDK_SO_FILES "${_odin_sdk_tgt}")
    set(_odin_sdk_cur "${_odin_sdk_tgt}")
  endwhile()
endif()
if(ODIN_ROS_VERSION EQUAL 1)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# =============================================================================
# Source Files
# =============================================================================
# post_process_node aggregates the feature modules that share a process with
# it (jpeg decode, color undistort, cloud render, stereo rectify).
# Explicit list (instead of aux_source_directory) keeps the dependency graph
# obvious as more snake_case subfolders are added.
set(POST_PROCESS_SOURCES
  src/post_process_node.cpp
  src/color_undistort/color_undistort.cpp
  src/cloud_render/rawcloud_render.cpp
  src/decoder/jpeg_decoder/jpeg_decoder_factory.cpp
  src/decoder/jpeg_decoder/software_decoder/software_decoder.cpp
  src/decoder/jpeg_decoder/hybrid_decoder/hybrid_decoder.cpp
  src/decoder/h264_decoder/h264_decoder_factory.cpp
  ${H264_DECODER_SOURCES}
  ${H264_NVDEC_SOURCES}
  ${H264_RK_SOURCES}
)

# =============================================================================
# Executables
# =============================================================================

# --- odin_ros_driver_node ---
add_executable(odin_ros_driver_node src/odin_ros_driver_node.cpp)
target_link_libraries(odin_ros_driver_node ${ODIN_SDK_LIBRARIES} Threads::Threads yaml-cpp)
# Find libodin_sdk.so at runtime: alongside the binary ($ORIGIN) and in the
# per-arch source folder (helpful for development without install).
# 通过 RPATH 让运行时定位到动态库（与可执行文件同目录或 SDK 源码目录）。
set_target_properties(odin_ros_driver_node PROPERTIES
  BUILD_RPATH "$ORIGIN;${ODIN_SDK_LIBRARY_DIRS}"
  INSTALL_RPATH "$ORIGIN;${ODIN_SDK_LIBRARY_DIRS}")

# --- post_process_node ---
if(USING_JETSON_DECODER)
  add_executable(post_process_node
    ${POST_PROCESS_SOURCES}
    src/decoder/jpeg_decoder/jetson_decoder/jetson_hardware_decoder.cpp
    ${JETSON_DECODER_SOURCES})
  target_compile_definitions(post_process_node PRIVATE USING_JETSON_DECODER)
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp ${NVBUF_UTILS} ${NVBUF_TRANSFORM} ${NVJPEG_LIB})
elseif(USE_RK_DECODER)
  add_executable(post_process_node
    ${POST_PROCESS_SOURCES}
    src/decoder/jpeg_decoder/rockchip_decoder/rockchip_hardware_decoder.cpp)
  target_compile_definitions(post_process_node PRIVATE USE_RK_DECODER)
  if(MPP_HAS_BUFFER_SYNC)
    target_compile_definitions(post_process_node PRIVATE MPP_HAS_BUFFER_SYNC)
  endif()
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp ${MPP_LIBRARY} ${RGA_LIBRARY})
else()
  add_executable(post_process_node ${POST_PROCESS_SOURCES})
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp)
endif()
target_compile_definitions(post_process_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})
# Link FFmpeg when available (applies regardless of JPEG decoder backend)
if(USE_FFMPEG_H264_DECODER)
  target_compile_definitions(post_process_node PRIVATE USE_FFMPEG_H264_DECODER)
  target_link_libraries(post_process_node ${FFMPEG_LIBRARIES})
endif()
# Link libyuv when available (fast YUV→BGR conversion in the H.264 decoder and
# in NV12/I420/NV21 image paths — applies regardless of the backend).
if(USE_LIBYUV)
  target_compile_definitions(post_process_node PRIVATE ODIN_HAVE_LIBYUV)
  target_include_directories(post_process_node PRIVATE ${LIBYUV_INCLUDE_DIRS})
  target_link_libraries(post_process_node ${LIBYUV_LIBRARIES})
endif()
# Compile/link Jetson NVDEC when available (applies regardless of JPEG decoder backend)
if(USE_JETSON_NVDEC_DECODER)
  target_compile_definitions(post_process_node PRIVATE USE_JETSON_NVDEC_DECODER)
  target_link_libraries(post_process_node ${H264_NVDEC_LIBS})
endif()
# Compile/link Rockchip MPP H.264 decoder when available (applies regardless
# of the JPEG decoder backend; MPP_LIBRARY/RGA_LIBRARY may already be linked
# via the USE_RK_DECODER branch above — duplicate link entries are harmless).
if(USE_RK_H264_DECODER)
  target_compile_definitions(post_process_node PRIVATE USE_RK_H264_DECODER)
  if(MPP_HAS_BUFFER_SYNC)
    target_compile_definitions(post_process_node PRIVATE MPP_HAS_BUFFER_SYNC)
  endif()
  target_link_libraries(post_process_node ${MPP_LIBRARY} ${RGA_LIBRARY})
endif()

# --- opencv_undistort_node ---
add_executable(opencv_undistort_node src/opencv_undistort_node.cpp)
target_link_libraries(opencv_undistort_node ${OpenCV_LIBS} yaml-cpp)
target_compile_definitions(opencv_undistort_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

# --- pointcloud_reprojector_node (lives in src/pointcloud_reprojector/) ---
add_executable(pointcloud_reprojector_node
  src/pointcloud_reprojector/pointcloud_reprojector_node.cpp
  src/pointcloud_reprojector/pointcloud_reprojector.cpp
)
target_link_libraries(pointcloud_reprojector_node ${OpenCV_LIBS} yaml-cpp)
target_compile_definitions(pointcloud_reprojector_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

# --- pointcloud_to_depth_node (library-free point-cloud -> depth projection) ---
# Lives in src/pointcloud_to_depth/ together with the header-only projection
# core (pointcloud_to_depth_node.hpp) and module docs (README.md).
add_executable(pointcloud_to_depth_node src/pointcloud_to_depth/pointcloud_to_depth_node.cpp)
target_link_libraries(pointcloud_to_depth_node ${OpenCV_LIBS} yaml-cpp)
target_compile_definitions(pointcloud_to_depth_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

# --- depth_to_pointcloud_node (library-free depth -> dense XYZRGB cloud) ---
# Lives in src/depth_to_pointcloud/ together with the header-only back-projection
# core (depth_to_pointcloud_node.hpp) and module docs.
add_executable(depth_to_pointcloud_node src/depth_to_pointcloud/depth_to_pointcloud_node.cpp)
target_link_libraries(depth_to_pointcloud_node ${OpenCV_LIBS} yaml-cpp)
target_compile_definitions(depth_to_pointcloud_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

# --- diag_subscriber_node (C++ subscriber-side FPS & latency diagnostic) ---
# Subscribes to all driver data topics and counts messages at the subscriber
# level. More accurate than `ros2 topic hz` (Python) for large PointCloud2.
add_executable(diag_subscriber_node src/diag_subscriber_node.cpp)
target_compile_definitions(diag_subscriber_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

# =============================================================================
# ROS-Specific Linking & Install
# =============================================================================
if(ODIN_ROS_VERSION EQUAL 2)
  # ROS2 dependencies
  ament_target_dependencies(odin_ros_driver_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs diagnostic_msgs point_cloud_transport ament_index_cpp)
  ament_target_dependencies(post_process_node rclcpp sensor_msgs std_msgs cv_bridge pcl_conversions ament_index_cpp)
  ament_target_dependencies(opencv_undistort_node rclcpp sensor_msgs std_msgs cv_bridge ament_index_cpp)
  ament_target_dependencies(pointcloud_reprojector_node rclcpp sensor_msgs nav_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  ament_target_dependencies(pointcloud_to_depth_node rclcpp sensor_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  ament_target_dependencies(depth_to_pointcloud_node rclcpp sensor_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  ament_target_dependencies(diag_subscriber_node rclcpp sensor_msgs nav_msgs std_msgs ament_index_cpp)
  target_link_libraries(odin_ros_driver_node ${cpp_typesupport_target})
  target_link_libraries(post_process_node ${cpp_typesupport_target})
  target_link_libraries(opencv_undistort_node ${cpp_typesupport_target})
  target_link_libraries(pointcloud_reprojector_node ${cpp_typesupport_target})
  target_link_libraries(pointcloud_to_depth_node ${cpp_typesupport_target})
  target_link_libraries(depth_to_pointcloud_node ${cpp_typesupport_target})
  target_link_libraries(diag_subscriber_node ${cpp_typesupport_target})

  # ROS2 install
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/version.py"
    "# Generated by CMake. Do not edit.\n"
    "APP_PACKAGE_NAME = \"${PROJECT_NAME}\"\n"
    "APP_VERSION = \"${APP_VERSION}\"\n"
  )
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/version.py" DESTINATION "lib/${PROJECT_NAME}")
  install(TARGETS odin_ros_driver_node post_process_node opencv_undistort_node pointcloud_reprojector_node pointcloud_to_depth_node depth_to_pointcloud_node diag_subscriber_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
  # Install the SDK shared library next to the executables (for $ORIGIN RPATH)
  # 将 SDK 动态库与可执行文件一起安装，配合 $ORIGIN RPATH 在运行时被加载到。
  if(ODIN_SDK_SO_FILES)
    install(FILES ${ODIN_SDK_SO_FILES} DESTINATION lib/${PROJECT_NAME})
  endif()
  find_library(ODIN2_SPECIAL_PLUGIN
    NAMES odin-device-Odin2Special
    PATHS ${ODIN_SDK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
  if(ODIN2_SPECIAL_PLUGIN)
    install(FILES ${ODIN2_SPECIAL_PLUGIN} DESTINATION lib/${PROJECT_NAME})
  endif()
  ament_package()
else()
  # ROS1 dependencies
  target_link_libraries(odin_ros_driver_node ${catkin_LIBRARIES})
  target_link_libraries(post_process_node ${catkin_LIBRARIES})
  target_link_libraries(opencv_undistort_node ${catkin_LIBRARIES})
  target_link_libraries(pointcloud_reprojector_node ${catkin_LIBRARIES})
  target_link_libraries(pointcloud_to_depth_node ${catkin_LIBRARIES})
  target_link_libraries(depth_to_pointcloud_node ${catkin_LIBRARIES})
  target_link_libraries(diag_subscriber_node ${catkin_LIBRARIES})
  add_dependencies(odin_ros_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(post_process_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(opencv_undistort_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(pointcloud_reprojector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(pointcloud_to_depth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(depth_to_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(diag_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # ROS1 install
  install(TARGETS odin_ros_driver_node post_process_node opencv_undistort_node pointcloud_reprojector_node pointcloud_to_depth_node depth_to_pointcloud_node diag_subscriber_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch rviz config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  # Install the SDK shared library next to the executables.
  # 将 SDK 动态库与可执行文件一起安装。
  if(ODIN_SDK_SO_FILES)
    install(FILES ${ODIN_SDK_SO_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endif()
endif()
