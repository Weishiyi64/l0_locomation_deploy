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
  ${ODIN_SDK_MODULE_DIR}/sdk
  # 添加更多头文件目录...
)
# Native targets share this workspace's toolchain and incremental build graph.
add_subdirectory(module/sdk_api/3rdparty/mongoose odin_sdk/3rdparty/mongoose EXCLUDE_FROM_ALL)
add_subdirectory(module/sdk_api/sdk odin_sdk/sdk EXCLUDE_FROM_ALL)
set(ODIN_SDK_LIBRARIES odin_sdk)

# =============================================================================
# Common Dependencies (shared by ROS1 and ROS2)
# =============================================================================
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
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
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetCalibration.srv"
    "srv/SetSensorMode.srv"
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  # --- ROS1 packages ---
  find_package(catkin REQUIRED COMPONENTS
    roscpp roslib sensor_msgs nav_msgs std_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs
    cv_bridge ddynamic_reconfigure message_generation
  )
  add_service_files(FILES GetCalibration.srv SetSensorMode.srv)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(
    CATKIN_DEPENDS roscpp roslib sensor_msgs nav_msgs std_msgs
                   geometry_msgs tf2 tf2_ros tf2_geometry_msgs
                   cv_bridge ddynamic_reconfigure message_runtime
  )
  
  # Suppress ROS1 rosconsole macro warnings and notes on Ubuntu 20.04
  add_compile_options(
    -Wno-gnu-zero-variadic-macro-arguments
    -Wno-format-security
  )
endif()

# =============================================================================
# Jetson Platform Detection & Hardware JPEG Decoder (NvJpegDecoder)
# =============================================================================
# >>> Toggle: Set to ON to force software decoder for CPU testing <<<
set(FORCE_SOFTWARE_DECODER OFF)

set(USE_JETSON_DECODER OFF)
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
  
  if(NOT FORCE_SOFTWARE_DECODER AND JETPACK_HW_DECODE_SUPPORTED AND EXISTS "${JETSON_MULTIMEDIA_API_DIR}/include/NvJpegDecoder.h")
    message(STATUS "  Jetson Multimedia API found")
    
    # Find required libraries
    find_library(NVBUF_UTILS NAMES nvbufsurface PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    find_library(NVBUF_TRANSFORM NAMES nvbufsurftransform PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    find_library(NVJPEG_LIB NAMES nvjpeg PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
    
    if(NVBUF_UTILS AND NVBUF_TRANSFORM AND NVJPEG_LIB)
      message(STATUS "  nvbufsurface: ${NVBUF_UTILS}")
      message(STATUS "  nvbufsurftransform: ${NVBUF_TRANSFORM}")
      message(STATUS "  nvjpeg: ${NVJPEG_LIB}")
      
      set(USE_JETSON_DECODER ON)
      
      # Include paths for Jetson Multimedia API
      include_directories(
        ${JETSON_MULTIMEDIA_API_DIR}/include
        ${JETSON_MULTIMEDIA_API_DIR}/include/libjpeg-8b
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes
      )
      
      # Source files to compile
      set(JETSON_DECODER_SOURCES
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvJpegDecoder.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvElement.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvElementProfiler.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvBuffer.cpp
        ${JETSON_MULTIMEDIA_API_DIR}/samples/common/classes/NvLogging.cpp
      )
      
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
# Only probed when Jetson HW is not in use. Set FORCE_SOFTWARE_DECODER=ON to
# skip RK detection too. Override paths with -DMPP_INCLUDE_DIR / -DMPP_LIBRARY.
set(USE_RK_DECODER OFF)

if(NOT USE_JETSON_DECODER AND NOT FORCE_SOFTWARE_DECODER)
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
  else()
    message(STATUS "Rockchip MPP/RGA not found - using software decoder")
    message(STATUS "  (install librockchip-mpp-dev + librga-dev, or pass")
    message(STATUS "   -DMPP_INCLUDE_DIR / -DMPP_LIBRARY / -DRGA_INCLUDE_DIR / -DRGA_LIBRARY)")
  endif()
endif()

if(NOT USE_JETSON_DECODER AND NOT USE_RK_DECODER)
  message(STATUS "  JPEG Decoder: OpenCV cv::imdecode (libjpeg-turbo)")
endif()

# Summary (use WARNING to ensure visibility in colcon build)
if(USE_JETSON_DECODER)
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

# =============================================================================
# Common Include Directories
# =============================================================================
# Source code lives in snake_case feature subfolders under src/. Each header
# is included by its bare filename (e.g. #include "color_undistort.hpp"), so
# every subfolder must be on the include path. Keep this list flat for now
# and revisit once we promote stable interfaces to include/.
include_directories(
  include
  include/utility
  include/camera
  src/camera_model
  src/cloud_render
  src/color_undistort
  src/decoder/jpeg_decoder
  src/depth_to_pointcloud
  src/disparity_to_pointcloud
  src/pointcloud_reprojector
  src/pointcloud_to_depth
  src/stereo_rectifier
  ${ODIN_SDK_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
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
  src/stereo_rectifier/stereo_rectifier.cpp
)

# =============================================================================
# Executables
# =============================================================================

# --- odin_ros_driver_node ---
add_executable(odin_ros_driver_node src/odin_ros_driver_node.cpp)
target_link_libraries(odin_ros_driver_node ${ODIN_SDK_LIBRARIES} OpenSSL::SSL OpenSSL::Crypto Threads::Threads yaml-cpp)

# --- post_process_node ---
if(USE_JETSON_DECODER)
  add_executable(post_process_node
    ${POST_PROCESS_SOURCES}
    src/decoder/jpeg_decoder/jetson_decoder/jetson_hardware_decoder.cpp
    ${JETSON_DECODER_SOURCES})
  target_compile_definitions(post_process_node PRIVATE USE_JETSON_DECODER)
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp ${NVBUF_UTILS} ${NVBUF_TRANSFORM} ${NVJPEG_LIB})
elseif(USE_RK_DECODER)
  add_executable(post_process_node
    ${POST_PROCESS_SOURCES}
    src/decoder/jpeg_decoder/rockchip_decoder/rockchip_hardware_decoder.cpp)
  target_compile_definitions(post_process_node PRIVATE USE_RK_DECODER)
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp ${MPP_LIBRARY} ${RGA_LIBRARY})
else()
  add_executable(post_process_node ${POST_PROCESS_SOURCES})
  target_link_libraries(post_process_node ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp)
endif()
target_compile_definitions(post_process_node PRIVATE ROS_VERSION_MAJOR=${ODIN_ROS_VERSION})

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

# =============================================================================
# ROS-Specific Linking & Install
# =============================================================================
if(ODIN_ROS_VERSION EQUAL 2)
  # ROS2 dependencies
  ament_target_dependencies(odin_ros_driver_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs ament_index_cpp)
  ament_target_dependencies(post_process_node rclcpp sensor_msgs std_msgs cv_bridge pcl_conversions ament_index_cpp)
  ament_target_dependencies(pointcloud_reprojector_node rclcpp sensor_msgs nav_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  ament_target_dependencies(pointcloud_to_depth_node rclcpp sensor_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  ament_target_dependencies(depth_to_pointcloud_node rclcpp sensor_msgs std_msgs cv_bridge message_filters ament_index_cpp)
  target_link_libraries(odin_ros_driver_node ${cpp_typesupport_target})
  target_link_libraries(post_process_node ${cpp_typesupport_target})
  target_link_libraries(pointcloud_reprojector_node ${cpp_typesupport_target})
  target_link_libraries(pointcloud_to_depth_node ${cpp_typesupport_target})
  target_link_libraries(depth_to_pointcloud_node ${cpp_typesupport_target})

  # ROS2 install
  install(TARGETS odin_ros_driver_node post_process_node pointcloud_reprojector_node pointcloud_to_depth_node depth_to_pointcloud_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  # ROS1 dependencies
  target_link_libraries(odin_ros_driver_node ${catkin_LIBRARIES})
  target_link_libraries(post_process_node ${catkin_LIBRARIES})
  target_link_libraries(pointcloud_reprojector_node ${catkin_LIBRARIES})
  target_link_libraries(pointcloud_to_depth_node ${catkin_LIBRARIES})
  target_link_libraries(depth_to_pointcloud_node ${catkin_LIBRARIES})
  add_dependencies(odin_ros_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(post_process_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(pointcloud_reprojector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(pointcloud_to_depth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(depth_to_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # ROS1 install
  install(TARGETS odin_ros_driver_node post_process_node pointcloud_reprojector_node pointcloud_to_depth_node depth_to_pointcloud_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch rviz config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
