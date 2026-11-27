# Device abstraction layer.
#
# Layout: every device implementation lives in its own subdirectory with its
# own CMakeLists.txt (siblings: OdinProtocolV2Base/, Odin1Lite/,
# Odin2Special/). A device subdirectory may optionally be a git submodule —
# the EXISTS guards below keep the build working when a submodule is not
# checked out.
#
#   - Builtin devices append their sources to odin_device_obj via
#     target_sources() and are merged into libodin_sdk.so.
#   - Custom plugin devices build a standalone .so (dlopen'ed by
#     DeviceLoader) and are gated by a per-device ODIN_BUILD_DEVICE_* option
#     so customer-specific code never enters a public SDK build unless
#     explicitly enabled.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- shared include surface --------------------------------------------------
# INTERFACE library carrying the include paths every device-layer target
# needs (loader, builtin devices, plugin devices). Keeps each per-device
# CMakeLists.txt to a few lines.
add_library(odin_device_includes INTERFACE)
target_include_directories(odin_device_includes INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/IDevice
    ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocolV2Base
    ${CMAKE_CURRENT_SOURCE_DIR}/Odin1Lite
    ${CMAKE_CURRENT_SOURCE_DIR}/Odin2Special
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../command
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/internal
    ${CMAKE_CURRENT_SOURCE_DIR}/../capture
    ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/IProtocol
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/OdinProtocol
    ${CMAKE_CURRENT_SOURCE_DIR}/../transport/ITransport
    ${CMAKE_CURRENT_SOURCE_DIR}/../transport/Network
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/slam_odom_synchronizer
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/payload_builder
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/logger
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cJson)
# Vendored Eigen3 headers (3rdparty/eigen3/Eigen/*). SYSTEM suppresses
# Eigen-internal warnings across GCC 9–13 (Ubuntu 20.04 – 24.04).
target_include_directories(odin_device_includes SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/eigen3)
# odin_ialgo provides the algo contract include dirs (ialgo_loader.h,
# iloopclosure_algo.h) consumed by OdinProtocolV2BaseDevice.h.
if(TARGET odin_ialgo)
    target_link_libraries(odin_device_includes INTERFACE odin_ialgo)
endif()

# ---- loader + shared device sources (merged into libodin_sdk.so) -------------
add_library(odin_device_obj OBJECT
    DeviceLoader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture/OdinCaptureBase.cpp)
target_link_libraries(odin_device_obj PUBLIC odin_device_includes)

# Dynamic loading for external device plugins (dlopen on POSIX, LoadLibrary
# on Windows — see the platform layer in DeviceLoader.cpp).
# Plugin binaries live next to libodin_sdk.so in lib/<arch>/ — the
# compiled-in ODIN_DEVICE_PLUGIN_DIR search path, so no env var is needed.
# ODIN_SDK_LIB_OUTPUT_DIR is set by sdk/CMakeLists.txt before this
# subdirectory is added; the fallback keeps this dir usable standalone.
if(NOT DEFINED ODIN_SDK_LIB_OUTPUT_DIR)
    set(ODIN_SDK_LIB_OUTPUT_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/${ODIN_LIB_SUBDIR}")
endif()
target_compile_definitions(odin_device_obj PRIVATE
    ODIN_HAS_DLOPEN
    ODIN_TARGET_ARCH="${ODIN_TARGET_ARCH}"
    ODIN_DEVICE_PLUGIN_DIR="${ODIN_SDK_LIB_OUTPUT_DIR}")
target_link_libraries(odin_device_obj PRIVATE ${CMAKE_DL_LIBS})

# ---- builtin devices (compiled into libodin_sdk.so) ---------------------------
add_subdirectory(OdinProtocolV2Base)
add_subdirectory(Odin1Lite)

# ---- custom plugin devices -----------------------------------------------------
# Each custom device subdirectory is fully self-contained: it declares its own
# ODIN_BUILD_DEVICE_* option and returns early when disabled. The parent only
# guards on directory existence, so an unchecked-out submodule never breaks
# the build.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Odin2Special/CMakeLists.txt")
    add_subdirectory(Odin2Special)
endif()
