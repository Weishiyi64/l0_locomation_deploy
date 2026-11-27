cmake_minimum_required(VERSION 3.10)
project(odin_foxglove_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===========================================================================
# Foxglove SDK — optional, local-only dependency
# ===========================================================================
# This sample requires the Foxglove SDK precompiled libraries.
# If the SDK is not found locally, this sample is SKIPPED — the rest of
# the project builds normally.
#
# To install the SDK, run (one-time, needs internet):
#   .\sample\foxglove_bridge\download_foxglove_sdk.ps1
#
# See sample/foxglove_bridge/README.md for details.
# ===========================================================================

# Import odin_sdk (same as other samples)
if (NOT TARGET odin_sdk)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sdk ${CMAKE_BINARY_DIR}/odin_sdk_build)
endif()

set(FOXGLOVE_SDK_VERSION "0.27.0")

# Determine platform subdirectory name
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(FOXGLOVE_PLATFORM "win64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(FOXGLOVE_PLATFORM "x86_64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(FOXGLOVE_PLATFORM "aarch64")
else()
    message(WARNING "foxglove_bridge: unsupported platform "
        "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}, skipping.")
    return()
endif()

# Check for local SDK copy
set(FOXGLOVE_LOCAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/foxglove-sdk/${FOXGLOVE_PLATFORM}")

if(NOT EXISTS "${FOXGLOVE_LOCAL_DIR}/lib/cmake/foxglove-sdk")
    message(WARNING
        "foxglove_bridge: Foxglove SDK not found — sample SKIPPED.\n"
        "  To enable, run:  .\\sample\\foxglove_bridge\\download_foxglove_sdk.ps1\n"
        "  Or see:          sample/foxglove_bridge/README.md"
    )
    return()
endif()

message(STATUS "foxglove_sdk: using LOCAL copy at 3rdparty/foxglove-sdk/${FOXGLOVE_PLATFORM}/")
set(foxglove_sdk_SOURCE_DIR "${FOXGLOVE_LOCAL_DIR}")

# --- Patch messages.hpp (idempotent — safe to re-run) ---
# logger.h defines DEBUG/INFO/ERROR/WARN as string macros which conflict
# with Foxglove SDK's enum class LogLevel.  The patch adds #undef guards.
execute_process(COMMAND ${CMAKE_COMMAND}
    -D "source_dir=${foxglove_sdk_SOURCE_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/foxglove_patch_messages.cmake"
)

# Find Foxglove SDK package
find_package(foxglove-sdk CONFIG REQUIRED HINTS "${foxglove_sdk_SOURCE_DIR}")

# Create C++ wrapper library from SDK
foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC)

# ===========================================================================
# Build target
# ===========================================================================

add_executable(odin_foxglove_bridge
    main.cpp
    foxglove_publisher.cpp)

target_include_directories(odin_foxglove_bridge
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty)

find_package(Threads REQUIRED)
target_link_libraries(odin_foxglove_bridge
    PRIVATE
        odin_sdk
        foxglove_cpp
        Threads::Threads)

# Foxglove SDK's C library references NtCreateNamedPipeFile from ntdll.dll.
if(WIN32)
    target_link_libraries(odin_foxglove_bridge PRIVATE ntdll)
endif()

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")

if(MSVC)
    # Defaults per config are fine.
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(odin_foxglove_bridge PRIVATE -g -O0)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(odin_foxglove_bridge PRIVATE -O2 -DNDEBUG)
endif()

install(TARGETS odin_foxglove_bridge RUNTIME DESTINATION bin/)
