# =============================================================================
# Cross-compilation toolchain: Linux aarch64 (typically from an x86_64 host).
#
# Usage:
#   cmake -S . -B build-arm \
#         -DCMAKE_TOOLCHAIN_FILE=toolchains/linux-aarch64.cmake
#   cmake --build build-arm
#
# The default compiler names follow Ubuntu/Debian's cross toolchain packages
# (gcc-aarch64-linux-gnu / g++-aarch64-linux-gnu). Override on the command
# line when using a different toolchain vendor:
#   -DCMAKE_C_COMPILER=/opt/gcc-arm/bin/aarch64-none-linux-gnu-gcc \
#   -DCMAKE_CXX_COMPILER=/opt/gcc-arm/bin/aarch64-none-linux-gnu-g++
#
# NOTE: toolchain files are ONLY for cross builds. A native Windows build
# (MSVC) does not use this directory at all — CMake auto-detects Visual
# Studio; see build.ps1. Same for native Linux builds (see build.sh).
# =============================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(NOT DEFINED CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
endif()
if(NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
endif()
