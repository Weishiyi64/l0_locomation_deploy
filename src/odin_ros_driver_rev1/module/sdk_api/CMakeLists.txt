cmake_minimum_required(VERSION 3.10)

# The SDK version & release date are defined in sdk/CMakeLists.txt, since only
# libodin_sdk consumes them (baked into odin_sdk.cpp). This keeps sdk/ buildable
# standalone without depending on this parent file.
project(odin_lidar LANGUAGES CXX)

option(ODIN_BUILD_SAMPLES "Build sample executables" ON)

# Arch normalization + lib/ subdirectory naming (x86_64/aarch64 on Linux,
# win64 on Windows). Keep before every consumer of ODIN_SDK_LIB_OUTPUT_DIR /
# ODIN_TARGET_ARCH / ODIN_LIB_SUBDIR.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OdinArch.cmake)

# Compiler flags (/utf-8 on MSVC, warning set on GCC) must be set BEFORE any
# add_subdirectory(): add_compile_options() is directory-scoped and only
# affects targets created afterwards in that directory tree. Including it here
# covers algo/ and sample/ too — the previous placement inside sdk/ left
# odin_ialgo and the samples reading UTF-8 sources as GBK on Windows.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OdinCompilerFlags.cmake)

# Per-arch shared library output directory: <sdk_api>/lib/<subdir>/ where
# <subdir> follows the platform layout (see cmake/OdinArch.cmake).
set(ODIN_SDK_LIB_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ODIN_LIB_SUBDIR}"
    CACHE PATH "odin_sdk per-arch shared library output directory")
file(MAKE_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")

# 算法插件框架（宿主侧）：AlgoLoader + 契约头（OBJECT 库，合并进 odin_sdk）。
# 编译期不依赖插件 .so，运行期 dlopen 按需加载。
add_subdirectory(algo)

# mongoose is added from sdk/CMakeLists.txt (merged into odin_sdk as an OBJECT
# library), so it is intentionally not added here.
add_subdirectory(sdk)
if(ODIN_BUILD_SAMPLES)
    add_subdirectory(sample)
endif()
