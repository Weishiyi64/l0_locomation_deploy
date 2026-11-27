# =============================================================================
# OdinArch.cmake — architecture detection & normalization (all platforms).
#
# Included by: CMakeLists.txt (root), sdk/CMakeLists.txt,
#             3rdparty/openssl/CMakeLists.txt. Safe to include repeatedly
#             (include_guard(GLOBAL)).
#
# Sets (in the including scope):
#   ODIN_TARGET_ARCH  normalized arch id matching `uname -m`:
#                     x86_64 | aarch64 (raw name kept for anything else)
#   ODIN_LIB_SUBDIR   per-platform library directory under lib/:
#                     Linux   → x86_64 / aarch64     (legacy layout, unchanged)
#                     Windows → win64 (MSVC x64)
#
# The Windows mapping exists because CMAKE_SYSTEM_PROCESSOR reports "AMD64"
# while the SDK layout uses x86_64 on Linux; keeping two names lets the Linux
# tree stay byte-compatible with what is already committed under lib/.
# =============================================================================

include_guard(GLOBAL)

if(NOT DEFINED ODIN_TARGET_ARCH)
    if(CMAKE_SYSTEM_PROCESSOR)
        set(_odin_proc "${CMAKE_SYSTEM_PROCESSOR}")
    elseif(UNIX)
        execute_process(COMMAND uname -m
            OUTPUT_VARIABLE _odin_proc
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    string(TOLOWER "${_odin_proc}" _odin_proc)
    if(_odin_proc MATCHES "^(x86_64|amd64|x64)")
        set(ODIN_TARGET_ARCH "x86_64")
    elseif(_odin_proc MATCHES "^(aarch64|arm64)")
        set(ODIN_TARGET_ARCH "aarch64")
    else()
        # Unknown arch: keep the raw name (Linux lib/<raw>/ layout stays usable).
        set(ODIN_TARGET_ARCH "${_odin_proc}")
    endif()
endif()

if(NOT DEFINED ODIN_LIB_SUBDIR)
    if(WIN32)
        if(ODIN_TARGET_ARCH STREQUAL "x86_64")
            set(ODIN_LIB_SUBDIR "win64")
        elseif(ODIN_TARGET_ARCH STREQUAL "aarch64")
            set(ODIN_LIB_SUBDIR "win-arm64")
        else()
            message(FATAL_ERROR
                "odin_arch: unsupported Windows architecture '${ODIN_TARGET_ARCH}'.")
        endif()
    else()
        set(ODIN_LIB_SUBDIR "${ODIN_TARGET_ARCH}")
    endif()
endif()

message(STATUS "odin_arch: arch=${ODIN_TARGET_ARCH} lib_subdir=${ODIN_LIB_SUBDIR}")
