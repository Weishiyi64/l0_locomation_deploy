# =============================================================================
# OdinPlatform.cmake — platform-specific settings for the SDK shared library.
#
# Included by: sdk/CMakeLists.txt.
#
# Public helper:
#   odin_apply_platform_settings(<target>)
#
#     Windows/MSVC:
#       - WINDOWS_EXPORT_ALL_SYMBOLS: CMake scans the target's object files
#         and generates a .def exporting every symbol — the Windows
#         counterpart of the Linux linker version script (sdk/odin_sdk.version),
#         keeping the public headers free of __declspec annotations.
#       - links the system libraries the SDK needs on Windows (Winsock, IP
#         Helper) plus the ones a statically-linked OpenSSL requires
#         (crypt32 and friends).
#     Linux:
#       - no-op (version script and release strip are handled directly in
#         sdk/CMakeLists.txt).
# =============================================================================

function(odin_apply_platform_settings target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "odin_apply_platform_settings: '${target}' is not a target")
    endif()

    if(WIN32)
        set_target_properties(${target} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

        # ws2_32  — Winsock2 (sockets; also covers the embedded mongoose)
        # iphlpapi— GetAdaptersAddresses (network_utils.cpp)
        # crypt32/user32/advapi32/gdi32 — required by the static OpenSSL libs
        #           on Windows; unused ones are simply ignored by the linker.
        target_link_libraries(${target} PRIVATE
            ws2_32
            iphlpapi
            crypt32
            user32
            advapi32
            gdi32)
    endif()
endfunction()
