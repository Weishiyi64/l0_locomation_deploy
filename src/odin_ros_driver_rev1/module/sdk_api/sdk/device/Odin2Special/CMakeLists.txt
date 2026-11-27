# Odin2Special — custom device, built as a standalone dlopen plugin
# (libodin-device-Odin2Special.so) instead of being merged into
# libodin_sdk.so.
#
# This directory is self-contained (optionally a git submodule): it declares
# its own build switch and returns early when disabled, so the parent only
# needs a plain add_subdirectory.
#
# The plugin resolves odin::sdk::* symbols (IDevice, OdinProtocolV2BaseDevice,
# logger C API) from libodin_sdk.so at runtime — all covered by the
# odin_sdk.version whitelist (_ZN4odin3sdk* / _ZTVN4odin3sdk* / odin_* / ...).

option(ODIN_BUILD_DEVICE_ODIN2SPECIAL
    "Build the Odin2Special custom device plugin" ON)
if(NOT ODIN_BUILD_DEVICE_ODIN2SPECIAL)
    return()
endif()

add_library(odin-device-Odin2Special SHARED
    Odin2SpecialDevice.cpp)

# ODIN_DEVICE_PLUGIN → ODIN_DEVICE_PLUGIN_API expands to __declspec(dllexport)
# on Windows (see DevicePluginEntry.h); no-op elsewhere.
target_compile_definitions(odin-device-Odin2Special PRIVATE ODIN_DEVICE_PLUGIN)

target_link_libraries(odin-device-Odin2Special PRIVATE
    odin_device_includes
    # odin_sdk is defined in the parent scope AFTER add_subdirectory(device);
    # CMake resolves plain target names at generate time, so this forward
    # reference is valid.
    odin_sdk)

# Output: lib/<arch>/libodin-device-Odin2Special.so — next to libodin_sdk.so,
# which is the compiled-in ODIN_DEVICE_PLUGIN_DIR search path, so no env var
# is needed. $ORIGIN lets the plugin locate libodin_sdk.so even when
# dlopen'ed before the SDK lib is on the loader path.
# On Windows the .dll is a RUNTIME artifact and its import .lib an ARCHIVE
# artifact — both go to the same dir; RPATH is ignored there (the loader
# resolves odin_sdk.dll via the standard DLL search order).
set_target_properties(odin-device-Odin2Special PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN")
