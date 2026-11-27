# OdinProtocolV2Base — V2 protocol base device (builtin).
# Sources are appended to odin_device_obj and merged into libodin_sdk.so.
# Include paths come from the odin_device_includes INTERFACE target
# (see ../CMakeLists.txt).

# Absolute path required: CMP0076 is OLD (cmake_minimum_required 3.10), so
# relative target_sources paths from another directory resolve against the
# target's source dir, not this one.
target_sources(odin_device_obj PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocolV2BaseDevice.cpp)
