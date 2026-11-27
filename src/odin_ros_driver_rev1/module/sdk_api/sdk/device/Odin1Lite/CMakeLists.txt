# Odin1Lite — builtin device (merged into libodin_sdk.so via odin_device_obj).
# Include paths come from the odin_device_includes INTERFACE target
# (see ../CMakeLists.txt).

# Absolute path required: CMP0076 is OLD (cmake_minimum_required 3.10), so
# relative target_sources paths from another directory resolve against the
# target's source dir, not this one.
target_sources(odin_device_obj PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Odin1LiteDevice.cpp)
