# cmake/foxglove_patch_messages.cmake
# Workaround: Odin SDK logger.h defines DEBUG/INFO/ERROR/WARN as string macros,
# which conflict with Foxglove SDK's enum class LogLevel { DEBUG = 1, INFO = 2, ... }.
# This script patches messages.hpp to undefine these macros.

set(file "${source_dir}/include/foxglove/messages.hpp")
file(READ "${file}" content)
string(FIND "${content}" "#undef DEBUG" pos)
if(pos EQUAL -1)
  string(REPLACE
    "#ifndef __wasm32__"
    "#ifdef DEBUG\n#undef DEBUG\n#endif\n#ifdef INFO\n#undef INFO\n#endif\n#ifdef ERROR\n#undef ERROR\n#endif\n#ifdef WARN\n#undef WARN\n#endif\n\n#ifndef __wasm32__"
    content "${content}")
  file(WRITE "${file}" "${content}")
  message(STATUS "Patched messages.hpp: added #undef DEBUG/INFO/ERROR/WARN")
else()
  message(STATUS "messages.hpp already patched")
endif()
