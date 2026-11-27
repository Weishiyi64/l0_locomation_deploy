# =============================================================================
# OdinCompilerFlags.cmake — default build type, warnings and optimization.
#
# Included by: the top-level CMakeLists.txt BEFORE any add_subdirectory()
# (directory-scoped add_compile_options must precede target creation), and
# again by sdk/CMakeLists.txt for standalone sdk/ builds (guarded, no-op the
# second time). Branches on the compiler so the main CMakeLists stay
# platform-neutral:
#   MSVC     → /W4 warnings; CMake's default Release flags (/O2 /Ob2 /DNDEBUG)
#   GCC/Clang→ legacy flags, identical to the previous unconditional set:
#              -Wall -Wextra ... + "-O3 -DNDEBUG" for Release
#
# Also provides odin_hide_inline_symbols(<targets...>), the GCC/Clang-only
# -fvisibility-inlines-hidden helper (no-op on MSVC: PE/COFF has no vague
# linkage / weak-symbol leakage problem).
# =============================================================================

include_guard(GLOBAL)

if(MSVC)
    # MSVC: only raise the warning level. Optimization flags stay at the
    # CMake defaults (/O2 /Ob2 /DNDEBUG for Release, /Od /Zi for Debug) so
    # the multi-config Visual Studio generator keeps per-config semantics.
    # /utf-8: sources are UTF-8 (Chinese comments everywhere); without it
    # MSVC reads them in the system codepage (GBK) and multibyte sequences
    # can swallow "*/" or line breaks, breaking otherwise-valid code.
    add_compile_options(/W4 /utf-8)
else()
    # Default to Release build with -O3 optimization (legacy behaviour).
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
    set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

    # Comprehensive warnings (legacy set).
    add_compile_options(
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-sign-compare
        -Wno-format-truncation
        -Wno-variadic-macros
        -Wno-unused-function
    )
endif()

# Hide inline / template-instantiation symbols on GCC/Clang (see the long
# comment in sdk/CMakeLists.txt for the STL weak-symbol rationale). No-op on
# MSVC.
function(odin_hide_inline_symbols)
    if(MSVC)
        return()
    endif()
    foreach(_t IN ITEMS ${ARGV})
        if(TARGET ${_t})
            target_compile_options(${_t} PRIVATE
                $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)
        endif()
    endforeach()
endfunction()
