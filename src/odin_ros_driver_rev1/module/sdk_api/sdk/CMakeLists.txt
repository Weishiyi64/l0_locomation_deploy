cmake_minimum_required(VERSION 3.10)

# =============================================================================
# SDK version & release date. This is the single source of truth (only
# libodin_sdk uses it, baked into odin_sdk.cpp). Edit the two lines below to
# bump them. CI/CD can override without editing the file:
#   -DODIN_SDK_VERSION=2.0.3 -DODIN_SDK_BUILD_DATE=20260720
# Fixed values (no build-time timestamp) keep the version reproducible.
# =============================================================================
set(ODIN_SDK_VERSION    "3.6.0"    CACHE STRING "SDK semantic version (X.Y.Z)")
set(ODIN_SDK_BUILD_DATE "20261016" CACHE STRING "SDK release date (YYYYMMDD)")

project(odin_sdk VERSION ${ODIN_SDK_VERSION} LANGUAGES C CXX)

set(ODIN_SDK_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(ODIN_SDK_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(ODIN_SDK_VERSION_PATCH "${PROJECT_VERSION_PATCH}")

set(ODIN_SDK_VERSION_STRING "${ODIN_SDK_VERSION}_${ODIN_SDK_BUILD_DATE}")
message(STATUS "odin_sdk: version = ${ODIN_SDK_VERSION_STRING}")

include(GNUInstallDirs)

# Platform- & compiler-specific settings live in cmake/ so this file stays
# platform-neutral: arch normalization (OdinArch), warning/optimization flags
# per compiler (OdinCompilerFlags), Windows link libs + DLL export policy
# (OdinPlatform).
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/OdinArch.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/OdinCompilerFlags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/OdinPlatform.cmake)

# Optional Link-Time Optimization for Release builds.
option(ODIN_SDK_ENABLE_LTO "Enable Link-Time Optimization for Release builds" OFF)
if(ODIN_SDK_ENABLE_LTO AND CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ODIN_SDK_LTO_OK OUTPUT ODIN_SDK_LTO_ERR)
    if(ODIN_SDK_LTO_OK)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
        message(STATUS "odin_sdk: LTO enabled")
    else()
        message(STATUS "odin_sdk: LTO not supported (${ODIN_SDK_LTO_ERR})")
    endif()
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Required for OBJECT libraries to be merged into a SHARED library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Per-arch output directory comes from OdinArch.cmake (ODIN_LIB_SUBDIR:
# x86_64/aarch64 on Linux, win64 on Windows).
if(NOT DEFINED ODIN_SDK_LIB_OUTPUT_DIR)
    set(ODIN_SDK_LIB_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/${ODIN_LIB_SUBDIR}")
    file(MAKE_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")
endif()
message(STATUS "odin_sdk: arch=${ODIN_TARGET_ARCH} output=${ODIN_SDK_LIB_OUTPUT_DIR}")

# Find dependencies before any add_library/add_subdirectory so that
# they are in scope for odin_sdk and its merged OBJECT libraries.
find_package(Threads REQUIRED)

# Everything compiled under sdk/ ends up inside odin_sdk(.dll/.so): mark the
# whole directory scope as "building the SDK" so logger.h expands
# ODIN_LOGGER_EXPORT to dllexport (not dllimport) for the global logger
# variables. Without this every internal OBJECT library references
# __imp_g_logPrint and the linker bridges it back to the local definition,
# emitting LNK4217/LNK4286 warnings for each referencing object. No-op on
# non-Windows (the macro is only consulted under _WIN32).
add_compile_definitions(ODIN_SDK_BUILDING)

# Vendored OpenSSL (static libs, 3rdparty/openssl): configured, built and
# installed out-of-source via ExternalProject. Sets OPENSSL_FOUND /
# OPENSSL_INCLUDE_DIR / OPENSSL_LIBRARIES in this scope, so consumers keep
# using ${OPENSSL_LIBRARIES} exactly as with the system package (imported
# targets are avoided on purpose for CMake 3.16 OBJECT-library compatibility).
# Static linking also removes the runtime dependency: neither libssl-dev
# nor libssl needs to be installed on the customer machine.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/openssl
                 ${CMAKE_CURRENT_BINARY_DIR}/openssl)

# Build protocol OBJECT library (will be merged into odin_sdk)
add_subdirectory(protocol)

# Build transport OBJECT library (will be merged into odin_sdk)
add_subdirectory(transport)

# Build command OBJECT library (will be merged into odin_sdk)
add_subdirectory(command)

# Build device OBJECT library (will be merged into odin_sdk)
add_subdirectory(device)

# Build https OBJECT library (will be merged into odin_sdk)
# TODO: Uncomment when mongoose is available in 3rdparty/mongoose/
# add_subdirectory(secure_transport)

# Build http_transfer library (HTTP/HTTPS client). Used internally by
# OdinCommand's HTTP backends (network mgmt + file transfer).
add_subdirectory(utils/http)

# Build mongoose OBJECT library (will be merged into odin_sdk). Added here
# rather than in the parent so sdk/ can also be configured standalone; a
# dedicated binary dir is required since the source lives outside this tree.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mongoose
                 ${CMAKE_CURRENT_BINARY_DIR}/mongoose)

# Vendored OpenSSL ordering: it must be built before compiling mongoose.c
# and http_client.cpp (both include <openssl/*.h> via MG_TLS_OPENSSL).
add_dependencies(mongoose openssl_vendor)
add_dependencies(http_transfer_obj openssl_vendor)

# SDK sources
# Note: odin_command_channel.cpp is compiled inside the device OBJECT library
# (device/CMakeLists.txt) together with its only consumer OdinCommand.
file(GLOB ODIN_SDK_SOURCES
    logger/logger.cpp
    src/odin_sdk.cpp
    src/OdinSdkImpl.cpp
    utils/slam_odom_synchronizer/SlamOdomSynchronizer.cpp
    hotplugListener/UdpHotplugListener/UdpHotplugListener.cpp
    hotplugListener/CompositeHotplugListener/CompositeHotplugListener.cpp
    hotplugListener/HotplugListenerFactory.cpp
    discovery/OdinUdpDiscovery/OdinUdpDiscovery.cpp
    utils/network_utils.cpp
    utils/payload_builder/payload_builder.cpp
    ../3rdparty/tinyxml2/tinyxml2.cpp
    ../3rdparty/cJson/cJSON.c)

# Merge all OBJECT libraries into odin_sdk SHARED library - external consumers
# only need to link libodin_sdk.so (+ OpenSSL).
add_library(odin_sdk SHARED ${ODIN_SDK_SOURCES}
    $<TARGET_OBJECTS:odin_protocol_obj>
    $<TARGET_OBJECTS:odin_transport_obj>
    $<TARGET_OBJECTS:odin_command_obj>
    $<TARGET_OBJECTS:odin_device_obj>
    $<TARGET_OBJECTS:odin_ialgo>
    $<TARGET_OBJECTS:http_transfer_obj>
    $<TARGET_OBJECTS:mongoose>)

set_target_properties(odin_sdk PROPERTIES
    VERSION ${ODIN_SDK_VERSION}
    SOVERSION ${ODIN_SDK_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")
# Multi-config generators (Visual Studio) append a per-config subdirectory
# (e.g. lib/win64/Release/) to the output directories above unless the
# per-config variants are set explicitly. Pin them to the same directory so
# the DLL (RUNTIME), the import lib (ARCHIVE) and Linux' .so (LIBRARY) all
# land directly in lib/<subdir>/ regardless of generator.
foreach(_odin_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(odin_sdk PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${_odin_cfg} "${ODIN_SDK_LIB_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_${_odin_cfg} "${ODIN_SDK_LIB_OUTPUT_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY_${_odin_cfg} "${ODIN_SDK_LIB_OUTPUT_DIR}")
endforeach()

# Inject the version string as a compile definition consumed by GetSdkVersion().
# The value (e.g. "2.0.2_20260611") contains no spaces or quotes, so the quoted
# form passes through the Makefile generator without escaping issues.
# (ODIN_SDK_BUILDING is set directory-wide above, covering odin_sdk itself and
# the merged OBJECT libraries; samples never see it.)
target_compile_definitions(odin_sdk PRIVATE
    ODIN_SDK_VERSION_STRING="${ODIN_SDK_VERSION_STRING}")

# Hide inline / template-instantiation symbols (the source of STL weak-symbol
# leakage such as std::__future_base::*) while keeping every non-inline
# function exported by default. Without this the SDK ships its own copies of
# inline STL symbols, and a consumer like rclcpp may bind its STL calls to
# the SDK-local copies (because libodin_sdk.so appears before libstdc++.so
# in the NEEDED order). The two copies are not necessarily byte-identical
# (different optimization level, LTO, ODR edge cases) and can crash with a
# NULL function-pointer jump.
# No-op on MSVC (see cmake/OdinCompilerFlags.cmake) — PE/COFF has no vague
# linkage / STL weak-symbol leakage problem.
odin_hide_inline_symbols(odin_sdk odin_protocol_obj odin_transport_obj
    odin_command_obj odin_device_obj http_transfer_obj)

# Restrict the .so dynamic export table to the SDK public API surface using a
# linker version script (positive whitelist). This is what actually prevents
# the STL weak symbols (std::__future_base::*, std::vector<>::_M_*, etc.) from
# leaking out of libodin_sdk.so. -fvisibility-inlines-hidden alone is not
# enough because vtables and template-instantiated member functions are
# vague-linkage data, not "inlines".
set(ODIN_SDK_VERSION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/odin_sdk.version")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${ODIN_SDK_VERSION_SCRIPT}")
    target_link_options(odin_sdk PRIVATE
        "LINKER:--version-script=${ODIN_SDK_VERSION_SCRIPT}")
    set_property(TARGET odin_sdk APPEND PROPERTY LINK_DEPENDS
        "${ODIN_SDK_VERSION_SCRIPT}")
endif()

# Windows counterpart of the version script: symbol export policy + system
# libraries (Winsock, IP Helper, static-OpenSSL deps). See
# cmake/OdinPlatform.cmake.
odin_apply_platform_settings(odin_sdk)

# Strip the shared library in Release builds to reduce its size while keeping
# the dynamic symbol table required for runtime linking. GNU strip only
# (not available / not applicable on Windows).
option(ODIN_SDK_STRIP_RELEASE "Strip release shared libraries to reduce size" ON)
if(ODIN_SDK_STRIP_RELEASE AND NOT WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Release" AND CMAKE_STRIP)
    add_custom_command(TARGET odin_sdk POST_BUILD
        COMMAND ${CMAKE_STRIP} --strip-unneeded $<TARGET_FILE:odin_sdk>
        COMMENT "Stripping $<TARGET_FILE_NAME:odin_sdk>")
endif()

# Public headers live under include/
target_include_directories(odin_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/device
        ${CMAKE_CURRENT_SOURCE_DIR}/device/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/device/OdinProtocolV2Base
        ${CMAKE_CURRENT_SOURCE_DIR}/device/Odin1Lite
        ${CMAKE_CURRENT_SOURCE_DIR}/device/Odin2Special
        ${CMAKE_CURRENT_SOURCE_DIR}/command
        ${CMAKE_CURRENT_SOURCE_DIR}/command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/capture
        ${CMAKE_CURRENT_SOURCE_DIR}/capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/IHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/UdpHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/payload_builder
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/slam_odom_synchronizer
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyxml2
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cJson)

# Vendored OpenSSL headers (SYSTEM: suppress third-party warnings).
target_include_directories(odin_sdk SYSTEM PRIVATE
    ${OPENSSL_INCLUDE_DIR})
# Link OpenSSL publicly so external consumers only need to link odin_sdk.
# ${OPENSSL_LIBRARIES} points at the vendored static libs (libssl.a +
# libcrypto.a) provided by 3rdparty/openssl via ExternalProject.
target_link_libraries(odin_sdk
    PUBLIC ${OPENSSL_LIBRARIES}
    PRIVATE Threads::Threads ${CMAKE_DL_LIBS})

# The static libs are produced by the openssl_vendor build step; odin_sdk
# must wait for them to be installed before linking.
add_dependencies(odin_sdk openssl_vendor)

# Link against odin_ialgo to get include paths for algo headers
if(TARGET odin_ialgo)
    target_link_libraries(odin_sdk PRIVATE odin_ialgo)
endif()

# Install odin_sdk shared library to per-arch lib folder (Windows: the DLL
# is the runtime component, the import lib goes to lib/).
install(TARGETS odin_sdk
    EXPORT odin_sdkTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${ODIN_TARGET_ARCH}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${ODIN_TARGET_ARCH}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Public headers (including logger/logger.h) all live under include/.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT odin_sdkTargets
    FILE odin_sdkTargets.cmake
    NAMESPACE odin::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odin_sdk)
    