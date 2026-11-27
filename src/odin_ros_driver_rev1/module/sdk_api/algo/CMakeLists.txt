# =============================================================================
# 算法插件框架（宿主侧）编译配置
#
# 分层说明（谁编译什么）：
#   本目录    = 框架契约层：契约头 + AlgoLoader 实现 → 编进 SDK（libodin_sdk.so）
#   plugin_loopclosure/ = 领域契约头（iloopclosure_algo.h/iloopclosure_def.h）
#   插件 .so  = 插件实现层：独立编译为 libloopclosure.so，
#               由算法/适配开发者维护，SDK 只 include 契约头，不编译它
#
# 接入方式：
#   顶层 CMakeLists.txt 中 add_subdirectory(algo)
#   SDK 链接 odin_ialgo OBJECT 库
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# odin_ialgo object files are merged into odin_sdk(.dll/.so), so they need
# the same "building the SDK" marker as sdk/: it makes logger.h expand
# ODIN_LOGGER_EXPORT to dllexport instead of dllimport for the global logger
# variables (avoids LNK4217/LNK4286 bridging warnings on Windows; no-op
# elsewhere since the macro is only consulted under _WIN32).
add_compile_definitions(ODIN_SDK_BUILDING)

# Required for OBJECT libraries to be merged into a SHARED library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(odin_ialgo OBJECT ialgo_loader.cpp)

# PUBLIC：链接 odin_ialgo 的目标（SDK）自动获得契约头路径
target_include_directories(odin_ialgo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}                    # ialgo_loader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/interface          # ialgo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin_loopclosure # iloopclosure_algo.h, iloopclosure_def.h
)

# dlopen/dlsym/dlclose
target_link_libraries(odin_ialgo PUBLIC ${CMAKE_DL_LIBS})

set_target_properties(odin_ialgo PROPERTIES
    CXX_VISIBILITY_PRESET hidden        # 框架符号默认隐藏，只暴露 SDK 需要的
    POSITION_INDEPENDENT_CODE ON)       # 随宿主共享库链接必须 PIC
