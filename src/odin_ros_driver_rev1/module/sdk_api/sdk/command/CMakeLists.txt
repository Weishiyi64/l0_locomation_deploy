set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMAND_SOURCES
    odin2Command/OdinCommand.cpp
    odin2Command/internal/http_config_backend.cpp
    odin2Command/internal/http_file_backend.cpp
    odin2Command/internal/odin_command_channel.cpp
)

# --- 1. OBJECT library: merged into libodin_sdk.so (builtin devices) ---
add_library(odin_command_obj OBJECT ${COMMAND_SOURCES})
target_include_directories(odin_command_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/odin2Command
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/payload_builder
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cJson
)
