cmake_minimum_required(VERSION 3.5)
project(legged_deploy)

# Optimize deployment builds while retaining symbols and explicit build-type choices.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AppVersion.cmake")
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(luvbot_control_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# ---- third party ----
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- Inference backend: RKNN on RK3588, OpenVINO CPU/ONNX on x86 ----
set(LEGGED_DEPLOY_BACKEND "AUTO" CACHE STRING "Inference backend: AUTO, OPENVINO, or RKNN")
set_property(CACHE LEGGED_DEPLOY_BACKEND PROPERTY STRINGS AUTO OPENVINO RKNN)
string(TOUPPER "${LEGGED_DEPLOY_BACKEND}" LEGGED_DEPLOY_BACKEND)

if(LEGGED_DEPLOY_BACKEND STREQUAL "AUTO")
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _processor)
  if(_processor MATCHES "aarch64|arm64|armv8")
    set(_device_tree_hex "")
    foreach(_compatible_file
        "/proc/device-tree/compatible"
        "/sys/firmware/devicetree/base/compatible")
      if(EXISTS "${_compatible_file}")
        file(READ "${_compatible_file}" _device_tree_hex HEX)
        string(TOLOWER "${_device_tree_hex}" _device_tree_hex)
        break()
      endif()
    endforeach()

    # Device-tree strings contain NUL bytes, so match their ASCII hex values.
    if(_device_tree_hex MATCHES "726f636b636869702c726b33353838") # rockchip,rk3588
      set(LEGGED_DEPLOY_BACKEND "RKNN")
      message(STATUS "legged_deploy: AUTO detected Rockchip RK3588")
    else()
      message(FATAL_ERROR "AUTO supports RK3588 on ARM64. Use OPENVINO explicitly for CPU ONNX inference.")
    endif()
  else()
    set(LEGGED_DEPLOY_BACKEND "OPENVINO")
    message(STATUS "legged_deploy: AUTO selected OpenVINO for ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
endif()

set(LEGGED_DEPLOY_USE_RKNN OFF)
if(LEGGED_DEPLOY_BACKEND STREQUAL "RKNN")
  set(LEGGED_DEPLOY_USE_RKNN ON)
  message(STATUS "legged_deploy: RKNN backend (RK3588)")
  add_compile_definitions(USE_RKNN)
  set(RKNN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn_runtime"
      CACHE PATH "RKNN Runtime or rknpu2 root")
  find_path(RKNN_INCLUDE_DIR rknn_api.h
    HINTS ${RKNN_ROOT}
    PATH_SUFFIXES include runtime/Linux/librknn_api/include
  )
  find_library(RKNN_LIBRARY NAMES rknnrt
    HINTS ${RKNN_ROOT}
    PATH_SUFFIXES lib lib64 aarch64 runtime/Linux/librknn_api/aarch64
  )
  if(NOT RKNN_INCLUDE_DIR OR NOT RKNN_LIBRARY)
    message(FATAL_ERROR "RKNN Runtime not found. Set -DRKNN_ROOT=<rknpu2 runtime root>.")
  endif()
  set(PLATFORM_INCLUDE_DIRS ${RKNN_INCLUDE_DIR})
  set(PLATFORM_LIBS ${RKNN_LIBRARY})
elseif(LEGGED_DEPLOY_BACKEND STREQUAL "OPENVINO")
  message(STATUS "legged_deploy: OpenVINO backend")
  add_compile_definitions(USE_OPENVINO)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  set(PLATFORM_INCLUDE_DIRS "")
  set(PLATFORM_LIBS openvino::runtime)
else()
  message(FATAL_ERROR "Unsupported LEGGED_DEPLOY_BACKEND='${LEGGED_DEPLOY_BACKEND}'")
endif()

# Keep your compile definitions (note: running from install may need extra handling)
add_compile_definitions(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_compile_definitions(LEGGED_POLICY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/policy")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)




# -----------------------
# executable
# -----------------------
find_package(OpenSSL REQUIRED)
add_executable(rl_controller
  src/main.cpp
  src/rl_controller.cpp
  src/rl_controller_imitation.cpp
  src/order_mapping.cpp
  src/base/base_rl_controller.cpp
  src/wbc_lora_assets.cpp
  src/rl_controller_wbc.cpp
  src/wbc_json_reference.cpp
  src/rl_controller_wbc_json.cpp
)
if(LEGGED_DEPLOY_USE_RKNN)
  target_sources(rl_controller PRIVATE src/rknn_model.cpp src/wbc_io_probe.cpp src/stop_reference_fp16.cpp src/rl_controller_wbc_soft_stop.cpp)
  set_target_properties(rl_controller PROPERTIES INSTALL_RPATH "$ORIGIN/..")
else()
  target_sources(rl_controller PRIVATE src/onnx_model.cpp)
endif()
ament_target_dependencies(rl_controller
  luvbot_control_interfaces
  rclcpp
  std_msgs
  robot_interfaces
  sensor_msgs
  geometry_msgs
)
target_include_directories(rl_controller PRIVATE ${PLATFORM_INCLUDE_DIRS})

# Suppress third-party Eigen warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rl_controller PRIVATE
    -Wno-class-memaccess
    -Wno-deprecated-declarations
  )
endif()

target_link_libraries(rl_controller
  OpenSSL::Crypto
  yaml-cpp
  ${PLATFORM_LIBS}
  rt
  pthread
)

add_executable(local_planner src/local_planner.cpp)
if(LEGGED_DEPLOY_USE_RKNN)
  target_sources(local_planner PRIVATE src/rknn_model.cpp)
  set_target_properties(local_planner PROPERTIES INSTALL_RPATH "$ORIGIN/..")
else()
  target_sources(local_planner PRIVATE src/onnx_model.cpp)
endif()
ament_target_dependencies(local_planner rclcpp std_msgs robot_interfaces sensor_msgs geometry_msgs)
target_include_directories(local_planner PRIVATE ${PLATFORM_INCLUDE_DIRS})
target_link_libraries(local_planner yaml-cpp pthread ${PLATFORM_LIBS})



# -----------------------
# install
# -----------------------
#install(TARGETS rl_controller ${PROJECT_NAME}
install(TARGETS rl_controller
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS local_planner RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

if(LEGGED_DEPLOY_USE_RKNN)
  install(FILES
    ${RKNN_INCLUDE_DIR}/rknn_api.h
    DESTINATION include
  )
  install(FILES
    ${RKNN_LIBRARY}
    DESTINATION lib
  )
endif()

# install policy folder if it exists
install(DIRECTORY policy/
  DESTINATION share/${PROJECT_NAME}/policy
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs robot_interfaces sensor_msgs geometry_msgs)

configure_file(cmake/inference_backend.cmake.in inference_backend.cmake @ONLY)
ament_package(CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/inference_backend.cmake")
