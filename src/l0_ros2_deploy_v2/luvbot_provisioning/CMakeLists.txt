cmake_minimum_required(VERSION 3.8)
project(luvbot_provisioning)

# Optimize deployment builds while retaining symbols and explicit build-type choices.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AppVersion.cmake")
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GIO2 REQUIRED IMPORTED_TARGET gio-2.0)

add_executable(luvbot_provd src/luvbot_provd.c)
target_compile_definitions(luvbot_provd PRIVATE L0_APP_VERSION="${APP_VERSION}")
target_link_libraries(luvbot_provd PkgConfig::GIO2)

add_executable(luvbot_bridge src/luvbot_bridge.cpp)
target_compile_features(luvbot_bridge PRIVATE cxx_std_17)
ament_target_dependencies(luvbot_bridge ament_index_cpp rclcpp std_msgs)

add_executable(network_ready_gate src/network_ready_gate.cpp)
target_compile_features(network_ready_gate PRIVATE cxx_std_14)
ament_target_dependencies(network_ready_gate rclcpp std_msgs)
target_link_libraries(network_ready_gate PkgConfig::GIO2)

install(TARGETS luvbot_provd luvbot_bridge network_ready_gate DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/configure_provisioning_sudo.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
