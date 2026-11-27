cmake_minimum_required(VERSION 3.8)
project(hpm_tactile)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/workspace_version.cmake")
find_package(hpm_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luvbot_hardware_interfaces REQUIRED)

add_library(hpm_tactile_lib SHARED src/tactile.cpp)
set_target_properties(hpm_tactile_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(hpm_tactile_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hpm_tactile_lib hpm_transport::hpm_transport)
ament_target_dependencies(hpm_tactile_lib rclcpp luvbot_hardware_interfaces)
target_compile_features(hpm_tactile_lib PUBLIC cxx_std_17)

install(
  TARGETS hpm_tactile_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tactile test/test_tactile.cpp)
  target_link_libraries(test_tactile hpm_tactile_lib)
  target_compile_features(test_tactile PRIVATE cxx_std_17)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hpm_transport rclcpp luvbot_hardware_interfaces)

# ponytail: clangd walks up from headers and misses colcon's build/<pkg>/compile_commands.json
execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    "${CMAKE_BINARY_DIR}/compile_commands.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json"
)

ament_package()
