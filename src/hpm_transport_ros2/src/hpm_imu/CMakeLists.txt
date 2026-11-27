cmake_minimum_required(VERSION 3.8)
project(hpm_imu)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/workspace_version.cmake")
find_package(geometry_msgs REQUIRED)
find_package(hpm_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/imu.cpp
  src/yesense_frame_parser.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} hpm_transport::hpm_transport)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp sensor_msgs)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(
  TARGETS ${PROJECT_NAME}
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
  ament_add_gtest(test_yesense_frame_parser
    test/test_yesense_frame_parser.cpp
  )
  target_link_libraries(test_yesense_frame_parser ${PROJECT_NAME})
  target_compile_features(test_yesense_frame_parser PRIVATE cxx_std_17)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs hpm_transport rclcpp sensor_msgs)

# ponytail: clangd walks up from headers and misses colcon's build/<pkg>/compile_commands.json
execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    "${CMAKE_BINARY_DIR}/compile_commands.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json"
)

ament_package()
