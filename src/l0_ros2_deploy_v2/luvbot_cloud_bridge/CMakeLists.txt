cmake_minimum_required(VERSION 3.16)
project(luvbot_cloud_bridge)

# Optimize deployment builds while retaining symbols and explicit build-type choices.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AppVersion.cmake")
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(luvbot_control_interfaces REQUIRED)
find_package(luvbot_hardware_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS proto/luvbot.proto)
add_library(luvbot_cloud_proto STATIC ${PROTO_SOURCES})
target_compile_features(luvbot_cloud_proto PRIVATE cxx_std_14)
target_include_directories(luvbot_cloud_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(luvbot_cloud_proto PUBLIC protobuf::libprotobuf)
add_executable(cloud_bridge_node src/cloud_bridge_node.cpp)
target_compile_features(cloud_bridge_node PRIVATE cxx_std_14)
ament_target_dependencies(cloud_bridge_node rclcpp std_msgs sensor_msgs ament_index_cpp robot_interfaces luvbot_control_interfaces luvbot_hardware_interfaces)
target_link_libraries(cloud_bridge_node luvbot_cloud_proto PkgConfig::MOSQUITTO nlohmann_json::nlohmann_json Threads::Threads)
install(TARGETS cloud_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_brain_mapping.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch proto DESTINATION share/${PROJECT_NAME})
# Device credentials are local, ignored by Git, and readable only by the installation owner.
install(DIRECTORY certs/ DESTINATION share/${PROJECT_NAME}/certs
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE
  DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
  FILES_MATCHING PATTERN "*.crt" PATTERN "*.key")
ament_package()
