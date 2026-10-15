cmake_minimum_required(VERSION 3.16)
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
project(kvms_battery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(socketcan_wrapper REQUIRED)
find_package(can_msgs REQUIRED)

add_executable(battery_node
  src/main.cpp
  src/battery_driver.cpp
  src/protocol_parser.cpp
  src/battery_monitor.cpp
)

target_include_directories(battery_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../socketcan_wrapper/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(battery_node
  rclcpp
  std_msgs
  legged_msgs
  socketcan_wrapper
  can_msgs
)

target_link_libraries(battery_node
  socketcan_wrapper::socketcan_wrapper
)

install(TARGETS battery_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
