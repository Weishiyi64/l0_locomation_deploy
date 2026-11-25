cmake_minimum_required(VERSION 3.8)
project(system_command_arbiter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luvbot_hardware_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(motor_mapping src/motor_mapping.cpp)
target_include_directories(motor_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_mapping luvbot_hardware_interfaces)
target_link_libraries(motor_mapping yaml-cpp)

add_executable(system_command_arbiter_node src/system_command_arbiter_node.cpp)
ament_target_dependencies(system_command_arbiter_node rclcpp luvbot_hardware_interfaces std_msgs)
target_link_libraries(system_command_arbiter_node motor_mapping)

add_executable(mock_motor_driver_node src/mock_motor_driver_node.cpp)
ament_target_dependencies(mock_motor_driver_node rclcpp luvbot_hardware_interfaces)

install(TARGETS motor_mapping
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(TARGETS system_command_arbiter_node mock_motor_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(luvbot_hardware_interfaces yaml_cpp_vendor)
ament_export_libraries(motor_mapping)

ament_package()
