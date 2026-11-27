cmake_minimum_required(VERSION 3.8)
project(hpm_system)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/workspace_version.cmake")
find_package(ament_index_cpp REQUIRED)
find_package(hpm_transport REQUIRED)
find_package(hpm_imu REQUIRED)
find_package(hpm_ultrasonic REQUIRED)
find_package(hpm_motor REQUIRED)
find_package(hpm_tactile REQUIRED)
find_package(hpm_battery REQUIRED)
find_package(hpm_display REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luvbot_hardware_interfaces REQUIRED)
find_package(Threads REQUIRED)

add_executable(hpm_system_node src/hpm_system_node.cpp src/motor_can_log.cpp)
ros2_add_version_header(hpm_system_node)
target_link_libraries(hpm_system_node
  hpm_transport::hpm_transport
  hpm_imu::hpm_imu
  hpm_ultrasonic::hpm_ultrasonic
  hpm_motor::hpm_motor
  hpm_tactile::hpm_tactile_lib
  hpm_battery::hpm_battery
  hpm_display::hpm_display
)
target_link_libraries(hpm_system_node Threads::Threads)
ament_target_dependencies(hpm_system_node ament_index_cpp rclcpp luvbot_hardware_interfaces)
target_compile_features(hpm_system_node PRIVATE cxx_std_17)

install(
  TARGETS hpm_system_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(rcl REQUIRED)
  find_package(std_msgs REQUIRED)
  add_library(display_publish_fault SHARED test/display_publish_fault.cpp)
  ament_target_dependencies(display_publish_fault rcl std_msgs)
  target_link_libraries(display_publish_fault ${CMAKE_DL_LIBS})
  ament_add_test(test_display_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_display_runtime.py"
      "$<TARGET_FILE:hpm_system_node>"
    TIMEOUT 100
  )
  ament_add_test(test_display_system_launch
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_display_system_launch.py"
      "$<TARGET_FILE:display_publish_fault>"
    TIMEOUT 55
  )
  ament_add_test(test_motor_rates
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_rates.py"
    TIMEOUT 50
  )
  ament_add_test(test_realtime_launch
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_realtime_launch.py"
  )
  ament_add_test(test_motor_can_log
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_can_log.py"
    TIMEOUT 70
  )
  ament_add_test(test_ultrasonic_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ultrasonic_runtime.py"
      "$<TARGET_FILE:hpm_system_node>"
    ENV ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1
    TIMEOUT 40
  )
  ament_add_test(test_motor_set_zero_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_set_zero_runtime.py"
      "$<TARGET_FILE:hpm_system_node>"
    TIMEOUT 40
  )
  ament_add_test(test_read_ls_error_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_read_ls_error_runtime.py"
      "$<TARGET_FILE:hpm_system_node>"
    TIMEOUT 60
  )
  ament_add_test(test_battery_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_battery_runtime.py"
      "$<TARGET_FILE:hpm_system_node>"
    ENV ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1
    TIMEOUT 30
  )
endif()

ament_package()
