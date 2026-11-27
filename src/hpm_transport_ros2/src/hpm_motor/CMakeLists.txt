cmake_minimum_required(VERSION 3.8)
project(hpm_motor)
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
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/motor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} hpm_transport::hpm_transport yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp luvbot_hardware_interfaces)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_config test/test_motor_config.cpp)
  target_link_libraries(test_motor_config ${PROJECT_NAME})
  target_compile_definitions(test_motor_config PRIVATE
    HPM_TEST_MOTOR_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motors.yaml")
  target_compile_features(test_motor_config PRIVATE cxx_std_17)
  ament_add_gtest(test_motor_set_zero test/test_motor_set_zero.cpp)
  target_link_libraries(test_motor_set_zero ${PROJECT_NAME})
  target_compile_definitions(test_motor_set_zero PRIVATE
    HPM_TEST_MOTOR_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motors.yaml")
  target_compile_features(test_motor_set_zero PRIVATE cxx_std_17)
  ament_add_test(test_motor_id_calibration_script
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/motor/test_motor_id_calibration.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hpm_transport rclcpp luvbot_hardware_interfaces)

ament_package()
