cmake_minimum_required(VERSION 3.5)
project(legged_controllers)

# Optimize deployment builds while retaining symbols and explicit build-type choices.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- find dependencies ----
find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AppVersion.cmake")
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(odin_ros_driver_rev1 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

find_package(luvbot_control_interfaces REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(legged_deploy REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- shared library ----
# Pure controller logic: state machine, neural network inference, hardware
# abstraction interface.  Hardware-specific code lives in legged_real.
add_library(${PROJECT_NAME} SHARED
  src/ArmSwingController.cpp
  src/FaceFollow.cpp
  src/JoystickInput.cpp
  src/ControllerFileUtils.cpp
  src/PolicyMux.cpp
  src/JointCommandComposer.cpp
  src/LeggedHardwareInterface.cpp
  src/LeggedControlloop.cpp
  src/LeggedControllerInit.cpp
  src/LeggedControllerStatus.cpp
  src/LeggedControllerRuntime.cpp
  src/LeggedControllerRosIo.cpp
  src/LeggedControllerActions.cpp
  src/LeggedControllerWbc.cpp
  src/LeggedControllerMotion.cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  urdf
  robot_interfaces
  legged_deploy
  ament_index_cpp
)

add_executable(planner_sensor_bridge src/planner_sensor_bridge.cpp)
ament_target_dependencies(planner_sensor_bridge rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros urdf odin_ros_driver_rev1)
target_include_directories(planner_sensor_bridge PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(planner_sensor_bridge yaml-cpp Eigen3::Eigen)

# The depth bridge always requires the robot's GPU OpenCL runtime.
find_package(OpenCL REQUIRED)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/planner_depth_opencl.cl")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/planner_depth_opencl.cl" PLANNER_DEPTH_KERNEL_SOURCE)
configure_file(include/PlannerDepthKernel.h.in generated/PlannerDepthKernel.h @ONLY)
add_library(planner_depth_opencl STATIC src/PlannerDepthOpenCl.cpp)
target_include_directories(planner_depth_opencl PRIVATE include ${EIGEN3_INCLUDE_DIR}
  ${OpenCL_INCLUDE_DIRS} "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(planner_depth_opencl PUBLIC Eigen3::Eigen ${OpenCL_LIBRARIES})
target_link_libraries(planner_sensor_bridge planner_depth_opencl)

add_executable(goal_to_planner src/goal_to_planner.cpp)
add_executable(radar_map_mode src/radar_map_mode.cpp)
ament_target_dependencies(radar_map_mode rclcpp luvbot_control_interfaces odin_ros_driver_rev1)
target_include_directories(goal_to_planner PRIVATE include)
ament_target_dependencies(goal_to_planner rclcpp rclcpp_action luvbot_control_interfaces std_msgs sensor_msgs nav_msgs visualization_msgs)

# Match the inference backend selected by legged_deploy, including explicit overrides.
if(LEGGED_DEPLOY_INFERENCE_BACKEND STREQUAL "RKNN")
  set(TEXT_INFERENCE_SOURCE src/TextMotionRknn.cpp)
  find_path(TEXT_RKNN_INCLUDE_DIR rknn_api.h
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../legged_deploy/third_party/rknn_runtime/include"
      ${legged_deploy_INCLUDE_DIRS})
  if(NOT TEXT_RKNN_INCLUDE_DIR)
    message(FATAL_ERROR "RKNN text inference requires rknn_api.h")
  endif()
else()
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  set(TEXT_INFERENCE_SOURCE src/TextMotionOnnx.cpp)
endif()
add_executable(motion_runtime
  src/motion_runtime.cpp
  src/MotionData.cpp
  src/MotionGateway.cpp
  src/MotionFrontend.cpp
  src/TextMotionPipeline.cpp
  ${TEXT_INFERENCE_SOURCE}
)
target_include_directories(motion_runtime PRIVATE include)
if(LEGGED_DEPLOY_INFERENCE_BACKEND STREQUAL "RKNN")
  target_compile_definitions(motion_runtime PRIVATE USE_RKNN)
  target_include_directories(motion_runtime PRIVATE ${TEXT_RKNN_INCLUDE_DIR})
else()
  target_compile_definitions(motion_runtime PRIVATE USE_OPENVINO)
  target_compile_features(motion_runtime PRIVATE cxx_std_17)
  target_link_libraries(motion_runtime openvino::runtime)
endif()
target_link_libraries(motion_runtime yaml-cpp Eigen3::Eigen nlohmann_json::nlohmann_json Threads::Threads ${CMAKE_DL_LIBS})
ament_target_dependencies(motion_runtime rclcpp rclcpp_action std_msgs robot_interfaces ament_index_cpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(motion_runtime PRIVATE -fno-fast-math -ffp-contract=off)
endif()

# ---- install ----
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner_sensor_bridge goal_to_planner radar_map_mode motion_runtime
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  PATTERN "*.in" EXCLUDE
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(
    DIRECTORY docs
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/check_hardware_ready.py
  scripts/rosbag_record.sh
  scripts/sequential_launch_real.sh
  scripts/sequential_launch_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/launch_processes.bash
  DESTINATION lib/${PROJECT_NAME}
)

# Text models/materials/runtime are installed once by legged_deploy's policy/ rule.

# ---- export ----
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs urdf
  luvbot_control_interfaces robot_interfaces legged_deploy Eigen3
)

ament_package()
