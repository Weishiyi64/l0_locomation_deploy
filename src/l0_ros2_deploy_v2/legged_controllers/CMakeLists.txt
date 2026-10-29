cmake_minimum_required(VERSION 3.5)
project(legged_controllers)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(luvbot_control_interfaces REQUIRED)
find_package(legged_deploy REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- shared library ----
# Pure controller logic: state machine, neural network inference, hardware
# abstraction interface.  Hardware-specific code lives in legged_real.
add_library(${PROJECT_NAME} SHARED
  src/ActionSequence.cpp
  src/ArmSwingController.cpp
  src/FaceFollow.cpp
  src/JointCsvFormat.cpp
  src/HeadScript.cpp
  src/JointCSV.cpp
  src/JoystickImitationActions.cpp
  src/JoystickInput.cpp
  src/JoystickSequenceActions.cpp
  src/ControllerFileUtils.cpp
  src/ControllerBanners.cpp
  src/JointGroups.cpp
  src/PolicyMux.cpp
  src/SequenceInspection.cpp
  src/SequenceRuntime.cpp
  src/JointCommandComposer.cpp
  src/LeggedHardwareInterface.cpp
  src/LeggedControlloop.cpp
  src/LeggedController.cpp
  src/LeggedControllerInit.cpp
  src/LeggedControllerJoystickImitation.cpp
  src/LeggedControllerJoystickSequence.cpp
  src/LeggedControllerRuntime.cpp
  src/LeggedControllerRosIo.cpp
  src/LeggedControllerActions.cpp
  src/LeggedControllerWbc.cpp
  src/LeggedControllerSequenceBridge.cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
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
  luvbot_control_interfaces
  legged_deploy
  ament_index_cpp
)

add_executable(planner_depth_preprocessor src/planner_depth_preprocessor.cpp)
ament_target_dependencies(planner_depth_preprocessor rclcpp sensor_msgs)

add_executable(goal_to_planner src/goal_to_planner.cpp)
target_include_directories(goal_to_planner PRIVATE src)
ament_target_dependencies(goal_to_planner rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs)

# ---- install ----
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner_depth_preprocessor goal_to_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config pcmd_yaml csv action_sequences
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
  scripts/text_wbc_generate
  scripts/check_hardware_ready.bash
  scripts/follow_goal_test.py
  scripts/rosbag_record.sh
  scripts/sequential_launch_real.sh
  scripts/sequential_launch_sim.sh
  scripts/wait_for_hardware_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../texttomotion/ DESTINATION share/${PROJECT_NAME}/texttomotion
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

# ---- export ----
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs urdf
  luvbot_control_interfaces legged_deploy Eigen3
)

ament_package()
