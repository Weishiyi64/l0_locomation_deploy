cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AppVersion.cmake")
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RequestMotion.action"
  "msg/PolicyObservation.msg"
  "msg/PolicyCommand.msg"
  "msg/MotorDebugFrame.msg"
  "srv/ControlWbcMotion.srv"
  "srv/MotionTransaction.srv"
  "srv/SwitchPolicy.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
