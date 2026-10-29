cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorDebugFrame.msg"
  "msg/MotorEnableCommand.msg"
  "msg/MotorRoute.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
  "msg/MotorTarget.msg"
  "srv/MotorSetZero.srv"
  "srv/MotorCalibrateId.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
