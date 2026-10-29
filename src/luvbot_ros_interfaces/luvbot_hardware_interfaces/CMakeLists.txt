cmake_minimum_required(VERSION 3.8)
project(luvbot_hardware_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorRoute.msg"
  "msg/JointMotorCommand.msg"
  "msg/ActuatorMotorCommand.msg"
  "msg/MotorCommandCandidate.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
  "srv/SetMotorEnabled.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
