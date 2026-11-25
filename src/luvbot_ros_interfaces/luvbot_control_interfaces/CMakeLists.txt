cmake_minimum_required(VERSION 3.8)
project(luvbot_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyObservation.msg"
  "msg/PolicyCommand.msg"
  "msg/NavigationGoal.msg"
  "msg/NavigationPlannerCommand.msg"
  "srv/SwitchPolicy.srv"
  "srv/ControlWbcMotion.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
