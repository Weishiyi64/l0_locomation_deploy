cmake_minimum_required(VERSION 3.16)
project(yesense_imu)

add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
add_definitions(-DPTHREAD_STACK_MIN=16384)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages
set(msg_files
  "msg/YesenseImuEulerAngle.msg"
  "msg/YesenseImuLocation.msg"
  "msg/YesenseImuQuaternion.msg"
  "msg/YesenseImuStatus.msg"
  "msg/YesenseImuUtcTime.msg"
  "msg/YesenseImuVelocity.msg"
  "msg/YesenseImuMasterGnssData.msg"
  "msg/YesenseImuSlaveGnssData.msg"
  "msg/YesenseImuGnssData.msg"
  "msg/YesenseImuGpsData.msg"
  "msg/YesenseImuNavData.msg"
  "msg/YesenseImuInertialData.msg"
  "msg/YesenseImuSensorData.msg"
  "msg/YesenseImuAllData.msg"
  "msg/YesenseImuCmdResp.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

include_directories(
  include
  ./src
)

# Declare a C++ executable
add_executable(yesense_imu_node
  src/yesense_driver.cpp
  src/yesense_node.cpp
  src/analysis_data.cpp
)

target_include_directories(yesense_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(yesense_imu_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  visualization_msgs
  nav_msgs
  serial
)
target_link_libraries(yesense_imu_node Boost::thread)

# Link generated messages (replaces deprecated rosidl_target_interfaces)
rosidl_get_typesupport_target(yesense_imu_ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(yesense_imu_node ${yesense_imu_ts_target})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++17")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g")

install(TARGETS yesense_imu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
