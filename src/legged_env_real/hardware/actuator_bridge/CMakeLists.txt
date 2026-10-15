cmake_minimum_required(VERSION 3.16)
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
project(actuator_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(socketcan_wrapper REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp QUIET)

add_library(actuator_drivers
  src/drivers/dm_driver.cpp
  src/drivers/rs_driver.cpp
  src/drivers/ec_driver.cpp
  src/drivers/motorevo_driver.cpp
  src/driver_registry.cpp
)

target_include_directories(actuator_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(actuator_drivers
  rclcpp
  std_msgs
  can_msgs
  socketcan_wrapper
  Boost
)

# Core bridge library for reuse by other nodes
add_library(actuator_bridge_core
  src/actuator_bridge.cpp
  src/config_loader.cpp
  src/coordinate_mapping.cpp
)

target_include_directories(actuator_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}

)

ament_target_dependencies(actuator_bridge_core
  rclcpp
  std_msgs
  can_msgs
  socketcan_wrapper
  Boost
)

target_link_libraries(actuator_bridge_core actuator_drivers)

if(yaml-cpp_FOUND)
  target_link_libraries(actuator_bridge_core yaml-cpp)
  target_link_libraries(actuator_drivers yaml-cpp)
else()
  message(WARNING "yaml-cpp not found; will build but policy_config won't be available")
endif()

install(TARGETS actuator_drivers actuator_bridge_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
