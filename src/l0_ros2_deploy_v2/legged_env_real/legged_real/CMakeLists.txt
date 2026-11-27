cmake_minimum_required(VERSION 3.16)
project(legged_real)

# Optimize deployment builds while retaining symbols and explicit build-type choices.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AppVersion.cmake")
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(legged_controllers REQUIRED)
find_package(luvbot_hardware_interfaces REQUIRED)
find_package(system_command_arbiter REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
add_definitions(-DPTHREAD_STACK_MIN=16384)

###########
## Build ##
###########

add_executable(topic_hardware_adapter
        src/main.cpp
        src/LeggedReal.cpp
        src/HardwareCommand.cpp
        src/HardwareStatus.cpp
        src/RealHardwareInterface.cpp
)

target_include_directories(topic_hardware_adapter PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(topic_hardware_adapter
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        legged_controllers
        luvbot_hardware_interfaces
        system_command_arbiter
)

target_link_libraries(topic_hardware_adapter
        yaml-cpp
        nlohmann_json::nlohmann_json
)

#############
## Install ##
#############

install(TARGETS topic_hardware_adapter
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
