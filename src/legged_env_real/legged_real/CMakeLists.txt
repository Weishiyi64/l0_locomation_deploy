cmake_minimum_required(VERSION 3.16)
project(legged_real)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(legged_controllers REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(actuator_bridge REQUIRED)
find_package(can_msgs REQUIRED)
find_package(socketcan_wrapper REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
add_definitions(-DPTHREAD_STACK_MIN=16384)

###########
## Build ##
###########

add_executable(legged_real
        src/main.cpp
        src/LeggedReal.cpp
        src/HardwareCommand.cpp
        src/RealHardwareInterface.cpp
        src/transmission/transmission_mapping.cpp
        src/transmission/transmission_math.cpp
        src/transmission/transmission_export.cpp
)

target_include_directories(legged_real PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(legged_real
        rclcpp
        legged_controllers
        legged_msgs
        actuator_bridge
        can_msgs
        socketcan_wrapper
        tf2_ros
        tf2_geometry_msgs
)

target_link_libraries(legged_real
        Boost::boost
        Eigen3::Eigen
)

#############
## Install ##
#############

install(TARGETS legged_real
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
