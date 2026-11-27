cmake_minimum_required(VERSION 3.12)
project(legged_mujoco)

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/AppVersion.cmake")

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
