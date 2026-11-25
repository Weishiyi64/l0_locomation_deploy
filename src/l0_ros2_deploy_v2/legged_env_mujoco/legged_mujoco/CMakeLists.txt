cmake_minimum_required(VERSION 3.12)
project(legged_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
