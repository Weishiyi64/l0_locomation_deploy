cmake_minimum_required(VERSION 3.5)
project(l0_v5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf mjcf launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
