cmake_minimum_required(VERSION 3.5)
project(l0_v1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf mjcf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
