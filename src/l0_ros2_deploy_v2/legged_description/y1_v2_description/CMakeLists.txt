cmake_minimum_required(VERSION 3.5)

project(y1_v2_description)

find_package(ament_cmake REQUIRED)

foreach(dir meshes urdf mjcf images rviz)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch/display.launch.py)
  install(PROGRAMS launch/display.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
endif()

ament_package()
