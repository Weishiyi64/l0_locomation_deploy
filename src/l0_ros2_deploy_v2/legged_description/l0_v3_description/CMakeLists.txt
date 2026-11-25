cmake_minimum_required(VERSION 3.5)

project(l0_v3_description)

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

install(FILES urdf/l0_v3_description.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME l0_v3.urdf)

install(FILES mjcf/l0_v3_description.xml
  DESTINATION share/${PROJECT_NAME}/mjcf
  RENAME l0_v3.xml)

ament_package()
