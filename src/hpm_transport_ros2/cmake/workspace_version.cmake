# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

if(NOT _AMENT_PACKAGE_NAME)
  ament_package_xml()
endif()

set(workspace_version_file "${CMAKE_CURRENT_LIST_DIR}/../VERSION")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${workspace_version_file}")
file(READ "${workspace_version_file}" APP_VERSION)
string(STRIP "${APP_VERSION}" APP_VERSION)

if(NOT APP_VERSION MATCHES "^(0|[1-9][0-9]*)[.](0|[1-9][0-9]*)[.](0|[1-9][0-9]*)(-(dev|rc)[.][1-9][0-9]*)?$")
  message(FATAL_ERROR "Invalid ROS 2 workspace version: '${APP_VERSION}'")
endif()
set(workspace_base_version "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")
if(NOT "${${PROJECT_NAME}_VERSION}" STREQUAL "${workspace_base_version}")
  message(FATAL_ERROR
    "${PROJECT_NAME}/package.xml must match the base version in VERSION (${workspace_base_version}). "
    "Use scripts/set_version.py to update the workspace version.")
endif()

set(version_template_dir "${CMAKE_CURRENT_LIST_DIR}")
configure_file("${version_template_dir}/version.py.in" "${CMAKE_CURRENT_BINARY_DIR}/version.py" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/version.py" DESTINATION "lib/${PROJECT_NAME}")

function(ros2_add_version_header target)
  set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
  configure_file("${version_template_dir}/version.hpp.in"
    "${generated_dir}/${PROJECT_NAME}/version.hpp" @ONLY)
  target_include_directories(${target} PRIVATE "${generated_dir}")
endfunction()
