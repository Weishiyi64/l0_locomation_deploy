cmake_minimum_required(VERSION 3.5)
project(mujoco)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(legged_msgs REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv8")
  message(STATUS "mujoco: ARM platform detected, skipping x86-only MuJoCo simulator build")
else()
  message(STATUS "mujoco: x86 platform detected, building MuJoCo simulator")

  find_package(Threads REQUIRED)
  find_package(glfw3 REQUIRED)
  # Prefer the modern GLVND targets when both legacy libGL and GLVND exist.
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)

  find_library(MUJOCO_LIB mujoco HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "MuJoCo library not found in ${CMAKE_CURRENT_SOURCE_DIR}/lib. Please place libmujoco.so there.")
  endif()

  include_directories(
    include
    include/simulate
  )

  aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/simulate SIMULATE_SRCS)
  aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sim_interface SIM_INTERFACE_SRCS)

  add_executable(${PROJECT_NAME}_simulate
    src/main.cc
    ${SIMULATE_SRCS}
    ${SIM_INTERFACE_SRCS}
  )

  ament_target_dependencies(${PROJECT_NAME}_simulate
    rclcpp
    realtime_tools
    legged_msgs
  )

  target_link_libraries(${PROJECT_NAME}_simulate
    ${MUJOCO_LIB}
    Threads::Threads
    glfw
    ${glfw3_LIBRARIES}
    OpenGL::GL
  )

  target_compile_options(${PROJECT_NAME}_simulate PRIVATE -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(${PROJECT_NAME}_simulate PRIVATE -fsanitize=address)

  install(TARGETS ${PROJECT_NAME}_simulate
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY launch scripts
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY lib/
    DESTINATION lib
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
