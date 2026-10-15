cmake_minimum_required(VERSION 3.5)
project(legged_deploy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# ---- third party ----
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- Inference backend: TensorRT on ARM/Jetson, OpenVINO on x86 ----
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv8")
  message(STATUS "legged_deploy: ARM/Jetson platform detected -> TensorRT backend")
  add_compile_definitions(USE_TENSORRT)
  find_package(CUDA REQUIRED)
  set(TENSORRT_INCLUDE_DIRS "/usr/include/aarch64-linux-gnu")
  set(TENSORRT_LIBRARIES "/usr/lib/aarch64-linux-gnu/libnvinfer.so")
  set(PLATFORM_INCLUDE_DIRS
    ${TENSORRT_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    "/usr/local/cuda/include"
  )
  set(PLATFORM_LIBS ${TENSORRT_LIBRARIES} ${CUDA_LIBRARIES})
else()
  message(STATUS "legged_deploy: x86 platform detected -> OpenVINO backend")
  add_compile_definitions(USE_OPENVINO)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  set(PLATFORM_INCLUDE_DIRS "")
  set(PLATFORM_LIBS openvino::runtime)
endif()

# Keep your compile definitions (note: running from install may need extra handling)
add_compile_definitions(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_compile_definitions(LEGGED_POLICY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/policy")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)




# -----------------------
# executable
# -----------------------
add_executable(rl_controller
  src/main.cpp
  src/rl_controller.cpp
  src/rl_controller_imitation.cpp
  src/rl_controller_text_motion.cpp
  src/order_mapping.cpp
  src/base/base_rl_controller.cpp
)
ament_target_dependencies(rl_controller
  rclcpp
  std_msgs
  legged_msgs
  sensor_msgs
)
target_include_directories(rl_controller PRIVATE ${PLATFORM_INCLUDE_DIRS})

# Suppress third-party warnings: Eigen NEON memcpy, TensorRT deprecated API
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rl_controller PRIVATE
    -Wno-class-memaccess
    -Wno-deprecated-declarations
  )
endif()

target_link_libraries(rl_controller
  yaml-cpp
  ${PLATFORM_LIBS}
  rt
  pthread
)


# -----------------------
# install
# -----------------------
#install(TARGETS rl_controller ${PROJECT_NAME}
install(TARGETS rl_controller
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# install policy folder if it exists
install(DIRECTORY policy/
  DESTINATION share/${PROJECT_NAME}/policy
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Runtime assets for the self-contained text-to-WBC deployment path.  The
# installed package now carries the base stg25 policy and the minimal
# wbc_deploy toolchain needed by text_motion_wbc_packager.py.
install(
  DIRECTORY resources third_party
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/infer.py
  scripts/text_motion_publish.py
  scripts/text_motion_wbc_packager.py
  scripts/json_to_motion_onnx.py
  scripts/build_trt_engines.sh
  scripts/build_dynamic_wbc_trt.sh
  scripts/build_wbc_motion_from_amp_stand.py
  scripts/wbc_motion_onnx_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs legged_msgs)

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
