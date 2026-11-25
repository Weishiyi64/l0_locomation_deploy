cmake_minimum_required(VERSION 3.16)
project(luvbot_cloud_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS proto/luvbot.proto)
add_executable(voltage_mqtt_test_node src/voltage_mqtt_test_node.cpp ${PROTO_SOURCES})
target_compile_features(voltage_mqtt_test_node PRIVATE cxx_std_14)
target_include_directories(voltage_mqtt_test_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
ament_target_dependencies(voltage_mqtt_test_node rclcpp ament_index_cpp)
target_link_libraries(voltage_mqtt_test_node protobuf::libprotobuf PkgConfig::MOSQUITTO)
install(TARGETS voltage_mqtt_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch proto DESTINATION share/${PROJECT_NAME})
# Device credentials are local, ignored by Git, and readable only by the installation owner.
install(DIRECTORY certs/ DESTINATION share/${PROJECT_NAME}/certs
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE
  DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
  FILES_MATCHING PATTERN "*.crt" PATTERN "*.key")
ament_package()
