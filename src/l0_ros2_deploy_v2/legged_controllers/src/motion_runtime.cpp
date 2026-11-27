#include "MotionRuntime.h"

#include <iostream>
#include "AppVersion.h"

int main(int argc, char** argv) {
  l0::printAppVersion("motion_runtime");
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    legged::motion::require(arguments.size() == 3 && arguments[1] == "--role" &&
        (arguments[2] == "backend" || arguments[2] == "wbc" || arguments[2] == "txt2motion"),
        "Usage: motion_runtime --role backend|wbc|txt2motion [--ros-args ...]");
    auto node = arguments[2] == "backend" ? legged::motion::makeMotionBackend() :
        legged::motion::makeMotionFrontend(arguments[2]);
    // One callback can wait for the NPU while the other handles cancellation/status.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception& error) {
    std::cerr << "motion_runtime: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
