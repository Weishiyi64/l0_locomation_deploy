#include <rclcpp/rclcpp.hpp>

#include "kvms_battery/battery_driver.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("battery_node");

  RCLCPP_INFO(node->get_logger(), "Starting KVMS Battery Monitor Node...");

  kvms_battery::BatteryDriver driver(node);

  if (!driver.init())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize battery driver");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Battery monitor node running...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  RCLCPP_INFO(node->get_logger(), "Battery monitor node terminated");
  return 0;
}
