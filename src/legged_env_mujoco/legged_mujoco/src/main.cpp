#include <legged_controllers/LeggedControlloop.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "legged_mujoco/LeggedMujocoSim.h"
#include "legged_mujoco/SimHardwareAdapter.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "legged_mujoco",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  try
  {
    // 1. Create and configure the sim I/O class
    auto mujocoSim = std::make_shared<legged::LeggedMujocoSim>();

    std::string robot_cfg_path;
    if (!node->get_parameter("robot_cfg_path", robot_cfg_path) || robot_cfg_path.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "robot_cfg_path parameter not set");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "robot_cfg_path: " << robot_cfg_path);

    mujocoSim->robot_configure(node, robot_cfg_path);

    // 2. Create SimHardwareAdapter wrapping the sim
    auto adapter = std::make_shared<legged::SimHardwareAdapter>(mujocoSim);
    adapter->robot_configure(node, robot_cfg_path);

    // 3. Start the control loop (same class as real robot)
    legged::LeggedControlloop controlLoop(node, adapter);

    while (rclcpp::ok())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  } catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(node->get_logger(), "Fatal error: " << e.what());
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
