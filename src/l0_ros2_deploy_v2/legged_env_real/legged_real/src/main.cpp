// Copyright information
//
// Copyright (c) 2022, Qiayuan (Original author)
// Copyright (c) 2024, Shibo Mu (Modifications)
// All rights reserved.

#include <legged_controllers/LeggedControlloop.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <stdexcept>

#include "legged_real/LeggedReal.h"
#include "rclcpp/rclcpp.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  // Keep DDS alive until the final disable command has been sent.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto legged_node =
      std::make_shared<rclcpp::Node>("legged_real", rclcpp::NodeOptions().allow_undeclared_parameters(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(legged_node);
  auto stop_timer = legged_node->create_wall_timer(std::chrono::milliseconds(20), [&executor]() {
    if (stop_requested) executor.cancel();
  });
  auto legged_real = std::make_shared<legged::LeggedReal>(legged_node);
  std::unique_ptr<legged::LeggedControlloop> control_loop;
  int exit_code = 0;

  try
  {
    if (!legged_real->robot_configure(legged_node, ""))
    {
      throw std::runtime_error("Failed to configure legged_real hardware interface");
    }

    if (!stop_requested)
    {
      control_loop = std::make_unique<legged::LeggedControlloop>(legged_node, legged_real);
      executor.spin();
    }
  } catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(legged_node->get_logger(), "Error in the hardware interface:\n\t" << e.what());
    exit_code = 1;
  }

  stop_timer->cancel();
  // Callbacks have stopped; keep the control thread publishing zero-gain MIT after disable.
  try
  {
    if (!legged_real->disableMotorsBeforeShutdown()) exit_code = 1;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(legged_node->get_logger(), "Shutdown motor disable failed: %s", e.what());
    exit_code = 1;
  }
  // End MIT transmission by terminating the process, without stopping the control loop first.
  std::_Exit(exit_code);
}
