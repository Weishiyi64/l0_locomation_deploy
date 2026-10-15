// Copyright information
//
// Copyright (c) 2022, Qiayuan (Original author)
// Copyright (c) 2024, Shibo Mu (Modifications)
// All rights reserved.

#include <legged_controllers/LeggedControlloop.h>

#include "legged_real/LeggedReal.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto legged_node =
      std::make_shared<rclcpp::Node>("legged_real", rclcpp::NodeOptions().allow_undeclared_parameters(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(legged_node);

  try
  {
    std::shared_ptr<legged::LeggedReal> legged_real = std::make_shared<legged::LeggedReal>(legged_node);

    if (!legged_real->robot_configure(legged_node, ""))
    {
      RCLCPP_ERROR(legged_node->get_logger(), "Failed to configure legged_real hardware interface");
      return 1;
    }

    legged::LeggedControlloop control_loop(legged_node, legged_real);

    executor.spin();
  } catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(legged_node->get_logger(), "Error in the hardware interface:\n\t" << e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
