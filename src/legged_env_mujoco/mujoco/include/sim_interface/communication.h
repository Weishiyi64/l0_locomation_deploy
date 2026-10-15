#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include "legged_msgs/msg/low_cmd.hpp"
#include "legged_msgs/msg/low_state.hpp"

// void HandleLowCmd(const legged_msgs::msg::LowCmd::ConstPtr& msg);
void HandleLowCmd(const legged_msgs::msg::LowCmd::ConstSharedPtr msg);
void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  rclcpp::Publisher<legged_msgs::msg::LowState>::SharedPtr state_pub);
// ros::Publisher& statePublisher_);
