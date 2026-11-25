#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <robot_interfaces/msg/motor_command.hpp>
#include <robot_interfaces/msg/motor_state_array.hpp>
#include <luvbot_control_interfaces/msg/sim_support_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

void HandleMotorCommand(const robot_interfaces::msg::MotorCommand::ConstSharedPtr msg);
void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  const rclcpp::Publisher<robot_interfaces::msg::MotorStateArray>::SharedPtr& motor_pub,
                  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub,
                  const rclcpp::Publisher<luvbot_control_interfaces::msg::SimSupportState>::SharedPtr& support_pub);
