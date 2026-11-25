#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <luvbot_hardware_interfaces/msg/motor_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/motor_command.hpp>
#include <robot_interfaces/msg/motor_state_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

void HandleMotorCommand(const luvbot_hardware_interfaces::msg::MotorCommand::ConstSharedPtr msg);
void HandleLegacyMotorCommand(const robot_interfaces::msg::MotorCommand::ConstSharedPtr msg);
void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  const rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorStateArray>::SharedPtr& motor_pub,
                  const rclcpp::Publisher<robot_interfaces::msg::MotorStateArray>::SharedPtr& legacy_motor_pub,
                  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub);
