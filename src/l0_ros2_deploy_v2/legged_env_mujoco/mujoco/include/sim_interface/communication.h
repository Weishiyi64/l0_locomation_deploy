#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <cstdint>
#include <luvbot_hardware_interfaces/msg/motor_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_enable_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Caller holds recvMutex_; shared by callbacks and the physics control loop.
void ExpireMotorCommand(int64_t now_steady_ns);
void HandleMotorEnable(const luvbot_hardware_interfaces::msg::MotorEnableCommand::ConstSharedPtr msg);
void HandleMotorCommand(const luvbot_hardware_interfaces::msg::MotorCommand::ConstSharedPtr msg);
void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  const rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorStateArray>::SharedPtr& motor_pub,
                  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub);
