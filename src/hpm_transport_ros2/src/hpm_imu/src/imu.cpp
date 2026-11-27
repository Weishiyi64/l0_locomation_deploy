/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_imu/imu.hpp"

#include <cmath>

namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

}  // namespace

namespace hpmImu
{

HpmImu::HpmImu(rclcpp::Node & node)
: node_(node)
{
  frame_id_ = node_.declare_parameter<std::string>("imu_frame_id", "imu_link");

  imu_publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS());
  euler_publisher_ = node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/imu/euler_deg", rclcpp::SensorDataQoS());
}

void HpmImu::handle_frame(const frameTransport::Frame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);

  ImuSample sample;
  if (!parser_.feed(frame, sample)) {
    return;
  }

  const auto stamp = node_.now();

  sensor_msgs::msg::Imu imu_message;
  imu_message.header.stamp = stamp;
  imu_message.header.frame_id = frame_id_;
  imu_message.linear_acceleration.x = sample.acceleration_mps2[0];
  imu_message.linear_acceleration.y = sample.acceleration_mps2[1];
  imu_message.linear_acceleration.z = sample.acceleration_mps2[2];
  imu_message.angular_velocity.x = sample.angular_velocity_deg_s[0] * kDegreesToRadians;
  imu_message.angular_velocity.y = sample.angular_velocity_deg_s[1] * kDegreesToRadians;
  imu_message.angular_velocity.z = sample.angular_velocity_deg_s[2] * kDegreesToRadians;
  imu_message.orientation.x = sample.quaternion[1];
  imu_message.orientation.y = sample.quaternion[2];
  imu_message.orientation.z = sample.quaternion[3];
  imu_message.orientation.w = sample.quaternion[0];
  imu_message.orientation_covariance[0] = -1.0;
  imu_message.angular_velocity_covariance[0] = -1.0;
  imu_message.linear_acceleration_covariance[0] = -1.0;

  geometry_msgs::msg::Vector3Stamped euler_message;
  euler_message.header.stamp = stamp;
  euler_message.header.frame_id = frame_id_;
  euler_message.vector.x = sample.euler_deg[0] * kDegreesToRadians;
  euler_message.vector.y = sample.euler_deg[1] * kDegreesToRadians;
  euler_message.vector.z = sample.euler_deg[2] * kDegreesToRadians;

  imu_publisher_->publish(imu_message);
  euler_publisher_->publish(euler_message);
}

void HpmImu::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  parser_.reset();
}

}  // namespace hpmImu
