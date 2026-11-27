/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HPM_IMU__IMU_HPP_
#define HPM_IMU__IMU_HPP_

#include "hpm_transport/frame.hpp"
#include "hpm_imu/yesense_frame_parser.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mutex>
#include <string>

namespace hpmImu
{

/**
 * IMU frame consumer owned by the system node.
 *
 * This class does not own a transport or a ROS node. The system node passes
 * every received HPM frame to handle_frame().
 */
class HpmImu
{
public:
  explicit HpmImu(rclcpp::Node & node);

  void handle_frame(const frameTransport::Frame & frame);
  void reset();

private:
  rclcpp::Node & node_;
  YesenseFrameParser parser_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;
  std::string frame_id_;
  mutable std::mutex mutex_;
};

}  // namespace hpmImu

#endif  // HPM_IMU__IMU_HPP_
