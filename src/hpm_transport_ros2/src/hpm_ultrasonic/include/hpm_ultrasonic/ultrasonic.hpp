/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HPM_ULTRASONIC__ULTRASONIC_HPP_
#define HPM_ULTRASONIC__ULTRASONIC_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "hpm_ultrasonic/fd07_parser.hpp"

namespace hpmUltrasonic
{
// Owned by hpm_system; no independent node, UART device or transport.
class HpmUltrasonic
{
public:
  explicit HpmUltrasonic(rclcpp::Node & node);
  void handle_frame(const frameTransport::Frame & frame);

private:
  void check_timeout();

  rclcpp::Node & node_;
  std::string frame_id_;
  double field_of_view_;
  std::chrono::milliseconds timeout_;
  std::chrono::steady_clock::time_point last_valid_;
  bool timed_out_{false};
  std::mutex mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace hpmUltrasonic

#endif  // HPM_ULTRASONIC__ULTRASONIC_HPP_
