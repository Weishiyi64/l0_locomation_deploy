/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hpm_ultrasonic/ultrasonic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hpmUltrasonic
{
HpmUltrasonic::HpmUltrasonic(rclcpp::Node & node)
: node_(node), last_valid_(std::chrono::steady_clock::now())
{
  frame_id_ = node_.declare_parameter<std::string>("ultrasonic_frame_id", "ultrasonic_link");
  field_of_view_ = node_.declare_parameter<double>("ultrasonic_field_of_view", 0.6981317);
  const int timeout_ms = node_.declare_parameter<int>("ultrasonic_timeout_ms", 500);
  if (frame_id_.empty() || !std::isfinite(field_of_view_) ||
    field_of_view_ <= 0.0 || field_of_view_ > 6.283185307179586 || timeout_ms <= 0)
  {
    throw std::invalid_argument("ultrasonic requires nonempty frame_id, FOV in (0, 2*pi], "
            "and positive timeout_ms");
  }
  timeout_ = std::chrono::milliseconds(timeout_ms);
  publisher_ = node_.create_publisher<sensor_msgs::msg::Range>(
    "/sensor/ultrasonic/range", rclcpp::SensorDataQoS());
  timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(std::max(1, std::min(100, timeout_ms / 2))),
    [this]() {check_timeout();});
}

void HpmUltrasonic::handle_frame(const frameTransport::Frame & frame)
{
  if (frame.msg_id != frameTransport::kMsgIdUltrasonicRaw) {
    return;
  }
  double range_m;
  if (!decode_frame(frame, range_m)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
      "FD07-3 invalid raw frame (metadata, length, checksum or range)");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_valid_ = std::chrono::steady_clock::now();
  if (timed_out_) {
    RCLCPP_INFO(node_.get_logger(), "FD07-3 valid data recovered");
    timed_out_ = false;
  }
  sensor_msgs::msg::Range message;
  message.header.stamp = node_.now();  // Host receipt time, not sensor acquisition time.
  message.header.frame_id = frame_id_;
  message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  message.field_of_view = static_cast<float>(field_of_view_);
  message.min_range = 0.03F;
  message.max_range = kMaxRangeMm / 1000.0F;
  message.range = static_cast<float>(range_m);
  publisher_->publish(message);
}

void HpmUltrasonic::check_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!timed_out_ && std::chrono::steady_clock::now() - last_valid_ >= timeout_) {
    timed_out_ = true;
    RCLCPP_WARN(node_.get_logger(), "FD07-3 valid data timeout (%lld ms)",
      static_cast<long long>(timeout_.count()));
  }
}
}  // namespace hpmUltrasonic
