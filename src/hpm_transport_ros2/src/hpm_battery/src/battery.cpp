/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause */
#include "hpm_battery/battery.hpp"

namespace hpmBattery {
HpmBattery::HpmBattery(rclcpp::Node &node) : node_(node)
{
  publisher_ = node_.create_publisher<luvbot_hardware_interfaces::msg::BatteryState>(
    "/battery/state", rclcpp::SensorDataQoS().keep_last(5));
  RCLCPP_INFO(node_.get_logger(), "Battery snapshot decoder enabled");
}

void HpmBattery::handle_battery_snapshot(const frameTransport::BatterySnapshot &snapshot)
{
  auto message = decode_snapshot(snapshot);
  message.header.stamp = node_.now();
  publisher_->publish(message);
}
}
