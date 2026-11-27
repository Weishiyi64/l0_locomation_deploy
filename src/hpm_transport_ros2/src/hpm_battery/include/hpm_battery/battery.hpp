/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause */
#ifndef HPM_BATTERY_BATTERY_HPP
#define HPM_BATTERY_BATTERY_HPP
#include "hpm_battery/decode.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hpmBattery {
class HpmBattery {
public:
  explicit HpmBattery(rclcpp::Node &node);
  void handle_battery_snapshot(const frameTransport::BatterySnapshot &snapshot);
private:
  rclcpp::Node &node_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::BatteryState>::SharedPtr publisher_;
};
}
#endif
