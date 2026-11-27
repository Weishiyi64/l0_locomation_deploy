/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HPM_TACTILE__TACTILE_HPP_
#define HPM_TACTILE__TACTILE_HPP_

#include "hpm_transport/frame.hpp"
#include "luvbot_hardware_interfaces/msg/tactile_adc.hpp"

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace hpmTactile
{

constexpr uint8_t kMsgIdTactile = 0x20U;

struct TactileSample
{
  uint8_t sensor_group{0U};
  std::array<uint16_t, 4> adc{};
};

using TactileSamples = std::array<TactileSample, 2>;

std::size_t decode_frame(const frameTransport::Frame & frame, TactileSamples & samples);

class HpmTactile
{
public:
  explicit HpmTactile(rclcpp::Node & node);

  void handle_frame(const frameTransport::Frame & frame);

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::TactileAdc>::SharedPtr adc_publisher_;
};

}  // namespace hpmTactile

#endif  // HPM_TACTILE__TACTILE_HPP_
