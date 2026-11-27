/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_tactile/tactile.hpp"

#include <cstddef>

namespace hpmTactile
{

std::size_t decode_frame(const frameTransport::Frame & frame, TactileSamples & samples)
{
  std::size_t count = 0U;
  std::array<uint8_t, 2> groups{};
  switch (frame.msg_id) {
    case 0x20U:
      groups = {0U, 1U};
      count = 2U;
      break;
    case 0x21U:
      groups = {2U, 3U};
      count = 2U;
      break;
    case 0x22U:
      groups = {4U, 0U};
      count = 1U;
      break;
    default:
      return 0U;
  }

  for (std::size_t sample = 0U; sample < count; ++sample) {
    samples[sample].sensor_group = groups[sample];
    for (std::size_t i = 0U; i < samples[sample].adc.size(); ++i) {
      const std::size_t offset = sample * 8U + i * 2U;
      samples[sample].adc[i] = static_cast<uint16_t>(
        (static_cast<uint16_t>(frame.payload[offset]) << 8U) |
        static_cast<uint16_t>(frame.payload[offset + 1U]));
    }
  }
  return count;
}

HpmTactile::HpmTactile(rclcpp::Node & node)
: node_(node)
{
  adc_publisher_ = node_.create_publisher<luvbot_hardware_interfaces::msg::TactileAdc>(
    "/sensor/tactile/adc_raw", rclcpp::SensorDataQoS());
}

void HpmTactile::handle_frame(const frameTransport::Frame & frame)
{
  TactileSamples samples{};
  const std::size_t count = decode_frame(frame, samples);
  if (count == 0U) {
    return;
  }

  const auto stamp = node_.now();
  for (std::size_t sample = 0U; sample < count; ++sample) {
    luvbot_hardware_interfaces::msg::TactileAdc message;
    message.header.stamp = stamp;
    message.sensor_group = samples[sample].sensor_group;
    message.adc = samples[sample].adc;
    adc_publisher_->publish(message);
  }
}

}  // namespace hpmTactile
