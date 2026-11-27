/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause */
#ifndef HPM_BATTERY_DECODE_HPP
#define HPM_BATTERY_DECODE_HPP
#include "hpm_transport/frame.hpp"
#include "luvbot_hardware_interfaces/msg/battery_state.hpp"

namespace hpmBattery {
luvbot_hardware_interfaces::msg::BatteryState decode_snapshot(
  const frameTransport::BatterySnapshot &snapshot);
}
#endif
