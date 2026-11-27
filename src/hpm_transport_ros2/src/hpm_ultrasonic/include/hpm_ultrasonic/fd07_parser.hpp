/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HPM_ULTRASONIC__FD07_PARSER_HPP_
#define HPM_ULTRASONIC__FD07_PARSER_HPP_

#include "hpm_transport/frame.hpp"

namespace hpmUltrasonic
{
constexpr uint16_t kMaxRangeMm = 1800U;

// Each HPM frame contains one UART idle-delimited burst. No stream reassembly.
bool decode_frame(const frameTransport::Frame & frame, double & range_m);
}  // namespace hpmUltrasonic

#endif  // HPM_ULTRASONIC__FD07_PARSER_HPP_
