/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hpm_ultrasonic/fd07_parser.hpp"

namespace hpmUltrasonic
{
bool decode_frame(const frameTransport::Frame & frame, double & range_m)
{
  if (frame.identifier != frameTransport::kFrameIdentifier ||
    frame.version != frameTransport::kFrameVersion ||
    frame.msg_id != frameTransport::kMsgIdUltrasonicRaw ||
    frame.reserved != 0U || frame.port != frameTransport::kUltrasonicPort ||
    frame.ide != frameTransport::kFrameIdeUnused ||
    frame.can_id != frameTransport::kFrameCanIdUnused || frame.dlc != 3U)
  {
    return false;
  }
  if (static_cast<uint8_t>(frame.data[0] + frame.data[1]) != frame.data[2]) {
    return false;
  }
  const uint16_t mm = (static_cast<uint16_t>(frame.data[0]) << 8U) | frame.data[1];
  if (mm < 30U || mm > kMaxRangeMm) {
    return false;
  }
  range_m = mm / 1000.0;
  return true;
}
}  // namespace hpmUltrasonic
