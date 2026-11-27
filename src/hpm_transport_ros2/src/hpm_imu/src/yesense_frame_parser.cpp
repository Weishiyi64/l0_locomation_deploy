/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_imu/yesense_frame_parser.hpp"

namespace hpmImu
{

int32_t YesenseFrameParser::read_be_i32(const uint8_t * data)
{
  const uint32_t raw =
    (static_cast<uint32_t>(data[0]) << 24U) |
    (static_cast<uint32_t>(data[1]) << 16U) |
    (static_cast<uint32_t>(data[2]) << 8U) |
    static_cast<uint32_t>(data[3]);

  const int64_t signed_value = (raw & 0x80000000U) != 0U
    ? static_cast<int64_t>(raw) - 0x100000000LL
    : static_cast<int64_t>(raw);
  return static_cast<int32_t>(signed_value);
}

void YesenseFrameParser::decode_vec3(
  const std::array<uint8_t, frameTransport::kFramePayloadLen> & payload,
  std::array<double, 3> & output)
{
  for (size_t i = 0U; i < output.size(); ++i) {
    output[i] = static_cast<double>(read_be_i32(&payload[i * sizeof(int32_t)])) * kYesenseScale;
  }
}

void YesenseFrameParser::decode_quaternion(
  const std::array<uint8_t, frameTransport::kFramePayloadLen> & payload,
  std::array<double, 4> & output)
{
  for (size_t i = 0U; i < output.size(); ++i) {
    output[i] = static_cast<double>(read_be_i32(&payload[i * sizeof(int32_t)])) * kYesenseScale;
  }
}

bool YesenseFrameParser::feed(const frameTransport::Frame & frame, ImuSample & sample)
{
  switch (frame.msg_id) {
    case kMsgIdAcceleration:
      reset();
      decode_vec3(frame.payload, current_.acceleration_mps2);
      have_acceleration_ = true;
      return false;

    case kMsgIdGyroscope:
      if (!have_acceleration_ || have_gyroscope_) {
        reset();
        return false;
      }
      decode_vec3(frame.payload, current_.angular_velocity_deg_s);
      have_gyroscope_ = true;
      return false;

    case kMsgIdEuler:
      if (!have_gyroscope_ || have_euler_) {
        reset();
        return false;
      }
      decode_vec3(frame.payload, current_.euler_deg);
      have_euler_ = true;
      return false;

    case kMsgIdQuaternion:
      if (!have_euler_ || have_quaternion_) {
        reset();
        return false;
      }
      decode_quaternion(frame.payload, current_.quaternion);
      have_quaternion_ = true;
      break;

    default:
      return false;
  }

  if (!have_acceleration_ || !have_gyroscope_ || !have_euler_ || !have_quaternion_) {
    return false;
  }

  sample = current_;
  reset();
  return true;
}

void YesenseFrameParser::reset()
{
  current_ = ImuSample{};
  have_acceleration_ = false;
  have_gyroscope_ = false;
  have_euler_ = false;
  have_quaternion_ = false;
}

}  // namespace hpmImu
