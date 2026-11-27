/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_transport/frame.hpp"
#include "hpm_imu/yesense_frame_parser.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace
{

frameTransport::Frame make_frame(
  uint8_t msg_id,
  const std::array<uint8_t, frameTransport::kFramePayloadLen> & payload)
{
  frameTransport::Frame frame;
  frame.msg_id = msg_id;
  frame.payload = payload;
  return frame;
}

constexpr std::array<uint8_t, frameTransport::kFramePayloadLen> kAcceleration{
  0xFF, 0xD0, 0x6B, 0x4D, 0x00, 0x5B, 0xD6, 0x97,
  0x00, 0x6B, 0x45, 0xB5, 0x00, 0x00, 0x00, 0x00};
constexpr std::array<uint8_t, frameTransport::kFramePayloadLen> kGyroscope{
  0x00, 0x1C, 0xDC, 0x73, 0xFF, 0xFB, 0xD1, 0x5C,
  0x00, 0x02, 0x57, 0xFE, 0x00, 0x00, 0x00, 0x00};
constexpr std::array<uint8_t, frameTransport::kFramePayloadLen> kEuler{
  0x01, 0x1D, 0x37, 0x4C, 0x02, 0x6B, 0x62, 0x68,
  0x02, 0xC8, 0x7A, 0x8C, 0x00, 0x00, 0x00, 0x00};
constexpr std::array<uint8_t, frameTransport::kFramePayloadLen> kQuaternion{
  0x00, 0x0D, 0x4E, 0x4E, 0x00, 0x03, 0xDF, 0xBA,
  0x00, 0x04, 0x34, 0x16, 0x00, 0x04, 0xCE, 0x9A};

}  // namespace

TEST(YesenseFrameParser, DecodesBoardFrameOrderAndScaling)
{
  hpmImu::YesenseFrameParser parser;
  hpmImu::ImuSample sample;

  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdAcceleration, kAcceleration), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdGyroscope, kGyroscope), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdEuler, kEuler), sample));
  EXPECT_TRUE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));

  EXPECT_NEAR(sample.acceleration_mps2[0], -3.118259, 1e-9);
  EXPECT_NEAR(sample.acceleration_mps2[1], 6.018711, 1e-9);
  EXPECT_NEAR(sample.acceleration_mps2[2], 7.030197, 1e-9);
  EXPECT_NEAR(sample.angular_velocity_deg_s[0], 1.891443, 1e-9);
  EXPECT_NEAR(sample.angular_velocity_deg_s[1], -0.274084, 1e-9);
  EXPECT_NEAR(sample.angular_velocity_deg_s[2], 0.153598, 1e-9);
  EXPECT_NEAR(sample.euler_deg[0], 18.691916, 1e-9);
  EXPECT_NEAR(sample.euler_deg[1], 40.591976, 1e-9);
  EXPECT_NEAR(sample.euler_deg[2], 46.693004, 1e-9);
  EXPECT_NEAR(sample.quaternion[0], 0.872014, 1e-9);
  EXPECT_NEAR(sample.quaternion[1], 0.253882, 1e-9);
  EXPECT_NEAR(sample.quaternion[2], 0.275478, 1e-9);
  EXPECT_NEAR(sample.quaternion[3], 0.315034, 1e-9);

  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));
}

TEST(YesenseFrameParser, RequiresQuaternionTriggerAndAllDataTypes)
{
  hpmImu::YesenseFrameParser parser;
  hpmImu::ImuSample sample;

  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdAcceleration, kAcceleration), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdGyroscope, kGyroscope), sample));
  EXPECT_FALSE(parser.feed(make_frame(0x50U, kEuler), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdEuler, kEuler), sample));
  EXPECT_TRUE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));
}

TEST(YesenseFrameParser, SupportsOutOfOrderFrames)
{
  hpmImu::YesenseFrameParser parser;
  hpmImu::ImuSample sample;

  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdEuler, kEuler), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdAcceleration, kAcceleration), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));
  EXPECT_FALSE(parser.feed(make_frame(hpmImu::kMsgIdGyroscope, kGyroscope), sample));
  EXPECT_TRUE(parser.feed(make_frame(hpmImu::kMsgIdQuaternion, kQuaternion), sample));
}
