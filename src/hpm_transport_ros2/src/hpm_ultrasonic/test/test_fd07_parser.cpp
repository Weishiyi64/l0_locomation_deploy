/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <gtest/gtest.h>
#include "hpm_ultrasonic/fd07_parser.hpp"

namespace
{
frameTransport::Frame sample(uint16_t mm)
{
  frameTransport::Frame frame;
  frame.msg_id = frameTransport::kMsgIdUltrasonicRaw;
  frame.port = frameTransport::kUltrasonicPort;
  frame.dlc = 3U;
  frame.data[0] = static_cast<uint8_t>(mm >> 8U);
  frame.data[1] = static_cast<uint8_t>(mm);
  frame.data[2] = static_cast<uint8_t>(frame.data[0] + frame.data[1]);
  return frame;
}
}

TEST(Fd07Parser, ExampleAndBoundaries)
{
  double range = -1.0;
  for (const auto mm : {30U, 1000U, 1800U}) {
    EXPECT_TRUE(hpmUltrasonic::decode_frame(sample(mm), range));
    EXPECT_DOUBLE_EQ(range, mm / 1000.0);
  }
  for (const auto mm : {0U, 29U, 1801U, 65535U}) {
    range = -1.0;
    EXPECT_FALSE(hpmUltrasonic::decode_frame(sample(mm), range));
    EXPECT_DOUBLE_EQ(range, -1.0);
  }
}

TEST(Fd07Parser, RejectsChecksumLengthAndMetadata)
{
  double range;
  auto frame = sample(1000);
  frame.data[2] ^= 1U;
  EXPECT_FALSE(hpmUltrasonic::decode_frame(frame, range));
  for (uint8_t length = 0U; length <= 9U; ++length) {
    frame = sample(1000);
    frame.dlc = length;
    EXPECT_EQ(hpmUltrasonic::decode_frame(frame, range), length == 3U);
  }
  for (int field = 0; field < 7; ++field) {
    frame = sample(1000);
    switch (field) {
      case 0: frame.msg_id = 0x51U; break;
      case 1: frame.port = 5U; break;
      case 2: frame.reserved = 1U; break;
      case 3: frame.ide = 0U; break;
      case 4: frame.can_id = 0U; break;
      case 5: frame.identifier = 0U; break;
      case 6: frame.version = 1U; break;
    }
    EXPECT_FALSE(hpmUltrasonic::decode_frame(frame, range));
  }
}

TEST(Fd07Parser, IndependentBurstsAndWireCrc)
{
  double range;
  auto frame = sample(1000);
  frame.dlc = 2U;
  EXPECT_FALSE(hpmUltrasonic::decode_frame(frame, range));
  frame.dlc = 1U;
  EXPECT_FALSE(hpmUltrasonic::decode_frame(frame, range));
  EXPECT_TRUE(hpmUltrasonic::decode_frame(sample(1000), range));

  frame = sample(1000);
  // Non-motor serialization uses the complete 16-byte payload.
  frame.payload = {0, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 3,
    0x03, 0xe8, 0xeb, 0, 0, 0, 0, 0};
  std::array<uint8_t, frameTransport::kFrameWireSize> wire;
  ASSERT_EQ(frameTransport::frame_serialize(frame, wire), frameTransport::FrameStatus::Ok);
  frameTransport::Frame decoded;
  ASSERT_EQ(frameTransport::frame_deserialize(decoded, wire.data(), wire.size()),
    frameTransport::FrameStatus::Ok);
  EXPECT_TRUE(hpmUltrasonic::decode_frame(decoded, range));
  EXPECT_DOUBLE_EQ(range, 1.0);
  wire[20] ^= 1U;
  EXPECT_EQ(frameTransport::frame_deserialize(decoded, wire.data(), wire.size()),
    frameTransport::FrameStatus::ErrCrc);
}
