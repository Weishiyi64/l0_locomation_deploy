/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_tactile/tactile.hpp"

#include <gtest/gtest.h>

TEST(Tactile, DecodesTwoBigEndianGroups)
{
  frameTransport::Frame frame;
  frame.msg_id = hpmTactile::kMsgIdTactile;
  frame.payload = {
    0x00, 0x01, 0x01, 0x02, 0x12, 0x34, 0xFF, 0xFF,
    0x00, 0x02, 0x03, 0x04, 0x56, 0x78, 0xAB, 0xCD};

  hpmTactile::TactileSamples samples{};
  ASSERT_EQ(hpmTactile::decode_frame(frame, samples), 2U);
  EXPECT_EQ(samples[0].sensor_group, 0U);
  EXPECT_EQ(samples[0].adc, (std::array<uint16_t, 4>{1U, 0x0102U, 0x1234U, 0xFFFFU}));
  EXPECT_EQ(samples[1].sensor_group, 1U);
  EXPECT_EQ(samples[1].adc, (std::array<uint16_t, 4>{2U, 0x0304U, 0x5678U, 0xABCDU}));

  frame.msg_id = 0x2FU;
  EXPECT_EQ(hpmTactile::decode_frame(frame, samples), 0U);

  frame.msg_id = 0x21U;
  ASSERT_EQ(hpmTactile::decode_frame(frame, samples), 2U);
  EXPECT_EQ(samples[0].sensor_group, 2U);
  EXPECT_EQ(samples[1].sensor_group, 3U);

  frame.msg_id = 0x22U;
  ASSERT_EQ(hpmTactile::decode_frame(frame, samples), 1U);
  EXPECT_EQ(samples[0].sensor_group, 4U);

  frame.msg_id = 0x30U;
  EXPECT_EQ(hpmTactile::decode_frame(frame, samples), 0U);
}
