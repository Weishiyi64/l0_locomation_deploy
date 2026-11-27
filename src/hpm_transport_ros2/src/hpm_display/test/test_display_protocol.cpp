// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#include "hpm_display/protocol.hpp"
#include <gtest/gtest.h>
using namespace hpmDisplay;
TEST(DisplayProtocol, StreamBoundariesCrcAndBounds)
{
  const uint8_t vector[]="123456789";
  EXPECT_EQ(crc32(vector,9),0xcbf43926U);
  Frame f{DISPLAY_SET_STATE,3,4,{1,2,3}}; auto raw=encode(f);
  Parser parser; Frame out;
  parser.append(raw.data(),7); EXPECT_FALSE(parser.next(out));
  parser.append(raw.data()+7,raw.size()-7); ASSERT_TRUE(parser.next(out));
  EXPECT_EQ(out.payload,f.payload); EXPECT_EQ(out.session,3U); EXPECT_EQ(out.sequence,4U);
  auto damaged=raw; damaged.back()^=1;
  Bytes stream={0,0,0,'P'}; stream.insert(stream.end(),damaged.begin(),damaged.end());
  stream.insert(stream.end(),raw.begin(),raw.end()); stream.insert(stream.end(),raw.begin(),raw.end());
  parser.append(stream.data(),stream.size()); EXPECT_TRUE(parser.next(out)); EXPECT_TRUE(parser.next(out)); EXPECT_FALSE(parser.next(out));
  f.payload.resize(1101); EXPECT_THROW(encode(f),std::invalid_argument);
  auto bad_size=raw; bad_size[16]=0xff; bad_size[17]=0xff;
  parser.append(bad_size.data(),bad_size.size()); parser.append(raw.data(),raw.size()); EXPECT_TRUE(parser.next(out));
}
TEST(DisplayProtocol, SemanticValidation)
{
  uint8_t battery[]={100,2,0,0}; EXPECT_EQ(display_validate(DISPLAY_BATTERY,battery,4),DISPLAY_OK);
  battery[0]=101; EXPECT_EQ(display_validate(DISPLAY_BATTERY,battery,4),DISPLAY_INVALID);
  uint8_t volume[]={30}; EXPECT_EQ(display_validate(DISPLAY_VOLUME,volume,1),DISPLAY_OK);
  EXPECT_EQ(display_validate(DISPLAY_VOLUME,volume,0),DISPLAY_INVALID);
  EXPECT_EQ(display_validate(0x7777,volume,1),DISPLAY_UNSUPPORTED);
  EXPECT_FALSE(display_event_valid(DISPLAY_VOLUME_REQUEST,101));
  EXPECT_TRUE(display_event_valid(DISPLAY_PROVISION_REQUEST,0));
  uint8_t invalid_name[]={1,2,0xc0,0x80}; EXPECT_EQ(display_validate(DISPLAY_IDENTITY,invalid_name,4),DISPLAY_INVALID);
  EXPECT_EQ(display_validate(0x12,volume,1),DISPLAY_UNSUPPORTED); // retired actuator command
  uint8_t source[]={3};
  EXPECT_EQ(display_validate(DISPLAY_INTERACTION,source,1),DISPLAY_OK);
  EXPECT_EQ(display_validate(DISPLAY_TASK_SUCCESS,source,1),DISPLAY_INVALID);
}
