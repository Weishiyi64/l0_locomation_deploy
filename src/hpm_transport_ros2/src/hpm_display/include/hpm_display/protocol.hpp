// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "hpm_display/display_wire.h"
#include <cstdint>
#include <vector>
namespace hpmDisplay {
using Bytes=std::vector<uint8_t>;
struct Frame { uint8_t type=0; uint32_t session=0,sequence=0; Bytes payload; };
uint32_t crc32(const uint8_t *data,size_t size);
Bytes encode(const Frame &frame);
void put16(Bytes &out,uint16_t value);
void put32(Bytes &out,uint32_t value);
class Parser {
public:
  void append(const uint8_t *data,size_t size);
  bool next(Frame &out);
  void reset() { bytes_.clear(); }
private:
  Bytes bytes_;
};
}  // namespace hpmDisplay
