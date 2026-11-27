// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#include "hpm_display/protocol.hpp"
#include <algorithm>
#include <stdexcept>
namespace hpmDisplay {
uint32_t crc32(const uint8_t *data,size_t size)
{
  uint32_t crc=0xffffffffU;
  while(size--) { crc^=*data++; for(unsigned i=0;i<8;++i) crc=(crc>>1)^(0xedb88320U & (0U-(crc&1U))); }
  return ~crc;
}
void put16(Bytes &b,uint16_t v) { b.push_back(v&255); b.push_back(v>>8); }
void put32(Bytes &b,uint32_t v) { put16(b,v&65535); put16(b,v>>16); }
Bytes encode(const Frame &f)
{
  if(f.payload.size()>1100) throw std::invalid_argument("P4 payload exceeds 1100 bytes");
  Bytes b={'P','4','U','P',1,f.type,0,0};
  put32(b,f.session); put32(b,f.sequence); put16(b,f.payload.size()); put16(b,0);
  b.insert(b.end(),f.payload.begin(),f.payload.end()); put32(b,crc32(b.data(),b.size())); return b;
}
void Parser::append(const uint8_t *data,size_t size)
{
  // ponytail: bounded vector resynchronization; a ring is unnecessary at UART921600.
  if(size>4096 || bytes_.size()+size>8192) { bytes_.clear(); throw std::runtime_error("P4 parser backlog overflow"); }
  bytes_.insert(bytes_.end(),data,data+size);
}
bool Parser::next(Frame &out)
{
  static const uint8_t magic[]={'P','4','U','P'};
  for(;;) {
    auto it=std::search(bytes_.begin(),bytes_.end(),magic,magic+4);
    if(it==bytes_.end()) { if(bytes_.size()>3) bytes_.erase(bytes_.begin(),bytes_.end()-3); return false; }
    bytes_.erase(bytes_.begin(),it);
    if(bytes_.size()<20) return false;
    auto *p=bytes_.data(); size_t n=display_u16(p+16);
    if(p[4]!=1 || display_u16(p+6) || display_u16(p+18) || n>1100) { bytes_.erase(bytes_.begin()); continue; }
    if(bytes_.size()<n+24) return false;
    if(crc32(p,n+20)!=display_u32(p+n+20)) { bytes_.erase(bytes_.begin()); continue; }
    out={p[5],display_u32(p+8),display_u32(p+12),Bytes(p+20,p+20+n)};
    bytes_.erase(bytes_.begin(),bytes_.begin()+n+24); return true;
  }
}
}  // namespace hpmDisplay
