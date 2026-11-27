#include "hpm_transport/frame.hpp"

#include <cstring>

namespace frameTransport {

namespace {

bool frame_is_motor_msg(uint8_t msg_id) {
  return msg_id >= 0x10U && msg_id <= 0x1FU;
}

uint16_t be16(const uint8_t *p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8U) |
                               static_cast<uint16_t>(p[1]));
}

uint32_t be32(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24U) |
         (static_cast<uint32_t>(p[1]) << 16U) |
         (static_cast<uint32_t>(p[2]) << 8U) | static_cast<uint32_t>(p[3]);
}

void set_be16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8U);
  p[1] = static_cast<uint8_t>(v & 0xFFU);
}

void set_be32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v >> 24U);
  p[1] = static_cast<uint8_t>((v >> 16U) & 0xFFU);
  p[2] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
  p[3] = static_cast<uint8_t>(v & 0xFFU);
}

} // namespace

void frame_init(Frame &frame) {
  frame = Frame{};
  frame.identifier = kFrameIdentifier;
  frame.version = kFrameVersion;
  frame.reserved = 0U;
  frame.port = kFramePortUnused;
  frame.ide = kFrameIdeUnused;
  frame.can_id = kFrameCanIdUnused;
}

uint16_t frame_crc16_modbus(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFU;

  while (len-- > 0U) {
    crc ^= *data++;
    for (uint8_t i = 0U; i < 8U; ++i) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1U);
      }
    }
  }

  return crc;
}

FrameStatus frame_serialize(const Frame &frame,
                            std::array<uint8_t, kFrameWireSize> &out) {
  if (frame.identifier != kFrameIdentifier) {
    return FrameStatus::ErrIdentifier;
  }
  if (frame.version != kFrameVersion) {
    return FrameStatus::ErrVersion;
  }
  if (frame_is_motor_msg(frame.msg_id) && frame.dlc > kFrameDataLen) {
    return FrameStatus::ErrDlc;
  }

  set_be32(&out[0], frame.identifier);
  out[4] = frame.version;
  out[5] = frame.msg_id;
  set_be16(&out[6], frame.heartbeat);
  set_be32(&out[8], frame.timestamp);
  if (frame_is_motor_msg(frame.msg_id)) {
    out[12] = frame.reserved;
    out[13] = frame.port;
    out[14] = frame.ide;
    set_be32(&out[15], frame.can_id);
    out[19] = frame.dlc;
    std::memcpy(&out[20], frame.data.data(), kFrameDataLen);
  } else {
    std::memcpy(&out[12], frame.payload.data(), kFramePayloadLen);
  }

  set_be16(&out[kFrameCrcOffset],
           frame_crc16_modbus(out.data(), kFrameWireSize - 2U));

  return FrameStatus::Ok;
}

FrameStatus frame_deserialize(Frame &frame, const uint8_t *in, size_t len) {
  if (in == nullptr) {
    return FrameStatus::ErrParam;
  }
  if (len < kFrameWireSize) {
    return FrameStatus::ErrLen;
  }

  const uint16_t rx_crc = be16(&in[kFrameCrcOffset]);
  if (frame_crc16_modbus(in, kFrameWireSize - 2U) != rx_crc) {
    return FrameStatus::ErrCrc;
  }

  frame.identifier = be32(&in[0]);
  frame.version = in[4];
  if (frame.version != kFrameVersion) {
    return FrameStatus::ErrVersion;
  }
  frame.msg_id = in[5];
  frame.heartbeat = be16(&in[6]);
  frame.timestamp = be32(&in[8]);
  frame.reserved = in[12];
  frame.port = in[13];
  frame.ide = in[14];
  frame.can_id = be32(&in[15]);
  frame.dlc = in[19];
  std::memcpy(frame.data.data(), &in[20], kFrameDataLen);
  std::memcpy(frame.payload.data(), &in[12], kFramePayloadLen);
  frame.crc = rx_crc;

  if (frame.identifier != kFrameIdentifier) {
    return FrameStatus::ErrIdentifier;
  }
  if (frame_is_motor_msg(frame.msg_id) && frame.dlc > kFrameDataLen) {
    return FrameStatus::ErrDlc;
  }

  return FrameStatus::Ok;
}

} // namespace frameTransport
