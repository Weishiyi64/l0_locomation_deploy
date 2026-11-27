#ifndef HPM_TRANSPORT__FRAME_HPP_
#define HPM_TRANSPORT__FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include "hpm_transport/battery_wire.h"

namespace frameTransport
{

/* v2 wire layout: 12-byte common header + 16-byte payload + 2-byte CRC. */
constexpr uint32_t kFrameIdentifier = 0x0A4D5048U;
constexpr uint8_t kFrameVersion = 0x02U;
constexpr uint8_t kFrameDataLen = 8U;
constexpr size_t kFrameHeaderLen = 12U;
constexpr size_t kFramePayloadLen = 16U;
constexpr size_t kFrameCrcLen = 2U;
constexpr size_t kFrameWireSize = 30U;
constexpr size_t kFrameCrcOffset = 28U;

static_assert(kFrameCrcOffset == kFrameHeaderLen + kFramePayloadLen,
              "v2 crc offset must follow the payload");
static_assert(kFrameWireSize == kFrameCrcOffset + kFrameCrcLen,
              "v2 wire size must include the crc");

constexpr uint8_t kFrameIdeStandard = 0U;
constexpr uint8_t kFrameIdeExtended = 1U;
constexpr uint8_t kFrameIdeUnused = 0xFFU;
constexpr uint8_t kFramePortUnused = 0xFFU;
constexpr uint32_t kFrameCanIdUnused = 0xFFFFFFFFU;

// FD07-3 raw UART burst: reserved=0, port=6, ide/can_id unused, dlc=1..8.
constexpr uint8_t kMsgIdUltrasonicRaw = 0x50U;  // category 0x50..0x5F
constexpr uint8_t kUltrasonicPort = 6U;

enum class FrameStatus
{
  Ok = 0,
  ErrParam,
  ErrLen,
  ErrIdentifier,
  ErrVersion,
  ErrDlc,
  ErrCrc,
};

struct Frame
{
  uint32_t identifier{kFrameIdentifier};
  uint8_t version{kFrameVersion};
  uint8_t msg_id{0U};
  uint16_t heartbeat{0U};
  uint32_t timestamp{0U};
  uint8_t reserved{0U};
  uint8_t port{kFramePortUnused};
  uint8_t ide{kFrameIdeUnused};
  uint32_t can_id{kFrameCanIdUnused};
  uint8_t dlc{0U};
  std::array<uint8_t, kFrameDataLen> data{};
  /* v2 non-motor frames use all 16 bytes at wire offsets 12..27. */
  std::array<uint8_t, kFramePayloadLen> payload{};
  uint16_t crc{0U};
};

struct MotorSnapshot
{
  uint32_t control_sequence{0U};
  uint32_t window_start_us{0U};
  uint32_t snapshot_us{0U};
  uint32_t feedback_mask{0U};
  uint8_t finish_reason{0U};
  std::array<Frame, 21U> slots{};
};

using BatterySnapshot = battery_snapshot_t;

void frame_init(Frame & frame);
uint16_t frame_crc16_modbus(const uint8_t * data, size_t len);
FrameStatus frame_serialize(const Frame & frame, std::array<uint8_t, kFrameWireSize> & out);
FrameStatus frame_deserialize(Frame & frame, const uint8_t * in, size_t len);

}  // namespace frameTransport

#endif  // HPM_TRANSPORT__FRAME_HPP_
