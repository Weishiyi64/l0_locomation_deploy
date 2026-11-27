#pragma once

/**
 * @file OdinProtocol.hpp
 * @brief Odin on-wire protocol frame headers and constants.
 * @details Defines the data frame header (protocol v2, 32B naturally aligned) and the
 *          command frame header, plus the OdinConst size/offset constants derived via
 *          sizeof()/offsetof(). The data wire is v2 (breaking change vs the legacy 27B
 *          header); the command wire is unchanged.
 */

#include <cstddef>  // for offsetof

#include "../IProtocol/IProtocol.hpp"
#include "FastCRC/FastCRC.h"
#include "logger.h"

namespace odin {
namespace sdk {

/* ============================================================================
 * Odin Data Frame Header (wire protocol v2)
 * ============================================================================ */

/**
 * @brief On-wire data frame header (protocol v2, 32 bytes, naturally aligned).
 * @details Prefixes every data chunk (point cloud / image / IMU / odom) on UDP/TCP.
 *          v2 supersedes the legacy 27B header: first byte `sof` is fixed to 0xAD,
 *          second byte `version` carries the data protocol generation. The SDK
 *          is forward compatible: it accepts any frame whose version is >=
 *          OdinConst::kDataVersion (0x02) and parses it with the layouts it
 *          knows; see PeekDataFrameHeader. `payload_len` is the pure payload
 *          size (excludes this header); reassembly is driven by `chunk_index` +
 *          `chunk_count` (+ `flags` bit0 = last_chunk); per-sample IMU timestamps
 *          are reconstructed as `timestamp + j*interval_ns`; the data-plane
 *          `crc32` defaults to 0 (structural fields are used for validation).
 *          Layout is verified by static_asserts below.
 */
struct OdinDataFrameHeader {
  uint8_t sof = 0;             ///< Start of frame marker, fixed 0xAD (OdinConst::kDataSof)
  uint8_t version = 0;         ///< Protocol version byte: one global compatibility epoch (see OdinConst)
  uint8_t data_type = 0;       ///< Channel type (0x00 raw/0x01 slam/0x10-0x11 image/0x20 imu/0x30 odom/0x40 algo/0x41 calib/0x42 telemetry)
  uint8_t flags = 0;           ///< Bit0 = last_chunk; remaining bits reserved
  uint32_t frame_seq = 0;      ///< Frame sequence number
  uint16_t chunk_index = 0;    ///< Zero-based chunk index within the frame
  uint16_t chunk_count = 0;    ///< Total chunk count of the frame
  uint16_t element_count = 0;  ///< Element count in this chunk (points/samples/bytes)
  uint16_t payload_len = 0;    ///< Pure payload byte count of this chunk (excludes header)
  uint64_t timestamp = 0;      ///< Timestamp (ns) of the first element in this chunk
  uint32_t interval_ns = 0;    ///< IMU only: inter-sample interval (ns); 0 for other channels
  uint32_t crc32 = 0;          ///< Payload CRC32 (data plane defaults to 0)
};
static_assert(sizeof(OdinDataFrameHeader) == 32, "OdinDataFrameHeader (v2) must be 32 bytes");
static_assert(offsetof(OdinDataFrameHeader, frame_seq) == 4, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, chunk_index) == 8, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, chunk_count) == 10, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, element_count) == 12, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, payload_len) == 14, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, timestamp) == 16, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, interval_ns) == 24, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, crc32) == 28, "v2 header layout");

/* ============================================================================
 * Odin Command Frame Header
 * ============================================================================
 * Frame format (18-byte header + payload):
 *   [0]      SOF (0xAE)
 *   [1]      Version
 *   [2-3]    Length (payload size)
 *   [4-7]    Sequence (uint32_t)
 *   [8-9]    Command ID
 *   [10]     Command Type (req/ack)
 *   [11]     Send Type (host/device)
 *   [12-13]  CRC16 (header checksum, covers [0-11])
 *   [14-17]  CRC32 (payload checksum)
 *   [18...]  Payload
 */
#pragma pack(push, 1)
struct OdinCommandFrameHeader {
  uint8_t sof;
  uint8_t version;
  uint16_t length;      // payload size
  uint32_t seq;
  uint16_t cmd_id;
  uint8_t cmd_type;
  uint8_t send_type;
  uint16_t crc16;       // header checksum (covers sof to send_type)
  uint32_t crc32;       // payload checksum
};
#pragma pack(pop)

/* ============================================================================
 * Odin Protocol Constants
 * ============================================================================
 * Uses sizeof() and offsetof() for auto-calculation, constants auto-update when struct changes
 */
namespace OdinConst {
constexpr uint8_t kSof = 0xAE;
// Current (newest) command epoch this SDK speaks: default dialect for
// unversioned packets and the newest entry in probe lists. Older epochs
// stay accepted and probed for compatibility (0x02 = first data-wire
// epoch is the frozen floor) - see OdinUdpDiscovery::GetProbeVersions.
constexpr uint8_t kVersion = 0x03;      // control protocol version (subscribe/port decoupled)
constexpr uint8_t kDataSof = 0xAD;     // data frame start-of-frame marker

// Version byte: ONE global compatibility epoch per incompatibility. Any
// incompatible change - the frame header layout, any payload type's base
// layout, command wire semantics - bumps the version by exactly one
// (0x02 -> 0x03 -> ...). Tail-appended fields on the newest payload
// generation do NOT bump it (the lenient tail window covers those).
// Current: 0x02 = first data-wire epoch (32B v2 data header, 18B command
// header, payload generation-2 layouts); 0x03 = odom cov[36]. This
// encoding is FROZEN: every future version must keep sof at byte 0 and
// this version byte at byte 1 with this exact meaning - those two bytes
// are the only thing every version shares.
constexpr uint8_t kDataVersion = 0x02;  // oldest version this SDK supports

// Forward-compat ceiling: the newest version this SDK will even try to
// frame. Versions within (kDataVersion, kMaxForwardCompatVersion] are
// parsed with the layouts this SDK knows (per-type guards drop layouts
// that actually changed); versions above the ceiling are refused - by
// then the SDK deserves an upgrade. The ceiling also bounds the routing
// key below to the version byte itself.
constexpr uint8_t kMaxForwardCompatVersion = 0x0F;

// Routing-key decode: within the accepted range [kDataVersion,
// kMaxForwardCompatVersion] this IS the version byte (the mask only
// defends against unvalidated callers). Kept as a named function so call
// sites state their intent: "which payload-generation layout parses this
// frame".
constexpr uint8_t PayloadGenerationOf(uint8_t version) { return version & 0x0F; }

/// True when the SDK can FRAME this version: it lies within
/// [kDataVersion, kMaxForwardCompatVersion]. Versions newer than the
/// baseline are parsed forward-compatibly (per-type guards drop layouts
/// that changed); versions beyond the ceiling cannot be trusted to share
/// this header layout at all, so they are refused instead of misparsed.
inline bool IsSupportedDataFrameVersion(uint8_t version) {
  if (version > kMaxForwardCompatVersion) {
    // Hint at an SDK upgrade, once per process (function-local statics in an
    // inline function are shared across translation units; a benign race
    // can emit the hint twice at most).
    static bool hinted = false;
    if (!hinted) {
      hinted = true;
      LOG_WARN("Version 0x%02X is beyond this SDK's forward-compat ceiling "
               "0x%02X: frame layout cannot be parsed, dropping frames - "
               "upgrade the SDK for this firmware\n",
               version, kMaxForwardCompatVersion);
    }
    return false;
  }
  return version >= kDataVersion;
}

constexpr uint8_t kCmdTypeReq = 0x00;
constexpr uint8_t kCmdTypeAck = 0x01;
constexpr uint8_t kSendTypeHost = 0x00;

// Auto-calculated size constants (auto-update when struct changes)
constexpr size_t kHeaderSize = sizeof(OdinCommandFrameHeader);
constexpr size_t kDataHeaderSize = sizeof(OdinDataFrameHeader);

// CRC16 checksum range: from sof to send_type (excludes crc16 and crc32)
constexpr size_t kCrc16EndOffset = offsetof(OdinCommandFrameHeader, crc16);
}  // namespace OdinConst

// Compile-time struct size validation
static_assert(sizeof(OdinDataFrameHeader) == 32, "Unexpected data header size (v2)");
static_assert(sizeof(OdinCommandFrameHeader) == 18, "Unexpected command header size");
static_assert(OdinConst::kCrc16EndOffset == 12, "Unexpected CRC16 offset");

/* ============================================================================
 * Odin Data Frame Parsing (single validation point for UDP and TCP)
 * ============================================================================ */

/**
 * @brief Validated view over one received data frame.
 * @details Non-owning: `header` and `payload` point into the caller's buffer and
 *          stay valid only as long as that buffer does. A view obtained from
 *          ParseDataFrame() is guaranteed to have a valid v2 header and a payload
 *          slice fully contained in the input buffer.
 */
struct OdinDataFrameView {
  const OdinDataFrameHeader* header = nullptr;  ///< Frame header (validated sof/version)
  const uint8_t* payload = nullptr;             ///< Payload start (header end)
  size_t payload_size = 0;                      ///< header->payload_len, bounds-checked
};

/**
 * @brief Validate only the fixed-size header of a data frame.
 * @details Used by stream (TCP) reassembly, which must read `payload_len` to know
 *          the total frame length before the payload has fully arrived.
 *
 *          Validation order mirrors the wire: the first two bytes (sof,
 *          version) are the permanent prefix shared by every version:
 *            1. sof must match - else this is not an Odin frame at all;
 *            2. the version must lie within [kDataVersion,
 *               kMaxForwardCompatVersion]: versions beyond the ceiling
 *               cannot be trusted to share this header layout, so the frame
 *               cannot be split at all and is refused (never parsed with a
 *               guessed header layout);
 *            3. newer versions within the ceiling pass and are parsed with
 *               the layouts this SDK knows - per-type geometry checks drop
 *               data types whose layout actually changed (no silent
 *               misparse).
 * @return Header pointer, or nullptr if the buffer is too short, sof is
 *         wrong, or the version is outside the supported range
 *         [kDataVersion, kMaxForwardCompatVersion].
 */
inline const OdinDataFrameHeader* PeekDataFrameHeader(const uint8_t* data, size_t length) {
  if (data == nullptr || length < OdinConst::kDataHeaderSize) return nullptr;
  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  if (header->sof != OdinConst::kDataSof) return nullptr;
  if (!OdinConst::IsSupportedDataFrameVersion(header->version)) return nullptr;
  return header;
}

/**
 * @brief Validate a complete data frame and expose it as an OdinDataFrameView.
 * @return true if the header is valid and the declared payload fits within `length`.
 */
inline bool ParseDataFrame(const uint8_t* data, size_t length, OdinDataFrameView* out) {
  if (out == nullptr) return false;
  const OdinDataFrameHeader* header = PeekDataFrameHeader(data, length);
  if (header == nullptr) return false;
  const size_t payload_size = header->payload_len;
  if (OdinConst::kDataHeaderSize + payload_size > length) return false;
  out->header = header;
  out->payload = data + OdinConst::kDataHeaderSize;
  out->payload_size = payload_size;
  return true;
}

/* ============================================================================
 * OdinPacket - Concrete IPacket Implementation
 * ============================================================================
 *
 * Compatible with existing OdinCommandPacket structure:
 *   - version, length, seq, cmd_id, cmd_type, send_type, crc16, crc32, payload
 */

class OdinPacket : public IPacket {
 public:
  OdinPacket() = default;

  // Construct from existing data (convenience for migration from legacy code)
  OdinPacket(uint16_t cmd_id, uint32_t seq, uint8_t cmd_type, uint8_t send_type,
             const uint8_t* payload = nullptr, size_t payload_size = 0)
      : seq_(seq), cmd_id_(cmd_id), cmd_type_(cmd_type), send_type_(send_type) {
    if (payload && payload_size > 0) {
      payload_.assign(payload, payload + payload_size);
    }
  }

  // IPacket interface implementation
  uint32_t GetCommandId() const override { return cmd_id_; }
  uint32_t GetSequence() const override { return seq_; }
  const uint8_t* GetPayload() const override {
    return payload_.empty() ? nullptr : payload_.data();
  }
  size_t GetPayloadSize() const override { return payload_.size(); }

  void SetCommandId(uint32_t id) override { cmd_id_ = static_cast<uint16_t>(id); }
  void SetSequence(uint32_t seq) override { seq_ = seq; }
  void SetPayload(const uint8_t* data, size_t size) override {
    if (data && size > 0) {
      payload_.assign(data, data + size);
    } else {
      payload_.clear();
    }
  }

  uint32_t GetAttribute(uint32_t key) const override {
    switch (key) {
      case PacketAttr::kCmdType:
        return cmd_type_;
      case PacketAttr::kSendType:
        return send_type_;
      case PacketAttr::kVersion:
        return version_;
      default:
        return 0;
    }
  }

  void SetAttribute(uint32_t key, uint32_t value) override {
    switch (key) {
      case PacketAttr::kCmdType:
        cmd_type_ = static_cast<uint8_t>(value);
        break;
      case PacketAttr::kSendType:
        send_type_ = static_cast<uint8_t>(value);
        break;
      case PacketAttr::kVersion:
        version_ = static_cast<uint8_t>(value);
        break;
    }
  }

  // Odin-specific accessors (convenient direct access)
  uint8_t GetCmdType() const { return cmd_type_; }
  uint16_t GetCmdId() const { return cmd_id_; }

 private:
  // 0 = epoch unset: Pack falls back to the protocol instance's dialect.
  // Populated by Parse with the frame's version byte.
  uint8_t version_ = 0;
  uint32_t seq_ = 0;
  uint16_t cmd_id_ = 0;
  uint8_t cmd_type_ = OdinConst::kCmdTypeReq;
  uint8_t send_type_ = OdinConst::kSendTypeHost;
  std::vector<uint8_t> payload_;
};

/* ============================================================================
 * OdinProtocolV1 - Concrete IProtocol Implementation
 * ============================================================================
 *
 * Frame format (18-byte header + payload):
 *   [0]     SOF (0xAE)
 *   [1]     Version
 *   [2-3]   Length (payload size)
 *   [4-7]   Sequence (uint32_t)
 *   [8-9]   Command ID
 *   [10]    Command Type (req/ack)
 *   [11]    Send Type (host/device)
 *   [12-13] CRC16 (header checksum)
 *   [14-17] CRC32 (payload checksum)
 *   [18...] Payload
 */

class OdinProtocolV1 : public IProtocol {
 public:
  // The instance speaks `version` (its command dialect) on every packed
  // frame whose packet does not carry an explicit non-zero epoch attribute.
  // Parse stays lenient: any well-formed frame is accepted regardless of
  // its version byte.
  explicit OdinProtocolV1(uint8_t version = OdinConst::kVersion) : version_(version) {}

  bool Pack(const IPacket& packet, std::vector<uint8_t>& output) override;

  int Parse(const uint8_t* buffer, size_t length, std::unique_ptr<IPacket>& packet) override;

  std::unique_ptr<IPacket> CreatePacket() override {
    return std::unique_ptr<IPacket>(new OdinPacket());
  }

  // Legacy API compatibility (kept for migration convenience)
  bool Pack(uint16_t cmd_id, uint32_t seq, uint8_t cmd_type, uint8_t send_type,
            const uint8_t* payload, size_t payload_size, std::vector<uint8_t>& output) {
    OdinPacket pkt(cmd_id, seq, cmd_type, send_type, payload, payload_size);
    return Pack(pkt, output);
  }

 private:
  uint8_t version_;
  mutable FastCRC16 crc16_;
  mutable FastCRC32 crc32_;
};

/* ============================================================================
 * Protocol version routing
 * ========================================================================= */

/**
 * @brief Protocol version registry and negotiator.
 * @details Single decision point mapping the protocol version reported by a
 *          device (DeviceQuery discovery ACK, DiscoveredDevice::protocol_version)
 *          to a protocol implementation. Transport-agnostic: applies to
 *          any bearer (TCP/UDP today, USB or wireless later).
 *
 *          The command version byte uses the same single-axis epoch as
 *          the data plane (see OdinConst). Negotiation policy: speak the
 *          device's own epoch - firmware that validates the header
 *          version accepts only its own epoch, while lenient firmware
 *          accepts older dialects too, so echoing the device's epoch
 *          serves both. The ceiling bounds the echoed value to epochs
 *          whose frame layout this SDK knows.
 *
 *          To register a new command generation, route it to its
 *          implementation here. Callers stay untouched.
 *
 * @param version Protocol version from the discovery ACK.
 * @return Negotiated protocol instance, or nullptr when the version is
 *         beyond the forward-compat ceiling or older than the oldest
 *         registered implementation.
 */
inline std::unique_ptr<IProtocol> CreateOdinProtocol(uint8_t version) {
  if (version > OdinConst::kMaxForwardCompatVersion) {
    return nullptr;  // unknown future wire: refuse fast, never guess
  }
  // 0x02 is the frozen first epoch (the single-axis floor shared with the
  // data plane): every device at or above it is accepted and served in its
  // own dialect.
  if (version >= OdinConst::kDataVersion) {
    // Speak the device's own epoch; the ceiling above already bounds it to
    // epochs whose frame layout this SDK knows. Single implementation
    // today (18B header, subscribe/port decoupled); future generations
    // register their own mappings here.
    return std::unique_ptr<IProtocol>(new OdinProtocolV1(version));
  }
  // Device version is older than the oldest registered one.
  return nullptr;
}

}  // namespace sdk
}  // namespace odin
