#pragma once

/**
 * @file V3/OdomEntry.hpp
 * @brief Odometry payload entry, payload generation 3 (197 bytes, cov[36]).
 * @details One generation of the odometry payload layout, exactly as the
 *          firmware serializes it before transmission: the covariance was
 *          extended in place from cov[18] to the full 6x6 cov[36] (the
 *          trailing `type` byte moved from offset 124 to 196), which is a
 *          modification of an existing field and therefore bumped the
 *          payload generation from 2 to 3 (version byte 0x03).
 *          Firmware sending this layout must stamp version 0x03; frames
 *          carrying these bytes under 0x02 are rejected by the SDK.
 *
 *          The struct IS the layout (packed + static asserts); the offsetof
 *          chain below locks it field-by-field to the public OdinOdomData
 *          (lockstep contract: a drifting public struct is a compile error
 *          in the same commit). Conversion is field-wise memcpy, mirroring
 *          the struct declaration one-to-one.
 *
 *          Evolution: a generation's BASE layout (through `type`) never
 *          changes - the struct above is a frozen record. When firmware
 *          changes it, the new layout gets its own folder (V4, V5, ...) and
 *          its ONE line in OdomGenerations.def (the manifest that generates
 *          the routing); this file is never touched again. As the NEWEST
 *          generation, V3 additionally tolerates append-only TAIL extensions:
 *          firmware appends fields after `type` without a version bump, and
 *          the conversion below learns them at their frozen tail offsets
 *          (reshuffling the tail shifts downstream offsets into wrong
 *          fields and requires a generation bump instead).
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "odin_lidar_def.h"  // OdinOdomData: public, generation-independent

namespace odin {
namespace sdk {
namespace protocol {
namespace v3 {

#pragma pack(push, 1)
/// One odometry entry in a generation-3 payload (cov[36], type @ 196).
struct OdomEntry {
  float pos[3];
  float orient[4];  // quaternion
  float linear_velocity[3];
  float angular_velocity[3];
  float cov[36];  // 6x6 covariance, row-major over (x, y, z, roll, pitch, yaw)
  uint8_t type;   // OdomSourceType
};
#pragma pack(pop)

static_assert(sizeof(OdomEntry) == 197, "Generation-3 odom entry must be 197 bytes");
static_assert(offsetof(OdomEntry, cov) == 52, "Generation-3 odom entry field order");
static_assert(offsetof(OdomEntry, type) == 196, "Generation-3 odom entry field order");

// Field-by-field layout lock with the public struct (lockstep contract).
// The conversion below copies field-wise with the SOURCE field width, so it
// is safe on its own terms (never over-reads the source); what these
// asserts add is the contract: generation 3 is the public struct's BASE
// layout, so if the public base ever drifts, they fire in the same commit
// and force a review of every generation's conversion. The public struct
// may additionally carry learned tail-extension fields beyond this base -
// every such field is REGISTERED in LearnedFields.def, which generates both
// the learned lines and the kLearnedTail ledger below.
static_assert(offsetof(OdinOdomData, orient) == offsetof(OdomEntry, orient),
              "Public OdinOdomData drifted from the generation-3 payload layout");
static_assert(offsetof(OdinOdomData, linear_velocity) == offsetof(OdomEntry, linear_velocity),
              "Public OdinOdomData drifted from the generation-3 payload layout");
static_assert(offsetof(OdinOdomData, angular_velocity) == offsetof(OdomEntry, angular_velocity),
              "Public OdinOdomData drifted from the generation-3 payload layout");
static_assert(offsetof(OdinOdomData, cov) == offsetof(OdomEntry, cov),
              "Public OdinOdomData drifted from the generation-3 payload layout");
static_assert(offsetof(OdinOdomData, type) == offsetof(OdomEntry, type),
              "Public OdinOdomData drifted from the generation-3 payload layout");
// Learned-tail ledger: kLearnedTail is the SUM of the learned tail-field
// widths, COMPUTED from LearnedFields.def - never a hand-maintained number.
// The assert below is therefore the pairing lock between the two touches of
// learning a field: a field declared at the tail of OdinOdomData in
// odin_lidar_def.h WITHOUT its X(field) line in LearnedFields.def makes the
// public struct larger than base + ledger and fails HERE (never a silently
// always-default field); a manifest line without its declaration fails to
// compile at the offsetof in the conversion below.
constexpr size_t kLearnedTail = 0
#define X_LEARNED(FIELD) + sizeof(OdinOdomData::FIELD)
#include "LearnedFields.def"
#undef X_LEARNED
    ;
static_assert(sizeof(OdinOdomData) == sizeof(OdomEntry) + kLearnedTail,
              "Public OdinOdomData carries a tail field not listed in "
              "LearnedFields.def: add its X_LEARNED(field) line there");

/// Adapt one generation-3 entry to the public struct (field-wise; the public
/// struct is not trivially copyable, so whole-struct memcpy is not allowed).
/// Copy length is always the SOURCE field width (sizeof(src.x)), which reads
/// exactly what the source holds and can never over-read; the equal-width
/// guarantee for the base prefix comes from the assert chain above.
///
/// Same-generation tail extension: `tail`/`tail_bytes` describe the bytes the
/// firmware appended after `type` (newest generation only; the frame
/// assembler guards divisibility, the 64 B cap and base validity). Learned
/// fields are read from the tail at their frozen append-only offsets;
/// fields the tail does not carry keep their default values (out.resize()
/// value-initializes, and the public struct's in-place initializers hold).
inline void ConvertToPublic(const OdomEntry& src, const uint8_t* tail,
                            size_t tail_bytes, OdinOdomData& dst) {
  std::memcpy(dst.pos, src.pos, sizeof(src.pos));
  std::memcpy(dst.orient, src.orient, sizeof(src.orient));
  std::memcpy(dst.linear_velocity, src.linear_velocity, sizeof(src.linear_velocity));
  std::memcpy(dst.angular_velocity, src.angular_velocity, sizeof(src.angular_velocity));
  std::memcpy(dst.cov, src.cov, sizeof(src.cov));
  dst.type = src.type;
  // Learned tail-extension fields are read HERE, expanded from
  // LearnedFields.def (see that file for the append-only contract and the
  // two-touch learning recipe). Each line's tail offset is derived from the
  // PUBLIC struct's own layout (offsetof minus the frozen base), so the
  // manifest order is irrelevant; a field whose bytes are not fully carried
  // by the tail keeps its default (out.resize() value-initializes, and the
  // public struct's in-place initializers hold).
#define X_LEARNED(FIELD)                                                   \
  {                                                                        \
    constexpr size_t kTailOffset =                                         \
        offsetof(OdinOdomData, FIELD) - sizeof(OdomEntry);                 \
    if (tail_bytes >= kTailOffset + sizeof(dst.FIELD))                     \
      std::memcpy(&dst.FIELD, tail + kTailOffset, sizeof(dst.FIELD));      \
  }
#include "LearnedFields.def"
#undef X_LEARNED
}

}  // namespace v3
}  // namespace protocol
}  // namespace sdk
}  // namespace odin
