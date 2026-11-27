#pragma once

/**
 * @file V2/OdomEntry.hpp
 * @brief Odometry payload entry, payload generation 2 (125 bytes, cov[18]).
 * @details One generation of the odometry payload layout, exactly as the
 *          firmware serializes it before transmission. The payload
 *          generation - the low nibble of the frame header version byte -
 *          is the SOLE routing key: version 0x02 frames are parsed with the
 *          structs in this V2 folder and nothing else. The cov[36] layout
 *          does NOT belong here - it is generation 3 (V3/OdomEntry.hpp);
 *          firmware sending it must stamp version 0x03.
 *
 *          The struct IS the layout: field order, sizes and offsets are
 *          declared once and verified by the compiler (packed + static
 *          asserts). Conversion to the generation-independent public
 *          OdinOdomData is declared right below, so this file is complete
 *          and self-contained; nothing outside this folder may reinterpret
 *          generation-2 odom bytes.
 *
 *          Evolution: a generation NEVER changes its layout. When firmware
 *          needs to, it bumps the payload generation and the new layout gets
 *          its own folder (V4, V5, ...) with its own entry structs - this
 *          file is then never touched again.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "odin_lidar_def.h"  // OdinOdomData: public, generation-independent

namespace odin {
namespace sdk {
namespace protocol {
namespace v2 {

#pragma pack(push, 1)
/// One odometry entry in a generation-2 payload (cov[18], type @ 124).
struct OdomEntry {
  float pos[3];
  float orient[4];  // quaternion
  float linear_velocity[3];
  float angular_velocity[3];
  float cov[18];  // 6x6 covariance, first 18 entries (position rows)
  uint8_t type;   // OdomSourceType
};
#pragma pack(pop)

static_assert(sizeof(OdomEntry) == 125, "Generation-2 odom entry must be 125 bytes");
static_assert(offsetof(OdomEntry, cov) == 52, "Generation-2 odom entry field order");
static_assert(offsetof(OdomEntry, type) == 124, "Generation-2 odom entry field order");

// Prefix-share lockstep contract with the public struct: generation 2 shares
// the pos..cov field offsets with OdinOdomData (the generation-3 covariance
// extension grew cov in place; everything before cov is unchanged). Same
// start offset + same next-field offset => same width, so these asserts lock
// the prefix fields field-by-field - if the public struct ever drifts, they
// fire in the same commit and force a review of the conversion below.
static_assert(offsetof(OdinOdomData, orient) == offsetof(OdomEntry, orient),
              "Public OdinOdomData drifted from the generation-2 prefix layout");
static_assert(offsetof(OdinOdomData, linear_velocity) ==
                  offsetof(OdomEntry, linear_velocity),
              "Public OdinOdomData drifted from the generation-2 prefix layout");
static_assert(offsetof(OdinOdomData, angular_velocity) ==
                  offsetof(OdomEntry, angular_velocity),
              "Public OdinOdomData drifted from the generation-2 prefix layout");
static_assert(offsetof(OdinOdomData, cov) == offsetof(OdomEntry, cov),
              "Public OdinOdomData drifted from the generation-2 prefix layout");

/// Adapt one generation-2 entry to the public struct. Copy length is always
/// the SOURCE field width (sizeof(src.x)): the copy reads exactly what the
/// source holds and can never over-read, whatever happens to the public
/// struct (the prefix asserts above make a drift a compile error anyway).
/// The orientation covariance rows (entries 18..35) are not in this layout
/// and are zero-filled, so consumers always see the full 6x6 layout.
///
/// The tail parameters mirror the newest generation's conversion signature
/// (the frame-assembler template calls both uniformly); the STRICT
/// generation-2 route always passes tail_bytes = 0, so they are unused here
/// and tail-extension fields keep their defaults.
inline void ConvertToPublic(const OdomEntry& src, const uint8_t* /*tail*/,
                            size_t /*tail_bytes*/, OdinOdomData& dst) {
  std::memcpy(dst.pos, src.pos, sizeof(src.pos));
  std::memcpy(dst.orient, src.orient, sizeof(src.orient));
  std::memcpy(dst.linear_velocity, src.linear_velocity, sizeof(src.linear_velocity));
  std::memcpy(dst.angular_velocity, src.angular_velocity, sizeof(src.angular_velocity));
  std::memcpy(dst.cov, src.cov, sizeof(src.cov));
  std::memset(dst.cov + 18, 0, sizeof(dst.cov) - sizeof(src.cov));
  dst.type = src.type;
}

}  // namespace v2
}  // namespace protocol
}  // namespace sdk
}  // namespace odin
