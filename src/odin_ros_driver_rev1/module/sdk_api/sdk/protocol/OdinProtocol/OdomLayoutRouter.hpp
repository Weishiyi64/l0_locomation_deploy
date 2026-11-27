#pragma once

/**
 * @file OdomLayoutRouter.hpp
 * @brief Odom payload-generation -> registered-layout routing.
 * @details Everything here is GENERATED from OdomGenerations.def - the
 *          one-line-per-generation manifest that also carries the layout
 *          includes. Registering a new generation touches only the
 *          manifest: this file and OdinFrameAssembler.cpp re-expand it.
 *
 *          Routing semantics: the version byte (one global
 *          compatibility epoch; see OdinConst) is the SOLE key. The
 *          manifest expands into an if-chain, newest first, so each entry
 *          claims every generation >= its MIN_GEN - forward compatibility
 *          for free (a generation that did not change the odom layout
 *          keeps the previous layout). Historical layouts are strict
 *          (zero tail: frozen records); the newest alone is lenient
 *          (same-generation tail extensions up to kOdomMaxTailExt bytes
 *          per entry).
 *
 *          Kept as a small standalone header (pure functions, no state)
 *          so scripts/verify_odom_routing.cpp can exercise the REAL
 *          routing tables: the two order invariants the macro system
 *          cannot enforce structurally (newest-first; lenient only on
 *          the newest entry) are pinned there as executable assertions.
 */

#include <cstddef>
#include <cstdint>

// First manifest pass: pull in the layout structs (the V2/V3 includes live
// inside the manifest). X expands to NOTHING here - the meaningful
// expansions (enum / routing / tail budget) come below, each with its own X.
#define X(MIN_GEN, ENTRY_TYPE, TAG, LENIENT)
#include "OdomGenerations.def"
#undef X

namespace odin {
namespace sdk {
namespace protocol {

/// Tail-extension budget per entry, accepted on the newest generation only.
constexpr size_t kOdomMaxTailExt = 64;

/// Which registered layout a payload generation parses odom with.
enum class OdomLayoutGen {
  kNone,  // unreachable in practice: ParseDataFrame rejects payload generations < 2
#define X(MIN_GEN, ENTRY_TYPE, TAG, LENIENT) TAG,
#include "OdomGenerations.def"
#undef X
};

/// Layout for a payload generation: the first manifest entry (newest first)
/// whose MIN_GEN the generation meets. kNone only for generations older than
/// every registered entry.
inline OdomLayoutGen OdomLayoutForGeneration(uint8_t payload_gen) {
#define X(MIN_GEN, ENTRY_TYPE, TAG, LENIENT) \
  if (payload_gen >= (MIN_GEN)) return OdomLayoutGen::TAG;
#include "OdomGenerations.def"
#undef X
  return OdomLayoutGen::kNone;
}

/// Tail budget for a layout: kOdomMaxTailExt on the lenient (newest) entry
/// only, 0 (strict frozen record) on every historical one.
inline size_t OdomLayoutTailBudget(OdomLayoutGen layout) {
  switch (layout) {
#define X(MIN_GEN, ENTRY_TYPE, TAG, LENIENT) \
  case OdomLayoutGen::TAG: return (LENIENT) ? kOdomMaxTailExt : 0;
#include "OdomGenerations.def"
#undef X
    case OdomLayoutGen::kNone:
      return 0;
  }
  return 0;
}

}  // namespace protocol
}  // namespace sdk
}  // namespace odin
