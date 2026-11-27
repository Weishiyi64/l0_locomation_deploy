/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 * Shared C/C++ raw battery snapshot contract. No physical-value decoding here.
 */
#ifndef HPM_TRANSPORT_BATTERY_WIRE_H
#define HPM_TRANSPORT_BATTERY_WIRE_H
#include "udp_wire.h"

#define BATTERY_SLOT_COUNT 29U
#define BATTERY_SLOT_SIZE 23U
#define BATTERY_META_SIZE 20U
#define BATTERY_MASK 0x1FFFFFFFU
#define BATTERY_PAYLOAD_SIZE (BATTERY_META_SIZE + BATTERY_SLOT_COUNT * BATTERY_SLOT_SIZE + 2U)
#define BATTERY_DATAGRAM_SIZE (UDP_WIRE_HEADER_SIZE + BATTERY_PAYLOAD_SIZE)

typedef struct {
    uint32_t sequence;
    uint32_t board_ms;
    uint32_t received_mask;
    uint32_t updated_mask;
    /* msg(1), update counter(2), age_ms(4), reserved(1), port(1), ide(1),
     * CAN ID(4), DLC(1), unchanged CAN data(8). Multi-byte metadata is BE. */
    uint8_t slots[BATTERY_SLOT_COUNT][BATTERY_SLOT_SIZE];
} battery_snapshot_t;

static inline uint32_t battery_slot_id(unsigned slot)
{
    static const uint16_t ids[14] = {
        0x210, 0x211, 0x212, 0x214, 0x215, 0x216, 0x217,
        0x218, 0x219, 0x21A, 0x21B, 0x21C, 0x21D, 0x21E
    };
    return slot < 14U ? ids[slot] : 0x212U;
}

static inline unsigned battery_serial_slot(unsigned index)
{
    return index == 0U ? 2U : 13U + index;
}

static inline int battery_slot_index(uint8_t port, uint8_t ide, uint32_t id,
                                      uint8_t dlc, const uint8_t *data)
{
    if (port != 6U || ide != 0U || dlc != 8U || data == NULL) {
        return -1;
    }
    if (id == 0x212U) {
        return (int)battery_serial_slot(data[0] & 15U);
    }
    for (unsigned i = 0U; i < 14U; ++i) {
        if (battery_slot_id(i) == id) {
            return (int)i;
        }
    }
    return -1;
}

typedef struct {
    battery_snapshot_t value;
    uint64_t received_ms[BATTERY_SLOT_COUNT];
    volatile uint32_t rejected_frames;
} battery_cache_t;

static inline bool battery_cache_update(battery_cache_t *cache, uint8_t msg,
    uint8_t port, uint8_t ide, uint32_t id, uint8_t dlc, const uint8_t *data,
    uint64_t now_ms)
{
    const int index = battery_slot_index(port, ide, id, dlc, data);
    if (msg != 0x40U || index < 0) {
        cache->rejected_frames++;
        return false;
    }
    uint8_t *slot = cache->value.slots[index];
    slot[0] = msg;
    udp_wire_put_be16(slot + 1U, (uint16_t)(udp_wire_get_be16(slot + 1U) + 1U));
    slot[8] = port;
    slot[9] = ide;
    udp_wire_put_be32(slot + 10U, id);
    slot[14] = dlc;
    memcpy(slot + 15U, data, 8U);
    cache->received_ms[index] = now_ms;
    cache->value.received_mask |= 1UL << index;
    cache->value.updated_mask |= 1UL << index;
    return true;
}

/* Call under the cache mutex. A failed network send never queues old snapshots. */
static inline void battery_cache_take(battery_cache_t *cache,
                                      battery_snapshot_t *out, uint64_t now_ms)
{
    cache->value.sequence++;
    cache->value.board_ms = (uint32_t)now_ms;
    *out = cache->value;
    cache->value.updated_mask = 0U;
    for (unsigned i = 0U; i < BATTERY_SLOT_COUNT; ++i) {
        if ((out->received_mask & (1UL << i)) != 0U) {
            const uint64_t age = now_ms - cache->received_ms[i];
            udp_wire_put_be32(out->slots[i] + 3U,
                             age > UINT32_MAX ? UINT32_MAX : (uint32_t)age);
        }
    }
}

static inline void battery_payload_encode(uint8_t *payload, const battery_snapshot_t *s)
{
    memset(payload, 0, BATTERY_PAYLOAD_SIZE);
    payload[0] = 1U;
    payload[1] = BATTERY_SLOT_COUNT;
    payload[2] = 6U;
    udp_wire_put_be32(payload + 4U, s->sequence);
    udp_wire_put_be32(payload + 8U, s->board_ms);
    udp_wire_put_be32(payload + 12U, s->received_mask);
    udp_wire_put_be32(payload + 16U, s->updated_mask);
    memcpy(payload + BATTERY_META_SIZE, s->slots, sizeof(s->slots));
}

/* Structural validation only; callers also validate CRC and UDP session/sequence. */
static inline bool battery_payload_decode(const uint8_t *p, size_t length,
                                          battery_snapshot_t *out)
{
    if (p == NULL || out == NULL || length != BATTERY_PAYLOAD_SIZE ||
        p[0] != 1U || p[1] != BATTERY_SLOT_COUNT || p[2] != 6U || p[3] != 0U) {
        return false;
    }
    const uint32_t received = udp_wire_get_be32(p + 12U);
    const uint32_t updated = udp_wire_get_be32(p + 16U);
    if ((received & ~BATTERY_MASK) != 0U || (updated & ~received) != 0U) {
        return false;
    }
    for (unsigned i = 0U; i < BATTERY_SLOT_COUNT; ++i) {
        const uint8_t *slot = p + BATTERY_META_SIZE + i * BATTERY_SLOT_SIZE;
        if ((received & (1UL << i)) == 0U) {
            for (unsigned j = 0U; j < BATTERY_SLOT_SIZE; ++j) {
                if (slot[j] != 0U) return false;
            }
        } else if (slot[0] != 0x40U || slot[7] != 0U ||
                   battery_slot_index(slot[8], slot[9], udp_wire_get_be32(slot + 10U),
                                      slot[14], slot + 15U) != (int)i) {
            return false;
        }
    }
    out->sequence = udp_wire_get_be32(p + 4U);
    out->board_ms = udp_wire_get_be32(p + 8U);
    out->received_mask = received;
    out->updated_mask = updated;
    memcpy(out->slots, p + BATTERY_META_SIZE, sizeof(out->slots));
    return true;
}
#endif
