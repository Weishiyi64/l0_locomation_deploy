/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HPM_TRANSPORT__UDP_WIRE_H_
#define HPM_TRANSPORT__UDP_WIRE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UDP_WIRE_HEADER_SIZE       (20U)
#define UDP_WIRE_FRAME_SIZE        (30U)
#define UDP_WIRE_MAX_FRAMES        (48U)
#define UDP_WIRE_MAX_PAYLOAD       (UDP_WIRE_FRAME_SIZE * UDP_WIRE_MAX_FRAMES)
#define UDP_WIRE_MAX_DATAGRAM_SIZE (UDP_WIRE_HEADER_SIZE + UDP_WIRE_MAX_PAYLOAD)
#define UDP_WIRE_MOTOR_SNAPSHOT_SLOTS       (21U)
#define UDP_WIRE_MOTOR_SNAPSHOT_SLOT_SIZE   (23U)
#define UDP_WIRE_MOTOR_SNAPSHOT_META_SIZE   (20U)
#define UDP_WIRE_MOTOR_SNAPSHOT_PAYLOAD_SIZE \
    (UDP_WIRE_MOTOR_SNAPSHOT_META_SIZE + \
     UDP_WIRE_MOTOR_SNAPSHOT_SLOTS * UDP_WIRE_MOTOR_SNAPSHOT_SLOT_SIZE + 2U)
#define UDP_WIRE_MOTOR_SNAPSHOT_SIZE \
    (UDP_WIRE_HEADER_SIZE + UDP_WIRE_MOTOR_SNAPSHOT_PAYLOAD_SIZE)
#define UDP_WIRE_BATTERY_SNAPSHOT_SLOTS 29U
#define UDP_WIRE_BATTERY_SNAPSHOT_PAYLOAD_SIZE (20U + 29U * 23U + 2U)

typedef enum {
    UDP_WIRE_DATA = 1U,
    UDP_WIRE_HELLO = 2U,
    UDP_WIRE_HELLO_ACK = 3U,
    UDP_WIRE_PING = 4U,
    UDP_WIRE_PONG = 5U,
    UDP_WIRE_CLOSE = 6U,
    UDP_WIRE_BUSY = 7U,
    UDP_WIRE_RESET = 8U,
    UDP_WIRE_OTA_BUSY = 9U,
    UDP_WIRE_MOTOR_SNAPSHOT = 10U,
    UDP_WIRE_BATTERY_SNAPSHOT = 11U,
} udp_wire_type_t;

typedef struct {
    udp_wire_type_t type;
    uint8_t frame_count;
    uint16_t payload_len;
    uint64_t session_id;
    uint32_t sequence;
} udp_wire_header_t;

static inline void udp_wire_put_be16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8U);
    out[1] = (uint8_t)value;
}

static inline void udp_wire_put_be32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value >> 24U);
    out[1] = (uint8_t)(value >> 16U);
    out[2] = (uint8_t)(value >> 8U);
    out[3] = (uint8_t)value;
}

static inline void udp_wire_put_be64(uint8_t *out, uint64_t value)
{
    udp_wire_put_be32(out, (uint32_t)(value >> 32U));
    udp_wire_put_be32(out + 4U, (uint32_t)value);
}

static inline uint16_t udp_wire_get_be16(const uint8_t *in)
{
    return (uint16_t)(((uint16_t)in[0] << 8U) | (uint16_t)in[1]);
}

static inline uint32_t udp_wire_get_be32(const uint8_t *in)
{
    return ((uint32_t)in[0] << 24U) | ((uint32_t)in[1] << 16U) |
           ((uint32_t)in[2] << 8U) | (uint32_t)in[3];
}

static inline uint64_t udp_wire_get_be64(const uint8_t *in)
{
    return ((uint64_t)udp_wire_get_be32(in) << 32U) |
           (uint64_t)udp_wire_get_be32(in + 4U);
}

static inline bool udp_wire_control_payload_valid(udp_wire_type_t type,
                                                   uint16_t payload_len)
{
    if ((type == UDP_WIRE_HELLO) || (type == UDP_WIRE_PING)) {
        return payload_len == 4U;
    }
    return (type >= UDP_WIRE_HELLO_ACK) && (type <= UDP_WIRE_OTA_BUSY) &&
           (payload_len == 0U);
}

static inline bool udp_wire_header_encode(uint8_t *out, size_t capacity,
                                          const udp_wire_header_t *header)
{
    static const uint8_t magic[4] = {'H', 'P', 'U', '1'};

    if ((out == NULL) || (header == NULL) ||
        (capacity < UDP_WIRE_HEADER_SIZE) || (header->session_id == 0U) ||
        (header->type < UDP_WIRE_DATA) ||
        (header->type > UDP_WIRE_BATTERY_SNAPSHOT)) {
        return false;
    }
    if (header->type == UDP_WIRE_DATA) {
        if ((header->frame_count == 0U) ||
            (header->frame_count > UDP_WIRE_MAX_FRAMES) ||
            (header->payload_len != (uint16_t)(header->frame_count * UDP_WIRE_FRAME_SIZE))) {
            return false;
        }
    } else if (header->type == UDP_WIRE_MOTOR_SNAPSHOT) {
        if ((header->frame_count != UDP_WIRE_MOTOR_SNAPSHOT_SLOTS) ||
            (header->payload_len != UDP_WIRE_MOTOR_SNAPSHOT_PAYLOAD_SIZE)) {
            return false;
        }
    } else if (header->type == UDP_WIRE_BATTERY_SNAPSHOT) {
        if (header->frame_count != UDP_WIRE_BATTERY_SNAPSHOT_SLOTS ||
            header->payload_len != UDP_WIRE_BATTERY_SNAPSHOT_PAYLOAD_SIZE) return false;
    } else if ((header->frame_count != 0U) ||
               !udp_wire_control_payload_valid(header->type, header->payload_len) ||
               (header->sequence != 0U)) {
        return false;
    }

    memcpy(out, magic, sizeof(magic));
    out[4] = (uint8_t)header->type;
    out[5] = header->frame_count;
    udp_wire_put_be16(out + 6U, header->payload_len);
    udp_wire_put_be64(out + 8U, header->session_id);
    udp_wire_put_be32(out + 16U, header->sequence);
    return true;
}

static inline bool udp_wire_header_decode(const uint8_t *in, size_t datagram_len,
                                          udp_wire_header_t *header)
{
    static const uint8_t magic[4] = {'H', 'P', 'U', '1'};
    uint16_t payload_len;
    udp_wire_type_t type;

    if ((in == NULL) || (header == NULL) ||
        (datagram_len < UDP_WIRE_HEADER_SIZE) ||
        (memcmp(in, magic, sizeof(magic)) != 0)) {
        return false;
    }

    type = (udp_wire_type_t)in[4];
    payload_len = udp_wire_get_be16(in + 6U);
    if ((type < UDP_WIRE_DATA) || (type > UDP_WIRE_BATTERY_SNAPSHOT) ||
        (payload_len != datagram_len - UDP_WIRE_HEADER_SIZE)) {
        return false;
    }

    header->type = type;
    header->frame_count = in[5];
    header->payload_len = payload_len;
    header->session_id = udp_wire_get_be64(in + 8U);
    header->sequence = udp_wire_get_be32(in + 16U);

    if (header->session_id == 0U) {
        return false;
    }
    if (type == UDP_WIRE_DATA) {
        return (header->frame_count > 0U) &&
               (header->frame_count <= UDP_WIRE_MAX_FRAMES) &&
               (payload_len == (uint16_t)(header->frame_count * UDP_WIRE_FRAME_SIZE));
    }
    if (type == UDP_WIRE_MOTOR_SNAPSHOT) {
        return (header->frame_count == UDP_WIRE_MOTOR_SNAPSHOT_SLOTS) &&
               (payload_len == UDP_WIRE_MOTOR_SNAPSHOT_PAYLOAD_SIZE);
    }
    if (type == UDP_WIRE_BATTERY_SNAPSHOT) {
        return header->frame_count == UDP_WIRE_BATTERY_SNAPSHOT_SLOTS &&
               payload_len == UDP_WIRE_BATTERY_SNAPSHOT_PAYLOAD_SIZE;
    }
    return (header->frame_count == 0U) && (header->sequence == 0U) &&
           udp_wire_control_payload_valid(type, payload_len);
}

#endif  // HPM_TRANSPORT__UDP_WIRE_H_
