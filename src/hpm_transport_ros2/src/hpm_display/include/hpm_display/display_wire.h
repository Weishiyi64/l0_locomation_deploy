/* SPDX-License-Identifier: Apache-2.0
 * Shared wire contract. Keep identical to hpm_display/display_wire.h. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

enum { DISPLAY_SET_STATE=0x10, DISPLAY_INPUT_EVENT, DISPLAY_RESULT, DISPLAY_REPORT_STATE };
enum { DISPLAY_OK, DISPLAY_REJECTED, DISPLAY_BUSY, DISPLAY_INVALID,
       DISPLAY_UNSUPPORTED, DISPLAY_FAILED, DISPLAY_TIMEOUT, DISPLAY_RECEIVED };
enum { DISPLAY_BATTERY=1, DISPLAY_DOCK=4, DISPLAY_FAULT=6, DISPLAY_IDENTITY=8,
       DISPLAY_VOICE=0x11, DISPLAY_VOLUME=0x20,
       DISPLAY_MUTE, DISPLAY_MODE, DISPLAY_VOLUME_LOCK, DISPLAY_BLUETOOTH, DISPLAY_WIFI,
       DISPLAY_PROVISION=0x30, DISPLAY_UPGRADE, DISPLAY_HOMING, DISPLAY_NIGHT,
       DISPLAY_INTERACTION, DISPLAY_TASK_SUCCESS, DISPLAY_AMBIENT_LIGHT, DISPLAY_OPERATION,
       DISPLAY_PANEL=0x8000, DISPLAY_BACKLIGHT, DISPLAY_TOUCH };
/* 0x8003 is reserved for retired indicator telemetry. */
enum { DISPLAY_BT_REQUEST=1, DISPLAY_MODE_REQUEST, DISPLAY_VOLUME_REQUEST,
       DISPLAY_MUTE_REQUEST, DISPLAY_FAULT_ACK, DISPLAY_PROVISION_REQUEST };
enum { DISPLAY_OFFLINE, DISPLAY_ONLINE, DISPLAY_WAITING_FOR_CHARGE, DISPLAY_RETURNING_TO_DOCK, DISPLAY_ON_CHARGER };
#define DISPLAY_MAX_VALUE 257U
#define DISPLAY_MAX_FAULTS 32U
static inline uint16_t display_u16(const uint8_t *p)
{ return (uint16_t)(p[0] | (uint16_t)p[1]<<8); }
static inline uint32_t display_u32(const uint8_t *p)
{ return (uint32_t)display_u16(p) | (uint32_t)display_u16(p+2)<<16; }
static inline void display_put16(uint8_t *p, uint16_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void display_put32(uint8_t *p, uint32_t v)
{ display_put16(p,(uint16_t)v); display_put16(p+2,(uint16_t)(v>>16)); }
/* ROS Illuminance and the wire both use IEEE 754 binary64 in lux. */
static inline double display_lux(const uint8_t *p)
{
    uint64_t bits=(uint64_t)display_u32(p+4)<<32 | display_u32(p);
    double lux;
    memcpy(&lux,&bits,sizeof(lux));
    return lux;
}
static inline void display_put_lux(uint8_t *p, double lux)
{
    uint64_t bits;
    memcpy(&bits,&lux,sizeof(bits));
    display_put32(p,(uint32_t)bits);
    display_put32(p+4,(uint32_t)(bits>>32));
}
static inline bool display_utf8(const uint8_t *p, size_t n)
{
    for (size_t i=0; i<n;) {
        uint32_t c=p[i++], min=0; unsigned more=0;
        if (!c) return false;
        if (c<0x80) continue;
        if (c>=0xc2 && c<=0xdf) { c&=0x1f; more=1; min=0x80; }
        else if (c>=0xe0 && c<=0xef) { c&=0x0f; more=2; min=0x800; }
        else if (c>=0xf0 && c<=0xf4) { c&=7; more=3; min=0x10000; }
        else return false;
        if (more>n-i) return false;
        while (more--) { if ((p[i]&0xc0)!=0x80) return false; c=(c<<6)|(p[i++]&0x3f); }
        if (c<min || c>0x10ffff || (c>=0xd800 && c<=0xdfff)) return false;
    }
    return true;
}
/* Format/range validation only. No thresholds, mode policy or light arbitration. */
static inline unsigned display_validate(uint16_t key, const uint8_t *v, size_t n)
{
    if (!v || n>DISPLAY_MAX_VALUE) return DISPLAY_INVALID;
    switch (key) {
    case DISPLAY_BATTERY:
        return n==4 && (v[0]<=100 || v[0]==255) && v[1]<=4 && !(display_u16(v+2)&~15U) ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_DOCK: return n==1 && (v[0]<=1 || v[0]==255) ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_VOLUME: return n==1 && v[0]<=100 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_OPERATION: return n==1 && v[0]<=DISPLAY_ON_CHARGER ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_AMBIENT_LIGHT: {
        if (n!=8) return DISPLAY_INVALID;
        double lux=display_lux(v);
        return isfinite(lux) && lux>=0 ? DISPLAY_OK:DISPLAY_INVALID;
    }
    case DISPLAY_MUTE: case DISPLAY_VOLUME_LOCK: case DISPLAY_HOMING: case DISPLAY_NIGHT:
        return n==1 && v[0]<=1 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_MODE: return n==1 && v[0]<=5 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_BLUETOOTH: return n==1 && v[0]<=4 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_WIFI: return n==2 && v[0]<=3 && v[1]>=1 && v[1]<=4 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_PROVISION:
        return n==10 && v[4]<=8 && v[5]<=4 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_FAULT:
        if (!n || v[0]>DISPLAY_MAX_FAULTS || n!=1U+8U*v[0]) return DISPLAY_INVALID;
        for (size_t i=0;i<v[0];++i) {
            const uint8_t *f=v+1+8*i;
            if (!display_u16(f) || f[2]<1 || f[2]>3 || f[3]>1) return DISPLAY_INVALID;
            for (size_t j=0;j<i;++j) if (display_u16(f)==display_u16(v+1+8*j)) return DISPLAY_INVALID;
        }
        return DISPLAY_OK;
    case DISPLAY_UPGRADE:
        return n==8 && v[4]>=1 && v[4]<=5 && v[5]<=7 && (v[6]<=100 || v[6]==255) && v[7]<=3 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_IDENTITY:
        return n>=2 && v[0]<=1 && v[1]<=63 && n==2U+v[1] && display_utf8(v+2,v[1]) ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_INTERACTION: return n==1 && v[0]>=1 && v[0]<=3 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_TASK_SUCCESS: return n==1 && v[0]==1 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_VOICE: return n>=2 && v[0]<=3 && v[1]<=64 && n==2U+v[1] ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_PANEL: case DISPLAY_TOUCH:
        return n==3 && v[0]<=3 ? DISPLAY_OK:DISPLAY_INVALID;
    case DISPLAY_BACKLIGHT:
        return n==4 && v[0]<=3 && v[3]<=100 ? DISPLAY_OK:DISPLAY_INVALID;
    default: return DISPLAY_UNSUPPORTED;
    }
}
static inline bool display_event_valid(uint16_t action, uint16_t value)
{
    switch (action) {
    case DISPLAY_BT_REQUEST: case DISPLAY_MUTE_REQUEST: return value<=1;
    case DISPLAY_MODE_REQUEST: return value<=5;
    case DISPLAY_VOLUME_REQUEST: return value<=100;
    case DISPLAY_FAULT_ACK: return value!=0;
    case DISPLAY_PROVISION_REQUEST: return value==0;
    default: return false;
    }
}
static inline const char *display_function(uint16_t key)
{
    switch (key) {
    case DISPLAY_BATTERY:return "battery"; case DISPLAY_DOCK:return "dock";
    case DISPLAY_AMBIENT_LIGHT:return "ambient_light"; case DISPLAY_OPERATION:return "operation";
    case DISPLAY_PROVISION:return "provision"; case DISPLAY_FAULT:return "fault";
    case DISPLAY_UPGRADE:return "upgrade"; case DISPLAY_IDENTITY:return "identity";
    case DISPLAY_TASK_SUCCESS:return "task"; case DISPLAY_INTERACTION:return "interaction"; case DISPLAY_VOICE:return "voice";
    case DISPLAY_HOMING:return "homing"; case DISPLAY_NIGHT:return "night"; case DISPLAY_VOLUME:return "volume";
    case DISPLAY_MUTE:return "mute"; case DISPLAY_MODE:return "mode";
    case DISPLAY_VOLUME_LOCK:return "volume_lock"; case DISPLAY_BLUETOOTH:return "bluetooth";
    case DISPLAY_WIFI:return "wifi"; case DISPLAY_PANEL:return "panel";
    case DISPLAY_BACKLIGHT:return "backlight"; case DISPLAY_TOUCH:return "touch";
    default:return "unknown";
    }
}
