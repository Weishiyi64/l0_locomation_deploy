/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "hpm_battery/decode.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

using frameTransport::BatterySnapshot;
static void put(battery_cache_t &cache, uint32_t id, const std::array<uint8_t, 8> &d,
                uint64_t time)
{
  assert(battery_cache_update(&cache, 0x40, 6, 0, id, 8, d.data(), time));
}
static BatterySnapshot take(battery_cache_t &cache, uint64_t time)
{
  BatterySnapshot s{};
  battery_cache_take(&cache, &s, time);
  return s;
}
static bool near(float a, float b) { return std::fabs(a - b) < .002f; }

int main(int argc, char **argv)
{
  // Reference checker feeds raw CAN records and receives the real decoder's ROS message.
  if (argc == 2 && std::strcmp(argv[1], "--decode") == 0) {
    battery_cache_t input{};
    unsigned count;
    if (!(std::cin >> count)) return 2;
    for (unsigned n = 0; n < count; ++n) {
      uint32_t id; uint64_t timestamp; std::array<uint8_t, 8> bytes{};
      if (!(std::cin >> std::hex >> id >> std::dec >> timestamp)) return 2;
      for (auto &byte : bytes) {
        unsigned value;
        if (!(std::cin >> std::hex >> value) || value > 255) return 2;
        byte = uint8_t(value);
      }
      put(input, id, bytes, timestamp);
    }
    std::cout << luvbot_hardware_interfaces::msg::to_yaml(hpmBattery::decode_snapshot(take(input, 100000)));
    return 0;
  }
  static_assert(BATTERY_DATAGRAM_SIZE == 709);
  battery_cache_t cache{};
  auto s = take(cache, 0);
  auto m = hpmBattery::decode_snapshot(s);
  assert(s.received_mask == 0 && std::isnan(m.voltage));
  assert(m.bms_capacity_cycle_count == UINT32_MAX && m.bms_com_matrix_version == 255);
  assert(std::isnan(m.bcu_vol_11) && std::isnan(m.bms_temp_psdg));

  // Establish charging state for the positive-current DBC fixtures below.
  put(cache, 0x211, {0, 2, 0, 0, 0, 0, 0, 0}, 0);
  // Last-writer wins, including unchanged bytes; slow messages survive multiple takes.
  put(cache, 0x210, {1, 2, 3, 4, 5, 0, 0, 0}, 0);
  put(cache, 0x215, {80, 100, 0xf4, 1, 0xa0, 0x0f, 0xf4, 0x81}, 25);
  put(cache, 0x215, {81, 100, 0xf4, 1, 0xa0, 0x0f, 0xf4, 0x81}, 75);
  s = take(cache, 100);
  assert(udp_wire_get_be16(s.slots[4] + 1) == 2);
  assert(udp_wire_get_be32(s.slots[4] + 3) == 25);
  m = hpmBattery::decode_snapshot(s);
  assert(m.soc == 81 && m.soh == 100 && m.sop == 50);
  assert(near(m.voltage, 40) && near(m.current, 5));
  assert(m.bms_com_matrix_version == 1 && m.bms_device_info_hv_major == 2 && m.bms_device_info_sv_minor == 5);
  s = take(cache, 200);
  assert(s.updated_mask == 0 && (s.received_mask & 1));
  assert(udp_wire_get_be32(s.slots[0] + 3) == 200);
  put(cache, 0x215, {81, 100, 0xf4, 1, 0xa0, 0x0f, 0xff, 0xff}, 201);
  s = take(cache, 202);
  assert(s.updated_mask == (1U << 4));
  assert(near(hpmBattery::decode_snapshot(s).current, 327.67f));
  put(cache, 0x215, {255, 255, 255, 255, 255, 255, 0, 0x80}, 203);
  m = hpmBattery::decode_snapshot(take(cache, 204));
  assert(std::isnan(m.soc) && std::isnan(m.soh) && std::isnan(m.voltage));
  assert(near(m.current, 0));
  put(cache, 0x211, {0, 1, 0, 0, 0, 0, 0, 0}, 205);
  // Captured real 0x215: native DBC current scale/offset, without a sign override.
  put(cache, 0x215, {0x29, 0x64, 0x56, 0, 0xe6, 0x0f, 0xee, 0x7f}, 205);
  m = hpmBattery::decode_snapshot(take(cache, 206));
  assert(m.soc == 41 && near(m.voltage, 40.70f) && near(m.current, -.18f));
  put(cache, 0x211, {100, 0xfd, 7, 1, 0, 1, 0, 1}, 250);
  m = hpmBattery::decode_snapshot(take(cache, 550));
  assert(m.bms_status_is_awake && m.full_charge && m.bms_status_is_balancing);
  assert(m.bms_status_charge_allowed && m.bms_status_discharge_allowed && m.bms_status_discharge_mos_state && m.bms_status_charge_mos_state);
  assert(m.bms_status_protection_state && m.bms_status_warning_state && m.charge_state == 1 && m.bms_status_pre_mos_state == 1);
  assert(m.bms_status_charge_mos2_state == 1 && m.bms_status_discharge_mos2_state == 0);
  assert(m.bms_status_charge_mos3_state == 1 && m.bms_status_discharge_mos3_state == 0);
  assert(hpmBattery::decode_snapshot(take(cache, 551)).charge_state == 1);
  put(cache, 0x216, {40, 77, 76, 5, 0x7c, 0x0e, 0x77, 0x0e}, 550);
  put(cache, 0x214, {1, 0x11, 11, 0x4c, 0x0f, 0, 0, 0}, 550);
  put(cache, 0x217, {0x9b, 1, 0x9b, 1, 0xa8, 0, 0, 0}, 550);
  put(cache, 0x218, {0x17, 0x12, 0x80, 2, 0x46, 0, 0, 0}, 550);
  for (unsigned id = 0x219; id <= 0x21b; ++id) {
    put(cache, id, {0x7a, 0x0e, 0xff, 0xff, 0x7c, 0x0e, 0x79, 0x0e}, 550);
  }
  put(cache, 0x21c, {0, 0, 2, 4, 0, 0, 0x95, 1}, 550);
  put(cache, 0x21d, {5, 2, 0, 0, 0, 0, 0, 0}, 550);
  put(cache, 0x21e, {0, 49, 50, 75, 100, 250, 255, 0}, 550);
  s = take(cache, 600); m = hpmBattery::decode_snapshot(s);
  assert(m.bms_pack_status_soc_display == 40 && m.bms_pack_status_temp_max == 27 && m.bms_pack_status_temp_min == 26);
  assert(m.bms_pack_status_cell_volt_diff == 5 && m.bms_pack_status_cell_volt_max == 3708 && m.bms_pack_status_cell_volt_min == 3703);
  assert(m.bms_pack_info_cell_series_num == 11 && m.bms_pack_info_cell_parallel_num == 1 && near(m.bms_pack_info_rated_voltage, 39.16f));
  assert(m.bms_capacity_design == 411 && m.bms_capacity_current == 168 && m.bms_capacity_cycle_count == 0);
  assert(near(m.bms_charge_voltage_req, 46.31f) && near(m.bms_charge_crrent_req, 6.4f));
  assert(near(m.bms_charged_capacity, .7f));
  assert(near(m.bcu_vol_1, 3.706f) && near(m.bcu_vol_2, 65.535f));
  assert(near(m.bcu_vol_11, 3.708f));
  assert(near(m.bcu_vol_3, 3.708f) && near(m.bcu_vol_4, 3.705f));
  assert(near(m.bcu_vol_5, 3.706f) && near(m.bcu_vol_6, 65.535f));
  assert(near(m.bcu_vol_7, 3.708f) && near(m.bcu_vol_8, 3.705f));
  assert(near(m.bcu_vol_9, 3.706f) && near(m.bcu_vol_10, 65.535f));
  assert(m.key_wakeup && !m.chg_det_wakeup && m.io_det_wakeup);
  assert(m.bms_temp_cell1 == -50 && m.bms_temp_cell2 == -1);
  assert(m.bms_temp_mos1 == 0 && m.bms_temp_mos2 == 25 && m.bms_temp_psdg == 50);
  assert(near(m.sum_volt, 37.05f));
  assert(m.get_bms_fault_level == 0 && m.asw_soc_get_clbrt_flag == 0);
  assert(m.get_sys_state == 2 && m.get_step == 4 && near(m.realsoc, 40.5f));
  assert(m.get_fault_list == 0 && m.get_fault_num == 0 && m.bms_cell_volt_num == 0);
  assert(near(m.ex_volt2, .2f) && m.chg_volt == 0 && m.inter_volt == 0);
  assert(!m.get_sys_key_io && !m.get_chg_wake_up);
  assert(s.slots[11][15 + 2] == 2); // 0x21C is preserved and decoded.
  // Trimming ROS diagnostics does not discard cached physical values.
  assert(hpmBattery::decode_snapshot(take(cache, 701)).bms_pack_status_soc_display == 40);
  s = take(cache, uint64_t(UINT32_MAX) + 10000);
  assert(udp_wire_get_be32(s.slots[0] + 3) == UINT32_MAX);
  assert(hpmBattery::decode_snapshot(s).bms_pack_status_soc_display == 40);

  uint8_t raw[8]{};
  battery_cache_t fragments{};
  put(fragments, 0x212, {0x4f, 7, 6, 5, 4, 3, 2, 1}, 0);
  s = take(fragments, 0);
  assert((s.received_mask & (1U << 28)) != 0 && s.slots[28][15] == 0x4f);
  // Serial-only snapshots have no physical values to decode.
  m = hpmBattery::decode_snapshot(s);
  assert(std::isnan(m.voltage) && m.bms_com_matrix_version == 255);
  assert(!battery_cache_update(&cache, 0x40, 6, 0, 0x500, 8, raw, 1));
  assert(!battery_cache_update(&cache, 0x40, 6, 1, 0x215, 8, raw, 1));
  assert(!battery_cache_update(&cache, 0x40, 6, 0, 0x215, 7, raw, 1));

  // Offset-binary boundaries and actual captures; sign must not depend on port state.
  battery_cache_t direction{};
  const uint16_t codes[] = {0x0000, 0x7e0c, 0x7fe8, 0x7feb, 0x7fee, 0x7ff1,
                            0x7fff, 0x8000, 0x8001, 0x81f4, 0xffff};
  const float amperes[] = {-327.68f, -5, -.24f, -.21f, -.18f, -.15f,
                           -.01f, 0, .01f, 5, 327.67f};
  for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
    put(direction, 0x215, {50, 100, 0, 0, 0xa0, 0x0f, uint8_t(codes[i]),
                          uint8_t(codes[i] >> 8U)}, 0);
    assert(near(hpmBattery::decode_snapshot(take(direction, 0)).current, amperes[i]));
    for (unsigned state = 0; state < 4; ++state) {
      put(direction, 0x211, {0, uint8_t(state), 0, 0, 0, 0, 0, 0}, 0);
      assert(near(hpmBattery::decode_snapshot(take(direction, 0)).current, amperes[i]));
    }
  }

  // Shared wire encoder/decoder with real protocol CRC, including empty slots.
  s = take(cache, uint64_t(UINT32_MAX) + 11000);
  uint8_t packet[BATTERY_DATAGRAM_SIZE]{};
  udp_wire_header_t h{UDP_WIRE_BATTERY_SNAPSHOT, BATTERY_SLOT_COUNT, BATTERY_PAYLOAD_SIZE, 7, 1};
  assert(udp_wire_header_encode(packet, sizeof(packet), &h));
  battery_payload_encode(packet + UDP_WIRE_HEADER_SIZE, &s);
  udp_wire_put_be16(packet + sizeof(packet) - 2,
    frameTransport::frame_crc16_modbus(packet, sizeof(packet) - 2));
  udp_wire_header_t decoded_header{};
  assert(udp_wire_header_decode(packet, sizeof(packet), &decoded_header));
  BatterySnapshot decoded{};
  assert(battery_payload_decode(packet + 20, sizeof(packet) - 20, &decoded));
  assert(decoded.received_mask == s.received_mask && std::memcmp(decoded.slots, s.slots, sizeof(s.slots)) == 0);
  packet[20] = 2;
  assert(!battery_payload_decode(packet + 20, sizeof(packet) - 20, &decoded));
  packet[20] = 1; packet[23] = 1;
  assert(!battery_payload_decode(packet + 20, sizeof(packet) - 20, &decoded));
  packet[23] = 0; packet[20 + 12] |= 0x80;
  assert(!battery_payload_decode(packet + 20, sizeof(packet) - 20, &decoded));
  std::cout << "PASS: latest cache, ages, masks, published DBC fields, ignored serial and wire contract\n";
}
