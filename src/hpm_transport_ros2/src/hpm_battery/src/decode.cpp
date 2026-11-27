/* Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 * Source: YB-BMS_CAN_V2.0_2026101301.dbc (native DBC scale, offset and current sign).
 */
#include "hpm_battery/decode.hpp"
#include <array>
#include <limits>

namespace hpmBattery {
namespace {
constexpr float invalid = std::numeric_limits<float>::quiet_NaN();
uint16_t le16(const uint8_t *d) { return uint16_t(d[0]) | (uint16_t(d[1]) << 8U); }
float value(uint32_t raw, uint32_t maximum, float scale = 1.0f, float offset = 0.0f)
{
  return raw <= maximum ? raw * scale + offset : invalid;
}
}

luvbot_hardware_interfaces::msg::BatteryState decode_snapshot(
  const frameTransport::BatterySnapshot &s)
{
  luvbot_hardware_interfaces::msg::BatteryState m;
  m.bms_com_matrix_version = m.bms_device_info_hv_major = m.bms_device_info_hv_minor = m.bms_device_info_sv_major = m.bms_device_info_sv_minor = 255;
  m.bms_alive_cnt4_charger = m.charge_state = 255;
  m.bms_status_discharge_mos2_state = m.bms_status_charge_mos2_state = m.bms_status_discharge_mos3_state = m.bms_status_charge_mos3_state = m.bms_status_pre_mos_state = 255;
  m.bms_pack_info_chemical_type = m.bms_pack_info_port_type = m.bms_pack_info_cell_parallel_num = m.bms_pack_info_cell_series_num = 255;
  m.bms_capacity_cycle_count = UINT32_MAX;
  m.bms_cell_volt_num = m.get_bms_fault_level = m.asw_soc_get_clbrt_flag = 255;
  m.get_sys_state = m.get_step = m.get_fault_num = m.get_fault_list = 255;
  for (float *p : {&m.bms_pack_info_rated_voltage, &m.soc, &m.bms_pack_status_soc_display, &m.soh, &m.sop,
      &m.voltage, &m.current, &m.bms_pack_status_temp_max, &m.bms_pack_status_temp_min, &m.bms_pack_status_cell_volt_diff,
      &m.bms_pack_status_cell_volt_max, &m.bms_pack_status_cell_volt_min, &m.bms_capacity_design, &m.bms_capacity_full_charge, &m.bms_capacity_current,
      &m.bms_charge_voltage_req, &m.bms_charge_crrent_req, &m.bms_charged_capacity,
      &m.sum_volt, &m.realsoc, &m.inter_volt, &m.chg_volt, &m.ex_volt2}) *p = invalid;
  const std::array<float *, 11> cell_voltages = {
    &m.bcu_vol_1, &m.bcu_vol_2, &m.bcu_vol_3, &m.bcu_vol_4, &m.bcu_vol_5, &m.bcu_vol_6,
    &m.bcu_vol_7, &m.bcu_vol_8, &m.bcu_vol_9, &m.bcu_vol_10, &m.bcu_vol_11
  };
  const std::array<float *, 5> temperatures = {
    &m.bms_temp_cell1, &m.bms_temp_cell2,
    &m.bms_temp_mos1, &m.bms_temp_mos2, &m.bms_temp_psdg
  };
  for (auto *p : cell_voltages) *p = invalid;
  for (auto *p : temperatures) *p = invalid;
  // 0x212 remains in the transport snapshot but is not decoded or published.
  const auto has = [&s](unsigned i) { return (s.received_mask & (1UL << i)) != 0U; };
  const auto data = [&s](unsigned i) { return s.slots[i] + 15U; };
  if (has(0)) {
    const auto *d = data(0);
    m.bms_com_matrix_version = d[0]; m.bms_device_info_hv_major = d[1]; m.bms_device_info_hv_minor = d[2];
    m.bms_device_info_sv_major = d[3]; m.bms_device_info_sv_minor = d[4];
  }
  if (has(1)) {
    const auto *d = data(1);
    m.bms_alive_cnt4_charger = d[0]; m.charge_state = d[1] & 3U;
    m.bms_status_is_awake = (d[1] & 4U) != 0U;
    m.full_charge = (d[1] & 8U) != 0U;
    m.bms_status_is_balancing = (d[1] & 16U) != 0U;
    m.bms_status_charge_allowed = (d[1] & 32U) != 0U;
    m.bms_status_discharge_allowed = (d[1] & 64U) != 0U;
    m.bms_status_discharge_mos_state = (d[1] & 128U) != 0U;
    m.bms_status_charge_mos_state = (d[2] & 1U) != 0U;
    m.bms_status_protection_state = (d[2] & 2U) != 0U;
    m.bms_status_warning_state = (d[2] & 4U) != 0U;
    m.bms_status_charge_mos2_state = d[3]; m.bms_status_discharge_mos2_state = d[4];
    m.bms_status_charge_mos3_state = d[5]; m.bms_status_discharge_mos3_state = d[6]; m.bms_status_pre_mos_state = d[7];
  }
  if (has(3)) {
    const auto *d = data(3);
    m.bms_pack_info_chemical_type = d[0] & 15U; m.bms_pack_info_port_type = d[1] & 15U;
    m.bms_pack_info_cell_parallel_num = d[1] >> 4U; m.bms_pack_info_cell_series_num = d[2];
    m.bms_pack_info_rated_voltage = value(le16(d + 3), 65500, .01f);
  }
  if (has(4)) {
    const auto *d = data(4);
    m.soc = value(d[0], 120); m.soh = value(d[1], 100);
    m.sop = value(le16(d + 2), 65535, .1f); // DBC unit: kW
    m.voltage = value(le16(d + 4), 65534, .01f);
    const uint32_t raw = le16(d + 6);
    // Unsigned little-endian raw, scale 0.01 A, offset -327.68 A (DBC native sign).
    m.current = (int32_t(raw) - 32768) * .01f;
  }
  if (has(5)) {
    const auto *d = data(5);
    m.bms_pack_status_soc_display = value(d[0], 100);
    m.bms_pack_status_temp_max = value(d[1], 250, 1, -50);
    m.bms_pack_status_temp_min = value(d[2], 250, 1, -50);
    m.bms_pack_status_cell_volt_diff = value(d[3], 250);
    m.bms_pack_status_cell_volt_max = value(le16(d + 4), 5000);
    m.bms_pack_status_cell_volt_min = value(le16(d + 6), 5000);
  }
  if (has(6)) {
    const auto *d = data(6);
    // DBC start bits 0/16/32/48.
    m.bms_capacity_design = value(le16(d), 65000);
    m.bms_capacity_full_charge = value(le16(d + 2), 65000);
    m.bms_capacity_current = value(le16(d + 4), 65000);
    if (le16(d + 6) <= 65000) m.bms_capacity_cycle_count = le16(d + 6);
  }
  if (has(7)) {
    const auto *d = data(7);
    m.bms_charge_voltage_req = value(le16(d), 65534, .01f);
    m.bms_charge_crrent_req = value(le16(d + 2), 65534, .01f);
    m.bms_charged_capacity = value(le16(d + 4), 65534, .01f);
    m.bms_cell_volt_num = d[6];
  }
  for (unsigned slot = 8; slot <= 10; ++slot) {
    if (!has(slot)) continue;
    const auto *d = data(slot);
    for (unsigned i = 0; i < (slot == 10 ? 3U : 4U); ++i) {
      *cell_voltages[(slot - 8U) * 4U + i] = value(le16(d + 2U * i), 65535, .001f);
    }
  }
  if (has(10)) m.sum_volt = value(le16(data(10) + 6), 65535, .01f);
  if (has(11)) {
    const auto *d = data(11);
    m.asw_soc_get_clbrt_flag = d[0]; m.get_bms_fault_level = d[1];
    m.get_sys_state = d[2]; m.get_step = d[3];
    m.get_fault_list = d[4]; m.get_fault_num = d[5];
    m.realsoc = value(le16(d + 6), 65535, .1f);
  }
  if (has(12)) {
    const auto bits = data(12)[0];
    m.key_wakeup = (bits & 1U) != 0U;
    m.chg_det_wakeup = (bits & 2U) != 0U;
    m.io_det_wakeup = (bits & 4U) != 0U;
    const auto *d = data(12);
    m.ex_volt2 = value(le16(d + 1), 65535, .1f);
    m.chg_volt = value(le16(d + 3), 65535, .1f);
    m.inter_volt = value(le16(d + 5), 65535, .1f);
    m.get_sys_key_io = (d[7] & 1U) != 0U;
    m.get_chg_wake_up = (d[7] & 2U) != 0U;
  }
  if (has(13)) {
    for (unsigned i = 0; i < 5; ++i) *temperatures[i] = value(data(13)[i], 255, 1, -50);
  }

  return m;
}
}
