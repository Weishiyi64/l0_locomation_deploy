#pragma once

#include <socketcan_wrapper/socketcan_manager.h>

#include <cstdint>
#include <map>
#include <rclcpp/rclcpp.hpp>

namespace kvms_battery {

struct BatteryRealtimeData {
  // 总信息0 (0x040280xx)
  float sum_voltage;  // 总电压 (V) - 0.1V/bit
  float current;      // 电流 (A) - 0.1A/bit, 偏移-30000
  float soc;          // 电量 (%) - 0.1%/bit
  uint8_t life;       // 0~255

  // 总信息1 (0x040380xx)
  float power;         // 功率 (W) - 1W/bit
  float total_energy;  // 总能量 (Wh) - 1Wh/bit
  float mos_temp;      // MOS温度 (°C) - 1°C/bit, 偏移-40
  float board_temp;    // 环境温度 (°C) - 1°C/bit, 偏移-40
  float heat_temp;     // 加热膜温度 (°C) - 1°C/bit, 偏移-40
  float heat_current;  // 加热电流 (A) - 1A/bit

  // 状态信息2 (0x040880xx)
  uint8_t cell_number;     // 电池串数
  uint8_t ntc_number;      // 温度个数
  float remain_capacity;   // 剩余容量 (Ah) - mAh存储
  uint32_t charge_cycles;  // 充电循环次数

  rclcpp::Time timestamp;
  std::map<uint32_t, rclcpp::Time> can_id_last_update;

  BatteryRealtimeData()
      : sum_voltage(0.0f),
        current(0.0f),
        soc(0.0f),
        life(0),
        power(0.0f),
        total_energy(0.0f),
        mos_temp(0.0f),
        board_temp(0.0f),
        heat_temp(0.0f),
        heat_current(0.0f),
        cell_number(0),
        ntc_number(0),
        remain_capacity(0.0f),
        charge_cycles(0) {}
};

class ProtocolParser {
 public:
  ProtocolParser();

  bool parseCANFrame(const can::Frame& in, BatteryRealtimeData& out);
  bool isDataComplete(const BatteryRealtimeData& data, double timeout = 8.0);

 private:
  bool parseTotalInfo0(const uint8_t* data,
                       BatteryRealtimeData& out);  // 0x040280xx
  bool parseTotalInfo1(const uint8_t* data,
                       BatteryRealtimeData& out);  // 0x040380xx
  bool parseStatusInfo2(const uint8_t* data,
                        BatteryRealtimeData& out);  // 0x040880xx

  uint16_t parseUint16(const uint8_t* data, int offset, bool big_endian = true);
  int16_t parseInt16(const uint8_t* data, int offset, bool big_endian = true);
  uint32_t parseUint32(const uint8_t* data, int offset, bool big_endian = true);
};

}  // namespace kvms_battery
