#include "kvms_battery/protocol_parser.h"

namespace kvms_battery {

ProtocolParser::ProtocolParser() {}

bool ProtocolParser::parseCANFrame(const can::Frame& in, BatteryRealtimeData& out) {
  if (!in.is_extended)
  {
    return false;  // KVMS协议只使用扩展帧
  }

  uint32_t can_id = in.id;

  // 掩码：0xFFFFF00 -> 基础ID (忽略最后2位版本号)
  uint32_t base_id = can_id & 0xFFFFF00;

  bool parsed = false;

  switch (base_id)
  {
    case 0x4028000:  // 总信息0
      parsed = parseTotalInfo0(in.data.data(), out);
      if (parsed)
      {
        // debug logging omitted or can use RCLCPP_DEBUG if node logger
        // available
      }
      break;

    case 0x4038000:  // 总信息1
      parsed = parseTotalInfo1(in.data.data(), out);
      if (parsed)
      {
        // debug logging omitted
      }
      break;

    case 0x4088000:  // 状态信息2
      parsed = parseStatusInfo2(in.data.data(), out);
      if (parsed)
      {
        // debug logging omitted
      }
      break;

    default:
      // 忽略不需要的帧
      return false;
  }

  if (parsed)
  {
    out.timestamp = rclcpp::Clock().now();
    out.can_id_last_update[base_id] = out.timestamp;
  }

  return parsed;
}

bool ProtocolParser::parseTotalInfo0(const uint8_t* data, BatteryRealtimeData& out) {
  // Data0-1: SumV_H, SumV_L - 总电压 0.1V/bit
  uint16_t sum_v_raw = parseUint16(data, 0, true);
  out.sum_voltage = sum_v_raw * 0.1f;

  // Data2-3: Curr_H, Curr_L - 电流 0.1A/bit, 偏移-30000
  uint16_t curr_raw = parseUint16(data, 2, true);
  out.current = (static_cast<int32_t>(curr_raw) - 30000) * 0.1f;

  // Data4-5: SOC_H, SOC_L - 电量 0.1%/bit
  uint16_t soc_raw = parseUint16(data, 4, true);
  out.soc = soc_raw * 0.1f;

  // Data6: Life - 寿命 0~255
  out.life = data[6];

  return true;
}

bool ProtocolParser::parseTotalInfo1(const uint8_t* data, BatteryRealtimeData& out) {
  // Data0-1: POWER_H, POWER_L - 功率 1W/bit
  uint16_t power_raw = parseUint16(data, 0, true);
  out.power = static_cast<float>(power_raw);

  // Data2-3: Total_Energy_H, Total_Energy_L - 总能量 1Wh/bit
  uint16_t energy_raw = parseUint16(data, 2, true);
  out.total_energy = static_cast<float>(energy_raw);

  // Data4: MOS_Temp - MOS温度 1°C/bit, 偏移-40
  out.mos_temp = static_cast<float>(static_cast<int16_t>(data[4]) - 40);

  // Data5: Board_Temp - 环境温度 1°C/bit, 偏移-40
  out.board_temp = static_cast<float>(static_cast<int16_t>(data[5]) - 40);

  // Data6: Heat_Temp - 加热膜温度 1°C/bit, 偏移-40
  out.heat_temp = static_cast<float>(static_cast<int16_t>(data[6]) - 40);

  // Data7: Heat_Cur - 加热电流 1A/bit
  out.heat_current = static_cast<float>(data[7]);

  return true;
}

bool ProtocolParser::parseStatusInfo2(const uint8_t* data, BatteryRealtimeData& out) {
  // Data0: cell_number - 电池串数
  out.cell_number = data[0];

  // Data1: ntc_number - 温度个数
  out.ntc_number = data[1];

  // Data2-5: Remain_Capacity (mAh) - 剩余容量，转换为Ah
  uint32_t remain_cap_mah = parseUint32(data, 2, true);
  out.remain_capacity = remain_cap_mah / 1000.0f;  // mAh -> Ah

  // Data6-7: Cycle_Time_H, Cycle_Time_L - 充电循环次数
  out.charge_cycles = parseUint16(data, 6, true);

  return true;
}

bool ProtocolParser::isDataComplete(const BatteryRealtimeData& data, double timeout) {
  rclcpp::Time now = rclcpp::Clock().now();

  // 需要检查3个帧是否都在超时时间内更新过
  std::vector<uint32_t> required_ids = {0x4028000, 0x4038000, 0x4088000};

  for (uint32_t id : required_ids)
  {
    auto it = data.can_id_last_update.find(id);
    if (it == data.can_id_last_update.end())
    {
      return false;  // 该ID从未收到过
    }

    double elapsed = (now - it->second).seconds();
    if (elapsed > timeout)
    {
      return false;  // 超时
    }
  }

  return true;
}

uint16_t ProtocolParser::parseUint16(const uint8_t* data, int offset, bool big_endian) {
  if (big_endian)
  {
    return (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
  }
  else
  {
    return (static_cast<uint16_t>(data[offset + 1]) << 8) | data[offset];
  }
}

int16_t ProtocolParser::parseInt16(const uint8_t* data, int offset, bool big_endian) {
  return static_cast<int16_t>(parseUint16(data, offset, big_endian));
}

uint32_t ProtocolParser::parseUint32(const uint8_t* data, int offset, bool big_endian) {
  if (big_endian)
  {
    return (static_cast<uint32_t>(data[offset]) << 24) | (static_cast<uint32_t>(data[offset + 1]) << 16) |
           (static_cast<uint32_t>(data[offset + 2]) << 8) | static_cast<uint32_t>(data[offset + 3]);
  }
  else
  {
    return (static_cast<uint32_t>(data[offset + 3]) << 24) | (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) | static_cast<uint32_t>(data[offset]);
  }
}

}  // namespace kvms_battery
