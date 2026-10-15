#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "kvms_battery/protocol_parser.h"

namespace kvms_battery {

enum AlarmLevel { NORMAL = 0, WARNING = 1, ERROR = 2, EMERGENCY = 3 };

class BatteryMonitor {
 public:
  explicit BatteryMonitor(std::shared_ptr<rclcpp::Node> node);

  bool loadConfig(std::shared_ptr<rclcpp::Node> node);

  AlarmLevel checkStatus(const BatteryRealtimeData& data);

  void printAlarms(const BatteryRealtimeData& data, AlarmLevel level);

  void printStartupInfo(const BatteryRealtimeData& data);

 private:
  std::shared_ptr<rclcpp::Node> node_;
  struct ThresholdConfig {
    float soc_warning;
    float soc_error;
    float soc_emergency;
  } config_;

  bool startup_info_printed_;
};

}  // namespace kvms_battery
