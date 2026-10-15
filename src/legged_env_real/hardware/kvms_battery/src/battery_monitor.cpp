#include "kvms_battery/battery_monitor.h"

#include <chrono>
#include <sstream>

namespace kvms_battery {

BatteryMonitor::BatteryMonitor(std::shared_ptr<rclcpp::Node> node) : node_(node), startup_info_printed_(false) {}

bool BatteryMonitor::loadConfig(std::shared_ptr<rclcpp::Node> node) {
  try
  {
    int error = 0;

    node->declare_parameter<double>("battery.soc.warning", 20.0);
    node->declare_parameter<double>("battery.soc.error", 10.0);
    node->declare_parameter<double>("battery.soc.emergency", 5.0);

    node->get_parameter("battery.soc.warning", config_.soc_warning);
    node->get_parameter("battery.soc.error", config_.soc_error);
    node->get_parameter("battery.soc.emergency", config_.soc_emergency);

    // no simple way of detecting missing parameter in ROS2; assume defaults
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Error in battery monitor loadConfig: %s", e.what());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Battery monitor thresholds:");
  RCLCPP_INFO(node->get_logger(), "  SOC: WARNING < %.1f%%, ERROR < %.1f%%, EMERGENCY < %.1f%%", config_.soc_warning,
              config_.soc_error, config_.soc_emergency);

  return true;
}

AlarmLevel BatteryMonitor::checkStatus(const BatteryRealtimeData& data) {
  if (data.soc < config_.soc_emergency)
  {
    return EMERGENCY;
  }
  else if (data.soc < config_.soc_error)
  {
    return ERROR;
  }
  else if (data.soc < config_.soc_warning)
  {
    return WARNING;
  }

  return NORMAL;
}

void BatteryMonitor::printAlarms(const BatteryRealtimeData& data, AlarmLevel level) {
  switch (level)
  {
    case NORMAL:
      break;

    case WARNING:
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 60000,
                           "[BATTERY] Low battery! SOC: %.1f%%, please charge soon", data.soc);
      break;

    case ERROR:
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 30000,
                            "[BATTERY] Critical battery! SOC: %.1f%%, please "
                            "shutdown and charge NOW!",
                            data.soc);
      break;

    case EMERGENCY:
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "[BATTERY EMERGENCY] Battery depleted! SOC: "
                            "%.1f%%, entering emergency stop!",
                            data.soc);
      break;
  }
}

void BatteryMonitor::printStartupInfo(const BatteryRealtimeData& data) {
  if (startup_info_printed_)
  {
    return;
  }

  // 根据SOC值选择颜色
  const char* soc_color;
  if (data.soc < config_.soc_emergency)
  {
    soc_color = "\033[1;31m";  // 加粗红色
  }
  else if (data.soc < config_.soc_error)
  {
    soc_color = "\033[31m";  // 红色
  }
  else if (data.soc < config_.soc_warning)
  {
    soc_color = "\033[33m";  // 黄色
  }
  else
  {
    soc_color = "\033[32m";  // 绿色
  }

  RCLCPP_INFO(node_->get_logger(), "\033[32m================ Battery Info =================\033[0m");
  RCLCPP_INFO(node_->get_logger(), "\033[32m  Voltage:          %.1f V\033[0m", data.sum_voltage);
  RCLCPP_INFO(node_->get_logger(), "%s  SOC:              %.1f%%\033[0m", soc_color, data.soc);
  RCLCPP_INFO(node_->get_logger(), "\033[32m  Charge Cycles:    %d\033[0m", data.charge_cycles);
  RCLCPP_INFO(node_->get_logger(), "\033[32m===============================================\033[0m");

  startup_info_printed_ = true;
}

}  // namespace kvms_battery
