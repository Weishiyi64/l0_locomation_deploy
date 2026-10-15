#pragma once

#include <socketcan_wrapper/socketcan_manager.h>

#include <legged_msgs/msg/battery_status.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "kvms_battery/battery_monitor.h"
#include "kvms_battery/protocol_parser.h"

namespace kvms_battery {

class BatteryDriver {
 public:
  explicit BatteryDriver(std::shared_ptr<rclcpp::Node> node);
  ~BatteryDriver();

  bool init();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<socketcan_wrapper::SocketCANManager> can_manager_;

  void CANRxCallback(const can::Frame& frame);
  void sendQueryFrame();

  rclcpp::Publisher<legged_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr query_timer_;

  legged_msgs::msg::BatteryStatus batterystatus_;

  std::string can_device_;
  double query_interval_;
  double communication_timeout_;
  bool enable_logging_;

  ProtocolParser parser_;
  BatteryMonitor monitor_;
  BatteryRealtimeData current_data_;

  bool first_data_received_;
  bool feedback_complete_;
  bool startup_info_printed_;
  bool emergency_stop_sent_;
  rclcpp::Time last_query_time_;
  rclcpp::Time last_receive_time_;
  rclcpp::TimerBase::SharedPtr feedback_check_timer_;
  std::mutex data_mutex_;

  void checkFeedbackComplete();
};

}  // namespace kvms_battery
