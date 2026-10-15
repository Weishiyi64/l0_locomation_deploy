#include "kvms_battery/battery_driver.h"

#include <socketcan_wrapper/can_error_parser.h>

#include <boost/make_shared.hpp>
#include <std_msgs/msg/bool.hpp>

namespace kvms_battery {

BatteryDriver::BatteryDriver(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      monitor_(node),
      first_data_received_(false),
      feedback_complete_(false),
      startup_info_printed_(false),
      emergency_stop_sent_(false) {}

BatteryDriver::~BatteryDriver() {
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "Shutting down battery driver...");
  }
  if (can_manager_)
  {
    can_manager_->shutdown();
  }
  // timers automatically stop when nodes are destroyed, but safe to reset
  query_timer_.reset();
  feedback_check_timer_.reset();
}

bool BatteryDriver::init() {
  try
  {
    int error = 0;

    node_->declare_parameter<std::string>("battery.can_device", "");
    node_->declare_parameter<double>("battery.query_interval", 1.0);
    node_->declare_parameter<double>("battery.communication_timeout", 1.0);
    node_->declare_parameter<bool>("battery.enable_logging", true);

    node_->get_parameter("battery.can_device", can_device_);
    node_->get_parameter("battery.query_interval", query_interval_);
    node_->get_parameter("battery.communication_timeout", communication_timeout_);
    node_->get_parameter("battery.enable_logging", enable_logging_);

    error += static_cast<int>(!monitor_.loadConfig(node_));

    if (error > 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load all battery driver parameters");
      return false;
    }
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error in battery driver init: %s", e.what());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Battery driver configuration:");
  RCLCPP_INFO(node_->get_logger(), "  CAN device: %s", can_device_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  Query interval: %.0f s", query_interval_);
  RCLCPP_INFO(node_->get_logger(), "  Logging enabled: %s", enable_logging_ ? "true" : "false");

  can_manager_ = std::make_shared<socketcan_wrapper::SocketCANManager>();
  if (!can_manager_->init(can_device_, false))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SocketCAN for battery on device: %s", can_device_.c_str());
    return false;
  }

  can_manager_->setReceiveCallback([this](const can::Frame& frame) { this->CANRxCallback(frame); });

  if (enable_logging_)
    battery_status_pub_ = node_->create_publisher<legged_msgs::msg::BatteryStatus>("/battery/status", 10);
  emergency_stop_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/emergency_stop", 10);

  query_timer_ = node_->create_wall_timer(std::chrono::duration<double>(query_interval_),
                                          std::bind(&BatteryDriver::sendQueryFrame, this));

  feedback_check_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.5),
                                                   std::bind(&BatteryDriver::checkFeedbackComplete, this));

  RCLCPP_INFO(node_->get_logger(), "KVMS Battery Driver initialized successfully");

  ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/hardware_status/battery_ready", 1);

  // 开机查询
  sendQueryFrame();

  return true;
}

void BatteryDriver::sendQueryFrame() {
  // KVMS协议查询帧：ID=0x400FF80 (扩展帧), Data=0000000000000000
  uint8_t query_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (!can_manager_)
  {
    RCLCPP_ERROR(node_->get_logger(), "SocketCAN not initialized");
    return;
  }

  errno = 0;
  if (!can_manager_->send(0x400FF80, query_data, 8, /*is_extended=*/true))
  {
    int err = errno;
    RCLCPP_ERROR(node_->get_logger(), "[BATTERY] Failed to send CAN frame (ID=0x400FF80) on %s: %s",
                 can_device_.c_str(), strerror(err));
  }
  else
  {
    last_query_time_ = node_->now();
  }

  if (first_data_received_)
  {
    rclcpp::Time now = node_->now();
    if ((now - last_receive_time_).seconds() > communication_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(), "[BATTERY] Communication timeout (>%.1fs)", communication_timeout_);
    }
  }
}

void BatteryDriver::CANRxCallback(const can::Frame& frame) {
  // 转换为 can_msgs::msg::Frame
  // frame 已经是 socketcan_wrapper::can::Frame
  if (frame.is_error)
  {
    // std::string error_details = socketcan_wrapper::parseCanRxErrorFrame(frame);
    // RCLCPP_ERROR(node_->get_logger(),
    //              "[BATTERY] CAN Error Frame on %s - ID=0x%X, extended=%d, dlc=%d, "
    //              "data=[%02X %02X %02X %02X %02X %02X %02X %02X], Details: %s",
    //              can_device_.c_str(), frame.id, frame.is_extended, frame.dlc, frame.data[0], frame.data[1],
    //              frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7],
    //              error_details.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (parser_.parseCANFrame(frame, current_data_))
  {
    last_receive_time_ = node_->now();
    current_data_.timestamp = last_receive_time_;

    bool was_first = !first_data_received_;
    first_data_received_ = true;
    feedback_complete_ = false;

    if (!was_first || parser_.isDataComplete(current_data_))
    {
      AlarmLevel level = monitor_.checkStatus(current_data_);
      monitor_.printAlarms(current_data_, level);

      if (level == EMERGENCY && !emergency_stop_sent_)
      {
        std_msgs::msg::Float32 stop_msg;
        stop_msg.data = 1.0f;
        emergency_stop_pub_->publish(stop_msg);
        emergency_stop_sent_ = true;
        RCLCPP_ERROR(node_->get_logger(), "[BATTERY] Emergency stop signal sent!");
      }
    }

    if (parser_.isDataComplete(current_data_))
    {
      if (enable_logging_)
      {
        batterystatus_.header.stamp = current_data_.timestamp;

        batterystatus_.sum_voltage = current_data_.sum_voltage;
        batterystatus_.current = current_data_.current;
        batterystatus_.soc = current_data_.soc;

        batterystatus_.power = current_data_.power;
        batterystatus_.total_energy = current_data_.total_energy;
        batterystatus_.mos_temp = current_data_.mos_temp;
        batterystatus_.board_temp = current_data_.board_temp;

        batterystatus_.remain_capacity = current_data_.remain_capacity;
        batterystatus_.charge_cycles = current_data_.charge_cycles;

        battery_status_pub_->publish(batterystatus_);
      }
    }
  }
}

void BatteryDriver::checkFeedbackComplete() {
  if (!first_data_received_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  rclcpp::Time now = node_->now();
  double elapsed = (now - last_receive_time_).seconds();

  // 如果2秒内没有新帧，且之前标记为正在接收，则认为本轮反馈完成
  if (elapsed >= 2.0 && !feedback_complete_)
  {
    feedback_complete_ = true;

    // 只在第一次反馈完成后打印开机信息并发布ready状态
    if (!startup_info_printed_)
    {
      monitor_.printStartupInfo(current_data_);
      startup_info_printed_ = true;

      // 发布电池就绪状态
      std_msgs::msg::Bool ready_msg;
      ready_msg.data = true;
      ready_pub_->publish(ready_msg);
      RCLCPP_INFO(node_->get_logger(),
                  "\033[32m[BATTERY] Ready status published to "
                  "/hardware_status/battery_ready\033[0m");
    }
  }
}

}  // namespace kvms_battery
