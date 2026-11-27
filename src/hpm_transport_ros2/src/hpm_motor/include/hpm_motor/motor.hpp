/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HPM_MOTOR__MOTOR_HPP_
#define HPM_MOTOR__MOTOR_HPP_

#include "hpm_transport/motor.hpp"
#include "hpm_transport/transport.hpp"
#include "luvbot_hardware_interfaces/msg/motor_command.hpp"
#include "luvbot_hardware_interfaces/msg/motor_enable_command.hpp"
#include "luvbot_hardware_interfaces/msg/motor_state_array.hpp"
#include "luvbot_hardware_interfaces/srv/motor_set_zero.hpp"
#include "luvbot_hardware_interfaces/srv/motor_calibrate_id.hpp"
#include "luvbot_hardware_interfaces/srv/motor_read_ls_error.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <variant>
#include <vector>

namespace hpmMotor
{

struct MotorDefinition
{
  uint8_t index{0U};
  std::string name;
  frameTransport::MotorType protocol{frameTransport::MotorType::MOTOR_TYPE_UNKNOWN};
  frameTransport::MotorModel model{frameTransport::MotorModel::UNKNOWN};
  uint8_t bus_index{0U};
  uint32_t motor_id{0U};
};

std::vector<MotorDefinition> load_motor_config(
  const std::string & path, frameTransport::MotorProtocolConfig * protocols = nullptr);

// Source item of a successfully transmitted topic command, before CAN quantization.
struct TxTopic {
  std::string topic;
  builtin_interfaces::msg::Time stamp;
  uint32_t sequence_id;
  uint8_t control_mode;
  uint8_t motor_type;
  std::variant<luvbot_hardware_interfaces::msg::MotorTarget, bool> item;
};

class HpmMotor
{
public:
  HpmMotor(rclcpp::Node & node, frameTransport::HpmFrameTransport & transport);

  const std::vector<MotorDefinition> & definitions() const { return motors_; }

  using TopicTxCallback = std::function<void(const frameTransport::Frame &, uint32_t, TxTopic)>;
  void set_topic_tx_callback(TopicTxCallback callback) { topic_tx_ = std::move(callback); }
  const char * state_topic() const { return state_publisher_->get_topic_name(); }
  void handle_motor_snapshot(const frameTransport::MotorSnapshot & snapshot,
                             luvbot_hardware_interfaces::msg::MotorStateArray * published);
  void handle_frame(const frameTransport::Frame & frame);
  void handle_motor_snapshot(const frameTransport::MotorSnapshot & snapshot);

private:
  void report_motor_rates();
  void handle_command(const luvbot_hardware_interfaces::msg::MotorCommand & message);
  void handle_enable(const luvbot_hardware_interfaces::msg::MotorEnableCommand & message);
  void handle_set_zero(
    const luvbot_hardware_interfaces::srv::MotorSetZero::Request & request,
    luvbot_hardware_interfaces::srv::MotorSetZero::Response & response, bool wait_for_feedback = true);
  void handle_set_zero_feedback(const frameTransport::Frame & frame);
  bool decode_motor_state(const frameTransport::Frame & frame, uint8_t index,
                          luvbot_hardware_interfaces::msg::MotorState & state);
  void complete_set_zero(uint8_t index, bool ok);
  void handle_calibrate_id(
    const luvbot_hardware_interfaces::srv::MotorCalibrateId::Request & request,
    luvbot_hardware_interfaces::srv::MotorCalibrateId::Response & response);
  bool handle_calibrate_id_feedback(const frameTransport::Frame & frame);
  using ReadLsError = luvbot_hardware_interfaces::srv::MotorReadLsError;
  void read_ls_error(std::shared_ptr<rmw_request_id_t> header, uint8_t index);
  void finish_ls_error(const frameTransport::Frame * frame);
  void reply_ls_error(rmw_request_id_t & header, ReadLsError::Response & response);
  enum class IdPhase { Query, Write, Save };
  bool exchange_id(const MotorDefinition & motor, IdPhase phase, uint16_t current_id,
    uint16_t expected_id, luvbot_hardware_interfaces::srv::MotorCalibrateId::Response & response,
    std::array<uint8_t, 8> & uid);
  const MotorDefinition * motor_by_type(uint8_t type) const;

  TopicTxCallback topic_tx_;
  rclcpp::Node & node_;
  frameTransport::HpmFrameTransport & transport_;
  std::vector<MotorDefinition> motors_;
  struct MotorCounts
  {
    std::atomic<uint32_t> tx{0};
    std::atomic<uint32_t> rx{0};
  };
  std::array<MotorCounts, 256> motor_counts_{};
  std::chrono::steady_clock::time_point rate_window_start_;
  rclcpp::CallbackGroup::SharedPtr rate_group_;
  rclcpp::TimerBase::SharedPtr rate_timer_;
  uint32_t state_sequence_{0U};
  std::mutex set_zero_mutex_;
  struct PendingSetZero
  {
    const MotorDefinition * motor;
    size_t response_index;
    bool done{false};
    uint8_t result{luvbot_hardware_interfaces::srv::MotorSetZero::Response::TIMEOUT};
    std::string message;
  };
  int set_zero_timeout_ms_{1000};
  std::mutex set_zero_feedback_mutex_;
  std::condition_variable set_zero_feedback_cv_;
  std::vector<PendingSetZero> pending_set_zero_;
  std::set<uint8_t> uncertain_set_zero_;
  std::chrono::steady_clock::time_point set_zero_deadline_;
  bool id_calibration_mode_{false};
  int id_calibration_timeout_ms_{1000};
  std::mutex id_operation_mutex_;
  std::mutex id_feedback_mutex_;
  std::condition_variable id_feedback_cv_;
  struct PendingId
  {
    const MotorDefinition * motor{nullptr};
    IdPhase phase{IdPhase::Query};
    uint16_t current_id{0U};
    uint16_t expected_id{0U};
    bool done{false};
    bool failed{false};
    std::set<uint16_t> ids;
    std::array<uint8_t, 8> uid{};
    std::chrono::steady_clock::time_point deadline;
  } pending_id_;
  rclcpp::CallbackGroup::SharedPtr id_callback_group_;
  rclcpp::Service<luvbot_hardware_interfaces::srv::MotorCalibrateId>::SharedPtr id_service_;
  std::mutex ls_error_mutex_;
  struct PendingLsError
  {
    std::shared_ptr<rmw_request_id_t> header;
    const MotorDefinition * motor{nullptr};
    std::chrono::steady_clock::time_point deadline;
  } pending_ls_error_;
  rclcpp::TimerBase::SharedPtr ls_error_timer_;
  rclcpp::Service<ReadLsError>::SharedPtr ls_error_service_;
  rclcpp::CallbackGroup::SharedPtr set_zero_callback_group_;
  rclcpp::Service<luvbot_hardware_interfaces::srv::MotorSetZero>::SharedPtr set_zero_service_;
  rclcpp::Service<luvbot_hardware_interfaces::srv::MotorSetZero>::SharedPtr set_zero_send_service_;
  rclcpp::Subscription<luvbot_hardware_interfaces::msg::MotorCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<luvbot_hardware_interfaces::msg::MotorEnableCommand>::SharedPtr enable_subscription_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorStateArray>::SharedPtr state_publisher_;
};

}  // namespace hpmMotor

#endif  // HPM_MOTOR__MOTOR_HPP_
