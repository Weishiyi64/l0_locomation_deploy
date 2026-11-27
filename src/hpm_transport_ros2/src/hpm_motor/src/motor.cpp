/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_motor/motor.hpp"
#include "hpm_transport/motor_config.hpp"
#include "hpm_transport/udp_wire.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace hpmMotor
{
namespace
{

constexpr auto kLsAlarmTimeout = std::chrono::seconds(1);

frameTransport::MotorType parse_protocol(const std::string & value)
{
  using frameTransport::MotorType;
  if (value == "DM") return MotorType::MOTOR_TYPE_DM;
  if (value == "RS") return MotorType::MOTOR_TYPE_RS;
  if (value == "ENC") return MotorType::MOTOR_TYPE_ENC;
  if (value == "MR") return MotorType::MOTOR_TYPE_MR;
  if (value == "LS") return MotorType::MOTOR_TYPE_LS;
  throw std::runtime_error("unsupported motor_type: " + value);
}

frameTransport::MotorModel parse_model(
  const std::string & name, frameTransport::MotorType protocol, const YAML::Node & node)
{
  using frameTransport::MotorModel;
  using frameTransport::MotorType;
  if (protocol != MotorType::MOTOR_TYPE_MR) {
    return MotorModel::UNKNOWN;
  }
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("motor '" + name + "' requires a model for MR protocol");
  }
  const std::string value = node.as<std::string>();
  if (value == "PA43-10-40") return MotorModel::MR_PA43_10_40;
  if (value == "PA43-10-20.25") return MotorModel::MR_PA43_10_20_25;
  if (value == "PA35-10-8") return MotorModel::MR_PA35_10_8;
  throw std::runtime_error("motor '" + name + "' has unsupported MR model '" + value + "'");
}

const char * protocol_name(frameTransport::MotorType type)
{
  using frameTransport::MotorType;
  switch (type) {
    case MotorType::MOTOR_TYPE_DM: return "DM";
    case MotorType::MOTOR_TYPE_RS: return "RS";
    case MotorType::MOTOR_TYPE_ENC: return "ENC";
    case MotorType::MOTOR_TYPE_MR: return "MR";
    case MotorType::MOTOR_TYPE_LS: return "LS";
    default: return "?";
  }
}

uint32_t decoded_motor_id(
  frameTransport::MotorType type, const frameTransport::MotorRx & rx)
{
  using frameTransport::MotorType;
  if (type == MotorType::MOTOR_TYPE_DM || type == MotorType::MOTOR_TYPE_ENC) {
    return rx.mit.motor_id;
  }
  return rx.motor_id;
}

bool finite_target(const luvbot_hardware_interfaces::msg::MotorTarget & target)
{
  return std::isfinite(target.q) && std::isfinite(target.dq) &&
         std::isfinite(target.tau) && std::isfinite(target.kp) &&
         std::isfinite(target.kd);
}

uint32_t parse_motor_id(const YAML::Node & node)
{
  const std::string text = node.as<std::string>();
  std::size_t idx = 0;
  const unsigned long value = std::stoul(text, &idx, 0);
  if (idx != text.size()) {
    throw std::runtime_error("invalid motor_id: " + text);
  }
  return static_cast<uint32_t>(value);
}

}  // namespace

std::vector<MotorDefinition> load_motor_config(
  const std::string & path, frameTransport::MotorProtocolConfig * protocols)
{
  const YAML::Node root = YAML::LoadFile(path);
  const auto protocol_config = frameTransport::parse_motor_protocol_config(root, path);
  const YAML::Node motors = root["motors"];
  if (!motors || !motors.IsMap()) {
    throw std::runtime_error("motor config must contain a 'motors' map");
  }

  std::vector<MotorDefinition> result;
  std::set<std::string> names;
  std::set<uint8_t> indices;
  std::set<std::pair<uint8_t, uint32_t>> routes;
  for (const auto & item : motors) {
    MotorDefinition motor;
    motor.name = item.first.as<std::string>();
    if (motor.name.empty()) {
      throw std::runtime_error("motor alias must not be empty");
    }
    if (!names.emplace(motor.name).second) {
      throw std::runtime_error("duplicate motor alias '" + motor.name + "'");
    }
    const YAML::Node config = item.second;
    const unsigned int index = config["index"].as<unsigned int>();
    if (index > 255U) {
      throw std::runtime_error("motor '" + motor.name + "' index must be 0..255");
    }
    motor.index = static_cast<uint8_t>(index);
    if (!indices.emplace(motor.index).second) {
      throw std::runtime_error("duplicate motor index " + std::to_string(index));
    }
    motor.protocol = parse_protocol(config["motor_type"].as<std::string>());
    motor.model = parse_model(motor.name, motor.protocol, config["model"]);
    const unsigned int bus_index = config["bus_index"].as<unsigned int>();
    if (bus_index > 7U) {
      throw std::runtime_error("motor '" + motor.name + "' bus_index must be 0..7");
    }
    motor.bus_index = static_cast<uint8_t>(bus_index);
    motor.motor_id = parse_motor_id(config["motor_id"]);
    if (!routes.emplace(motor.bus_index, motor.motor_id).second) {
      throw std::runtime_error("duplicate bus_index/motor_id route for motor '" + motor.name + "'");
    }
    result.push_back(std::move(motor));
  }
  if (result.empty()) {
    throw std::runtime_error("motor config must contain at least one motor");
  }
  if (protocols != nullptr) {
    *protocols = protocol_config;
  }
  return result;
}

HpmMotor::HpmMotor(rclcpp::Node & node, frameTransport::HpmFrameTransport & transport)
: node_(node), transport_(transport)
{
  const std::string config_path =
    node_.declare_parameter<std::string>("motor_config_file", "");
  if (config_path.empty()) {
    throw std::runtime_error("motor_config_file must not be empty");
  }
  frameTransport::MotorProtocolConfig protocols;
  motors_ = load_motor_config(config_path, &protocols);
  rcl_interfaces::msg::ParameterDescriptor firmware_descriptor;
  firmware_descriptor.read_only = true;
  firmware_descriptor.description = "LS type 1 feedback firmware format: old or new";
  const auto ls_firmware_version = node_.declare_parameter<std::string>(
    "ls_firmware_version", "old", firmware_descriptor);
  if (ls_firmware_version != "old" && ls_firmware_version != "new") {
    throw std::invalid_argument("ls_firmware_version must be 'old' or 'new'");
  }
  protocols.ls_type1_new_firmware = ls_firmware_version == "new";
  transport_.set_motor_protocol_config(protocols);
  RCLCPP_INFO(node_.get_logger(), "LS type 1 feedback firmware: %s", ls_firmware_version.c_str());

  rcl_interfaces::msg::ParameterDescriptor id_descriptor;
  id_descriptor.read_only = true;
  id_descriptor.description = "ID commissioning mode: blocks motor motion, enable and SETZERO";
  id_calibration_mode_ = node_.declare_parameter<bool>(
    "motor_id_calibration_mode", false, id_descriptor);
  id_descriptor.description = "Per-stage ID feedback deadline in milliseconds";
  id_calibration_timeout_ms_ = node_.declare_parameter<int>(
    "motor_id_calibration_timeout_ms", 1000, id_descriptor);
  if (id_calibration_timeout_ms_ <= 0) {
    throw std::invalid_argument("motor_id_calibration_timeout_ms must be positive");
  }
  id_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  id_service_ = node_.create_service<luvbot_hardware_interfaces::srv::MotorCalibrateId>(
    "/actuator/motor/calibrate_id",
    [this](const luvbot_hardware_interfaces::srv::MotorCalibrateId::Request::SharedPtr request,
      luvbot_hardware_interfaces::srv::MotorCalibrateId::Response::SharedPtr response) {
      handle_calibrate_id(*request, *response);
    }, rmw_qos_profile_services_default, id_callback_group_);
  ls_error_timer_ = node_.create_wall_timer(kLsAlarmTimeout, [this]() { finish_ls_error(nullptr); });
  ls_error_timer_->cancel();
  ls_error_service_ = node_.create_service<ReadLsError>(
    "/actuator/motor/read_ls_error",
    [this](std::shared_ptr<rmw_request_id_t> header, ReadLsError::Request::SharedPtr request) {
      read_ls_error(std::move(header), request->motor_index);
    });
  if (id_calibration_mode_) {
    RCLCPP_WARN(node_.get_logger(),
      "ID calibration mode: motor control, enable/disable and SETZERO are blocked. "
      "Only power the motor being commissioned; restart in normal mode afterwards.");
  }

  rcl_interfaces::msg::ParameterDescriptor timeout_descriptor;
  timeout_descriptor.description =
    "Batch SETZERO feedback deadline in milliseconds";
  timeout_descriptor.read_only = true;
  set_zero_timeout_ms_ = node_.declare_parameter<int>(
    "motor_set_zero_timeout_ms", 1000, timeout_descriptor);
  if (set_zero_timeout_ms_ <= 0)
  {
    throw std::invalid_argument("motor_set_zero_timeout_ms must be positive");
  }
  set_zero_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  set_zero_service_ = node_.create_service<luvbot_hardware_interfaces::srv::MotorSetZero>(
    "/actuator/motor/set_zero",
    [this](
      const luvbot_hardware_interfaces::srv::MotorSetZero::Request::SharedPtr request,
      luvbot_hardware_interfaces::srv::MotorSetZero::Response::SharedPtr response)
    {
      handle_set_zero(*request, *response);
    }, rmw_qos_profile_services_default, set_zero_callback_group_);
  set_zero_send_service_ = node_.create_service<luvbot_hardware_interfaces::srv::MotorSetZero>(
    "/actuator/motor/set_zero_send",
    [this](const luvbot_hardware_interfaces::srv::MotorSetZero::Request::SharedPtr request,
      luvbot_hardware_interfaces::srv::MotorSetZero::Response::SharedPtr response) {
      handle_set_zero(*request, *response, false);
    }, rmw_qos_profile_services_default, set_zero_callback_group_);

  command_subscription_ = node_.create_subscription<luvbot_hardware_interfaces::msg::MotorCommand>(
    "/actuator/motor/command", rclcpp::QoS(10).reliable(),
    [this](const luvbot_hardware_interfaces::msg::MotorCommand & message) {
      handle_command(message);
    });
  enable_subscription_ = node_.create_subscription<luvbot_hardware_interfaces::msg::MotorEnableCommand>(
    "/actuator/motor/enable", rclcpp::QoS(10).reliable(),
    [this](const luvbot_hardware_interfaces::msg::MotorEnableCommand & message) {
      handle_enable(message);
    });
  state_publisher_ = node_.create_publisher<luvbot_hardware_interfaces::msg::MotorStateArray>(
    "/actuator/motor/state", rclcpp::SensorDataQoS());
  rate_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rate_window_start_ = std::chrono::steady_clock::now();
  rate_timer_ = node_.create_wall_timer(std::chrono::seconds(10),
    [this]() { report_motor_rates(); }, rate_group_);

  RCLCPP_INFO(node_.get_logger(), "Motor module loaded %zu configured motors", motors_.size());
  for (const auto & motor : motors_) {
    RCLCPP_INFO(
      node_.get_logger(),
      "motor_type=%u %s protocol=%s model=%u mcan=%u motor_id=0x%02X",
      motor.index, motor.name.c_str(), protocol_name(motor.protocol),
      static_cast<unsigned>(motor.model), motor.bus_index, motor.motor_id);
  }
}

void HpmMotor::report_motor_rates()
{
  const auto now = std::chrono::steady_clock::now();
  const double seconds = std::chrono::duration<double>(now - rate_window_start_).count();
  rate_window_start_ = now;
  std::size_t name_width = 0;
  for (const auto & motor : motors_) {
    name_width = std::max(name_width, motor.name.size());
  }
  const std::string separator(44 + name_width, '-');
  std::ostringstream report;
  report << separator << "\nMotor rates (window=" << std::fixed << std::setprecision(2)
         << seconds << " s):";
  for (const auto & motor : motors_) {
    auto & counts = motor_counts_[motor.index];
    const auto tx = counts.tx.exchange(0, std::memory_order_relaxed);
    const auto rx = counts.rx.exchange(0, std::memory_order_relaxed);
    report << "\n  index=" << std::left << std::setw(3) << unsigned(motor.index) << ' '
           << std::setw(name_width) << motor.name << std::right
           << "  TX=" << std::setw(8) << tx / seconds << " Hz"
           << "  RX=" << std::setw(8) << rx / seconds << " Hz";
  }
  report << '\n' << separator;
  RCLCPP_INFO(node_.get_logger(), "%s", report.str().c_str());
}

void HpmMotor::handle_command(const luvbot_hardware_interfaces::msg::MotorCommand & message)
{
  if (id_calibration_mode_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Motor control rejected in ID calibration mode");
    return;
  }
  if (message.control_mode != luvbot_hardware_interfaces::msg::MotorCommand::CONTROL_MODE_MIT) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 2000,
      "Ignoring unsupported motor control_mode=%u", message.control_mode);
    return;
  }

  std::vector<frameTransport::HpmFrameTransport::MotorControl> controls;
  std::array<const luvbot_hardware_interfaces::msg::MotorTarget *, UDP_WIRE_MAX_FRAMES> targets;
  controls.reserve(std::min(targets.size(), message.commands.size()));
  for (const auto & target : message.commands) {
    const MotorDefinition * motor = motor_by_type(target.route.motor_type);
    if (motor == nullptr) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Ignoring command for unknown motor_type=%u", target.route.motor_type);
      continue;
    }
    if (!finite_target(target)) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Ignoring non-finite command for motor_type=%u (%s)",
        motor->index, motor->name.c_str());
      continue;
    }

    frameTransport::mitctrl_t command;
    command.p_des = static_cast<float>(target.q);
    command.v_des = static_cast<float>(target.dq);
    command.t_ff = static_cast<float>(target.tau);
    command.kp = static_cast<float>(target.kp);
    command.kd = static_cast<float>(target.kd);
    command.model = motor->model;
    if (controls.size() == targets.size()) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
        "MIT command batch exceeds %zu frames; not sent", targets.size());
      return;
    }
    targets[controls.size()] = &target;
    controls.push_back({motor->protocol, motor->motor_id, motor->bus_index, command});
  }
  if (controls.empty()) {
    return;
  }
  frameTransport::HpmFrameTransport::BatchTxCallback on_sent;
  if (topic_tx_) {
    on_sent = [&](size_t pos, const auto &frame, uint32_t id) {
      const auto &target = *targets[pos];
      topic_tx_(frame, id, {command_subscription_->get_topic_name(), message.stamp,
        message.sequence_id, message.control_mode, target.route.motor_type, target});
    };
  }
  if (!transport_.send_motor_controls(controls, on_sent)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Failed to send MIT command batch (%zu motors)", controls.size());
    return;
  }
  for (size_t pos = 0; pos < controls.size(); ++pos) {
    motor_counts_[targets[pos]->route.motor_type].tx.fetch_add(1, std::memory_order_relaxed);
  }
}

void HpmMotor::handle_enable(const luvbot_hardware_interfaces::msg::MotorEnableCommand & message)
{
  if (id_calibration_mode_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Motor enable/disable rejected in ID calibration mode");
    return;
  }
  if (message.motor_types.size() != message.enabled.size()) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 2000,
      "Ignoring motor enable message: motor_types and enabled sizes differ");
    return;
  }

  std::vector<frameTransport::HpmFrameTransport::MotorEnable> commands;
  std::array<uint8_t, UDP_WIRE_MAX_FRAMES> types;
  commands.reserve(std::min(types.size(), message.motor_types.size()));
  for (size_t i = 0U; i < message.motor_types.size(); ++i) {
    const MotorDefinition * motor = motor_by_type(message.motor_types[i]);
    if (motor == nullptr) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Ignoring enable command for unknown motor_type=%u", message.motor_types[i]);
      continue;
    }
    if (commands.size() == types.size()) {
      RCLCPP_WARN(node_.get_logger(), "Enable batch exceeds maximum UDP frame count");
      return;
    }
    types[commands.size()] = motor->index;
    commands.push_back({motor->protocol, motor->motor_id, motor->bus_index,
                         message.enabled[i]});
  }
  frameTransport::HpmFrameTransport::BatchTxCallback on_sent;
  if (topic_tx_) {
    on_sent = [&](size_t pos, const auto &frame, uint32_t id) {
      topic_tx_(frame, id, {enable_subscription_->get_topic_name(), message.stamp,
        message.sequence_id, 0, types[pos], commands[pos].enabled});
    };
  }
  if (!commands.empty() && !transport_.send_motor_enables(commands, on_sent)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Failed to send motor enable/disable batch (%zu motors)", commands.size());
  }
}

void HpmMotor::handle_set_zero(
  const luvbot_hardware_interfaces::srv::MotorSetZero::Request & request,
  luvbot_hardware_interfaces::srv::MotorSetZero::Response & response, bool wait_for_feedback)
{
  using Response = luvbot_hardware_interfaces::srv::MotorSetZero::Response;
  response.motor_types = request.motor_types;
  response.results.assign(request.motor_types.size(), Response::REJECTED);
  response.messages.assign(request.motor_types.size(), "Another set_zero batch is in progress");
  if (id_calibration_mode_) {
    response.messages.assign(request.motor_types.size(), "SETZERO rejected in ID calibration mode");
    return;
  }

  if (request.motor_types.empty()) {
    // This per-motor interface has no result slot for an empty request.
    RCLCPP_WARN(node_.get_logger(), "Rejected empty set_zero request");
    return;
  }
  const std::set<uint8_t> unique_types(request.motor_types.begin(), request.motor_types.end());
  if (unique_types.size() != request.motor_types.size()) {
    response.messages.assign(request.motor_types.size(), "Duplicate motor index; entire batch rejected");
    return;
  }

  std::unique_lock<std::mutex> batch_lock(set_zero_mutex_, std::try_to_lock);
  if (!batch_lock.owns_lock()) {
    return;
  }
  // ponytail: one global batch; per-bus concurrency only if calibration throughput requires it.
  std::unique_lock<std::mutex> feedback_lock(set_zero_feedback_mutex_);
  pending_set_zero_.clear();
  set_zero_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(set_zero_timeout_ms_);
  for (size_t i = 0U; i < request.motor_types.size(); ++i) {
    const MotorDefinition * motor = motor_by_type(request.motor_types[i]);
    if (motor == nullptr) {
      response.messages[i] = "Unknown motor index " + std::to_string(request.motor_types[i]);
      continue;
    }

    if (uncertain_set_zero_.count(motor->index) != 0U) {
      response.messages[i] = "Previous SETZERO outcome is unknown; verify motor/bus before restarting node";
      continue;
    }
    if (std::chrono::steady_clock::now() >= set_zero_deadline_) {
      response.messages[i] = "Batch deadline reached before command could be sent";
      continue;
    }

    if (!wait_for_feedback) {
      const bool sent = transport_.send_motorcmd(
        motor->protocol, frameTransport::MotorCommand::MOTOR_CMD_SETZERO,
        motor->motor_id, motor->bus_index);
      // On set_zero_send, result=0 confirms transport submission only.
      response.results[i] = sent ? Response::CONFIRMED : Response::SEND_FAILED;
      response.messages[i] = sent ? "SETZERO sent; device write not verified" :
        "Failed to send SETZERO; device outcome unknown";
      if (!sent) {
        uncertain_set_zero_.insert(motor->index);
      }
      continue;
    }

    pending_set_zero_.push_back({motor, i, false, Response::TIMEOUT, ""});
    // Register before sending, holding the feedback mutex through the send so
    // an immediate reply cannot be lost. wait_until below releases this mutex.
    if (!transport_.send_motorcmd(
        motor->protocol, frameTransport::MotorCommand::MOTOR_CMD_SETZERO,
        motor->motor_id, motor->bus_index))
    {
      auto & pending = pending_set_zero_.back();
      pending.done = true;
      pending.result = Response::SEND_FAILED;
      pending.message = "Failed to send SETZERO; motor outcome may be unknown";
      // A failed socket write may already have transmitted part of the frame.
      uncertain_set_zero_.insert(motor->index);
    }
  }
  if (!wait_for_feedback) {
    return;
  }
  set_zero_feedback_cv_.wait_until(feedback_lock, set_zero_deadline_, [this]() {
      return std::all_of(pending_set_zero_.begin(), pending_set_zero_.end(),
        [](const PendingSetZero & pending) { return pending.done; });
    });
  for (const auto & pending : pending_set_zero_) {
    response.results[pending.response_index] = pending.result;
    response.messages[pending.response_index] = pending.done ? pending.message :
      "Timed out waiting for motor feedback; SETZERO outcome unknown";
    // LS retries the same idempotent zero command while held at the stop.
    // Keep transport-write failures and other protocols quarantined as before.
    if (!pending.done && pending.motor->protocol != frameTransport::MotorType::MOTOR_TYPE_LS) {
      uncertain_set_zero_.insert(pending.motor->index);
    }
  }
  pending_set_zero_.clear();
}

void HpmMotor::handle_set_zero_feedback(const frameTransport::Frame & frame)
{
  if (frame.msg_id < 0x10U || frame.msg_id > 0x1FU) {
    return;
  }
  using frameTransport::MotorType;
  using Response = luvbot_hardware_interfaces::srv::MotorSetZero::Response;
  std::lock_guard<std::mutex> lock(set_zero_feedback_mutex_);
  if (std::chrono::steady_clock::now() >= set_zero_deadline_) {
    return;
  }
  for (auto & pending : pending_set_zero_) {
    const auto & motor = *pending.motor;
    if (pending.done || motor.bus_index != frame.port) {
      continue;
    }
    bool ok = false;
    if (motor.protocol == MotorType::MOTOR_TYPE_ENC || motor.protocol == MotorType::MOTOR_TYPE_LS) {
      // ENC/LS share this reply format; no protocol discriminator is required.
      const uint32_t id = (static_cast<uint32_t>(frame.data[0]) << 8U) | frame.data[1];
      if (frame.ide != frameTransport::kFrameIdeStandard || frame.can_id != 0x7FFU ||
          frame.dlc != 4U || id != motor.motor_id || frame.data[2] != 0x01U ||
          (frame.data[3] != 0x03U && frame.data[3] != 0x00U))
      {
        continue;
      }
      frameTransport::MotorRx rx;
      if (!transport_.unpack_motorcmd(
          motor.protocol, frameTransport::MotorCommand::MOTOR_CMD_SETZERO, frame, rx))
      {
        continue;
      }
      ok = rx.ok;
    } else {
      continue;  // MIT feedback uses the existing unambiguous state matching path.
    }
    pending.done = true;
    pending.result = ok ? Response::CONFIRMED : Response::DEVICE_FAILED;
    pending.message = ok ? "Received matching motor feedback after SETZERO" :
      "Motor feedback reports SETZERO failure or fault";
  }
  set_zero_feedback_cv_.notify_all();
}

void HpmMotor::complete_set_zero(uint8_t index, bool ok)
{
  using Response = luvbot_hardware_interfaces::srv::MotorSetZero::Response;
  std::lock_guard<std::mutex> lock(set_zero_feedback_mutex_);
  if (std::chrono::steady_clock::now() >= set_zero_deadline_) {
    return;
  }
  for (auto & pending : pending_set_zero_) {
    if (!pending.done && pending.motor->index == index) {
      pending.done = true;
      pending.result = ok ? Response::CONFIRMED : Response::DEVICE_FAILED;
      pending.message = ok ? "Received matching MIT feedback after SETZERO" :
        "MIT feedback reports a motor fault";
      set_zero_feedback_cv_.notify_all();
      return;
    }
  }
}

void HpmMotor::handle_calibrate_id(
  const luvbot_hardware_interfaces::srv::MotorCalibrateId::Request & request,
  luvbot_hardware_interfaces::srv::MotorCalibrateId::Response & response)
{
  using Service = luvbot_hardware_interfaces::srv::MotorCalibrateId;
  using Response = Service::Response;
  using frameTransport::MotorType;
  response.result = Response::REJECTED;
  response.stage = "validate";
  const MotorDefinition * motor = motor_by_type(request.motor_index);
  if (motor == nullptr) {
    response.message = "Unknown motor index";
    return;
  }
  response.motor_name = motor->name;
  response.motor_type = protocol_name(motor->protocol);
  response.model = motor->model == frameTransport::MotorModel::MR_PA43_10_40 ?
    "PA43-10-40" : motor->model == frameTransport::MotorModel::MR_PA43_10_20_25 ?
    "PA43-10-20.25" : motor->model == frameTransport::MotorModel::MR_PA35_10_8 ?
    "PA35-10-8" : "";
  response.bus_index = motor->bus_index;
  if (motor->motor_id > 0xFFFFU) {
    response.message = "Configured target ID exceeds uint16 range";
    return;
  }
  response.target_id = static_cast<uint16_t>(motor->motor_id);
  if (request.operation != Service::Request::QUERY &&
      request.operation != Service::Request::CONFIGURE) {
    response.message = "Unknown operation";
    return;
  }
  if (motor->protocol != MotorType::MOTOR_TYPE_RS &&
      motor->protocol != MotorType::MOTOR_TYPE_MR &&
      motor->protocol != MotorType::MOTOR_TYPE_LS) {
    response.result = Response::UNSUPPORTED;
    response.message = "ID calibration supports RS, MR and LS only";
    return;
  }
  const uint32_t max_id = motor->protocol == MotorType::MOTOR_TYPE_LS ? 0x7FFU : 0xFFU;
  if (request.current_id == 0U || request.current_id > max_id ||
      motor->motor_id == 0U || motor->motor_id > max_id) {
    response.message = "Current/target ID outside protocol range (RS/MR 1..255, LS 1..2047)";
    return;
  }
  const bool configure = request.operation == Service::Request::CONFIGURE;
  if (configure && !id_calibration_mode_) {
    response.message = "Restart node with motor_id_calibration_mode=true before ID writes";
    return;
  }
  // ponytail: one global ID transaction; per-bus concurrency only if commissioning needs it.
  std::unique_lock<std::mutex> operation_lock(id_operation_mutex_, std::try_to_lock);
  if (!operation_lock.owns_lock()) {
    response.message = "Another ID operation is in progress";
    return;
  }
  std::array<uint8_t, 8> uid{};
  std::string stages;
  const auto run_stage = [&](IdPhase phase, uint16_t current, uint16_t expected) {
      const bool ok = exchange_id(*motor, phase, current, expected, response, uid);
      stages += response.stage + "=" + std::to_string(response.result) + "; ";
      // The final response/log includes every completed stage without changing the srv schema.
      response.message = stages + response.message;
      return ok;
    };
  response.stage = "query_current";
  if (!run_stage(IdPhase::Query, request.current_id, request.current_id)) {
    return;
  }
  if (!configure) {
    return;
  }
  if (response.detected_ids.size() != 1U || response.detected_ids.front() != request.current_id) {
    response.result = Response::REJECTED;
    response.message = stages + "Configuration requires exactly the requested current ID; isolate this motor";
    return;
  }
  if (request.current_id != response.target_id) {
    response.stage = "write_id";
    if (!run_stage(IdPhase::Write, request.current_id, response.target_id)) {
      response.message += "; ID may have changed. Query old/new IDs before another explicit attempt";
      return;
    }
    // LS commissioning requires a power cycle before checking the active ID.
    // A matching write acknowledgement confirms acceptance, not persistence.
    if (motor->protocol != MotorType::MOTOR_TYPE_LS) {
      response.stage = "query_new";
      if (!run_stage(IdPhase::Query, response.target_id, response.target_id)) {
        response.message += "; ID write was acknowledged but new ID verification failed";
        return;
      }
      if (response.detected_ids.size() != 1U || response.detected_ids.front() != response.target_id) {
        response.result = Response::REJECTED;
        response.message = stages + "New ID inventory differs from the target; no further commands sent";
        return;
      }
    }
    if (motor->protocol == MotorType::MOTOR_TYPE_MR) {
      response.stage = "save_id";
      if (!run_stage(IdPhase::Save, response.target_id, response.target_id)) {
        response.message += "; ID changed but Flash persistence is unknown; do not retry automatically";
        return;
      }
    }
  }
  response.result = Response::CONFIRMED;
  response.stage = "await_power_cycle";
  response.power_cycle_required = true;
  response.message = stages +
    "ID operation acknowledged only. Power cycle, then query three times to verify the active ID and persistence";
}

bool HpmMotor::exchange_id(
  const MotorDefinition & motor, IdPhase phase, uint16_t current_id, uint16_t expected_id,
  luvbot_hardware_interfaces::srv::MotorCalibrateId::Response & response, std::array<uint8_t, 8> & uid)
{
  using Response = luvbot_hardware_interfaces::srv::MotorCalibrateId::Response;
  using frameTransport::MotorCommand;
  std::unique_lock<std::mutex> lock(id_feedback_mutex_);
  pending_id_ = PendingId{};
  pending_id_.motor = &motor;
  pending_id_.phase = phase;
  pending_id_.current_id = current_id;
  pending_id_.expected_id = expected_id;
  pending_id_.uid = uid;
  // Read-only inventory queries collect at most 200 ms. Keep the configured
  // acknowledgement deadline for ID writes and Flash saves unchanged.
  pending_id_.deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(phase == IdPhase::Query ?
      std::min(200, id_calibration_timeout_ms_) : id_calibration_timeout_ms_);
  response.detected_ids.clear();
  // Register before send; waiting releases the receive-path mutex.
  const bool sent = phase == IdPhase::Write ?
    transport_.send_motor_id_write(motor.protocol, current_id, expected_id, motor.bus_index) :
    transport_.send_motorcmd(motor.protocol,
      phase == IdPhase::Query ? MotorCommand::MOTOR_CMD_GETID : MotorCommand::MOTOR_CMD_SAVEREG,
      current_id, motor.bus_index);
  if (!sent) {
    pending_id_.motor = nullptr;
    response.result = Response::SEND_FAILED;
    response.message = "Transport send failed; delivery and device outcome may be unknown";
    RCLCPP_WARN(node_.get_logger(), "ID calibration index=%u stage=%s result=%u: %s",
      motor.index, response.stage.c_str(), response.result, response.message.c_str());
    return false;
  }
  // Queries collect the whole window: LS broadcast may return more than one ID,
  // and RS can reveal different MCU UIDs responding at the same ID.
  id_feedback_cv_.wait_until(lock, pending_id_.deadline, [this, phase]() {
      return pending_id_.failed || (phase != IdPhase::Query && pending_id_.done);
    });
  response.detected_ids.assign(pending_id_.ids.begin(), pending_id_.ids.end());
  response.result = pending_id_.failed ? Response::DEVICE_FAILED :
    pending_id_.done ? Response::CONFIRMED : Response::TIMEOUT;
  response.message = pending_id_.failed ? "ID response reports failure or MCU identity mismatch" :
    pending_id_.done ? "Received matching protocol ID response" :
    "Timed out waiting for ID response; outcome unknown";
  uid = pending_id_.uid;
  pending_id_.motor = nullptr;
  RCLCPP_INFO(node_.get_logger(), "ID calibration index=%u bus=%u current=0x%X expected=0x%X stage=%s result=%u: %s",
    motor.index, motor.bus_index, current_id, expected_id, response.stage.c_str(),
    response.result, response.message.c_str());
  return response.result == Response::CONFIRMED;
}

bool HpmMotor::handle_calibrate_id_feedback(const frameTransport::Frame & frame)
{
  using frameTransport::MotorType;
  if (frame.msg_id < 0x10U || frame.msg_id > 0x1FU) {
    return false;
  }
  std::lock_guard<std::mutex> lock(id_feedback_mutex_);
  auto & pending = pending_id_;
  if (pending.motor == nullptr || frame.port != pending.motor->bus_index ||
      std::chrono::steady_clock::now() >= pending.deadline) {
    return false;
  }
  const auto type = pending.motor->protocol;
  uint16_t id = 0U;
  bool ok = false;
  if (type == MotorType::MOTOR_TYPE_RS) {
    if (frame.ide != frameTransport::kFrameIdeExtended || frame.dlc != 8U ||
        frame.can_id != ((static_cast<uint32_t>(pending.expected_id) << 8U) | 0xFEU)) {
      return false;
    }
    // MCU UID, not a MIT/status frame or our all-zero query command.
    const bool empty = std::all_of(frame.data.begin(), frame.data.end(),
      [](uint8_t byte) { return byte == 0U; });
    const bool erased = std::all_of(frame.data.begin(), frame.data.end(),
      [](uint8_t byte) { return byte == 0xFFU; });
    if (empty || erased) {
      return true;
    }
    const std::array<uint8_t, 8> unset{};
    if (pending.uid != unset && pending.uid != frame.data) {
      pending.failed = true;
    } else {
      pending.uid = frame.data;
      id = pending.expected_id;
      ok = true;
    }
  } else if (type == MotorType::MOTOR_TYPE_MR) {
    if (frame.ide != frameTransport::kFrameIdeStandard || frame.dlc != 8U ||
        (frame.can_id != 0x600U + pending.expected_id && frame.can_id != pending.expected_id)) {
      return false;
    }
    uint8_t returned_id = 0U;
    uint16_t addr = 0U;
    uint32_t value = 0U;
    if (frameTransport::motor_unpack_regr(type, frame.data.data(), returned_id, addr, value) != 0 ||
        returned_id != pending.expected_id ||
        addr != (pending.phase == IdPhase::Save ? 0U : 36U) ||
        value != (pending.phase == IdPhase::Save ? 0U : pending.expected_id)) {
      return false;
    }
    id = returned_id;
    ok = true;
  } else if (type == MotorType::MOTOR_TYPE_LS) {
    if (frame.ide != frameTransport::kFrameIdeStandard || frame.can_id != 0x7FFU) {
      return false;
    }
    if (pending.phase == IdPhase::Query) {
      if (frame.dlc != 5U || frame.data[0] != 0xFFU || frame.data[1] != 0xFFU ||
          frame.data[2] != 0x01U) {
        return false;
      }
      frameTransport::motor_unpack_getid(type, frame.data.data(), id, ok);
      if (id == 0U || id > 0x7FFU) {
        return false;
      }
    } else {
      id = static_cast<uint16_t>((frame.data[0] << 8U) | frame.data[1]);
      if (frame.dlc != 4U || id != pending.expected_id || frame.data[2] != 0x01U ||
          (frame.data[3] != 0x04U && frame.data[3] != 0x00U)) {
        return false;
      }
      frameTransport::motor_unpack_setid(type, frame.data.data(), id, ok);
      pending.failed = !ok;
    }
  } else {
    return false;
  }
  if (ok) {
    pending.ids.insert(id);
    pending.done = true;
  }
  id_feedback_cv_.notify_all();
  return true;
}

void HpmMotor::read_ls_error(std::shared_ptr<rmw_request_id_t> header, uint8_t index)
{
  ReadLsError::Response response;
  const auto * motor = motor_by_type(index);
  if (motor == nullptr) {
    response.message = "motor_index 未配置，未发送";
  } else if (motor->protocol != frameTransport::MotorType::MOTOR_TYPE_LS) {
    response.message = motor->name + " 是 " + protocol_name(motor->protocol) + " 电机，不是 LS，未发送";
  } else if (motor->motor_id == 0U || motor->motor_id > 0x7FEU) {
    response.message = "LS 单机 CAN ID 必须在 0x001..0x7FE，未发送";
  } else {
    std::lock_guard<std::mutex> lock(ls_error_mutex_);
    if (pending_ls_error_.header) {
      response.message = "已有 LS 报警查询进行中，未发送";
    } else {
      if (transport_.send_motorcmd(motor->protocol, frameTransport::MotorCommand::MOTOR_CMD_READ_LS_ERROR,
          motor->motor_id, motor->bus_index)) {
        pending_ls_error_ = {std::move(header), motor,
          std::chrono::steady_clock::now() + kLsAlarmTimeout};
        ls_error_timer_->reset();
        return;
      }
      response.message = "HPM 查询帧发送失败，请检查业务节点的现有连接";
    }
  }
  reply_ls_error(*header, response);
}

void HpmMotor::finish_ls_error(const frameTransport::Frame * frame)
{
  std::shared_ptr<rmw_request_id_t> header;
  ReadLsError::Response response;
  {
    std::lock_guard<std::mutex> lock(ls_error_mutex_);
    auto & pending = pending_ls_error_;
    if (!pending.header) {
      return;
    }
    const auto & motor = *pending.motor;
    const bool expired = std::chrono::steady_clock::now() >= pending.deadline;
    if ((frame == nullptr && !expired) ||
        (frame != nullptr && (frame->port != motor.bus_index || frame->can_id != motor.motor_id))) {
      return;
    }
    response.motor_name = motor.name;
    response.bus_index = motor.bus_index;
    response.motor_id = motor.motor_id;
    if (frame != nullptr && !expired) {
      response.success = true;
      response.error_code = frame->data[0] & 0x07U;
      response.alarm_code = static_cast<uint16_t>((frame->data[2] << 8U) | frame->data[3]);
    } else {
      response.message = "超时：未收到目标电机匹配的 LS 报警回包";
    }
    header = std::move(pending.header);
    ls_error_timer_->cancel();
  }
  reply_ls_error(*header, response);
}

void HpmMotor::reply_ls_error(rmw_request_id_t & header, ReadLsError::Response & response)
{
  try {
    ls_error_service_->send_response(header, response);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_.get_logger(), "Could not deliver LS alarm query response: %s", error.what());
  }
}

void HpmMotor::handle_frame(const frameTransport::Frame & frame)
{
  if ((frame.msg_id & 0xF0U) == 0x10U && frame.ide == frameTransport::kFrameIdeStandard &&
      frame.dlc == 4U && (frame.data[0] >> 5U) == 5U && frame.data[1] == frameTransport::kLsAlarmQuery) {
    finish_ls_error(&frame);
  }
  if (handle_calibrate_id_feedback(frame)) {
    return;
  }
  handle_set_zero_feedback(frame);
}

bool HpmMotor::decode_motor_state(
  const frameTransport::Frame & frame, uint8_t index,
  luvbot_hardware_interfaces::msg::MotorState & state)
{
  if (frame.msg_id < 0x10U || frame.msg_id > 0x1FU || frame.dlc != 8U) {
    return false;
  }

  const MotorDefinition * matched = nullptr;
  frameTransport::MotorRx matched_rx;
  for (const auto & motor : motors_) {
    if (motor.index != index || motor.bus_index != frame.port ||
        (motor.motor_id & 0x0FU) != static_cast<uint32_t>(frame.msg_id - 0x10U))
    {
      continue;
    }
    frameTransport::MotorRx rx;
    if (!transport_.unpack_motorcmd(
        motor.protocol, frameTransport::MotorCommand::MOTOR_CMD_CTRL, frame, rx, motor.model) ||
        decoded_motor_id(motor.protocol, rx) != motor.motor_id)
    {
      continue;
    }
    if (matched != nullptr) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Ambiguous MIT feedback for index=%u bus=%u msg_id=0x%02X",
        index, frame.port, frame.msg_id);
      return false;
    }
    matched = &motor;
    matched_rx = rx;
  }

  if (matched == nullptr) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 2000,
      "No configured motor matches snapshot index=%u bus=%u can_id=0x%X msg_id=0x%02X",
      index, frame.port, frame.can_id, frame.msg_id);
    return false;
  }

  using frameTransport::MotorType;
  const std::array<uint8_t, 8> set_zero_command{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}};
  if ((matched->protocol == MotorType::MOTOR_TYPE_DM ||
       matched->protocol == MotorType::MOTOR_TYPE_RS ||
       matched->protocol == MotorType::MOTOR_TYPE_MR) &&
      frame.ide == (matched->protocol == MotorType::MOTOR_TYPE_RS ?
        frameTransport::kFrameIdeExtended : frameTransport::kFrameIdeStandard) &&
      frame.data != set_zero_command)
  {
    // DM status 0/1 is disabled/enabled; RS/MR use fault bits.
    complete_set_zero(matched->index, matched->protocol == MotorType::MOTOR_TYPE_DM ?
      matched_rx.mit.err <= 1U : matched_rx.mit.err == 0U);
  }

  const double unknown = std::numeric_limits<double>::quiet_NaN();
  state.route.motor_type = matched->index;
  state.route.bus_index = matched->bus_index;
  state.route.motor_id = matched->motor_id;
  state.valid = true;
  state.error_code = matched_rx.mit.err;
  using State = luvbot_hardware_interfaces::msg::MotorState;
  state.enable_state = State::ENABLE_UNKNOWN;
  if (matched_rx.mit.enabled_known) {
    state.enable_state = matched_rx.mit.enabled ? State::ENABLE_ENABLED : State::ENABLE_DISABLED;
  }
  state.operating_mode = luvbot_hardware_interfaces::msg::MotorState::OPERATING_MODE_UNKNOWN;
  state.q = matched_rx.mit.pos;
  state.dq = matched_rx.mit.vel;
  state.tau = matched_rx.mit.torque;
  state.mos_temperature = matched_rx.mit.t_mos;
  state.rotor_temperature = matched_rx.mit.t_rotor;

  if (matched->protocol == MotorType::MOTOR_TYPE_LS)
  {
    state.operating_mode = matched_rx.mit.mode;
  }
  if (matched->protocol == MotorType::MOTOR_TYPE_RS)
  {
    state.rotor_temperature = unknown;
  }
  if (matched->protocol == MotorType::MOTOR_TYPE_MR) {
    state.mos_temperature = unknown;
    state.rotor_temperature = matched_rx.mit.winding_temperature;
  }
  if (matched->protocol == MotorType::MOTOR_TYPE_LS && matched_rx.mit.mode == 2U) {
    state.mos_temperature = unknown;
  }

  return true;
}

void HpmMotor::handle_motor_snapshot(const frameTransport::MotorSnapshot & snapshot)
{
  handle_motor_snapshot(snapshot, nullptr);
}

void HpmMotor::handle_motor_snapshot(const frameTransport::MotorSnapshot & snapshot,
                                     luvbot_hardware_interfaces::msg::MotorStateArray * published)
{
  if ((snapshot.feedback_mask & ~0x001FFFFFU) != 0U) {
    RCLCPP_WARN(node_.get_logger(), "Motor feedback snapshot has invalid mask 0x%08X",
                snapshot.feedback_mask);
    return;
  }

  luvbot_hardware_interfaces::msg::MotorStateArray message;
  message.feedback_mask = snapshot.feedback_mask;
  for (uint8_t index = 0U; index < snapshot.slots.size(); ++index) {
    if ((snapshot.feedback_mask & (1UL << index)) == 0U) {
      continue;
    }
    const auto & frame = snapshot.slots[index];
    handle_set_zero_feedback(frame);
    luvbot_hardware_interfaces::msg::MotorState state;
    if (!decode_motor_state(frame, index, state)) {
      return;
    }
    message.states.push_back(std::move(state));
  }

  message.stamp = node_.now();
  message.sequence_id = ++state_sequence_;
  state_publisher_->publish(message);
  for (const auto & state : message.states) {
    motor_counts_[state.route.motor_type].rx.fetch_add(1, std::memory_order_relaxed);
  }
  if (published) *published = std::move(message);
}

const MotorDefinition * HpmMotor::motor_by_type(uint8_t type) const
{
  for (const auto & motor : motors_) {
    if (motor.index == type) {
      return &motor;
    }
  }
  return nullptr;
}

}  // namespace hpmMotor
