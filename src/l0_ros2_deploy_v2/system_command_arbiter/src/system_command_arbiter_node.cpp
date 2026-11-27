#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <luvbot_hardware_interfaces/msg/motor_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_command_candidate.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <luvbot_hardware_interfaces/srv/set_motor_enabled.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "motor_mapping.hpp"
#include "AppVersion.h"

namespace {

using MotorCommand = luvbot_hardware_interfaces::msg::MotorCommand;
using MotorCommandCandidate = luvbot_hardware_interfaces::msg::MotorCommandCandidate;
using MotorRoute = luvbot_hardware_interfaces::msg::MotorRoute;
using MotorStateArray = luvbot_hardware_interfaces::msg::MotorStateArray;
using SetMotorEnabled = luvbot_hardware_interfaces::srv::SetMotorEnabled;
using system_command_arbiter::MotorMapping;

int64_t nanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool same_route(const MotorRoute& lhs, const MotorRoute& rhs) {
  return lhs.motor_type == rhs.motor_type && lhs.bus_index == rhs.bus_index && lhs.motor_id == rhs.motor_id;
}

}  // namespace

class SystemCommandArbiter : public rclcpp::Node {
 public:
  SystemCommandArbiter() : Node("system_command_arbiter_node") {
    const std::string mapping_file = declare_parameter<std::string>("mapping_file", "");
    if (mapping_file.empty()) throw std::runtime_error("mapping_file is required");
    mapping_ = std::make_unique<MotorMapping>(mapping_file);
    if (mapping_->enabledSize() == 0U) throw std::runtime_error("mapping has no enabled motors");
    feedback_wrap_.resize(mapping_->size());

    allowed_sources_ =
        declare_parameter<std::vector<std::string>>("allowed_sources", std::vector<std::string>{"legged_controller"});
    command_validity_ms_ = declare_parameter<int>("command_validity_ms", 20);
    pilot_mode_ = declare_parameter<bool>("pilot_mode", false);
    allow_actuation_ = declare_parameter<bool>("allow_actuation", false);
    pilot_hold_kp_ = declare_parameter<double>("pilot_hold_kp", 4.0);
    pilot_hold_kd_ = declare_parameter<double>("pilot_hold_kd", 0.5);
    pilot_hold_seconds_ = declare_parameter<double>("pilot_hold_seconds", 2.0);
    pilot_feedback_timeout_ms_ = declare_parameter<int>("pilot_feedback_timeout_ms", 500);
    const int pilot_motor_type = declare_parameter<int>("pilot_motor_type", 4);
    if (pilot_motor_type < 0 || pilot_motor_type > 255) throw std::runtime_error("pilot_motor_type must be in 0..255");
    pilot_route_.motor_type = static_cast<uint8_t>(pilot_motor_type);
    const int pilot_bus_index = declare_parameter<int>("pilot_bus_index", 4);
    if (pilot_bus_index < 0 || pilot_bus_index > 255) throw std::runtime_error("pilot_bus_index must be in 0..255");
    pilot_route_.bus_index = static_cast<uint8_t>(pilot_bus_index);
    pilot_route_.motor_id = static_cast<uint32_t>(declare_parameter<int>("pilot_motor_id", 1));

    if (command_validity_ms_ <= 0 || pilot_feedback_timeout_ms_ <= 0 || pilot_hold_seconds_ <= 0.0 ||
        !std::isfinite(pilot_hold_kp_) || !std::isfinite(pilot_hold_kd_) || pilot_hold_kp_ < 0.0 ||
        pilot_hold_kd_ < 0.0)
    {
      throw std::runtime_error("invalid timing or pilot gain parameter");
    }

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    const auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    command_pub_ = create_publisher<MotorCommand>("motor_command", command_qos);
    candidate_sub_ = create_subscription<MotorCommandCandidate>(
        "motor_command_candidate", command_qos,
        std::bind(&SystemCommandArbiter::candidate_callback, this, std::placeholders::_1));
    state_sub_ = create_subscription<MotorStateArray>(
        "motor_state", state_qos, std::bind(&SystemCommandArbiter::state_callback, this, std::placeholders::_1));
    enable_client_ = create_client<SetMotorEnabled>("set_motor_enabled");
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", safety_qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) { emergency_stop_ = message->data; });
    fault_sub_ = create_subscription<std_msgs::msg::Bool>(
        "hardware_fault", safety_qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) { hardware_fault_ = message->data; });

    pilot_timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&SystemCommandArbiter::pilot_tick, this));

    RCLCPP_INFO(get_logger(), "Loaded %zu logical joints (%zu enabled motors) from %s", mapping_->size(),
                mapping_->enabledSize(), mapping_file.c_str());
    if (pilot_mode_ && !allow_actuation_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Pilot is disarmed: type=%u bus=%u id=0x%X. Set allow_actuation:=true only after the robot is secured.",
          pilot_route_.motor_type, pilot_route_.bus_index, pilot_route_.motor_id);
      pilot_state_ = PilotState::DONE;
    }
    else if (pilot_mode_)
    {
      if (mapping_->indexOfRoute(pilot_route_) < 0)
      {
        throw std::runtime_error("pilot route must match an enabled motor index, bus, and CAN ID in the mapping");
      }
      pilot_state_ = PilotState::WAIT_SERVICE;
      state_entered_ = now();
    }
    else
    {
      pilot_state_ = PilotState::DONE;
    }
  }

 private:
  enum class PilotState { WAIT_SERVICE, ENABLING, ACQUIRE_FEEDBACK, HOLD, DISABLING, DONE, FAILED };

  bool source_allowed(const std::string& source) const {
    return std::find(allowed_sources_.begin(), allowed_sources_.end(), source) != allowed_sources_.end();
  }

  void reject(const std::string& reason) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected motor command candidate: %s", reason.c_str());
  }

  void candidate_callback(const MotorCommandCandidate::ConstSharedPtr message) {
    if (pilot_mode_) return;
    if (emergency_stop_ || hardware_fault_) return reject("safety stop active");
    if (!source_allowed(message->source_id)) return reject("source is not allowed");
    if (message->control_mode != MotorCommandCandidate::CONTROL_MODE_HYBRID)
      return reject("the shared motor command requires MIT-compatible hybrid targets");

    const int64_t now_ns = now().nanoseconds();
    if (nanoseconds(message->stamp) > now_ns + 100000000LL) return reject("timestamp is in the future");
    if (nanoseconds(message->valid_until) <= now_ns || nanoseconds(message->valid_until) <= nanoseconds(message->stamp))
      return reject("command is expired");
    if (nanoseconds(message->valid_until) - nanoseconds(message->stamp) > command_validity_ms_ * 1000000LL)
      return reject("validity exceeds configured limit");

    const auto sequence = last_sequence_.find(message->source_id);
    const auto previous_deadline = last_valid_until_.find(message->source_id);
    if (sequence != last_sequence_.end() && message->sequence_id <= sequence->second &&
        previous_deadline != last_valid_until_.end() && previous_deadline->second > now_ns)
      return reject("sequence is stale");
    if (message->commands.size() != mapping_->enabledSize()) return reject("enabled command list is not complete");
    for (std::size_t i = 0; i < feedback_wrap_.size(); ++i)
      if (mapping_->entries()[i].enabled && !feedback_wrap_[i].ready())
        return reject("complete valid motor feedback turns have not been confirmed");

    MotorCommand output;
    output.stamp = message->stamp;
    output.control_mode = MotorCommand::CONTROL_MODE_MIT;
    output.commands.reserve(message->commands.size());
    std::unordered_set<std::string> joints;

    for (const auto& command : message->commands)
    {
      if (!joints.emplace(command.joint_name).second) return reject("duplicate joint: " + command.joint_name);
      const auto* entry = mapping_->findJoint(command.joint_name);
      if (entry == nullptr) return reject("unknown joint: " + command.joint_name);
      if (!entry->enabled) return reject("joint is disabled: " + command.joint_name);
      try
      {
        const int index = mapping_->indexOfJoint(command.joint_name);
        output.commands.push_back(mapping_->commandFor(*entry, command.q, command.dq, command.tau, command.kp,
                                                       command.kd, feedback_wrap_[index].offset));
      } catch (const std::exception& error)
      { return reject(error.what()); }
    }

    last_sequence_[message->source_id] = message->sequence_id;
    last_valid_until_[message->source_id] = nanoseconds(message->valid_until);
    const bool active_command = std::any_of(output.commands.begin(), output.commands.end(), [](const auto& command) {
      return command.kp != 0.0 || command.dq != 0.0 || command.tau != 0.0;
    });
    if (active_command)
      for (auto& wrap : feedback_wrap_) wrap.locked = true;
    output.sequence_id = ++command_sequence_;
    command_pub_->publish(output);
  }

  void state_callback(const MotorStateArray::ConstSharedPtr message) {
    for (const auto& state : message->states)
    {
      const int index = mapping_->indexOfRoute(state.route);
      if (index >= 0 && !pilot_mode_)
      {
        const auto& entry = mapping_->entries()[index];
        const bool was_faulted = feedback_wrap_[index].fault;
        const auto feedback = mapping_->updateFeedback(entry, state, feedback_wrap_[index]);
        if (!was_faulted && feedback_wrap_[index].fault)
          RCLCPP_ERROR(get_logger(),
                       "Motor %s feedback invalid after active control: motor_q=%.6f offset=%.6f joint_q=%.6f; "
                       "commands blocked until feedback is inspected and the arbiter is restarted",
                       entry.joint_name.c_str(), state.q, feedback_wrap_[index].offset, feedback.q);
      }
      if (!pilot_mode_) continue;
      if (!same_route(state.route, pilot_route_)) continue;
      pilot_feedback_valid_ = state.valid && state.error_code == 0U && std::isfinite(state.q);
      pilot_feedback_q_ = state.q;
      last_feedback_ = now();
      if (!pilot_feedback_valid_ && pilot_state_ != PilotState::DISABLING) fail_pilot("invalid motor feedback");
      return;
    }
  }

  void publish_pilot_command(double q, double kp, double kd) {
    MotorCommand output;
    const auto current = now();
    output.stamp = current;
    output.control_mode = MotorCommand::CONTROL_MODE_MIT;
    output.commands.resize(1);
    output.commands[0].route = pilot_route_;
    output.commands[0].q = static_cast<float>(q);
    output.commands[0].dq = 0.0F;
    output.commands[0].tau = 0.0F;
    output.commands[0].kp = static_cast<float>(kp);
    output.commands[0].kd = static_cast<float>(kd);
    output.sequence_id = ++command_sequence_;
    command_pub_->publish(output);
  }

  void request_enabled(bool enabled) {
    const uint64_t generation = ++service_generation_;
    service_pending_ = true;
    service_deadline_ = now() + rclcpp::Duration::from_nanoseconds(200000000LL);
    auto request = std::make_shared<SetMotorEnabled::Request>();
    request->motor = pilot_route_;
    request->enabled = enabled;
    enable_client_->async_send_request(
        request, [this, enabled, generation](rclcpp::Client<SetMotorEnabled>::SharedFuture future) {
          if (generation != service_generation_) return;
          service_pending_ = false;
          const auto response = future.get();
          if (!response->success)
          {
            RCLCPP_ERROR(get_logger(), "Motor %s failed: error=%u %s", enabled ? "enable" : "disable",
                         response->error_code, response->diagnostic_message.c_str());
            if (enabled)
              begin_disable();
            else
              pilot_state_ = PilotState::FAILED;
            return;
          }
          state_entered_ = now();
          if (enabled)
          {
            RCLCPP_INFO(get_logger(), "Pilot motor enabled; acquiring feedback with zero gains");
            pilot_state_ = PilotState::ACQUIRE_FEEDBACK;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Pilot motor disabled; test complete");
            pilot_state_ = PilotState::DONE;
          }
        });
  }

  void begin_disable() {
    if (pilot_state_ == PilotState::DISABLING || pilot_state_ == PilotState::DONE || pilot_state_ == PilotState::FAILED)
      return;
    pilot_state_ = PilotState::DISABLING;
    state_entered_ = now();
    if (enable_client_->service_is_ready())
      request_enabled(false);
    else
    {
      RCLCPP_ERROR(get_logger(), "Cannot disable pilot motor: service unavailable; driver watchdog must stop it");
      pilot_state_ = PilotState::FAILED;
    }
  }

  void fail_pilot(const std::string& reason) {
    RCLCPP_ERROR(get_logger(), "Pilot aborted: %s", reason.c_str());
    begin_disable();
  }

  void pilot_tick() {
    if (!pilot_mode_ || pilot_state_ == PilotState::DONE || pilot_state_ == PilotState::FAILED) return;
    const auto current = now();
    if (emergency_stop_ || hardware_fault_)
    {
      fail_pilot("safety stop active");
      return;
    }
    if (service_pending_ && current > service_deadline_)
    {
      service_pending_ = false;
      ++service_generation_;
      if (pilot_state_ == PilotState::DISABLING)
      {
        RCLCPP_ERROR(get_logger(), "Motor disable service timed out; driver watchdog must stop it");
        pilot_state_ = PilotState::FAILED;
      }
      else
      {
        fail_pilot("set_enabled service timed out after 200 ms");
      }
      return;
    }

    switch (pilot_state_)
    {
      case PilotState::WAIT_SERVICE:
        if (enable_client_->service_is_ready())
        {
          pilot_state_ = PilotState::ENABLING;
          request_enabled(true);
        }
        break;
      case PilotState::ENABLING:
      case PilotState::DISABLING:
        break;
      case PilotState::ACQUIRE_FEEDBACK:
        publish_pilot_command(0.0, 0.0, 0.0);
        if (pilot_feedback_valid_ && last_feedback_ >= state_entered_)
        {
          pilot_hold_q_ = pilot_feedback_q_;
          state_entered_ = current;
          RCLCPP_INFO(get_logger(), "Feedback acquired at q=%.5f; holding for %.2f s", pilot_hold_q_,
                      pilot_hold_seconds_);
          pilot_state_ = PilotState::HOLD;
        }
        else if ((current - state_entered_).nanoseconds() > pilot_feedback_timeout_ms_ * 1000000LL)
        {
          fail_pilot("feedback timeout");
        }
        break;
      case PilotState::HOLD:
        if (!pilot_feedback_valid_ || (current - last_feedback_).nanoseconds() > 100000000LL)
        {
          fail_pilot("feedback became stale");
        }
        else if ((current - state_entered_).seconds() >= pilot_hold_seconds_)
        {
          begin_disable();
        }
        else
        {
          publish_pilot_command(pilot_hold_q_, pilot_hold_kp_, pilot_hold_kd_);
        }
        break;
      case PilotState::DONE:
      case PilotState::FAILED:
        break;
    }
  }

  std::unique_ptr<MotorMapping> mapping_;
  std::vector<system_command_arbiter::FeedbackWrapState> feedback_wrap_;
  std::vector<std::string> allowed_sources_;
  std::unordered_map<std::string, uint64_t> last_sequence_;
  std::unordered_map<std::string, int64_t> last_valid_until_;
  rclcpp::Publisher<MotorCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<MotorCommandCandidate>::SharedPtr candidate_sub_;
  rclcpp::Subscription<MotorStateArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_sub_;
  rclcpp::Client<SetMotorEnabled>::SharedPtr enable_client_;
  rclcpp::TimerBase::SharedPtr pilot_timer_;

  bool emergency_stop_{false};
  bool hardware_fault_{false};
  bool pilot_mode_{false};
  bool allow_actuation_{false};
  int command_validity_ms_{20};
  int pilot_feedback_timeout_ms_{500};
  double pilot_hold_kp_{4.0};
  double pilot_hold_kd_{0.5};
  double pilot_hold_seconds_{2.0};
  MotorRoute pilot_route_;
  PilotState pilot_state_{PilotState::DONE};
  rclcpp::Time state_entered_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_{0, 0, RCL_ROS_TIME};
  rclcpp::Time service_deadline_{0, 0, RCL_ROS_TIME};
  bool service_pending_{false};
  uint64_t service_generation_{0};
  bool pilot_feedback_valid_{false};
  double pilot_feedback_q_{0.0};
  double pilot_hold_q_{0.0};
  uint32_t command_sequence_{0};
};

int main(int argc, char** argv) {
  l0::printAppVersion("system_command_arbiter_node");
  rclcpp::init(argc, argv);
  try
  { rclcpp::spin(std::make_shared<SystemCommandArbiter>()); } catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("system_command_arbiter_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
