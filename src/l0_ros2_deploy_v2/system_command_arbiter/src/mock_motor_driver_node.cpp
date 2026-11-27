#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <luvbot_hardware_interfaces/msg/motor_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <luvbot_hardware_interfaces/msg/motor_target.hpp>
#include <luvbot_hardware_interfaces/srv/set_motor_enabled.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <utility>
#include "AppVersion.h"

namespace {

using MotorCommand = luvbot_hardware_interfaces::msg::MotorCommand;
using MotorRoute = luvbot_hardware_interfaces::msg::MotorRoute;
using MotorState = luvbot_hardware_interfaces::msg::MotorState;
using MotorStateArray = luvbot_hardware_interfaces::msg::MotorStateArray;
using SetMotorEnabled = luvbot_hardware_interfaces::srv::SetMotorEnabled;

uint64_t route_key(const MotorRoute& route) {
  return (static_cast<uint64_t>(route.motor_type) << 56U) | (static_cast<uint64_t>(route.bus_index) << 48U) |
         route.motor_id;
}

bool valid_route(const MotorRoute& route) {
  return route.motor_type < 32U;
}

bool finite_command(const luvbot_hardware_interfaces::msg::MotorTarget& command) {
  return std::isfinite(command.q) && std::isfinite(command.dq) && std::isfinite(command.tau) &&
         std::isfinite(command.kp) && std::isfinite(command.kd);
}

}  // namespace

class MockMotorDriver : public rclcpp::Node {
 public:
  MockMotorDriver() : Node("mock_motor_driver_node") {
    const double state_rate = declare_parameter<double>("state_rate", 100.0);
    watchdog_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("watchdog_timeout", 0.1));
    command_validity_ = rclcpp::Duration::from_nanoseconds(
        1000000LL * declare_parameter<int>("command_validity_ms", 20));
    if (!std::isfinite(state_rate) || state_rate <= 0.0 || watchdog_timeout_.nanoseconds() <= 0 ||
        command_validity_.nanoseconds() <= 0)
      throw std::runtime_error("state_rate, watchdog_timeout and command_validity_ms must be positive");

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    state_pub_ = create_publisher<MotorStateArray>("state", state_qos);
    command_sub_ = create_subscription<MotorCommand>(
        "command", command_qos, std::bind(&MockMotorDriver::command_callback, this, std::placeholders::_1));
    enable_service_ = create_service<SetMotorEnabled>(
        "set_enabled", std::bind(&MockMotorDriver::set_enabled, this, std::placeholders::_1, std::placeholders::_2));
    state_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / state_rate),
                                     std::bind(&MockMotorDriver::publish_state, this));
    RCLCPP_INFO(get_logger(), "Mock motor driver is ready (no hardware access)");
  }

 private:
  void set_enabled(const SetMotorEnabled::Request::SharedPtr request, SetMotorEnabled::Response::SharedPtr response) {
    const uint64_t key = route_key(request->motor);
    for (const auto& item : routes_)
    {
      if (item.second.motor_type != request->motor.motor_type || item.first == key) continue;
      response->success = false;
      response->error_code = SetMotorEnabled::Response::ERROR_INVALID_ROUTE;
      response->diagnostic_message = "motor index is already associated with another route";
      return;
    }
    if (!valid_route(request->motor))
    {
      response->success = false;
      response->error_code = SetMotorEnabled::Response::ERROR_INVALID_ROUTE;
      response->diagnostic_message = "motor index must fit the feedback mask";
      return;
    }
    routes_[key] = request->motor;
    if (request->enabled)
    {
      enabled_.insert(key);
      MotorState state;
      state.route = request->motor;
      state.valid = true;
      state.operating_mode = MotorState::OPERATING_MODE_UNKNOWN;
      state.enable_state = MotorState::ENABLE_ENABLED;
      state.mos_temperature = 25.0F;
      state.rotor_temperature = 25.0F;
      command_state_[key] = state;
    }
    else
    {
      enabled_.erase(key);
      command_state_.erase(key);
    }
    response->success = true;
    response->error_code = SetMotorEnabled::Response::ERROR_NONE;
    response->diagnostic_message = request->enabled ? "enabled" : "disabled";
  }

  void command_callback(const MotorCommand::ConstSharedPtr message) {
    const rclcpp::Time current = now();
    if (message->stamp.sec < 0 || message->stamp.nanosec >= 1000000000U)
    {
      RCLCPP_WARN(get_logger(), "Rejected invalid MotorCommand timestamp");
      return;
    }
    const rclcpp::Time stamp(message->stamp);
    const rclcpp::Time valid_until = stamp + command_validity_;
    const uint32_t sequence_delta = message->sequence_id - last_command_sequence_;
    if (message->control_mode != MotorCommand::CONTROL_MODE_MIT || message->commands.empty() ||
        message->commands.size() != enabled_.size() || valid_until <= current ||
        stamp > current + watchdog_timeout_ ||
        (have_command_ && (sequence_delta == 0U || sequence_delta >= 0x80000000U)))
    {
      RCLCPP_WARN(get_logger(), "Rejected invalid, expired, incomplete, or out-of-order MotorCommand");
      return;
    }

    std::map<uint64_t, MotorState> next;
    for (const auto& command : message->commands)
    {
      const uint64_t key = route_key(command.route);
      if (!valid_route(command.route) || enabled_.count(key) == 0U || next.count(key) != 0U || !finite_command(command))
      {
        RCLCPP_WARN(get_logger(), "Rejected MotorCommand with duplicate, unknown, disabled, or non-finite route");
        return;
      }
      MotorState state;
      state.route = command.route;
      state.valid = true;
      state.operating_mode = MotorState::OPERATING_MODE_UNKNOWN;
      state.enable_state = MotorState::ENABLE_ENABLED;
      state.q = command.q;
      state.dq = command.dq;
      state.tau = command.tau;
      state.mos_temperature = 25.0F;
      state.rotor_temperature = 25.0F;
      next.emplace(key, state);
    }

    command_state_ = std::move(next);
    last_command_sequence_ = message->sequence_id;
    command_deadline_ = valid_until;
    last_command_received_ = current;
    have_command_ = true;
  }

  void publish_state() {
    const rclcpp::Time current = now();
    if (!enabled_.empty() && have_command_ &&
        (current > command_deadline_ || current - last_command_received_ > watchdog_timeout_))
    {
      if (have_command_) RCLCPP_WARN(get_logger(), "Motor command watchdog expired; disabling mock motors");
      enabled_.clear();
      command_state_.clear();
      have_command_ = false;
    }

    MotorStateArray state_array;
    state_array.stamp = current;
    state_array.sequence_id = ++state_sequence_;
    for (const auto& item : routes_)
    {
      state_array.feedback_mask |= uint32_t{1} << item.second.motor_type;
      const auto active = command_state_.find(item.first);
      if (active != command_state_.end())
      {
        state_array.states.push_back(active->second);
        continue;
      }
      MotorState state;
      state.route = item.second;
      state.valid = true;
      state.operating_mode = MotorState::OPERATING_MODE_UNKNOWN;
      state.enable_state = MotorState::ENABLE_DISABLED;
      state.mos_temperature = 25.0F;
      state.rotor_temperature = 25.0F;
      state_array.states.push_back(state);
    }
    if (!state_array.states.empty()) state_pub_->publish(state_array);
  }

  std::map<uint64_t, MotorRoute> routes_;
  std::set<uint64_t> enabled_;
  std::map<uint64_t, MotorState> command_state_;
  rclcpp::Duration watchdog_timeout_{0, 0};
  rclcpp::Duration command_validity_{0, 0};
  rclcpp::Time command_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_received_{0, 0, RCL_ROS_TIME};
  uint32_t last_command_sequence_{0};
  uint32_t state_sequence_{0};
  bool have_command_{false};
  rclcpp::Publisher<MotorStateArray>::SharedPtr state_pub_;
  rclcpp::Subscription<MotorCommand>::SharedPtr command_sub_;
  rclcpp::Service<SetMotorEnabled>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char** argv) {
  l0::printAppVersion("mock_motor_driver_node");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
