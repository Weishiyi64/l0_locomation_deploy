#pragma once
#include <atomic>
#include <rclcpp/rclcpp.hpp>

namespace legged {
class StateMachine {
 public:
  enum class State { Idle, Standby, Walking, EmergencyStop, Calibration };

  StateMachine(rclcpp::Logger logger) : state_(State::Standby), logger_(logger) {}

  State state() const { return state_.load(); }
  State prevState() const { return prev_state_.load(); }

  const char* stateName() const { return stateNameFrom(state()); }

  // Events
  void setWalk() {
    State previous = State::Standby;
    if (state_.compare_exchange_strong(previous, State::Walking))
    {
      prev_state_.store(previous);
      RCLCPP_INFO(logger_, "FSM: Standby -> Walking");
    }
    else if (previous != State::Walking)
    {
      RCLCPP_WARN(logger_, "FSM: setWalk ignored in state %s", stateNameFrom(previous));
    }
  }

  void emergencyStop() {
    const State previous = state_.exchange(State::EmergencyStop);
    if (previous != State::EmergencyStop)
    {
      prev_state_.store(previous);
      RCLCPP_WARN(logger_, "FSM: -> EmergencyStop (prev=%s)", stateNameFrom(previous));
    }
  }

  void resetFromEmergency() {
    transition(State::EmergencyStop, State::Standby, "reset");
  }

  void enterCalibrationMode() {
    transition(State::EmergencyStop, State::Calibration, "enterCalibrationMode");
  }

  void exitCalibrationMode() {
    transition(State::Calibration, State::EmergencyStop, "exitCalibrationMode");
  }

  bool isWalking() const { return state() == State::Walking; }
  bool isEmergency() const { return state() == State::EmergencyStop; }
  bool isStandby() const { return state() == State::Standby; }
  bool isCalibration() const { return state() == State::Calibration; }

 private:
  void transition(State from, State to, const char* event) {
    State previous = from;
    // An event must not overwrite a state changed by the control thread or another callback.
    if (state_.compare_exchange_strong(previous, to))
    {
      prev_state_.store(from);
      RCLCPP_INFO(logger_, "FSM: %s -> %s", stateNameFrom(from), stateNameFrom(to));
    }
    else
    {
      RCLCPP_WARN(logger_, "FSM: %s ignored in state %s", event, stateNameFrom(previous));
    }
  }

  static const char* stateNameFrom(State s) {
    switch (s)
    {
      case State::Idle:
        return "Idle";
      case State::Standby:
        return "Standby";
      case State::Walking:
        return "Walking";
      case State::EmergencyStop:
        return "EmergencyStop";
      case State::Calibration:
        return "Calibration";
    }
    return "Unknown";
  }
  std::atomic<State> state_;
  std::atomic<State> prev_state_{State::Standby};
  rclcpp::Logger logger_;
};
}  // namespace legged
