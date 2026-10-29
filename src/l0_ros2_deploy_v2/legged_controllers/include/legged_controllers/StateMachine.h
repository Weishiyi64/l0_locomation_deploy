#pragma once
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace legged {
class StateMachine {
 public:
  enum class State { Idle, Standby, Walking, EmergencyStop, Calibration };

  StateMachine(rclcpp::Logger logger) : state_(State::Standby), logger_(logger) {}

  State state() const { return state_; }
  State prevState() const { return prev_state_; }

  const char* stateName() const {
    switch (state_)
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

  // Events
  void setWalk() {
    if (state_ == State::Standby)
    {
      prev_state_ = state_;
      state_ = State::Walking;
      // ("FSM: Standby -> Walking");
      RCLCPP_INFO(logger_, "FSM: Standby -> Walking");
    }
    else if (state_ == State::Walking)
    {
      // no-op
    }
    else
    {
      // ("FSM: setWalk ignored in state %s", stateName());
      RCLCPP_WARN(logger_, "FSM: setWalk ignored in state %s", stateName());
    }
  }

  void emergencyStop() {
    if (state_ != State::EmergencyStop)
    {
      prev_state_ = state_;
      state_ = State::EmergencyStop;
      // ("FSM: -> EmergencyStop (prev=%s)",
      // stateNameFrom(prev_state_));
      RCLCPP_WARN(logger_, "FSM: -> EmergencyStop (prev=%s)", stateNameFrom(prev_state_));
    }
  }

  void resetFromEmergency() {
    if (state_ == State::EmergencyStop)
    {
      prev_state_ = state_;
      state_ = State::Standby;
      // ("FSM: EmergencyStop -> Standby");
      RCLCPP_INFO(logger_, "FSM: EmergencyStop -> Standby");
    }
    else
    {
      // ("FSM: reset ignored in state %s", stateName());
      RCLCPP_WARN(logger_, "FSM: reset ignored in state %s", stateName());
    }
  }

  void enterCalibrationMode() {
    if (state_ == State::EmergencyStop)
    {
      prev_state_ = state_;
      state_ = State::Calibration;
      // ("FSM: EmergencyStop -> Calibration");
      RCLCPP_INFO(logger_, "FSM: EmergencyStop -> Calibration");
    }
    else
    {
      // ("FSM: enterCalibrationMode ignored in state %s", stateName());
      RCLCPP_WARN(logger_, "FSM: enterCalibrationMode ignored in state %s", stateName());
    }
  }

  void exitCalibrationMode() {
    if (state_ == State::Calibration)
    {
      prev_state_ = state_;
      state_ = State::EmergencyStop;
      // ("FSM: Calibration -> EmergencyStop");
      RCLCPP_INFO(logger_, "FSM: Calibration -> EmergencyStop");
    }
    else
    {
      RCLCPP_WARN(logger_, "FSM: exitCalibrationMode ignored in state %s", stateName());
    }
  }

  bool isWalking() const { return state_ == State::Walking; }
  bool isEmergency() const { return state_ == State::EmergencyStop; }
  bool isStandby() const { return state_ == State::Standby; }
  bool isCalibration() const { return state_ == State::Calibration; }

 private:
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
  State state_;
  State prev_state_{State::Standby};
  rclcpp::Logger logger_;
};
}  // namespace legged
