#pragma once

#include <chrono>
#include <map>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

namespace legged {

struct JoystickButtons {
  bool a{false};
  bool b{false};
  bool x{false};
  bool y{false};
  bool lb{false};
  bool rb{false};
  bool lt{false};
  bool dpad_left{false};
  bool rt{false};
  bool back{false};
  bool start{false};
  bool ls{false};
  bool rs{false};
};

struct JoystickAxes {
  double ls_lr{0.0};
  double ls_ud{0.0};
  double rs_lr{0.0};
  double lt{0.0};
};

struct JoystickCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double lt_axis{0.0};
  bool emergency_stop_pressed{false};
  bool wbc_soft_stop_pressed{false};
  bool set_walk_pressed{false};
  bool reset_pressed{false};
  bool kp_add_pressed{false};
  bool kp_minus_pressed{false};
  bool toggle_calibration_pressed{false};
  bool calibrate_left_arm_pressed{false};
  bool calibrate_right_arm_pressed{false};
  bool calibrate_left_leg_pressed{false};
  bool calibrate_right_leg_pressed{false};
  bool calibrate_waist_pressed{false};
  bool calibrate_neck_pressed{false};
};

class JoystickInput {
 public:
  JoystickCommand decode(const sensor_msgs::msg::Joy& joy);

 private:
  JoystickButtons decodeButtons(const sensor_msgs::msg::Joy& joy) const;
  JoystickAxes decodeAxes(const sensor_msgs::msg::Joy& joy) const;
  JoystickCommand buildCommand(const JoystickButtons& buttons, const JoystickAxes& axes);
  bool isButtonRisingEdge(const std::string& button_name, bool current_state);

  std::map<std::string, bool> lastButtonStates_;
  bool calibration_toggle_armed_{true};
  std::chrono::steady_clock::time_point calibration_buttons_released_since_{};
};

}  // namespace legged
