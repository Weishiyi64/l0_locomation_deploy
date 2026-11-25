#include "legged_controllers/JoystickInput.h"

namespace legged {

namespace {

constexpr int ButtonA = 0;
constexpr int ButtonB = 1;
constexpr int ButtonX = 3;
constexpr int ButtonY = 4;
constexpr int ButtonLB = 6;
constexpr int ButtonRB = 7;
constexpr int ButtonLt = 8;
constexpr int ButtonRt = 9;
constexpr int ButtonBack = 10;
constexpr int ButtonStart = 11;
constexpr int ButtonLs = 13;
constexpr int ButtonRs = 14;

constexpr int AxisLsLr = 0;
constexpr int AxisLsUd = 1;
constexpr int AxisRsLr = 2;
constexpr int AxisLt = 5;

bool isPressed(const sensor_msgs::msg::Joy& joy, int index) {
  return index >= 0 && static_cast<size_t>(index) < joy.buttons.size() && joy.buttons[index] == 1;
}

double axisValue(const sensor_msgs::msg::Joy& joy, int index) {
  return index >= 0 && static_cast<size_t>(index) < joy.axes.size() ? joy.axes[index] : 0.0;
}

}  // namespace

JoystickCommand JoystickInput::decode(const sensor_msgs::msg::Joy& joy) {
  return buildCommand(decodeButtons(joy), decodeAxes(joy));
}

JoystickButtons JoystickInput::decodeButtons(const sensor_msgs::msg::Joy& joy) const {
  JoystickButtons buttons;
  buttons.a = isPressed(joy, ButtonA);
  buttons.b = isPressed(joy, ButtonB);
  buttons.x = isPressed(joy, ButtonX);
  buttons.y = isPressed(joy, ButtonY);
  buttons.lb = isPressed(joy, ButtonLB);
  buttons.rb = isPressed(joy, ButtonRB);
  buttons.lt = isPressed(joy, ButtonLt);
  buttons.rt = isPressed(joy, ButtonRt);
  buttons.back = isPressed(joy, ButtonBack);
  buttons.start = isPressed(joy, ButtonStart);
  buttons.ls = isPressed(joy, ButtonLs);
  buttons.rs = isPressed(joy, ButtonRs);
  return buttons;
}

JoystickAxes JoystickInput::decodeAxes(const sensor_msgs::msg::Joy& joy) const {
  JoystickAxes axes;
  axes.ls_lr = axisValue(joy, AxisLsLr);
  axes.ls_ud = axisValue(joy, AxisLsUd);
  axes.rs_lr = axisValue(joy, AxisRsLr);
  axes.lt = axisValue(joy, AxisLt);
  return axes;
}

JoystickCommand JoystickInput::buildCommand(const JoystickButtons& buttons, const JoystickAxes& axes) {
  JoystickCommand command;
  command.vx = axes.ls_ud;
  command.vy = axes.ls_lr;
  command.wz = axes.rs_lr;
  command.lt_axis = axes.lt;
  const bool single_a = buttons.a && !buttons.x;

  // LB+RB is the reset chord. Do not turn its RB edge into an emergency stop
  // immediately before processing the reset in the same control cycle.
  command.emergency_stop_pressed = isButtonRisingEdge("emergency_stop_manual", buttons.rb && !buttons.lb);
  command.set_walk_pressed = isButtonRisingEdge("set_walk", buttons.b && buttons.y);
  command.reset_pressed = isButtonRisingEdge("reset_controller", buttons.lb && buttons.rb);
  command.kp_add_pressed = isButtonRisingEdge("kp_add", buttons.b);
  command.kp_minus_pressed = isButtonRisingEdge("kp_minus", single_a);
  command.back_emergency_sequence_pressed = isButtonRisingEdge("back_emergency_sequence", buttons.back);
  command.toggle_calibration_pressed = isButtonRisingEdge("toggle_calibration_mode", buttons.ls && buttons.rs);
  command.calibrate_left_arm_pressed = isButtonRisingEdge("calibrate_left_arm", buttons.lt && buttons.lb);
  command.calibrate_right_arm_pressed = isButtonRisingEdge("calibrate_right_arm", buttons.rt && buttons.rb);
  command.calibrate_left_leg_pressed = isButtonRisingEdge("calibrate_left_leg", buttons.a && buttons.x);
  command.calibrate_right_leg_pressed = isButtonRisingEdge("calibrate_right_leg", buttons.b && buttons.y);
  command.calibrate_waist_pressed = isButtonRisingEdge("calibrate_waist", buttons.back);
  command.calibrate_neck_pressed = isButtonRisingEdge("calibrate_neck", buttons.start);
  return command;
}

bool JoystickInput::isButtonRisingEdge(const std::string& button_name, bool current_state) {
  bool last_state = lastButtonStates_[button_name];
  lastButtonStates_[button_name] = current_state;
  return current_state && !last_state;
}

}  // namespace legged
