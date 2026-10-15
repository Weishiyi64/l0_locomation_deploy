#include "legged_controllers/LeggedController.h"

namespace legged {

void LeggedController::processJoystickSequenceActions(const sensor_msgs::msg::Joy& joy) {
  if (!joystick_sequence_enabled_ || joystickSequenceActions_.empty()) return;

  const JoystickSequenceBinding* binding = joystickSequenceActions_.consumeTriggered(joy);
  if (binding == nullptr) return;

  const std::string action_label = binding->label.empty() ? binding->name : binding->label;

  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!canStartAmpDerivedAction("[JoystickSequence]", action_label, true, true))
  {
    return;
  }

  const auto log_trigger = [this, &action_label, binding]() {
    RCLCPP_INFO(nh_->get_logger(), "[JoystickSequence] %s -> %s", action_label.c_str(),
                displayControllerPath(binding->sequence_path).c_str());
  };
  (void)triggerSequenceFromPath(binding->sequence_path, "[JoystickSequence] Trigger", log_trigger);
}

}  // namespace legged
