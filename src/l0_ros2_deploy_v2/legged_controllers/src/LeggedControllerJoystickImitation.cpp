#include "legged_controllers/LeggedController.h"

namespace legged {

void LeggedController::processJoystickImitationActions(const sensor_msgs::msg::Joy& joy) {
  if (!joystick_imitation_enabled_ || joystickImitationActions_.empty()) return;

  const JoystickImitationBinding* binding = joystickImitationActions_.consumeTriggered(joy);
  if (binding == nullptr) return;

  const std::string action_label = binding->label.empty() ? binding->name : binding->label;

  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!canStartAmpDerivedAction("[JoystickImitation]", action_label, true, true))
  {
    return;
  }

  RCLCPP_INFO(nh_->get_logger(), "[JoystickImitation] %s -> %s", action_label.c_str(), binding->policy.c_str());

  if (!switchImitationPolicy(binding->policy, "[JoystickImitation]"))
  {
    return;
  }

  if (binding->shot_id >= 0 && shotIdPub_)
  {
    std_msgs::msg::Int64 shot_id_msg;
    shot_id_msg.data = binding->shot_id;
    shotIdPub_->publish(shot_id_msg);
    RCLCPP_INFO(nh_->get_logger(), "[JoystickImitation] Published /shot_id: %ld", static_cast<long>(binding->shot_id));
  }

  setPolicySource(PolicyCmdSource::kImitation);
  policyMux_.publishImitationReset();
  publishImitationActive(true);
  applyJoystickMotion(lastJoystickCommand_, last_controller_wall_sec_);
}

}  // namespace legged
