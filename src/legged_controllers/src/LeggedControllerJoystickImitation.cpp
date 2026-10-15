#include "legged_controllers/LeggedController.h"

#include <algorithm>

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
  if (!switchImitationPolicy(binding->policy, "[JoystickImitation]")) return;

  // Keep the outgoing AMP/RL command at the motor interface while the newly
  // selected imitation policy starts.  The transition is applied after the
  // policy mux in JointCommandComposer, so it smooths both positions and
  // gains even when the first motion-reference frame differs from AMP.
  // A zero duration deliberately preserves the old direct-switch behaviour.
  if (imitation_entry_blend_sec_ > 1e-9)
  {
    const bool walking = fsm_ && fsm_->isWalking();
    const PolicyInputs amp_inputs = policyMux_.selectInputs(walking);
    if (!amp_inputs.q_snapshot.empty())
    {
      const size_t n = jointNames_.size();
      const size_t q_count = std::min(n, amp_inputs.q_snapshot.size());
      std::vector<bool> joint_mask(n, false);
      std::vector<JointCommandSample> start_cmd(n);
      for (size_t j = 0; j < q_count; ++j)
      {
        joint_mask[j] = true;
        start_cmd[j].q = amp_inputs.q_snapshot[j];
        start_cmd[j].dq = 0.0;
        start_cmd[j].kp = (j < amp_inputs.gains.kps.size()) ? amp_inputs.gains.kps[j] : 0.0;
        start_cmd[j].kd = (j < amp_inputs.gains.kds.size()) ? amp_inputs.gains.kds[j] : 0.0;
        start_cmd[j].tau_ff = 0.0;
      }

      double wall_sec = last_controller_wall_sec_;
      if (wall_sec <= 0.0 && nh_)
      {
        wall_sec = nh_->get_clock()->now().seconds();
      }

      JointCommandTransition& handoff = sequenceRuntime_.releaseHandoff();
      handoff.clear();
      if (handoff.start(joint_mask, start_cmd, wall_sec, imitation_entry_blend_sec_))
      {
        RCLCPP_INFO(nh_->get_logger(),
                    "[PolicyMux] AMP/RL -> imitation handoff blend %.3fs (%zu joints)",
                    imitation_entry_blend_sec_, q_count);
      }
    }
  }

  active_joystick_imitation_finish_mode_.store(binding->on_finish);
  setPolicySource(PolicyCmdSource::kImitation);
  policyMux_.publishImitationReset();
  publishImitationActive(true);
  applyJoystickMotion(lastJoystickCommand_, last_controller_wall_sec_);
}

}  // namespace legged
