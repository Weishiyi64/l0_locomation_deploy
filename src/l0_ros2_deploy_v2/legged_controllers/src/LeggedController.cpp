//
// Common policy bridge for legged controllers.
//
#include "legged_controllers/LeggedController.h"

#include <algorithm>

namespace legged {

bool LeggedController::loadWalkingGainProfile(const YAML::Node& rc, const std::string& profile_name,
                                              size_t expected_size, JointGainProfile& out) const {
  return policyMux_.loadWalkingGainProfile(rc, profile_name, expected_size, out);
}

void LeggedController::setPolicySource(PolicyCmdSource source, const char* reason) {
  policyMux_.setSource(source, reason);
}

PolicyCmdSource LeggedController::walkStartPolicySource() const { return policyMux_.walkStartPolicySource(); }

void LeggedController::selectPolicyInputs(bool walking, std::vector<double>& q_snapshot,
                                          JointGainProfile& active_gains) const {
  const PolicyInputs inputs = policyMux_.selectInputs(walking);
  q_snapshot = inputs.q_snapshot;
  active_gains = inputs.gains;
}

void LeggedController::publishRlObsReset() { policyMux_.publishRlObsReset(); }

void LeggedController::returnToRlWithoutGainRamp(const char* reason) {
  if (imitation_return_blend_sec_ > 1e-9 && policyMux_.source() == PolicyCmdSource::kImitation)
  {
    const bool walking = fsm_ && fsm_->isWalking();
    const PolicyInputs inputs = policyMux_.selectInputs(walking);
    if (inputs.using_imitation && !inputs.q_snapshot.empty())
    {
      const size_t n = jointNames_.size();
      std::vector<bool> joint_mask(n, false);
      std::vector<JointCommandSample> start_cmd(n);
      const size_t q_count = std::min(n, inputs.q_snapshot.size());
      for (size_t j = 0; j < q_count; ++j)
      {
        joint_mask[j] = true;
        start_cmd[j].q = inputs.q_snapshot[j];
        start_cmd[j].dq = 0.0;
        start_cmd[j].kp = (j < inputs.gains.kps.size()) ? inputs.gains.kps[j] : 0.0;
        start_cmd[j].kd = (j < inputs.gains.kds.size()) ? inputs.gains.kds[j] : 0.0;
        start_cmd[j].tau_ff = 0.0;
      }

      double wall_sec = last_controller_wall_sec_;
      if (wall_sec <= 0.0 && nh_)
      {
        wall_sec = nh_->get_clock()->now().seconds();
      }

      JointCommandTransition& handoff = sequenceRuntime_.releaseHandoff();
      handoff.clear();
      if (handoff.start(joint_mask, start_cmd, wall_sec, imitation_return_blend_sec_))
      {
        RCLCPP_INFO(nh_->get_logger(), "[PolicyMux] Imitation -> RL handoff blend %.3fs", imitation_return_blend_sec_);
      }
    }
  }

  policyMux_.returnToRlWithoutGainRamp(reason);
}

void LeggedController::publishImitationActive(bool active, const char* reason) {
  policyMux_.publishImitationActive(active, reason);
}

bool LeggedController::switchImitationPolicy(const std::string& policy_name, const char* context) {
  return policyMux_.switchImitationPolicy(policy_name, context);
}

}  // namespace legged
