#include "JointCommandComposer.h"

#include <algorithm>
#include <cmath>

namespace legged {

bool JointCommandComposer::composeAll(const Context& ctx) {
  if (ctx.robot_hw == nullptr) return false;

  const int joint_motor_commands_size = ctx.robot_hw->getJointMotorCommandsSize();
  if (joint_motor_commands_size <= 0) return false;
  next_commands_.resize(static_cast<size_t>(joint_motor_commands_size));
  const auto& q_snapshot = ctx.policy_inputs.q_snapshot;
  const auto& active_gains = ctx.policy_inputs.gains;
  if (ctx.walking && !ctx.emergency &&
      (q_snapshot.size() != next_commands_.size() || active_gains.kps.size() != next_commands_.size() ||
       active_gains.kds.size() != next_commands_.size())) return false;

  if (ctx.release_handoff != nullptr)
  {
    ctx.release_handoff->clearIfExpired(ctx.wall_sec);
  }
  const bool release_handoff_available =
      ctx.release_handoff != nullptr && ctx.release_handoff->active();

  for (size_t j = 0; j < static_cast<size_t>(joint_motor_commands_size); ++j)
  {
    double q_j, dq_j, kp_j, kd_j, tau_ff_j;
    if (ctx.head_hold_positions != nullptr && j >= 18 && j < 21)
    {
      q_j = (*ctx.head_hold_positions)[j - 18];
      dq_j = kp_j = kd_j = tau_ff_j = 0.0;
    }
    else if (ctx.arm_swing_ctrl != nullptr && ctx.arm_swing_ctrl->getJointCmd(j, q_j, dq_j, kp_j, kd_j, tau_ff_j))
    {
      // Procedural walking arm swing.
    }
    else if (ctx.allow_auxiliary_overlays && ctx.head_ctrl != nullptr &&
             ctx.head_ctrl->getJointCmd(j, q_j, dq_j, kp_j, kd_j, tau_ff_j))
    {
      // Walking-only head overlay.
    }
    else
    {
      q_j = (j < q_snapshot.size()) ? q_snapshot[j] : 0.0;
      dq_j = 0.0;
      kp_j = kd_j = tau_ff_j = 0.0;
    }

    if (release_handoff_available && !ctx.emergency && !ctx.blend_policy_gains)
    {
      (void)ctx.release_handoff->apply(j, ctx.wall_sec, q_j, dq_j, kp_j, kd_j, tau_ff_j);
    }

    if (ctx.walking)
    {
      kp_j = (j < active_gains.kps.size()) ? active_gains.kps[j] : 0.0;
      kd_j = (j < active_gains.kds.size()) ? active_gains.kds[j] : 0.0;
    }
    else if (ctx.is_sim)
    {
      kp_j = 80.0;
      kd_j = 1.0;
    }
    else
    {
      double kp_limit = ctx.kp_standby;
      if (j < ctx.rl_gains.kps.size()) kp_limit = std::min(ctx.kp_standby, ctx.rl_gains.kps[j]);
      kp_j = kp_limit;
      kd_j = 0.05;
    }

    // WBC changes gain profiles as well as position targets.
    if (release_handoff_available && !ctx.emergency && ctx.blend_policy_gains)
      (void)ctx.release_handoff->apply(j, ctx.wall_sec, q_j, dq_j, kp_j, kd_j, tau_ff_j);

    if (ctx.emergency)
    {
      q_j = (ctx.motor_states != nullptr && j < ctx.motor_states->size()) ? (*ctx.motor_states)[j].q : q_j;
      if (!std::isfinite(q_j)) q_j = 0.0;
      dq_j = 0.0;
      kp_j = 0.0;
      kd_j = std::max(0.0, ctx.emergency_damping_kd);
      tau_ff_j = 0.0;
    }

    if (!std::isfinite(q_j) || !std::isfinite(dq_j) || !std::isfinite(kp_j) ||
        !std::isfinite(kd_j) || !std::isfinite(tau_ff_j) || kp_j < 0.0 || kd_j < 0.0) return false;
    next_commands_[j] = JointMotorCommand(q_j, dq_j, kp_j, kd_j, tau_ff_j);
  }
  // Validate the complete frame before replacing any part of the last usable command.
  for (size_t j = 0; j < next_commands_.size(); ++j)
  {
    if (ctx.upper_position_snapshot != nullptr && j >= 12 && j < 21)
      (*ctx.upper_position_snapshot)[j - 12] = next_commands_[j].q_des;
    ctx.robot_hw->set_joint_motor_commands(static_cast<int>(j), next_commands_[j]);
  }
  if (!ctx.emergency) last_commands_ = next_commands_;
  else last_commands_.clear();
  return true;
}

bool JointCommandComposer::holdLastCommand(IHardwareLayer* robot_hw) const {
  if (robot_hw == nullptr || last_commands_.empty() ||
      last_commands_.size() != static_cast<size_t>(robot_hw->getJointMotorCommandsSize())) return false;
  for (size_t j = 0; j < last_commands_.size(); ++j)
    robot_hw->set_joint_motor_commands(static_cast<int>(j), last_commands_[j]);
  return true;
}

}  // namespace legged
