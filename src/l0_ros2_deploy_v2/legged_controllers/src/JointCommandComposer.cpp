#include "legged_controllers/JointCommandComposer.h"

#include <algorithm>

namespace legged {

void JointCommandComposer::composeAll(const Context& ctx) const {
  if (ctx.robot_hw == nullptr) return;

  const int joint_motor_commands_size = ctx.robot_hw->getJointMotorCommandsSize();
  const auto& q_snapshot = ctx.policy_inputs.q_snapshot;
  const auto& active_gains = ctx.policy_inputs.gains;

  if (ctx.release_handoff != nullptr)
  {
    ctx.release_handoff->clearIfExpired(ctx.wall_sec);
  }
  const bool release_handoff_available =
      ctx.release_handoff != nullptr && ctx.release_handoff->active() && !ctx.action_sequence_active;

  for (size_t j = 0; j < static_cast<size_t>(joint_motor_commands_size); ++j)
  {
    double q_j, dq_j, kp_j, kd_j, tau_ff_j;
    const bool is_leg_joint = ctx.is_leg_joint != nullptr && j < ctx.is_leg_joint->size() && (*ctx.is_leg_joint)[j];

    const bool csv_allowed_for_joint = ctx.action_sequence_full_csv || !is_leg_joint;
    if (csv_allowed_for_joint && ctx.joint_csv != nullptr && ctx.joint_csv->isEnabled() &&
        ctx.joint_csv->getJointCmd(j, q_j, dq_j, kp_j, kd_j, tau_ff_j))
    {
      // Joint CSV timeline (auxiliary joints normally; full body only for back-emergency full_csv).
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
      q_j = (j < ctx.motor_states.size()) ? ctx.motor_states[j].q : q_j;
      dq_j = 0.0;
      kp_j = 0.0;
      kd_j = std::max(0.0, ctx.emergency_damping_kd);
      tau_ff_j = 0.0;
    }

    legged::JointMotorCommand cmd(q_j, dq_j, kp_j, kd_j, tau_ff_j);
    ctx.robot_hw->set_joint_motor_commands(static_cast<int>(j), cmd);
  }
}

}  // namespace legged
