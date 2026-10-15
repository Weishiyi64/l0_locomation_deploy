#include <algorithm>
#include <cmath>
#include <vector>

#include "legged_controllers/LeggedController.h"

namespace legged {

namespace {

constexpr double kFullCsvVxDeadzone = 0.05;
constexpr double kFullCsvWzDeadzone = 0.10;

std::string relativeToBasePath(const std::string& path, const std::string& base) {
  if (path.empty() || base.empty()) return path;

  std::string normalized_base = base;
  while (normalized_base.size() > 1 && normalized_base.back() == '/')
  {
    normalized_base.pop_back();
  }

  if (path == normalized_base) return ".";

  const std::string prefix = normalized_base + "/";
  if (path.rfind(prefix, 0) == 0) return path.substr(prefix.size());

  return path;
}

geometry_msgs::msg::Twist twistFromVelocityTimeline(const std::vector<ActionSequenceVelocityKeyframe>& timeline,
                                                    double elapsed_sec) {
  geometry_msgs::msg::Twist out;
  if (timeline.empty()) return out;
  std::size_t idx = 0;
  constexpr double kEps = 1e-9;
  for (std::size_t k = 0; k < timeline.size(); ++k)
  {
    if (timeline[k].time_from_phase_start_sec <= elapsed_sec + kEps) idx = k;
  }
  out.linear.x = timeline[idx].velocity.linear.x;
  out.linear.y = timeline[idx].velocity.linear.y;
  out.angular.z = timeline[idx].velocity.angular.z;
  return out;
}

}  // namespace

bool LeggedController::fullCsvVelocityAllowed() const {
  return std::abs(lowState_.cmd_vel.linear.x) <= kFullCsvVxDeadzone &&
         std::abs(lowState_.cmd_vel.angular.z) <= kFullCsvWzDeadzone;
}

void LeggedController::logFullCsvVelocityBlocked(const char* context, const std::string& target) const {
  RCLCPP_WARN(nh_->get_logger(),
              "%s blocked full_csv '%s': cmd_vel vx=%.3f m/s, wz=%.3f rad/s outside deadzone "
              "(|vx|<=%.3f, |wz|<=%.3f)",
              context != nullptr ? context : "[ActionSequence]", target.c_str(), lowState_.cmd_vel.linear.x,
              lowState_.cmd_vel.angular.z, kFullCsvVxDeadzone, kFullCsvWzDeadzone);
}

bool LeggedController::prepareImitationPolicyForLoadedSequence(const char* context) {
  const bool requires_walking_policy = sequenceRuntime_.requiresWalkingPolicy();
  const bool uses_rl_imitation = sequenceRuntime_.usesRlImitation();
  const std::string imitation_policy = sequenceRuntime_.imitationPolicy();

  if (requires_walking_policy && fsm_ && !fsm_->isWalking())
  {
    RCLCPP_WARN(nh_->get_logger(),
                "%s ignored: this sequence uses walking/imitation policy phases. Enter walk mode before triggering it.",
                context != nullptr ? context : "ActionSequence");
    return false;
  }
  if (!uses_rl_imitation) return true;
  return switchImitationPolicy(imitation_policy, context);
}

bool LeggedController::loadActionSequenceForPlayback(const std::string& path, const char* context) {
  return sequenceRuntime_.loadActionSequenceForPlayback(path, context);
}

void LeggedController::clearSequenceReleaseHandoff() { sequenceRuntime_.clearReleaseHandoff(); }

void LeggedController::beginSequenceReleaseHandoff(double wall_sec) {
  sequenceRuntime_.beginReleaseHandoff(wall_sec, is_leg_joint_);
}

bool LeggedController::loadEmotionSequenceMap(const std::string& path) {
  return sequenceRuntime_.loadEmotionSequenceMap(path);
}

bool LeggedController::loadKeyboardActionMap(const std::string& path) {
  return sequenceRuntime_.loadKeyboardActionMap(path);
}

std::string LeggedController::displayControllerPath(const std::string& path) const {
  return relativeToBasePath(path, legged_controllers_share_);
}

bool LeggedController::triggerSequenceFromPath(const std::string& sequence_path, const char* context,
                                               const std::function<void()>& after_start) {
  if (!sequenceRuntime_.loadedPathMatches(sequence_path))
  {
    if (!loadActionSequenceForPlayback(sequence_path, context))
    {
      return false;
    }
  }
  if (!sequenceRuntime_.isLoaded()) return false;

  if (sequenceRuntime_.usesFullCsv() && !fullCsvVelocityAllowed())
  {
    logFullCsvVelocityBlocked(context, sequence_path);
    return false;
  }

  if (!prepareImitationPolicyForLoadedSequence(context))
  {
    return false;
  }

  if (!startActionSequence())
  {
    return false;
  }
  if (after_start)
  {
    after_start();
  }
  return true;
}

bool LeggedController::startActionSequence() {
  // Timeline is anchored on the next controller update(time) — do not use node clock here or
  // remainingPhaseSeconds / base_vel ramp will disagree with step() (common in sim).
  sequence_finish_emergency_pending_ = false;
  return sequenceRuntime_.triggerLoaded();
}

void LeggedController::stopActionSequence(const char* reason, double wall_sec) {
  (void)reason;
  sequence_finish_emergency_pending_ = false;
  sequenceRuntime_.resetForAbort();

  if (walkStartPolicySource() == PolicyCmdSource::kRl)
  {
    returnToRlWithoutGainRamp(nullptr);
  }
  else
  {
    publishImitationActive(false);
    setPolicySource(PolicyCmdSource::kImitation);
  }
  applyJoystickMotion(lastJoystickCommand_, wall_sec);
}

void LeggedController::finishActionSequenceNormally(double wall_sec) {
  sequence_finish_emergency_pending_ = false;
  sequenceRuntime_.resetForNormalFinish(wall_sec, is_leg_joint_);

  if (walkStartPolicySource() == PolicyCmdSource::kRl)
  {
    returnToRlWithoutGainRamp(nullptr);
  }
  else
  {
    publishImitationActive(false);
    setPolicySource(PolicyCmdSource::kImitation);
  }
  applyJoystickMotion(lastJoystickCommand_, wall_sec);
}

void LeggedController::configureActionSequencePhase(const ActionSequencePhase& phase, double wall_sec) {
  if (phase.mode == ActionSequenceMode::kRlImitation)
  {
    sequenceRuntime_.resetPhaseFlags();
    sequenceRuntime_.jointCsv().setHoldLastFrame(false);
    sequenceRuntime_.jointCsv().stop();

    setPolicySource(PolicyCmdSource::kImitation);
    policyMux_.publishImitationReset();
    publishImitationActive(true);
    applyJoystickMotion(lastJoystickCommand_, wall_sec);
    return;
  }

  if (phase.mode == ActionSequenceMode::kFullCsv && !fullCsvVelocityAllowed())
  {
    logFullCsvVelocityBlocked("[ActionSequence] Phase guard", phase.name);
    stopActionSequence("[ActionSequence] full_csv blocked by cmd_vel deadzone -> default policy", wall_sec);
    return;
  }

  publishImitationActive(false);
  sequenceRuntime_.setFullCsvPhase(phase.mode == ActionSequenceMode::kFullCsv);
  if (phase.mode == ActionSequenceMode::kRlWithCsv)
  {
    setPolicySource(walkStartPolicySource());
  }
  else if (phase.mode == ActionSequenceMode::kFullCsv && policyMux_.source() == PolicyCmdSource::kImitation)
  {
    returnToRlWithoutGainRamp(nullptr);
  }

  sequenceRuntime_.jointCsv().setHoldLastFrame(true);
  sequenceRuntime_.jointCsv().stop();
  if (!sequenceRuntime_.jointCsv().load(phase.csv_path))
  {
    stopActionSequence("[ActionSequence] Failed to load phase CSV -> default policy", wall_sec);
    return;
  }
  if (phase.mode == ActionSequenceMode::kFullCsv && !sequenceRuntime_.jointCsv().coversAllJoints())
  {
    RCLCPP_WARN(nh_->get_logger(), "[ActionSequence] full_csv phase '%s' does not cover all robot joints",
                phase.name.c_str());
  }
  const double csv_playback_sec = sequenceRuntime_.jointCsv().duration();
  if (phase.duration_explicit && phase.mode == ActionSequenceMode::kFullCsv && csv_playback_sec > 1e-9 &&
      phase.duration + 1e-3 < csv_playback_sec)
  {
    RCLCPP_WARN(nh_->get_logger(),
                "[ActionSequence] Phase '%s': YAML duration %.3fs < CSV playback %.3fs — phase ends before clip; "
                "omit duration or use duration: auto to match the file.",
                phase.name.c_str(), phase.duration, csv_playback_sec);
  }

  applyJointCsvPhaseTriggerBlendAndCmdVel(wall_sec);
}

void LeggedController::handleActionSequenceUpdate(const rclcpp::Time& time) {
  if (!sequence_engine_enabled_) return;

  if (sequenceRuntime_.isActive())
  {
    sequenceRuntime_.step(time.seconds());

    ActionSequencePhase phase;
    if (sequenceRuntime_.consumePhaseStartEvent(phase))
    {
      configureActionSequencePhase(phase, time.seconds());
    }
  }

  if (sequenceRuntime_.consumeFinishedEvent())
  {
    if (sequence_finish_emergency_pending_)
    {
      sequence_finish_emergency_pending_ = false;
      RCLCPP_WARN(nh_->get_logger(), "[BackEmergency] Sequence finished: entering EmergencyStop");
      triggerEmergencyStop();
      return;
    }

    if (sequenceRuntime_.softReleaseOnFinish())
    {
      const auto motor_states = robot_hw_->getHyridJointMotorState();
      sequenceRuntime_.resetForSoftReleaseFinish(motor_states);
      publishImitationActive(false);
      RCLCPP_WARN(
          nh_->get_logger(),
          "[ActionSequence] Sequence finished: soft-release LATCHED (legs+arms => kp/kd/tau=0, head => hold last "
          "target; no default RL). Use emergency, then reset, then set_walk to restore.");
      applyJoystickMotion(lastJoystickCommand_, time.seconds());
    }
    else
    {
      finishActionSequenceNormally(time.seconds());
    }
  }
}

void LeggedController::requestKeyboardAction(const std::string& action_name) {
  if (!keyboard_action_enabled_) return;

  const KeyboardActionBinding* binding_ptr = sequenceRuntime_.findKeyboardAction(action_name);
  if (binding_ptr == nullptr)
  {
    RCLCPP_WARN(nh_->get_logger(), "[KeyboardAction] Ignoring unknown action '%s'", action_name.c_str());
    return;
  }
  const KeyboardActionBinding& binding = *binding_ptr;

  if (sequenceRuntime_.isActive())
  {
    RCLCPP_WARN(nh_->get_logger(), "[KeyboardAction] Ignoring action '%s' because an action sequence is already active",
                action_name.c_str());
    return;
  }

  if (sequenceRuntime_.softReleaseLatched() || (fsm_ && (fsm_->isEmergency() || fsm_->isCalibration())))
  {
    RCLCPP_WARN(nh_->get_logger(),
                "[KeyboardAction] Ignoring trigger because the controller is not in a sequence-safe state");
    return;
  }
  if (binding.requires_walking_policy && fsm_ && !fsm_->isWalking())
  {
    RCLCPP_WARN(nh_->get_logger(),
                "[KeyboardAction] Ignoring action '%s' because it uses walking/imitation policy phases. Enter walk "
                "mode before triggering it.",
                action_name.c_str());
    return;
  }

  const auto publish_auxiliary_commands_and_log = [this, &action_name, &binding]() {
    if (keyboardActionAudioPub_ && !binding.audio_cmd.empty())
    {
      std_msgs::msg::String audio_msg;
      audio_msg.data = binding.audio_cmd;
      keyboardActionAudioPub_->publish(audio_msg);
      RCLCPP_INFO(nh_->get_logger(), "[KeyboardAction] Action '%s' published audio command to %s: '%s'",
                  action_name.c_str(), sequenceRuntime_.keyboardAudioTopic().c_str(), binding.audio_cmd.c_str());
    }

    RCLCPP_INFO(nh_->get_logger(), "[KeyboardAction] Starting action '%s' sequence: %s", action_name.c_str(),
                binding.sequence_path.c_str());
  };
  (void)triggerSequenceFromPath(binding.sequence_path, "[KeyboardAction] Trigger", publish_auxiliary_commands_and_log);
}

geometry_msgs::msg::Twist LeggedController::computeJoystickTrim(const JoystickCommand& command) {
  geometry_msgs::msg::Twist input;
  input.linear.x = command.vx;
  input.linear.y = command.vy;
  input.angular.z = command.wz;

  updateSpeedMultiplier(command.lt_axis);
  return velLimiter_ ? velLimiter_->filter(input) : input;
}

geometry_msgs::msg::Twist LeggedController::computeCommandVelocity(const JoystickCommand& command, double wall_sec) {
  const geometry_msgs::msg::Twist trim = computeJoystickTrim(command);

  const ActiveSequencePhaseSnapshot snapshot = sequenceRuntime_.activePhaseSnapshot(wall_sec);
  if (!snapshot.active)
  {
    return trim;
  }
  const ActionSequencePhase& active_phase = snapshot.phase;

  if (active_phase.mode == ActionSequenceMode::kFullCsv)
  {
    return geometry_msgs::msg::Twist();
  }

  if (active_phase.mode == ActionSequenceMode::kRlImitation)
  {
    return trim;
  }

  geometry_msgs::msg::Twist out;
  if (!active_phase.velocity_timeline.empty())
  {
    out = twistFromVelocityTimeline(active_phase.velocity_timeline, snapshot.elapsed_sec);
  }
  else
  {
    out.linear.x = active_phase.base_velocity.linear.x;
    out.linear.y = active_phase.base_velocity.linear.y;
    out.angular.z = active_phase.base_velocity.angular.z;
  }

  double scale = 1.0;
  if (snapshot.base_vel_ramp_sec > 1e-9)
  {
    const double eff_ramp = std::min(snapshot.base_vel_ramp_sec, std::max(active_phase.duration, 1e-9));
    if (snapshot.remaining_sec < eff_ramp) scale = std::clamp(snapshot.remaining_sec / eff_ramp, 0.0, 1.0);
  }
  out.linear.x *= scale;
  out.linear.y *= scale;
  out.angular.z *= scale;

  out.linear.x += trim.linear.x;
  out.linear.y += trim.linear.y;
  out.angular.z += trim.angular.z;

  return out;
}

void LeggedController::applyJoystickMotion(const JoystickCommand& command, double wall_sec) {
  lowState_.cmd_vel = computeCommandVelocity(command, wall_sec);
  lowState_.behavior_ud = command.behavior_ud;
  lowState_.behavior_lr = command.behavior_lr;
}

}  // namespace legged
