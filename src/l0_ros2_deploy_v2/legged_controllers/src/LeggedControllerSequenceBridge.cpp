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
  return std::abs(policy_observation_.velocity_command.linear.x) <= kFullCsvVxDeadzone &&
         std::abs(policy_observation_.velocity_command.angular.z) <= kFullCsvWzDeadzone;
}

void LeggedController::logFullCsvVelocityBlocked(const char* context, const std::string& target) const {
  RCLCPP_WARN(nh_->get_logger(),
              "%s blocked full_csv '%s': cmd_vel vx=%.3f m/s, wz=%.3f rad/s outside deadzone "
              "(|vx|<=%.3f, |wz|<=%.3f)",
              context != nullptr ? context : "[ActionSequence]", target.c_str(),
              policy_observation_.velocity_command.linear.x, policy_observation_.velocity_command.angular.z,
              kFullCsvVxDeadzone, kFullCsvWzDeadzone);
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

    finishActionSequenceNormally(time.seconds());
  }
}

geometry_msgs::msg::Twist LeggedController::computeJoystickTrim(const JoystickCommand& command) {
  geometry_msgs::msg::Twist input;
  input.linear.x = command.vx;
  input.linear.y = command.vy;
  input.angular.z = command.wz;

  updateSpeedMultiplier(command.lt_axis);
  return velLimiter_ ? velLimiter_->filter(input) : input;
}

geometry_msgs::msg::Twist LeggedController::selectedOperatorVelocity(
    const geometry_msgs::msg::Twist& joystick_velocity) {
  if (!radarControlEnabled_)
  {
    return joystick_velocity;
  }

  if (!fsm_ || !fsm_->isWalking())
  {
    return geometry_msgs::msg::Twist();
  }

  std::lock_guard<std::mutex> lock(plannerCommandMutex_);
  if (!plannerCommandSeen_)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "Planner velocity command has not been received; commanding zero velocity");
    return geometry_msgs::msg::Twist();
  }

  if (!plannerCommandValid_)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "Planner velocity command is invalid; commanding zero velocity");
    return geometry_msgs::msg::Twist();
  }

  const bool fresh =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - plannerCommandReceiveTime_).count() <= 0.5;
  if (fresh)
  {
    return plannerCommand_;
  }

  RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                        "Planner velocity command is stale; commanding zero velocity");
  return geometry_msgs::msg::Twist();
}

geometry_msgs::msg::Twist LeggedController::computeCommandVelocity(const JoystickCommand& command, double wall_sec) {
  const geometry_msgs::msg::Twist trim = computeJoystickTrim(command);

  const ActiveSequencePhaseSnapshot snapshot = sequenceRuntime_.activePhaseSnapshot(wall_sec);
  if (!snapshot.active)
  {
    return selectedOperatorVelocity(trim);
  }
  const ActionSequencePhase& active_phase = snapshot.phase;

  if (active_phase.mode == ActionSequenceMode::kFullCsv)
  {
    return geometry_msgs::msg::Twist();
  }

  if (active_phase.mode == ActionSequenceMode::kRlImitation)
  {
    return selectedOperatorVelocity(trim);
  }

  const geometry_msgs::msg::Twist operator_velocity = selectedOperatorVelocity(trim);

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
    if (snapshot.remaining_sec < eff_ramp) scale = std::max(0.0, std::min(1.0, snapshot.remaining_sec / eff_ramp));
  }
  out.linear.x *= scale;
  out.linear.y *= scale;
  out.angular.z *= scale;

  out.linear.x += operator_velocity.linear.x;
  out.linear.y += operator_velocity.linear.y;
  out.angular.z += operator_velocity.angular.z;

  return out;
}

void LeggedController::applyJoystickMotion(const JoystickCommand& command, double wall_sec) {
  policy_observation_.velocity_command = computeCommandVelocity(command, wall_sec);
}

}  // namespace legged
