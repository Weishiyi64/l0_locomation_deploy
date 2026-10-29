#include "legged_controllers/SequenceRuntime.h"

#include <algorithm>

#include "legged_controllers/ControllerFileUtils.h"

namespace legged {

void SequenceRuntime::init(rclcpp::Logger logger, const std::string& package_share_dir) {
  logger_ = logger;
  package_share_dir_ = package_share_dir;
  action_sequence_.init(logger_, package_share_dir_);
}

void SequenceRuntime::initJoints(const std::vector<std::string>& joint_names) {
  joint_csv_.initJoints(joint_names, logger_);
  release_handoff_.resize(joint_names.size());
}

void SequenceRuntime::setEnabled(bool enabled) {
  sequence_engine_enabled_ = enabled;
  joint_csv_.setEnabled(enabled);
  joint_csv_.setHoldLastFrame(false);
  joint_csv_.setBlendDuration(0.0);
}

bool SequenceRuntime::loadActionSequenceForPlayback(const std::string& path, const char* context) {
  const std::string resolved_path = resolvePackagePath(path, package_share_dir_);
  if (resolved_path.empty() || !pathExists(resolved_path))
  {
    RCLCPP_ERROR(logger(), "%s failed: sequence path is missing or not a file: %s",
                 context != nullptr ? context : "ActionSequence", path.c_str());
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!action_sequence_.load(resolved_path))
  {
    RCLCPP_ERROR(logger(), "%s failed to load sequence: %s", context != nullptr ? context : "ActionSequence",
                 resolved_path.c_str());
    loaded_action_sequence_path_.clear();
    csv_entry_blend_sec_ = 0.0;
    csv_exit_blend_sec_ = 0.0;
    full_csv_phase_ = false;
    joint_csv_.setBlendDuration(0.0);
    return false;
  }

  loaded_action_sequence_path_ = resolved_path;
  csv_entry_blend_sec_ = std::max(action_sequence_.csvEntryBlendSec(), 0.0);
  csv_exit_blend_sec_ = std::max(action_sequence_.csvExitBlendSec(), 0.0);
  joint_csv_.setBlendDuration(csv_entry_blend_sec_);
  return true;
}

bool SequenceRuntime::isLoaded() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.isLoaded();
}

bool SequenceRuntime::isActive() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.isActive();
}

bool SequenceRuntime::isFullCsvPhase() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const ActionSequencePhase* active_phase = action_sequence_.activePhase();
  return action_sequence_.isActive() && active_phase != nullptr && active_phase->mode == ActionSequenceMode::kFullCsv;
}

bool SequenceRuntime::requiresWalkingPolicy() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.requiresWalkingPolicy();
}

bool SequenceRuntime::usesRlImitation() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.usesRlImitation();
}

bool SequenceRuntime::usesFullCsv() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.usesFullCsv();
}

std::string SequenceRuntime::imitationPolicy() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.imitationPolicy();
}

bool SequenceRuntime::loadedPathMatches(const std::string& path) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return loaded_action_sequence_path_ == path;
}

ActiveSequencePhaseSnapshot SequenceRuntime::activePhaseSnapshot(double wall_sec) const {
  ActiveSequencePhaseSnapshot snapshot;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const ActionSequencePhase* active_phase = action_sequence_.activePhase();
  snapshot.active = action_sequence_.isActive() && active_phase != nullptr;
  if (!snapshot.active) return snapshot;
  snapshot.phase = *active_phase;
  snapshot.elapsed_sec = action_sequence_.elapsedPhaseSeconds(wall_sec);
  snapshot.remaining_sec = action_sequence_.remainingPhaseSeconds(wall_sec);
  snapshot.base_vel_ramp_sec = action_sequence_.activePhaseBaseVelRampSec();
  return snapshot;
}

bool SequenceRuntime::triggerLoaded() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!sequence_engine_enabled_ || !action_sequence_.isLoaded()) return false;
  if (!action_sequence_.trigger()) return false;
  clearReleaseHandoff();
  full_csv_phase_ = false;
  return true;
}

void SequenceRuntime::step(double wall_sec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (action_sequence_.isActive())
  {
    action_sequence_.step(wall_sec);
  }
}

bool SequenceRuntime::consumePhaseStartEvent(ActionSequencePhase& out) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.consumePhaseStartEvent(out);
}

bool SequenceRuntime::consumeFinishedEvent() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return action_sequence_.consumeFinishedEvent();
}

void SequenceRuntime::completeImitationPhaseByExternalEvent(double wall_sec, bool& next_phase_is_imitation) {
  next_phase_is_imitation = false;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  action_sequence_.completeActivePhaseByExternalEvent(wall_sec);
  const ActionSequencePhase* next_phase = action_sequence_.activePhase();
  next_phase_is_imitation =
      action_sequence_.isActive() && next_phase != nullptr && next_phase->mode == ActionSequenceMode::kRlImitation;
}

void SequenceRuntime::setFullCsvPhase(bool active) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  full_csv_phase_ = active;
}

void SequenceRuntime::resetPhaseFlags() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  full_csv_phase_ = false;
}

void SequenceRuntime::resetForAbort() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clearReleaseHandoff();
  full_csv_phase_ = false;
  action_sequence_.stop();
  joint_csv_.setHoldLastFrame(false);
  joint_csv_.stop();
}

void SequenceRuntime::resetForNormalFinish(double wall_sec, const std::vector<bool>& is_leg_joint) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  beginReleaseHandoff(wall_sec, is_leg_joint);
  full_csv_phase_ = false;
  action_sequence_.stop();
  joint_csv_.setHoldLastFrame(false);
  joint_csv_.stop();
}

void SequenceRuntime::clearReleaseHandoff() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  release_handoff_.clear();
}

bool SequenceRuntime::releaseHandoffActive(double wall_sec) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return release_handoff_.active() && !release_handoff_.expired(wall_sec);
}

void SequenceRuntime::beginReleaseHandoff(double wall_sec, const std::vector<bool>& is_leg_joint) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clearReleaseHandoff();
  const double duration = std::max(csv_exit_blend_sec_, 0.0);
  if (duration <= 1e-9 || !joint_csv_.isEnabled() || joint_csv_.empty()) return;

  joint_csv_.step(0.0, wall_sec);
  const std::vector<bool>& csv_joint_mask = joint_csv_.csvJointMask();
  const size_t n = csv_joint_mask.size();
  if (n == 0) return;

  std::vector<JointCommandSample> release_start(n);
  std::vector<bool> release_mask(n, false);

  size_t captured = 0;
  for (size_t j = 0; j < n; ++j)
  {
    if (!csv_joint_mask[j]) continue;
    if (!full_csv_phase_ && j < is_leg_joint.size() && is_leg_joint[j]) continue;
    double q = 0.0, dq = 0.0, kp = 0.0, kd = 0.0, tau_ff = 0.0;
    if (!joint_csv_.getJointCmd(j, q, dq, kp, kd, tau_ff)) continue;
    release_mask[j] = true;
    release_start[j].q = q;
    release_start[j].dq = dq;
    release_start[j].kp = kp;
    release_start[j].kd = kd;
    release_start[j].tau_ff = tau_ff;
    ++captured;
  }

  if (captured == 0) return;
  if (!release_handoff_.start(release_mask, release_start, wall_sec, duration)) return;
}

void SequenceRuntime::triggerCsvBlend(const std::vector<double>& q_all, double wall_sec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  joint_csv_.onTrigger(q_all, wall_sec);
  action_sequence_.addBlendToCurrentPhaseTimeline(csv_entry_blend_sec_);
}

}  // namespace legged
