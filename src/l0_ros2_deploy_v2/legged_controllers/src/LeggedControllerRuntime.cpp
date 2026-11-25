#include <cmath>
#include <vector>

#include "legged_controllers/LeggedController.h"

namespace legged {

namespace {

constexpr double kPi = 3.14159265358979323846;

double radiansToDegrees(double radians) { return radians * 180.0 / kPi; }

}  // namespace

void LeggedController::starting(const rclcpp::Time& time) {
  startingTime_ = time;
  lastVizTick_ = time;
  lastJoyFeedbackTick_ = time;
}

void LeggedController::prepareControllerClock(const rclcpp::Time& time) {
  shifted_time = time - startingTime_;
  if (lastVizTick_.nanoseconds() == 0)
  {
    lastVizTick_ = time;
  }
}

bool LeggedController::isActionSequenceFullCsv() const { return sequenceRuntime_.isFullCsvPhase(); }

RuntimeControlState LeggedController::computeRuntimeControlState() const {
  RuntimeControlState state;
  state.walking = fsm_ && fsm_->isWalking();
  state.emergency_control = fsm_ && (fsm_->isEmergency() || fsm_->isCalibration());
  state.action_full_csv = isActionSequenceFullCsv();
  const bool amp_derived_action_active = sequenceRuntime_.isActive() || policyMux_.imitationActiveCommanded() ||
                                         policyMux_.wbcActiveCommanded() || wbc_return_pending_.load() ||
                                         wbc_handoff_in_progress_.load();
  state.policy_drives_legs = state.walking && !state.action_full_csv;
  state.rl_inference_enabled = state.policy_drives_legs;
  state.allow_auxiliary_overlays =
      state.walking && !state.action_full_csv && !state.emergency_control && !amp_derived_action_active;
  return state;
}

bool LeggedController::faceFollowAllowed() const {
  return face_follow_allowed_.load() && !sequenceRuntime_.isActive() && !policyMux_.imitationActiveCommanded() &&
         !policyMux_.wbcActiveCommanded() && !wbc_return_pending_.load() && !wbc_handoff_in_progress_.load();
}

void LeggedController::syncFaceFollowState(bool allowed, double wall_sec) {
  const bool effective = faceFollowEnabled_ && allowed;
  face_follow_allowed_.store(effective);
  faceFollowCtrl_.setAllowed(effective, wall_sec, headCtrl_, [this] { return readCurrentHeadRpyRad(); });
}

bool LeggedController::advanceSequenceAndWalkingPolicy(const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)period;
  handleActionSequenceUpdate(time);

  const RuntimeControlState runtime = computeRuntimeControlState();
  if (!runtime.policy_drives_legs && !sequenceRuntime_.isActive() && policyMux_.source() != PolicyCmdSource::kRl)
  {
    abortWbcImmediately("[WBC] Leaving Walking mode");
    setPolicySource(PolicyCmdSource::kRl, "[PolicyMux] Leaving walk -> default RL source");
  }
  policy_observation_.inference_enabled = runtime.rl_inference_enabled;
  return runtime.policy_drives_legs;
}

void LeggedController::runSafetyAndEmergencyFeedback(const rclcpp::Time& time, bool safety_armed) {
  const bool in_emergency = fsm_->isEmergency();
  const bool in_calibration = fsm_->isCalibration();
  if (!in_emergency && !in_calibration && safety_armed)
  {
    checkSafety();
  }
  else
  {
    orientation_violation_count_ = 0;
  }

  if (fsm_->isEmergency())
  {
    if (joyFeedbackPeriod_.seconds() > 0.0)
    {
      if (lastJoyFeedbackTick_.nanoseconds() == 0)
      {
        lastJoyFeedbackTick_ = time;
      }
      if ((time - lastJoyFeedbackTick_) >= (joyFeedbackPeriod_ * 0.5))
      {
        lastJoyFeedbackTick_ = time;
        joyFeedbackState_ = !joyFeedbackState_;
        const double intensity = joyFeedbackState_ ? 0.25 : 0.0;
        joyFeedbackArray_.array[0].intensity = intensity;
        joyFeedbackArray_.array[1].intensity = intensity;
        joyFeedbackPub_->publish(joyFeedbackArray_);
      }
    }
  }
}

void LeggedController::publishPolicyObservationWithJoystick(double wall_sec) {
  applyJoystickMotion(lastJoystickCommand_, wall_sec);
  policy_observation_.header.stamp = nh_->get_clock()->now();
  policy_observation_.header.frame_id = "odom";
  policy_observation_.imu.header.stamp = policy_observation_.header.stamp;
  policy_observation_.imu.header.frame_id = "base_link";
  policy_observation_.joints.header = policy_observation_.header;
  pubState_->publish(policy_observation_);
}

void LeggedController::stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Duration& period,
                                                  const rclcpp::Time& time) {
  if (allow_auxiliary_overlays && (headCtrl_.isTopicEnabled() || headCtrl_.isScriptEnabled()))
  {
    double t_loop_sec = 0.0;
    if (headCtrl_.isScriptEnabled() && !headCtrl_.empty() && headCtrl_.duration() > 0.0)
    {
      t_loop_sec = std::fmod(shifted_time.seconds(), headCtrl_.duration());
      if (t_loop_sec < 0.0) t_loop_sec += headCtrl_.duration();
    }
    headCtrl_.stepHeadOutput(period.seconds(), time.seconds(), t_loop_sec);
  }

  if (sequenceRuntime_.jointCsv().isEnabled() && !sequenceRuntime_.jointCsv().empty() &&
      sequenceRuntime_.jointCsv().duration() > 0.0)
  {
    sequenceRuntime_.jointCsv().step(period.seconds(), time.seconds());
    (void)sequenceRuntime_.jointCsv().consumeFinishedEvent();
  }
}

void LeggedController::maybePublishVisualization(const rclcpp::Time& time) {
  if ((time - lastVizTick_) >= vizPeriod_)
  {
    lastVizTick_ = time;
    rvizVisualization();
  }
}

void LeggedController::applyJointCsvPhaseTriggerBlendAndCmdVel(double wall_sec) {
  const auto motor_states = robot_hw_->getHyridJointMotorState();
  std::vector<double> q_all(motor_states.size());
  for (size_t i = 0; i < motor_states.size(); ++i)
  {
    q_all[i] = motor_states[i].q;
  }
  sequenceRuntime_.triggerCsvBlend(q_all, wall_sec);
  applyJoystickMotion(lastJoystickCommand_, wall_sec);
}

void LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!fsm_ || !robot_hw_)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "LeggedController::update called before init completed (fsm/robot_hw not ready)");
    return;
  }

  last_controller_wall_sec_ = time.seconds();
  prepareControllerClock(time);
  processSimWbcFinishEmergency();
  handleActionSequenceUpdate(time);
  RuntimeControlState runtime = computeRuntimeControlState();
  if (!runtime.policy_drives_legs && !sequenceRuntime_.isActive() && policyMux_.source() != PolicyCmdSource::kRl)
  {
    abortWbcImmediately("[WBC] Leaving Walking mode");
    setPolicySource(PolicyCmdSource::kRl, "[PolicyMux] Leaving walk -> default RL source");
  }
  policy_observation_.inference_enabled = runtime.rl_inference_enabled;
  updatePolicyObservationFromHardware();
  runSafetyAndEmergencyFeedback(time, runtime.policy_drives_legs);
  runtime = computeRuntimeControlState();
  if (runtime.emergency_control)
  {
    policy_observation_.inference_enabled = false;
  }
  publishPolicyObservationWithJoystick(time.seconds());
  updateWbcControlState();
  runtime = computeRuntimeControlState();
  syncFaceFollowState(runtime.allow_auxiliary_overlays && !sequenceRuntime_.releaseHandoffActive(time.seconds()),
                      time.seconds());
  stepAuxiliaryMotionSources(runtime.allow_auxiliary_overlays, period, time);
  applyJointCommands(runtime.policy_drives_legs, runtime.allow_auxiliary_overlays, runtime.emergency_control);
  maybePublishVisualization(time);
}

void LeggedController::applyJointCommands(bool walking, bool allow_auxiliary_overlays, bool emergency_control) {
  const bool action_sequence_active = sequenceRuntime_.isActive();
  const PolicyInputs policy_inputs = policyMux_.selectInputs(walking);
  const bool policy_expired = walking && !policy_inputs.valid;
  const bool effective_emergency = emergency_control || policy_expired;
  if (policy_expired)
  {
    if (policyMux_.source() == PolicyCmdSource::kWbc) triggerEmergencyStop();
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "Policy command expired or unavailable; applying measured-position damping hold");
  }
  const bool need_motor_state = effective_emergency;
  const auto motor_states =
      need_motor_state ? robot_hw_->getHyridJointMotorState() : std::vector<legged::JointMotorState>{};
  const bool arm_swing_allowed = allow_auxiliary_overlays && !action_sequence_active && !effective_emergency;
  armSwingCtrl_.update(last_controller_wall_sec_, policy_observation_.velocity_command.linear.x, arm_swing_allowed,
                       policy_inputs.q_snapshot);

  JointCommandComposer::Context ctx;
  ctx.robot_hw = robot_hw_.get();
  ctx.head_ctrl = &headCtrl_;
  ctx.joint_csv = &sequenceRuntime_.jointCsv();
  ctx.arm_swing_ctrl = &armSwingCtrl_;
  ctx.release_handoff = &sequenceRuntime_.releaseHandoff();
  ctx.policy_inputs = policy_inputs;
  ctx.blend_policy_gains = wbc_handoff_in_progress_.load();
  ctx.rl_gains = policyMux_.rlGainsSnapshot();
  ctx.motor_states = motor_states;
  ctx.is_leg_joint = &is_leg_joint_;
  // Active CSV/action phases use the AMP profile too; only a selected mimic policy
  // changes PolicyMux::gains away from the AMP profile.
  ctx.walking = walking || action_sequence_active;
  ctx.allow_auxiliary_overlays = allow_auxiliary_overlays;
  ctx.emergency = effective_emergency;
  ctx.is_sim = isSim_;
  ctx.action_sequence_full_csv = isActionSequenceFullCsv();
  ctx.action_sequence_active = action_sequence_active;
  ctx.kp_standby = kp_standby_;
  ctx.emergency_damping_kd = emergency_damping_kd_;
  ctx.wall_sec = last_controller_wall_sec_;
  jointCommandComposer_.composeAll(ctx);
}

void LeggedController::checkSafety() {
  if (isSim_ || !safetyChecker_)
  {
    orientation_violation_count_ = 0;
    return;
  }

  const auto& q = policy_observation_.imu.orientation;
  const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));

  SafetyChecker::OrientationResult orientationResult = safetyChecker_->checkOrientation(roll, pitch);
  if (orientationResult.ok)
  {
    orientation_violation_count_ = 0;
    return;
  }

  ++orientation_violation_count_;
  RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 500,
                       "[SafetyChecker] Orientation violation %d/%d: roll=%.1f deg, pitch=%.1f deg",
                       orientation_violation_count_, safetyChecker_->requiredConsecutiveOrientationViolations(),
                       radiansToDegrees(roll), radiansToDegrees(pitch));

  if (orientation_violation_count_ >= safetyChecker_->requiredConsecutiveOrientationViolations() &&
      !fsm_->isEmergency())
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "[SafetyChecker] Orientation violation persisted for %d cycles (roll=%.1f deg, pitch=%.1f deg)",
                 orientation_violation_count_, radiansToDegrees(roll), radiansToDegrees(pitch));
    triggerEmergencyStop();
  }
}

void LeggedController::updatePolicyObservationFromHardware() {
  std::vector<double> bodyPosition;
  std::vector<double> bodyOrientation;
  bodyPosition.resize(3);
  bodyOrientation.resize(4);
  bodyPosition = robot_hw_->getBodyPosition();
  bodyOrientation = robot_hw_->getBodyOrientation();

  if (bodyPosition.size() >= 3)
  {
    policy_observation_.base_pose.position.x = bodyPosition[0];
    policy_observation_.base_pose.position.y = bodyPosition[1];
    policy_observation_.base_pose.position.z = bodyPosition[2];
  }
  if (bodyOrientation.size() >= 4)
  {
    policy_observation_.base_pose.orientation.x = bodyOrientation[0];
    policy_observation_.base_pose.orientation.y = bodyOrientation[1];
    policy_observation_.base_pose.orientation.z = bodyOrientation[2];
    policy_observation_.base_pose.orientation.w = bodyOrientation[3];
  }

  std::vector<legged::JointMotorState> motor_states = robot_hw_->getHyridJointMotorState();
  const size_t state_count = std::min(motor_states.size(), policy_observation_.joints.position.size());
  for (size_t i = 0; i < state_count; ++i)
  {
    policy_observation_.joints.position[i] = motor_states[i].q;
    policy_observation_.joints.velocity[i] = motor_states[i].dq;
    policy_observation_.joints.effort[i] = motor_states[i].tau;
  }

  std::vector<double> imu_quaternion = robot_hw_->getImuQuaternion();
  std::vector<double> imu_angular_velocity = robot_hw_->getImuAngularVelocity();

  if (imu_quaternion.size() >= 4)
  {
    policy_observation_.imu.orientation.x = imu_quaternion[0];
    policy_observation_.imu.orientation.y = imu_quaternion[1];
    policy_observation_.imu.orientation.z = imu_quaternion[2];
    policy_observation_.imu.orientation.w = imu_quaternion[3];
  }
  if (imu_angular_velocity.size() >= 3)
  {
    policy_observation_.imu.angular_velocity.x = imu_angular_velocity[0];
    policy_observation_.imu.angular_velocity.y = imu_angular_velocity[1];
    policy_observation_.imu.angular_velocity.z = imu_angular_velocity[2];
  }
  policy_observation_.motion_time = shifted_time;
}

void LeggedController::rvizVisualization() {
  if (!enableRviz_)
  {
    return;
  }

  rvizVisualPos_.header.stamp = rclcpp::Clock().now();
  std::vector<legged::JointMotorState> motor_states = robot_hw_->getHyridJointMotorState();
  for (size_t i = 0; i < motor_states.size(); ++i)
  {
    rvizVisualPos_.position[i] = motor_states[i].q;
    rvizVisualPos_.velocity[i] = motor_states[i].dq;
    rvizVisualPos_.effort[i] = motor_states[i].tau;
  }
  vizPub_->publish(rvizVisualPos_);
}

}  // namespace legged
