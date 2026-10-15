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
  const bool amp_derived_action_active = sequenceRuntime_.isActive() || policyMux_.imitationActiveCommanded();
  state.policy_drives_legs = state.walking && !state.action_full_csv;
  state.rl_inference_enabled = state.policy_drives_legs && !sequenceRuntime_.softReleaseLatched();
  state.allow_auxiliary_overlays =
      state.walking && !state.action_full_csv && !state.emergency_control && !amp_derived_action_active;
  return state;
}

bool LeggedController::headTopicIntegrationAllowed(double wall_sec) const {
  (void)wall_sec;
  return head_topic_integration_allowed_.load() && !sequenceRuntime_.isActive() &&
         !policyMux_.imitationActiveCommanded();
}

void LeggedController::syncHeadTopicIntegrationState(bool allowed, double wall_sec) {
  const bool effective_allowed = allowed && headCtrl_.isTopicEnabled();
  head_topic_integration_allowed_.store(effective_allowed);
  auto read_head = [this]() -> std::pair<double, double> { return readCurrentHeadYawPitchRad(); };
  headTopicCtrl_.setIntegrationAllowed(effective_allowed, wall_sec, headCtrl_, read_head);
}

bool LeggedController::advanceSequenceAndWalkingPolicy(const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)period;
  handleActionSequenceUpdate(time);

  const RuntimeControlState runtime = computeRuntimeControlState();
  if (!runtime.policy_drives_legs && !sequenceRuntime_.isActive() && policyMux_.source() != PolicyCmdSource::kRl)
  {
    setPolicySource(PolicyCmdSource::kRl, "[PolicyMux] Leaving walk -> default RL source");
  }
  lowState_.rl_inference_signal = runtime.rl_inference_enabled;
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

void LeggedController::publishLowStateWithJoystick(double wall_sec) {
  applyJoystickMotion(lastJoystickCommand_, wall_sec);
  pubState_->publish(lowState_);
}

void LeggedController::stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Duration& period,
                                                  const rclcpp::Time& time) {
  if (allow_auxiliary_overlays && (headCtrl_.isTopicEnabled() || headCtrl_.isScriptEnabled()))
  {
    double t_loop_sec = 0.0;
    if (headCtrl_.isScriptEnabled() && !headCtrl_.empty() && headCtrl_.duration() > 0.0)
    {
      double t = shifted_time.seconds();
      t_loop_sec = std::fmod(t, headCtrl_.duration());
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
  handleActionSequenceUpdate(time);
  RuntimeControlState runtime = computeRuntimeControlState();
  if (!runtime.policy_drives_legs && !sequenceRuntime_.isActive() && policyMux_.source() != PolicyCmdSource::kRl)
  {
    setPolicySource(PolicyCmdSource::kRl, "[PolicyMux] Leaving walk -> default RL source");
  }
  lowState_.rl_inference_signal = runtime.rl_inference_enabled;
  updateLowStateFromHardware();
  runSafetyAndEmergencyFeedback(time, runtime.policy_drives_legs);
  runtime = computeRuntimeControlState();
  if (runtime.emergency_control)
  {
    lowState_.rl_inference_signal = false;
  }
  publishLowStateWithJoystick(time.seconds());
  syncHeadTopicIntegrationState(
      runtime.allow_auxiliary_overlays && !sequenceRuntime_.releaseHandoffActive(time.seconds()), time.seconds());
  stepAuxiliaryMotionSources(runtime.allow_auxiliary_overlays, period, time);
  applyJointCommands(runtime.policy_drives_legs, runtime.allow_auxiliary_overlays, runtime.emergency_control);
  maybePublishVisualization(time);
}

void LeggedController::applyJointCommands(bool walking, bool allow_auxiliary_overlays, bool emergency_control) {
  const bool action_sequence_active = sequenceRuntime_.isActive();
  const bool in_soft_release_hold = sequenceRuntime_.softReleaseLatched() && !action_sequence_active;
  const bool need_motor_state = emergency_control || in_soft_release_hold;
  const auto motor_states =
      need_motor_state ? robot_hw_->getHyridJointMotorState() : std::vector<legged::JointMotorState>{};
  const PolicyInputs policy_inputs = policyMux_.selectInputs(walking);
  const bool arm_swing_allowed =
      allow_auxiliary_overlays && !action_sequence_active && !in_soft_release_hold && !emergency_control;
  armSwingCtrl_.update(last_controller_wall_sec_, lowState_.cmd_vel.linear.x, arm_swing_allowed,
                       policy_inputs.q_snapshot);

  JointCommandComposer::Context ctx;
  ctx.robot_hw = robot_hw_.get();
  ctx.head_ctrl = &headCtrl_;
  ctx.head_topic_ctrl = &headTopicCtrl_;
  ctx.joint_csv = &sequenceRuntime_.jointCsv();
  ctx.arm_swing_ctrl = &armSwingCtrl_;
  ctx.release_handoff = &sequenceRuntime_.releaseHandoff();
  ctx.policy_inputs = policy_inputs;
  ctx.rl_gains = policyMux_.rlGainsSnapshot();
  ctx.motor_states = motor_states;
  ctx.is_leg_joint = &is_leg_joint_;
  ctx.is_arm_joint = &is_arm_joint_;
  ctx.is_head_joint = &is_head_joint_;
  ctx.soft_release_head_hold_q = &sequenceRuntime_.headHoldQ();
  ctx.walking = walking;
  ctx.allow_auxiliary_overlays = allow_auxiliary_overlays;
  ctx.emergency = emergency_control;
  ctx.is_sim = isSim_;
  ctx.action_sequence_full_csv = isActionSequenceFullCsv();
  ctx.in_soft_release_hold = in_soft_release_hold;
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

  const double roll = lowState_.imu.rpy[0];
  const double pitch = lowState_.imu.rpy[1];

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

void LeggedController::updateLowStateFromHardware() {
  std::vector<double> bodyPosition;
  std::vector<double> bodyOrientation;
  bodyPosition.resize(3);
  bodyOrientation.resize(4);
  bodyPosition = robot_hw_->getBodyPosition();
  bodyOrientation = robot_hw_->getBodyOrientation();

  for (int i = 0; i < 3; ++i)
  {
    lowState_.body_xpos[i] = bodyPosition[i];
  }
  for (int i = 0; i < 4; ++i)
  {
    lowState_.body_xquat[i] = bodyOrientation[i];
  }

  std::vector<legged::JointMotorState> motor_states = robot_hw_->getHyridJointMotorState();
  for (size_t i = 0; i < motor_states.size(); ++i)
  {
    lowState_.motor_state[i].q = motor_states[i].q;
    lowState_.motor_state[i].dq = motor_states[i].dq;
    lowState_.motor_state[i].tau = motor_states[i].tau;
  }

  std::vector<double> imu_quaternion = robot_hw_->getImuQuaternion();
  std::vector<double> imu_angular_velocity = robot_hw_->getImuAngularVelocity();

  for (size_t i = 0; i < 4; ++i)
  {
    lowState_.imu.quaternion[i] = imu_quaternion[i];
  }
  for (size_t i = 0; i < 3; ++i)
  {
    lowState_.imu.gyroscope[i] = imu_angular_velocity[i];
  }

  const double x = lowState_.imu.quaternion[0];
  const double y = lowState_.imu.quaternion[1];
  const double z = lowState_.imu.quaternion[2];
  const double w = lowState_.imu.quaternion[3];

  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y * y);
  lowState_.imu.rpy[0] = std::atan2(t0, t1);
  double t2 = +2.0 * (w * y - z * x);
  if (t2 > 1.0)
    t2 = 1.0;
  else if (t2 < -1.0)
    t2 = -1.0;
  lowState_.imu.rpy[1] = std::asin(t2);
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  lowState_.imu.rpy[2] = std::atan2(t3, t4);

  lowState_.shifted_time = rclcpp::Time(shifted_time.nanoseconds());
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
