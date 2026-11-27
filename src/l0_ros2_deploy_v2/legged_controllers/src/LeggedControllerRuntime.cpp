#include <algorithm>
#include <cmath>

#include "LeggedController.h"

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

RuntimeControlState LeggedController::computeRuntimeControlState() const {
  RuntimeControlState state;
  state.walking = fsm_ && fsm_->isWalking();
  state.emergency_control = fsm_ && (fsm_->isEmergency() || fsm_->isCalibration());
  // Callback admission must not remove overlays before the control thread starts the handoff.
  const bool wbc_controls_commands = policyMux_.source() == PolicyCmdSource::kWbc ||
      (wbc_handoff_in_progress_.load() && policy_handoff_.active() &&
       !policy_handoff_.expired(last_controller_wall_sec_));
  const bool amp_derived_action_active = policyMux_.imitationActiveCommanded() || wbc_controls_commands ||
                                         wbc_preposition_active_.load();
  state.policy_drives_legs = state.walking;
  state.rl_inference_enabled = state.policy_drives_legs;
  state.allow_auxiliary_overlays =
      state.walking && !state.emergency_control && !amp_derived_action_active;
  return state;
}

bool LeggedController::faceFollowAllowed() const {
  return face_follow_allowed_.load() && !policyMux_.imitationActiveCommanded() && !motion_reserved_.load();
}

void LeggedController::syncFaceFollowState(bool allowed, double wall_sec) {
  const bool effective = faceFollowEnabled_ && allowed && !motion_reserved_.load();
  face_follow_allowed_.store(effective);
  faceFollowCtrl_.setAllowed(effective, wall_sec, headCtrl_, [this] { return readCurrentHeadRpyRad(); });
}

void LeggedController::runSafetyAndEmergencyFeedback(const rclcpp::Time& time, bool safety_armed) {
  const bool in_emergency = fsm_->isEmergency();
  const bool in_calibration = fsm_->isCalibration();
  const bool orientation_monitoring = !isSim_ && !in_emergency && !in_calibration && safety_armed;
  if (safetyChecker_) safetyChecker_->setOrientationMonitoring(orientation_monitoring);
  if (orientation_monitoring)
  {
    checkSafety();
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

void LeggedController::publishPolicyObservationWithJoystick() {
  applyJoystickMotion(lastJoystickCommand_);
  policy_observation_.header.stamp = nh_->get_clock()->now();
  policy_observation_.header.frame_id = "odom";
  policy_observation_.imu.header.stamp = policy_observation_.header.stamp;
  policy_observation_.imu.header.frame_id = "base_link";
  policy_observation_.joints.header = policy_observation_.header;
  policyMux_.fillObservationPolicyCommand(policy_observation_);
  if ((wbc_preposition_active_.load() || (motion_head_hold_active_ && motion_upper_snapshot_valid_)) &&
      policy_observation_.source_policy == robot_interfaces::msg::PolicyObservation::SOURCE_AMP &&
      policy_observation_.source_command.target_positions.size() == 21)
  {
    // WBC history includes the composed upper targets, including held head and natural arm-swing return.
    std::copy(motion_upper_positions_.begin(), motion_upper_positions_.end(),
              policy_observation_.source_command.target_positions.begin() + 12);
  }
  pubState_->publish(policy_observation_);
}

void LeggedController::stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Time& time) {
  if (allow_auxiliary_overlays && !motion_reserved_.load() && headCtrl_.isTopicEnabled())
  {
    headCtrl_.stepHeadOutput(time.seconds());
  }
}

void LeggedController::maybePublishVisualization(const rclcpp::Time& time) {
  if ((time - lastVizTick_) >= vizPeriod_)
  {
    lastVizTick_ = time;
    rvizVisualization();
  }
}

void LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& /* period */) {
  if (!fsm_ || !robot_hw_)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "LeggedController::update called before init completed (fsm/robot_hw not ready)");
    return;
  }

  last_controller_wall_sec_ = time.seconds();
  prepareControllerClock(time);
  RuntimeControlState runtime = computeRuntimeControlState();
  if (!runtime.policy_drives_legs && policyMux_.source() != PolicyCmdSource::kRl)
  {
    abortWbcImmediately("[WBC] Leaving Walking mode");
    policyMux_.setSource(PolicyCmdSource::kRl, "[PolicyMux] Leaving walk -> default RL source");
  }
  policy_observation_.inference_enabled = runtime.rl_inference_enabled;
  updatePolicyObservationFromHardware();
  runSafetyAndEmergencyFeedback(time, runtime.policy_drives_legs);
  runtime = computeRuntimeControlState();
  if (runtime.emergency_control)
  {
    policy_observation_.inference_enabled = false;
  }
  updateMotionPreparation();
  publishPolicyObservationWithJoystick();
  updateWbcControlState();
  runtime = computeRuntimeControlState();
  const bool handoff_active = policy_handoff_.active() && !policy_handoff_.expired(time.seconds());
  syncFaceFollowState(runtime.allow_auxiliary_overlays && !handoff_active, time.seconds());
  stepAuxiliaryMotionSources(runtime.allow_auxiliary_overlays, time);
  applyJointCommands(runtime.policy_drives_legs, runtime.allow_auxiliary_overlays, runtime.emergency_control);
  maybePublishVisualization(time);
}

bool LeggedController::holdPolicyCommand(const PolicyInputs& inputs) {
  if (!fsm_->isWalking()) return false;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    text_capture_stamp_ns_ = 0;
  }
  if (jointCommandComposer_.holdLastCommand(robot_hw_.get()))
  {
    holding_policy_command_ = true;
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
        "[Policy Recovery] Holding last validated q/dq/kp/kd/tau until fresh output; no policy hold timeout");
    captureControllerStatus(true, inputs);
    return true;
  }
  holding_policy_command_ = false;
  RCLCPP_ERROR(nh_->get_logger(), "[Policy Recovery] No validated joint command exists to hold; entering emergency damping");
  triggerEmergencyStop(NoValidCommand);
  return false;
}

void LeggedController::applyJointCommands(bool walking, bool allow_auxiliary_overlays, bool emergency_control) {
  PolicyInputs policy_inputs = policyMux_.selectInputs(walking);
  {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    if (wbc_preposition_active_.load() && walking && policy_inputs.source == PolicyCmdSource::kRl &&
        policy_inputs.q_snapshot.size() == 21 && wbc_preposition_positions_.size() == 21)
      std::copy(wbc_preposition_positions_.begin() + 12, wbc_preposition_positions_.end(),
                policy_inputs.q_snapshot.begin() + 12);
  }
  if (walking && !policy_inputs.valid && !emergency_control)
  {
    if (holdPolicyCommand(policy_inputs)) return;
    emergency_control = true;
  }
  const bool arm_swing_allowed = allow_auxiliary_overlays && !emergency_control;
  armSwingCtrl_.update(last_controller_wall_sec_, policy_observation_.velocity_command.linear.x, arm_swing_allowed,
                       policy_inputs.q_snapshot);

  JointCommandComposer::Context ctx;
  ctx.robot_hw = robot_hw_.get();
  ctx.head_ctrl = motion_head_pause_applied_ ? nullptr : &headCtrl_;
  ctx.arm_swing_ctrl = &armSwingCtrl_;
  ctx.release_handoff = &policy_handoff_;
  ctx.policy_inputs = policy_inputs;
  ctx.blend_policy_gains = wbc_handoff_in_progress_.load() || wbc_preposition_active_.load();
  ctx.rl_gains = policyMux_.rlGainsSnapshot();
  ctx.motor_states = emergency_control ? &robot_hw_->getHyridJointMotorState() : nullptr;
  const bool holding_head = motion_head_hold_active_ && walking && policy_inputs.source == PolicyCmdSource::kRl &&
                            !wbc_preposition_active_.load();
  ctx.head_hold_positions = holding_head ? &motion_head_positions_ : nullptr;
  ctx.upper_position_snapshot = (wbc_preposition_active_.load() || holding_head) ? &motion_upper_positions_ : nullptr;
  ctx.walking = walking;
  ctx.allow_auxiliary_overlays = allow_auxiliary_overlays;
  ctx.emergency = emergency_control;
  ctx.is_sim = isSim_;
  ctx.kp_standby = kp_standby_;
  ctx.emergency_damping_kd = emergency_damping_kd_;
  ctx.wall_sec = last_controller_wall_sec_;
  if (!jointCommandComposer_.composeAll(ctx))
  {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                         "[Policy Recovery] Rejected incomplete/non-finite joint command frame or invalid gains");
    policy_inputs.valid = false;
    if (walking && !emergency_control)
    {
      if (policy_inputs.source == PolicyCmdSource::kWbc)
      {
        std::string message;
        requestWbcReturn("[WBC] Invalid composed command; requesting AMP", message, true);
      }
      if (holdPolicyCommand(policy_inputs)) return;
    }
    ctx.emergency = true;
    holding_policy_command_ = false;
    ctx.motor_states = &robot_hw_->getHyridJointMotorState();
    jointCommandComposer_.composeAll(ctx);
    captureControllerStatus(walking, policy_inputs);
    return;
  }
  if (holding_policy_command_ && walking && !emergency_control)
    RCLCPP_INFO(nh_->get_logger(), "[Policy Recovery] Fresh policy output applied; no reset required");
  holding_policy_command_ = false;
  if (ctx.upper_position_snapshot != nullptr) motion_upper_snapshot_valid_ = true;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (motion_reserved_.load() && motion_source_ == "txt2motion" && !motion_head_pending_ &&
        !motion_cancelled_.load() && holding_head &&
        !emergency_control && !armSwingCtrl_.outputActive() && policy_handoff_.expired(last_controller_wall_sec_))
    {
      if (text_capture_stamp_ns_ == 0)
      {
        // The gateway must use an observation newer than this first confirmed held-output cycle.
        text_capture_stamp_ns_ = rclcpp::Time(policy_observation_.header.stamp).nanoseconds();
        RCLCPP_DEBUG(nh_->get_logger(), "[Motion] Text start capture ready for %s; head held and arm swing idle",
                    motion_request_id_.c_str());
      }
    }
    else text_capture_stamp_ns_ = 0;
  }
  captureControllerStatus(walking, policy_inputs);
}

void LeggedController::checkSafety() {
  if (isSim_ || !safetyChecker_)
  {
    return;
  }

  const auto orientation = safetyChecker_->orientationState();
  if (orientation.consecutive_violations == 0) return;

  RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 500,
                       "[SafetyChecker] Orientation violation %d/%d IMU frames: roll=%.1f deg, pitch=%.1f deg",
                       orientation.consecutive_violations, safetyChecker_->requiredConsecutiveOrientationViolations(),
                       radiansToDegrees(orientation.roll), radiansToDegrees(orientation.pitch));

  if (orientation.consecutive_violations >= safetyChecker_->requiredConsecutiveOrientationViolations() &&
      !fsm_->isEmergency())
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "[SafetyChecker] Orientation violation persisted for %d consecutive IMU frames "
                 "(roll=%.1f deg, pitch=%.1f deg)",
                 orientation.consecutive_violations, radiansToDegrees(orientation.roll),
                 radiansToDegrees(orientation.pitch));
    triggerEmergencyStop(OrientationStop);
  }
}

void LeggedController::updatePolicyObservationFromHardware() {
  const auto bodyPosition = robot_hw_->getBodyPosition();
  const auto bodyOrientation = robot_hw_->getBodyOrientation();

  policy_observation_.base_pose.position.x = bodyPosition[0];
  policy_observation_.base_pose.position.y = bodyPosition[1];
  policy_observation_.base_pose.position.z = bodyPosition[2];
  policy_observation_.base_pose.orientation.x = bodyOrientation[0];
  policy_observation_.base_pose.orientation.y = bodyOrientation[1];
  policy_observation_.base_pose.orientation.z = bodyOrientation[2];
  policy_observation_.base_pose.orientation.w = bodyOrientation[3];

  const auto& motor_states = robot_hw_->getHyridJointMotorState();
  const size_t state_count = std::min(motor_states.size(), policy_observation_.joints.position.size());
  for (size_t i = 0; i < state_count; ++i)
  {
    policy_observation_.joints.position[i] = motor_states[i].q;
    policy_observation_.joints.velocity[i] = motor_states[i].dq;
    policy_observation_.joints.effort[i] = motor_states[i].tau;
  }

  const auto imu_quaternion = robot_hw_->getImuQuaternion();
  const auto imu_angular_velocity = robot_hw_->getImuAngularVelocity();

  policy_observation_.imu.orientation.x = imu_quaternion[0];
  policy_observation_.imu.orientation.y = imu_quaternion[1];
  policy_observation_.imu.orientation.z = imu_quaternion[2];
  policy_observation_.imu.orientation.w = imu_quaternion[3];
  policy_observation_.imu.angular_velocity.x = imu_angular_velocity[0];
  policy_observation_.imu.angular_velocity.y = imu_angular_velocity[1];
  policy_observation_.imu.angular_velocity.z = imu_angular_velocity[2];
  policy_observation_.motion_time = shifted_time;
}

void LeggedController::rvizVisualization() {
  rvizVisualPos_.header.stamp = rclcpp::Clock().now();
  const auto& motor_states = robot_hw_->getHyridJointMotorState();
  for (size_t i = 0; i < motor_states.size(); ++i)
  {
    rvizVisualPos_.position[i] = motor_states[i].q;
    rvizVisualPos_.velocity[i] = motor_states[i].dq;
    rvizVisualPos_.effort[i] = motor_states[i].tau;
  }
  vizPub_->publish(rvizVisualPos_);
}

}  // namespace legged
