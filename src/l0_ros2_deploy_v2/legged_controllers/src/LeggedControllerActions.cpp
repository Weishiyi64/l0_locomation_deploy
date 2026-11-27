#include <algorithm>
#include <cstddef>
#include <mutex>
#include <string>

#include "LeggedController.h"

namespace legged {
namespace {

constexpr std::size_t kBoxInnerWidth = 42;
constexpr const char* kBoxBorder = "+------------------------------------------+";

void logBlank(rclcpp::Logger logger) { RCLCPP_INFO(logger, " "); }

void logBorder(rclcpp::Logger logger) { RCLCPP_INFO(logger, "%s", kBoxBorder); }

void logBoxLine(rclcpp::Logger logger, const std::string& text = std::string(), bool centered = true) {
  std::string content = text;
  if (content.size() > kBoxInnerWidth)
  {
    content.resize(kBoxInnerWidth);
  }

  std::size_t left = 0;
  if (centered)
  {
    left = (kBoxInnerWidth - content.size()) / 2;
  }
  else
  {
    left = content.empty() ? (kBoxInnerWidth / 2) : 2;
    if (left + content.size() > kBoxInnerWidth) left = 0;
  }
  const std::size_t right = kBoxInnerWidth - content.size() - left;
  const std::string left_pad(left, ' ');
  const std::string right_pad(right, ' ');
  RCLCPP_INFO(logger, "|%s%s%s|", left_pad.c_str(), content.c_str(), right_pad.c_str());
}

void printCalibrationBanner(rclcpp::Logger logger, const std::string& banner) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, banner);
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printEmergencyStopBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "EMERGENCY STOP ACTIVATED");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printWalkModeBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "WALK MODE ACTIVATED");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printCalibrationEnabledBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CALIBRATION MODE ENABLED");
  logBoxLine(logger);
  logBoxLine(logger, "Use controller buttons to calibrate:", false);
  logBoxLine(logger, "- LB+LT: Left Arm", false);
  logBoxLine(logger, "- RB+RT: Right Arm", false);
  logBoxLine(logger, "- A+X: Left Leg", false);
  logBoxLine(logger, "- B+Y: Right Leg", false);
  logBoxLine(logger, "- BACK: Waist (if configured)", false);
  logBoxLine(logger, "- START: Neck", false);
  logBoxLine(logger, "Press LS+RS again to exit calibration", false);
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printCalibrationDisabledBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CALIBRATION MODE DISABLED");
  logBoxLine(logger);
  logBoxLine(logger, "Returning to Emergency Stop mode");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void logResetBanner(rclcpp::Logger logger) {
  RCLCPP_INFO(logger, " ");
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CONTROLLER RESET");
  logBoxLine(logger);
  logBoxLine(logger, "Emergency cleared - System ready");
  logBoxLine(logger, "Policy state reset for walk");
  logBoxLine(logger);
  logBorder(logger);
  RCLCPP_INFO(logger, " ");
}

}  // namespace

void LeggedController::processJoystickActions(const JoystickCommand& command) {
  if (command.emergency_stop_pressed) triggerEmergencyStop(ManualStop);
  if (command.set_walk_pressed) enterWalkMode();
  if (command.reset_pressed) resetController();
  if (command.kp_add_pressed) adjustStandbyKp(2.0);
  if (command.kp_minus_pressed) adjustStandbyKp(-2.0);
  if (command.toggle_calibration_pressed) toggleCalibrationMode();
  if (command.calibrate_left_arm_pressed) calibrateByTag("left_arm", "CALIBRATING LEFT ARM");
  if (command.calibrate_right_arm_pressed) calibrateByTag("right_arm", "CALIBRATING RIGHT ARM");
  if (command.calibrate_left_leg_pressed) calibrateByTag("left_leg", "CALIBRATING LEFT LEG");
  if (command.calibrate_right_leg_pressed) calibrateByTag("right_leg", "CALIBRATING RIGHT LEG");
  if (command.calibrate_waist_pressed) calibrateByTag("waist", "CALIBRATING WAIST");
  if (command.calibrate_neck_pressed) calibrateByTag("neck", "CALIBRATING NECK");
}

void LeggedController::updateSpeedMultiplier(double lt_axis_value) {
  if (velLimiter_)
  {
    velLimiter_->updateSpeedMultiplierFromAxis(lt_axis_value);
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "[SPEED MULTIPLIER] Velocity limiter not initialized");
  }
}

void LeggedController::triggerEmergencyStop(StopReason reason) {
  if (fsm_->isCalibration())
  {
    RCLCPP_WARN(nh_->get_logger(), "[EMERGENCY] Ignored while hardware calibration mode is active");
    return;
  }

  stop_reasons_.fetch_or(static_cast<uint32_t>(reason));
  policy_handoff_.clear();
  policyMux_.publishImitationActive(false, "[EMERGENCY] Deactivate imitation worker");
  abortWbcImmediately("[EMERGENCY] Deactivate WBC worker");
  policyMux_.setSource(policyMux_.walkStartPolicySource(), "[EMERGENCY] Policy source restored to default");

  fsm_->emergencyStop();

  kp_standby_ = 0;
  printEmergencyStopBanner(nh_->get_logger());
  RCLCPP_WARN(nh_->get_logger(), "[EMERGENCY] System stopped for safety!");
}

void LeggedController::enterWalkMode() {
  if (!fsm_->isStandby()) return;

  policy_handoff_.clear();
  fsm_->setWalk();
  startingTime_ = rclcpp::Clock().now();
  printWalkModeBanner(nh_->get_logger());
  RCLCPP_INFO(nh_->get_logger(), "[WALK] Robot entering walk mode - All systems go!");
}

void LeggedController::resetController() {
  if (!fsm_->isEmergency()) return;

  fsm_->resetFromEmergency();
  stop_reasons_.store(0);
  kp_standby_ = 0;
  if (safetyChecker_) safetyChecker_->setOrientationMonitoring(false);
  policy_handoff_.clear();
  policyMux_.publishImitationActive(false, "[RESET] Deactivate imitation worker");
  abortWbcImmediately("[RESET] Deactivate WBC worker");
  policyMux_.clearCommands(0.0);
  RCLCPP_INFO(nh_->get_logger(), "[RESET] Standby targets cleared to zero pose");
  policyMux_.setSource(policyMux_.walkStartPolicySource(), "[RESET] Policy source restored to startup default");
  policyMux_.publishRlObsReset();

  logResetBanner(nh_->get_logger());

  joyFeedbackArray_.array[0].intensity = 0.0;
  joyFeedbackArray_.array[1].intensity = 0.0;
  joyFeedbackPub_->publish(joyFeedbackArray_);
  RCLCPP_INFO(nh_->get_logger(), "[RESET] Controller reset - Ready for operation!");
}

void LeggedController::adjustStandbyKp(double delta) {
  if (fsm_->isStandby())
  {
    kp_standby_ += delta;
    RCLCPP_INFO(nh_->get_logger(), "[KP] Standby gain %s: %.1f", delta >= 0.0 ? "increased" : "decreased", kp_standby_);
    if (kp_standby_ > 80)
    {
      kp_standby_ = 80;
      RCLCPP_INFO(nh_->get_logger(), "[KP] Limit reached - max: 80");
    }
    else if (kp_standby_ < 0)
    {
      kp_standby_ = 0;
      RCLCPP_INFO(nh_->get_logger(), "[KP] Limit reached - min: 0");
    }
  }
}

void LeggedController::toggleCalibrationMode() {
  if (isSim_)
  {
    RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Calibration mode is not supported in simulation");
    return;
  }

  if (fsm_->isEmergency())
  {
    if (!robot_hw_->set_calibration_mode(true))
    {
      RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Failed to enable hardware calibration mode");
      return;
    }
    fsm_->enterCalibrationMode();
    printCalibrationEnabledBanner(nh_->get_logger());
    RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Calibration mode activated");
  }
  else if (fsm_->isCalibration())
  {
    if (!robot_hw_->set_calibration_mode(false))
    {
      RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Failed to disable hardware calibration mode");
      return;
    }
    fsm_->exitCalibrationMode();
    printCalibrationDisabledBanner(nh_->get_logger());
    RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Calibration mode deactivated, back to Emergency Stop");
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Can only enter calibration mode from Emergency Stop state");
  }
}

void LeggedController::calibrateByTag(const std::string& tag, const std::string& banner) {
  if (!fsm_->isCalibration()) return;

  printCalibrationBanner(nh_->get_logger(), banner);
  if (robot_hw_->zeroMotorsByTag(tag))
  {
    RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] %s request accepted; checking motor position feedback", tag.c_str());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] %s calibration failed", tag.c_str());
  }
}

bool LeggedController::imitationActionActive() const {
  return motion_reserved_.load() || policyMux_.source() != PolicyCmdSource::kRl || policyMux_.imitationActiveCommanded() ||
         policyMux_.wbcActiveCommanded() || wbc_return_pending_.load() || wbc_handoff_in_progress_.load();
}

bool LeggedController::canStartAmpDerivedAction(const char* context, const std::string& action_label,
                                                bool require_walking, bool block_imitation_active) const {
  const char* tag = context != nullptr ? context : "[ActionTrigger]";
  const std::string label = action_label.empty() ? "unnamed" : action_label;

  if (require_walking && (!fsm_ || !fsm_->isWalking()))
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': robot must be in Walking mode", tag, label.c_str());
    return false;
  }
  if (fsm_ && (fsm_->isEmergency() || fsm_->isCalibration()))
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': controller is not in a safe state", tag, label.c_str());
    return false;
  }
  if (block_imitation_active && imitationActionActive())
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': another policy action or WBC handoff is already active",
                tag, label.c_str());
    return false;
  }
  return true;
}

bool LeggedController::startImitationAction(const std::string& policy) {
  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!canStartAmpDerivedAction("[Imitation]", policy, true, true))
  {
    return false;
  }

  if (!policyMux_.switchImitationPolicy(policy, "[Imitation]"))
  {
    return false;
  }

  policyMux_.setSource(PolicyCmdSource::kImitation);
  policyMux_.publishImitationReset();
  policyMux_.publishImitationActive(true);
  applyJoystickMotion(lastJoystickCommand_);
  return true;
}

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

      JointCommandTransition& handoff = policy_handoff_;
      handoff.clear();
      if (handoff.start(joint_mask, start_cmd, wall_sec, imitation_return_blend_sec_))
      {
        RCLCPP_INFO(nh_->get_logger(), "[PolicyMux] Imitation -> RL handoff blend %.3fs", imitation_return_blend_sec_);
      }
    }
  }

  policyMux_.returnToRlWithoutGainRamp(reason);
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
    RCLCPP_DEBUG_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "Planner velocity command has not been received; commanding zero velocity");
    return geometry_msgs::msg::Twist();
  }

  if (!plannerCommandValid_)
  {
    RCLCPP_DEBUG_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "Planner velocity command is invalid; commanding zero velocity");
    return geometry_msgs::msg::Twist();
  }

  const bool fresh =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - plannerCommandReceiveTime_).count() <= 0.5;
  if (fresh)
  {
    return plannerCommand_;
  }

  RCLCPP_DEBUG_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                        "Planner velocity command is stale; commanding zero velocity");
  return geometry_msgs::msg::Twist();
}

void LeggedController::applyJoystickMotion(const JoystickCommand& command) {
  policy_observation_.velocity_command = selectedOperatorVelocity(computeJoystickTrim(command));
}

}  // namespace legged
