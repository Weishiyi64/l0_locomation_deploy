#include <mutex>

#include "legged_controllers/ControllerBanners.h"
#include "legged_controllers/LeggedController.h"

namespace legged {

void LeggedController::processJoystickActions(const JoystickCommand& command) {
  if (command.emergency_stop_pressed) triggerEmergencyStop();
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

void LeggedController::processJoystickBackEmergencySequence(const JoystickCommand& command) {
  if (!joystick_back_emergency_sequence_enabled_ || !command.back_emergency_sequence_pressed) return;
  if (!fsm_ || !fsm_->isWalking()) return;

  const std::string action_label = "Back emergency sequence";
  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!canStartAmpDerivedAction("[BackEmergency]", action_label, true, true))
  {
    return;
  }

  if (joystick_back_emergency_sequence_path_.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[BackEmergency] Ignoring trigger: sequence path is empty");
    return;
  }

  if (!fullCsvVelocityAllowed())
  {
    logFullCsvVelocityBlocked("[BackEmergency]", joystick_back_emergency_sequence_path_);
    return;
  }

  const auto log_trigger = [this]() {
    RCLCPP_WARN(nh_->get_logger(), "[BackEmergency] Back button -> %s; EmergencyStop after finish",
                displayControllerPath(joystick_back_emergency_sequence_path_).c_str());
  };
  if (triggerSequenceFromPath(joystick_back_emergency_sequence_path_, "[BackEmergency] Trigger", log_trigger))
  {
    sequence_finish_emergency_pending_ = true;
  }
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

void LeggedController::triggerEmergencyStop() {
  if (fsm_->isCalibration())
  {
    RCLCPP_WARN(nh_->get_logger(), "[EMERGENCY] Ignored while hardware calibration mode is active");
    return;
  }

  const bool had_active_sequence = sequenceRuntime_.isActive();
  sequence_finish_emergency_pending_ = false;
  sequenceRuntime_.resetForAbort();
  if (had_active_sequence)
  {
    RCLCPP_WARN(nh_->get_logger(), "[EMERGENCY] Active action sequence aborted");
  }
  publishImitationActive(false, "[EMERGENCY] Deactivate imitation worker");
  setPolicySource(walkStartPolicySource(), "[EMERGENCY] Policy source restored to default");

  if (!fsm_->isEmergency()) fsm_->emergencyStop();

  kp_standby_ = 0;
  printEmergencyStopBanner(nh_->get_logger());
  RCLCPP_WARN(nh_->get_logger(), "[EMERGENCY] System stopped for safety!");
}

void LeggedController::enterWalkMode() {
  if (!fsm_->isStandby()) return;

  clearSequenceReleaseHandoff();
  if (sequenceRuntime_.softReleaseLatched())
  {
    sequenceRuntime_.clearSoftReleaseLatch();
    returnToRlWithoutGainRamp("[SoftRelease] Latch cleared on set_walk: restore RL leg control");
    return;
  }
  fsm_->setWalk();
  startingTime_ = rclcpp::Clock().now();
  printWalkModeBanner(nh_->get_logger());
  RCLCPP_INFO(nh_->get_logger(), "[WALK] Robot entering walk mode - All systems go!");
}

void LeggedController::resetController() {
  if (!fsm_->isEmergency()) return;

  fsm_->resetFromEmergency();
  kp_standby_ = 0;
  orientation_violation_count_ = 0;
  sequenceRuntime_.resetForAbort();
  publishImitationActive(false, "[RESET] Deactivate imitation worker");
  policyMux_.clearCommands(0.0);
  RCLCPP_INFO(nh_->get_logger(), "[RESET] Standby targets cleared to zero pose");
  setPolicySource(walkStartPolicySource(), "[RESET] Policy source restored to startup default");
  publishRlObsReset();

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
    if (robot_hw_->resetAllMotors())
    {
      RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] All motors reset successfully");
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Motor reset failed");
    }

    if (robot_hw_->enableAllMotors())
    {
      RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] All motors enabled successfully");
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] Motor enable failed");
    }

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
    RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] %s calibrated successfully", tag.c_str());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] %s calibration failed", tag.c_str());
  }
}

bool LeggedController::imitationActionActive() const {
  return policyMux_.source() == PolicyCmdSource::kImitation || policyMux_.imitationActiveCommanded();
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
  if (sequenceRuntime_.isActive())
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': action sequence is already active", tag, label.c_str());
    return false;
  }
  if (sequenceRuntime_.softReleaseLatched())
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': soft-release latch is active", tag, label.c_str());
    return false;
  }
  if (block_imitation_active && imitationActionActive())
  {
    RCLCPP_WARN(nh_->get_logger(), "%s Ignoring action '%s': imitation is already active", tag, label.c_str());
    return false;
  }
  return true;
}

}  // namespace legged
