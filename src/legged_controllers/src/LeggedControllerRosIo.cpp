#include <functional>
#include <memory>
#include <utility>

#include "legged_controllers/ControllerFileUtils.h"
#include "legged_controllers/LeggedController.h"

namespace legged {

void LeggedController::ModeSubscribe(rclcpp::Node::SharedPtr /* controller_nh */) {
  if (!sequence_trigger_callback_group_)
  {
    sequence_trigger_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  subJoy_ = nh_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(10), std::bind(&LeggedController::JoyCallback, this, std::placeholders::_1));
  subEmgstop_ = nh_->create_subscription<std_msgs::msg::Float32>(
      "/emergency_stop", rclcpp::QoS(1),
      std::bind(&LeggedController::EmergencyStopCallback, this, std::placeholders::_1));
  subCmdRl_ = nh_->create_subscription<legged_msgs::msg::PosCmd>(
      "/rlCmd_rl", rclcpp::QoS(100), std::bind(&LeggedController::lowCmdRlCallback, this, std::placeholders::_1));
  subCmdImitation_ = nh_->create_subscription<legged_msgs::msg::PosCmd>(
      "/rlCmd_imitation", rclcpp::QoS(100),
      std::bind(&LeggedController::lowCmdImitationCallback, this, std::placeholders::_1));
  subHeadRpy_ = nh_->create_subscription<geometry_msgs::msg::Point>(
      "/eye_target", rclcpp::QoS(10), std::bind(&LeggedController::headRpyCallback, this, std::placeholders::_1));
  subHeadJoystick_ = nh_->create_subscription<geometry_msgs::msg::Point>(
      "/head_joystick", rclcpp::QoS(10), std::bind(&LeggedController::headRpyCallback, this, std::placeholders::_1));

  subImitationMotionDone_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "/imitation_motion_done", rclcpp::QoS(10),
      std::bind(&LeggedController::imitationMotionDoneCallback, this, std::placeholders::_1));

  if (text_motion_enabled_)
  {
    rclcpp::SubscriptionOptions text_motion_options;
    text_motion_options.callback_group = sequence_trigger_callback_group_;
    subTextMotionReady_ = nh_->create_subscription<std_msgs::msg::String>(
        text_motion_ready_topic_, rclcpp::QoS(1).reliable(),
        std::bind(&LeggedController::textMotionReadyCallback, this, std::placeholders::_1), text_motion_options);
  }

  if (keyboard_action_enabled_)
  {
    rclcpp::SubscriptionOptions sequence_options;
    sequence_options.callback_group = sequence_trigger_callback_group_;
    subKeyboardActionTrigger_ = nh_->create_subscription<std_msgs::msg::String>(
        keyboard_action_trigger_topic_, rclcpp::QoS(10),
        std::bind(&LeggedController::keyboardActionTriggerCallback, this, std::placeholders::_1), sequence_options);
  }
}

void LeggedController::ModePublish(rclcpp::Node::SharedPtr controller_nh) {
  pubState_ = nh_->create_publisher<legged_msgs::msg::LowState>("/rlState", rclcpp::QoS(100));

  joyFeedbackPub_ = nh_->create_publisher<sensor_msgs::msg::JoyFeedbackArray>("/joy/set_feedback", rclcpp::QoS(1));
  policyMux_.createRosInterfaces();

  if (keyboard_action_enabled_)
  {
    keyboardActionAudioPub_ =
        nh_->create_publisher<std_msgs::msg::String>(sequenceRuntime_.keyboardAudioTopic(), rclcpp::QoS(10));
  }

  if (enableRviz_)
  {
    RCLCPP_INFO(nh_->get_logger(),
                "RViz visualization enabled - creating publishers for "
                "/joint_states and /joint_rad_text");
    VisualizationPublisher::Params vp;
    vizPub_ = std::make_unique<VisualizationPublisher>(controller_nh, vp);
  }
  else
  {
    RCLCPP_INFO(nh_->get_logger(),
                "RViz visualization disabled - skipping /joint_states and "
                "/joint_rad_text publishers");
  }
}

void LeggedController::lowCmdRlCallback(const legged_msgs::msg::PosCmd::ConstSharedPtr msg) {
  policyMux_.setRlCommand(*msg);
}

void LeggedController::lowCmdImitationCallback(const legged_msgs::msg::PosCmd::ConstSharedPtr msg) {
  policyMux_.setImitationCommand(*msg);
}

void LeggedController::keyboardActionTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  const std::string action_name = trimCopy(msg->data);
  if (action_name.empty()) return;
  requestKeyboardAction(action_name);
}

void LeggedController::imitationMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data) return;

  if (sequence_engine_enabled_)
  {
    const ActiveSequencePhaseSnapshot snapshot = sequenceRuntime_.activePhaseSnapshot(last_controller_wall_sec_);
    if (snapshot.active && snapshot.phase.mode == ActionSequenceMode::kRlImitation)
    {
      const double t_wall = last_controller_wall_sec_;
      bool next_phase_is_imitation = false;
      sequenceRuntime_.completeImitationPhaseByExternalEvent(t_wall, next_phase_is_imitation);
      return;
    }
  }

  if (policyMux_.source() != PolicyCmdSource::kImitation) return;

  if (policyMux_.singleNodePublishesImitationPosCmd())
  {
    return;
  }

  const JoystickImitationFinishMode finish_mode =
      active_joystick_imitation_finish_mode_.exchange(JoystickImitationFinishMode::kReturnRl);
  if (finish_mode == JoystickImitationFinishMode::kEmergencyStop)
  {
    RCLCPP_WARN(nh_->get_logger(), "[JoystickImitation] Motion finished: entering EmergencyStop");
    triggerEmergencyStop();
    return;
  }
  if (finish_mode == JoystickImitationFinishMode::kReturnRlDirect)
  {
    // Text motion is deliberately a direct switch in both directions.  Clear
    // any handoff left by another action and bypass imitation_return_blend_sec_.
    sequenceRuntime_.releaseHandoff().clear();
    policyMux_.returnToRlWithoutGainRamp("[TextMotion] finished; switching directly to RL output");
    return;
  }

  returnToRlWithoutGainRamp(nullptr);
}

void LeggedController::textMotionReadyCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  const std::string policy = trimCopy(msg->data);
  if (!text_motion_enabled_ || policy.empty()) return;

  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!canStartAmpDerivedAction("[TextMotion]", policy, true, true)) return;
  if (!switchImitationPolicy(policy, "[TextMotion]")) return;

  // Keep the direct text reference (there are no AMP inner-arm frames), while
  // handing the outgoing AMP/RL command and gains to the first live WBC output
  // over a short non-blocking interval.  The target remains the WBC command at
  // every control tick; this transition never visits a fixed standing pose.
  JointCommandTransition& handoff = sequenceRuntime_.releaseHandoff();
  handoff.clear();
  if (text_motion_entry_blend_sec_ > 1e-9)
  {
    const PolicyInputs amp_inputs = policyMux_.selectInputs(fsm_ && fsm_->isWalking());
    if (!amp_inputs.q_snapshot.empty())
    {
      const size_t n = jointNames_.size();
      const size_t q_count = std::min(n, amp_inputs.q_snapshot.size());
      std::vector<bool> joint_mask(n, false);
      std::vector<JointCommandSample> start_cmd(n);
      for (size_t j = 0; j < q_count; ++j)
      {
        joint_mask[j] = true;
        start_cmd[j].q = amp_inputs.q_snapshot[j];
        start_cmd[j].dq = 0.0;
        start_cmd[j].kp = (j < amp_inputs.gains.kps.size()) ? amp_inputs.gains.kps[j] : 0.0;
        start_cmd[j].kd = (j < amp_inputs.gains.kds.size()) ? amp_inputs.gains.kds[j] : 0.0;
        start_cmd[j].tau_ff = 0.0;
      }
      double wall_sec = last_controller_wall_sec_;
      if (wall_sec <= 0.0 && nh_) wall_sec = nh_->get_clock()->now().seconds();
      if (handoff.start(joint_mask, start_cmd, wall_sec, text_motion_entry_blend_sec_))
      {
        RCLCPP_INFO(nh_->get_logger(),
                    "[TextMotion] AMP/RL -> WBC live handoff %.3fs (%zu joints, no stand pose)",
                    text_motion_entry_blend_sec_, q_count);
      }
    }
  }
  active_joystick_imitation_finish_mode_.store(JoystickImitationFinishMode::kReturnRlDirect);
  setPolicySource(PolicyCmdSource::kImitation,
                  "[TextMotion] complete trajectory ready; switching to WBC output with live handoff");
  policyMux_.publishImitationReset();
  publishImitationActive(true);
  applyJoystickMotion(lastJoystickCommand_, last_controller_wall_sec_);
}

std::pair<double, double> LeggedController::readCurrentHeadYawPitchRad() {
  double actual_yaw = 0.0, actual_pitch = 0.0;
  if (!robot_hw_) return {actual_yaw, actual_pitch};

  const auto motor_states = robot_hw_->getHyridJointMotorState();
  for (size_t i = 0; i < jointNames_.size() && i < motor_states.size(); ++i)
  {
    const std::string& joint_name = jointNames_[i];
    if (joint_name.find("neck_yaw") != std::string::npos || joint_name.find("head_yaw") != std::string::npos)
    {
      actual_yaw = motor_states[i].q;
    }
    else if (joint_name.find("neck_pitch") != std::string::npos || joint_name.find("head_pitch") != std::string::npos)
    {
      actual_pitch = motor_states[i].q;
    }
  }
  return {actual_yaw, actual_pitch};
}

void LeggedController::headRpyCallback(const geometry_msgs::msg::Point::ConstSharedPtr msg) {
  const double t_wall = rclcpp::Clock().now().seconds();
  const bool allow_integration = headTopicIntegrationAllowed(t_wall);
  auto read_head = [this]() -> std::pair<double, double> { return readCurrentHeadYawPitchRad(); };
  headTopicCtrl_.processEyeTarget(*msg, t_wall, headCtrl_, read_head, allow_integration);
}

void LeggedController::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  const JoystickCommand command = joystickInput_.decode(*msg);
  lastJoystickCommand_ = command;
  processJoystickActions(command);
  processJoystickBackEmergencySequence(command);
  processJoystickImitationActions(*msg);
  processJoystickSequenceActions(*msg);
}

void LeggedController::EmergencyStopCallback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
  if (msg->data >= 0.5f)
  {
    triggerEmergencyStop();
  }
}

}  // namespace legged
