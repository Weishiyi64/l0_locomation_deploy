//
// Base class for legged controllers: shared FSM, policy mux, safety, head control, and action sequences.
//
#pragma once

#include <legged_controllers/ActionSequence.h>
#include <legged_controllers/ArmSwingController.h>
#include <legged_controllers/ControllerTypes.h>
#include <legged_controllers/HeadScript.h>
#include <legged_controllers/HeadTopic.h>
#include <legged_controllers/IHardwareLayer.h>
#include <legged_controllers/JointCommandComposer.h>
#include <legged_controllers/JoystickImitationActions.h>
#include <legged_controllers/JoystickSequenceActions.h>
#include <legged_controllers/PolicyMux.h>
#include <legged_controllers/SequenceRuntime.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "legged_controllers/JoystickInput.h"
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/StateMachine.h"
#include "legged_controllers/VelocityLimiter.h"
#include "legged_controllers/VisualizationPublisher.h"
#include "legged_msgs/msg/low_cmd.hpp"
#include "legged_msgs/msg/low_state.hpp"
#include "legged_msgs/msg/pos_cmd.hpp"
#include "legged_msgs/srv/switch_imitation_policy.hpp"

namespace legged {

class LeggedController {
 private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<IHardwareLayer> robot_hw_;

 public:
  LeggedController(rclcpp::Node::SharedPtr controller_nh, std::shared_ptr<IHardwareLayer> robot_hw)
      : nh_(controller_nh),
        robot_hw_(robot_hw),
        vizPeriod_(rclcpp::Duration(0, 0)),
        joyFeedbackPeriod_(rclcpp::Duration(0, 0)),
        shifted_time(rclcpp::Duration(0, 0)) {}
  virtual ~LeggedController() = default;

  virtual bool init(rclcpp::Node::SharedPtr controller_nh);
  virtual void starting(const rclcpp::Time& time);
  virtual void update(const rclcpp::Time& time, const rclcpp::Duration& period);

 protected:
  // Hooks for derived specialization
  virtual void applyJointCommands(bool walking, bool allow_auxiliary_overlays, bool emergency);

  virtual void ModePublish(rclcpp::Node::SharedPtr controller_nh);
  virtual void checkSafety();  // Default uses SafetyChecker
  /// Refresh lowState_ from robot_hw_ (joint/IMU/body) for safety and /rlState.
  virtual void updateLowStateFromHardware();
  virtual void rvizVisualization();

  void ModeSubscribe(rclcpp::Node::SharedPtr controller_nh);

  void EmergencyStopCallback(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void lowCmdRlCallback(const legged_msgs::msg::PosCmd::ConstSharedPtr msg);
  void lowCmdImitationCallback(const legged_msgs::msg::PosCmd::ConstSharedPtr msg);
  void imitationMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void textMotionReadyCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void keyboardActionTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  void headRpyCallback(const geometry_msgs::msg::Point::ConstSharedPtr msg);

  geometry_msgs::msg::Twist computeJoystickTrim(const JoystickCommand& command);
  geometry_msgs::msg::Twist computeCommandVelocity(const JoystickCommand& command, double wall_sec);
  void applyJoystickMotion(const JoystickCommand& command, double wall_sec);
  void processJoystickActions(const JoystickCommand& command);
  void processJoystickBackEmergencySequence(const JoystickCommand& command);
  void processJoystickImitationActions(const sensor_msgs::msg::Joy& joy);
  void processJoystickSequenceActions(const sensor_msgs::msg::Joy& joy);
  void enterWalkMode();
  virtual void resetController();
  void triggerEmergencyStop();
  void updateSpeedMultiplier(double lt_axis_value);
  void adjustStandbyKp(double delta);
  void toggleCalibrationMode();
  void calibrateByTag(const std::string& tag, const std::string& banner);
  bool imitationActionActive() const;
  bool canStartAmpDerivedAction(const char* context, const std::string& action_label, bool require_walking,
                                bool block_imitation_active) const;
  void requestKeyboardAction(const std::string& action_name);
  bool startActionSequence();
  void stopActionSequence(const char* reason, double wall_sec);
  void finishActionSequenceNormally(double wall_sec);
  void configureActionSequencePhase(const ActionSequencePhase& phase, double wall_sec);
  void handleActionSequenceUpdate(const rclcpp::Time& time);
  bool loadActionSequenceForPlayback(const std::string& path, const char* context);
  bool loadEmotionSequenceMap(const std::string& path);
  bool loadKeyboardActionMap(const std::string& path);
  bool triggerSequenceFromPath(const std::string& sequence_path, const char* context,
                               const std::function<void()>& after_start = nullptr);
  std::string displayControllerPath(const std::string& path) const;
  bool prepareImitationPolicyForLoadedSequence(const char* context);
  bool fullCsvVelocityAllowed() const;
  void logFullCsvVelocityBlocked(const char* context, const std::string& target) const;
  bool switchImitationPolicy(const std::string& policy_name, const char* context);
  bool loadImitationWalkingGains(const std::string& robot_cfg_path, const std::string& policy_name);
  void publishImitationActive(bool active, const char* reason = nullptr);
  std::pair<double, double> readCurrentHeadYawPitchRad();
  bool headTopicIntegrationAllowed(double wall_sec) const;
  void syncHeadTopicIntegrationState(bool allowed, double wall_sec);

  void dataInit(const std::vector<std::string>& joint_names);
  bool loadWalkingGainProfile(const YAML::Node& rc, const std::string& profile_name, size_t expected_size,
                              JointGainProfile& out) const;
  void setPolicySource(PolicyCmdSource source, const char* reason = nullptr);
  PolicyCmdSource walkStartPolicySource() const;
  void selectPolicyInputs(bool walking, std::vector<double>& q_snapshot, JointGainProfile& active_gains) const;
  void publishRlObsReset();
  /// Switch policy mux to RL and reset RL observation history (no kp/kd ramp).
  void returnToRlWithoutGainRamp(const char* reason);
  void beginSequenceReleaseHandoff(double wall_sec);
  void clearSequenceReleaseHandoff();

 protected:
  rclcpp::Publisher<legged_msgs::msg::LowState>::SharedPtr pubState_;  // LowState publish
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joyFeedbackPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keyboardActionAudioPub_;
  rclcpp::CallbackGroup::SharedPtr sequence_trigger_callback_group_;

  rclcpp::Subscription<legged_msgs::msg::PosCmd>::SharedPtr subCmdRl_;
  rclcpp::Subscription<legged_msgs::msg::PosCmd>::SharedPtr subCmdImitation_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subEmgstop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subHeadRpy_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subHeadJoystick_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationMotionDone_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subKeyboardActionTrigger_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subTextMotionReady_;

  // State containers
  legged_msgs::msg::LowState lowState_;
  legged_msgs::msg::LowCmd lowCmd_;
  sensor_msgs::msg::JointState rvizVisualPos_;
  std::vector<std::string> jointNames_;
  /// True for leg joints (name contains hip/knee/ankle). Used for sequence end soft-release.
  std::vector<bool> is_leg_joint_;
  /// True for arm joints (name contains shoulder/elbow/wrist). Used for sequence end soft-release.
  std::vector<bool> is_arm_joint_;
  /// True for head joints (name contains neck/head). Used for sequence end head hold.
  std::vector<bool> is_head_joint_;

  // Motion source components
  HeadScriptController headCtrl_;
  HeadTopicController headTopicCtrl_;
  ArmSwingController armSwingCtrl_;
  PolicyMux policyMux_;
  SequenceRuntime sequenceRuntime_;
  JointCommandComposer jointCommandComposer_;

  rclcpp::Time startingTime_;
  rclcpp::Time lastVizTick_;
  rclcpp::Duration vizPeriod_;
  rclcpp::Time lastJoyFeedbackTick_;
  rclcpp::Duration joyFeedbackPeriod_;
  rclcpp::Duration shifted_time;  // controller time since startingTime_

  bool joyFeedbackState_{false};

  int policy_frequency{0};

  // Standby global kp (clamped per joint)
  double kp_standby_{0};
  bool sequence_engine_enabled_{false};
  bool keyboard_action_enabled_{false};
  bool joystick_imitation_enabled_{false};
  bool joystick_sequence_enabled_{false};
  bool joystick_back_emergency_sequence_enabled_{false};
  bool text_motion_enabled_{true};
  std::string keyboard_action_config_path_;
  std::string keyboard_action_trigger_topic_{"/keyboard_action_trigger"};
  std::string joystick_imitation_config_path_;
  std::string joystick_sequence_config_path_;
  std::string joystick_back_emergency_sequence_path_;
  std::string text_motion_ready_topic_{"/text_motion/ready"};
  std::string legged_controllers_share_;

  /// Last controller `time.seconds()` from `update()`; used for async callbacks (e.g. /imitation_motion_done).
  double last_controller_wall_sec_{0.0};
  std::atomic<bool> head_topic_integration_allowed_{false};

  double imitation_policy_switch_timeout_sec_{120.0};
  // Smooth AMP/RL -> imitation policy entry.  Set to 0.0 to retain the
  // historical direct policy switch behaviour.
  double imitation_entry_blend_sec_{0.8};
  // Text WBC keeps a direct motion reference, but briefly hands off the
  // outgoing AMP command/gains to the first live WBC command without blocking.
  double text_motion_entry_blend_sec_{0.2};
  double imitation_return_blend_sec_{0.35};
  double emergency_damping_kd_{5.0};
  std::atomic<JoystickImitationFinishMode> active_joystick_imitation_finish_mode_{
      JoystickImitationFinishMode::kReturnRl};
  bool sequence_finish_emergency_pending_{false};
  std::mutex auxiliary_action_trigger_mutex_;


  // Feedback rumble array
  // sensor_msgs::JoyFeedbackArray joyFeedbackArray_;
  sensor_msgs::msg::JoyFeedbackArray joyFeedbackArray_;

  // Components
  JoystickInput joystickInput_;
  JoystickImitationActions joystickImitationActions_;
  JoystickSequenceActions joystickSequenceActions_;
  JoystickCommand lastJoystickCommand_;
  std::unique_ptr<VelocityLimiter> velLimiter_;
  std::unique_ptr<VisualizationPublisher> vizPub_;
  std::unique_ptr<SafetyChecker> safetyChecker_;
  int orientation_violation_count_{0};
  std::unique_ptr<StateMachine> fsm_;

  // Flags (default false for real robot; set true in sim launch)
  bool isSim_{false};
  bool enableRviz_{false};

 private:
  void prepareControllerClock(const rclcpp::Time& time);
  RuntimeControlState computeRuntimeControlState() const;
  bool isActionSequenceFullCsv() const;
  bool advanceSequenceAndWalkingPolicy(const rclcpp::Time& time, const rclcpp::Duration& period);
  void runSafetyAndEmergencyFeedback(const rclcpp::Time& time, bool safety_armed);
  void publishLowStateWithJoystick(double wall_sec);
  void stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Duration& period,
                                  const rclcpp::Time& time);
  void maybePublishVisualization(const rclcpp::Time& time);
  void applyJointCsvPhaseTriggerBlendAndCmdVel(double wall_sec);
};

}  // namespace legged
