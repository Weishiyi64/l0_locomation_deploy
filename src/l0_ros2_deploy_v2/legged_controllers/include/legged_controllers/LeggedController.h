//
// Base class for legged controllers: shared FSM, policy mux, safety, head control, and action sequences.
//
#pragma once

#include <legged_controllers/ActionSequence.h>
#include <legged_controllers/ArmSwingController.h>
#include <legged_controllers/ControllerTypes.h>
#include <legged_controllers/FaceFollow.h>
#include <legged_controllers/HeadScript.h>
#include <legged_controllers/IHardwareLayer.h>
#include <legged_controllers/JointCommandComposer.h>
#include <legged_controllers/JoystickImitationActions.h>
#include <legged_controllers/JoystickSequenceActions.h>
#include <legged_controllers/PolicyMux.h>
#include <legged_controllers/SequenceRuntime.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
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
#include "luvbot_control_interfaces/msg/policy_command.hpp"
#include "luvbot_control_interfaces/msg/policy_observation.hpp"
#include "luvbot_control_interfaces/srv/control_wbc_motion.hpp"

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
  /// Refresh the policy observation from robot_hw_ for safety and inference.
  virtual void updatePolicyObservationFromHardware();
  virtual void rvizVisualization();

  void ModeSubscribe(rclcpp::Node::SharedPtr controller_nh);

  void EmergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void lowCmdRlCallback(const luvbot_control_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void lowCmdImitationCallback(const luvbot_control_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void lowCmdWbcCallback(const luvbot_control_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void wbcMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void wbcActionCommandCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void textWbcCancelCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void plannerCommandCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void followFaceCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void imitationMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  std::array<double, 3> readCurrentHeadRpyRad();
  bool faceFollowAllowed() const;
  void syncFaceFollowState(bool allowed, double wall_sec);

  geometry_msgs::msg::Twist computeJoystickTrim(const JoystickCommand& command);
  geometry_msgs::msg::Twist selectedOperatorVelocity(const geometry_msgs::msg::Twist& joystick_velocity);
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
  bool startActionSequence();
  void stopActionSequence(const char* reason, double wall_sec);
  void finishActionSequenceNormally(double wall_sec);
  void configureActionSequencePhase(const ActionSequencePhase& phase, double wall_sec);
  void handleActionSequenceUpdate(const rclcpp::Time& time);
  bool loadActionSequenceForPlayback(const std::string& path, const char* context);
  bool triggerSequenceFromPath(const std::string& sequence_path, const char* context,
                               const std::function<void()>& after_start = nullptr);
  std::string displayControllerPath(const std::string& path) const;
  bool prepareImitationPolicyForLoadedSequence(const char* context);
  bool fullCsvVelocityAllowed() const;
  void logFullCsvVelocityBlocked(const char* context, const std::string& target) const;
  bool switchImitationPolicy(const std::string& policy_name, const char* context);
  void publishImitationActive(bool active, const char* reason = nullptr);
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
  bool startWbcMotion(const std::string& motion_id, int64_t timestamp_ms,
                      std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Response> response);
  bool finishWbcMotion(const char* reason, std::string& message);
  void abortWbcImmediately(const char* reason);
  void processSimWbcFinishEmergency();
  void publishWbcActionFeedback(int64_t timestamp_ms, const std::string& action_id, const char* status);
  void publishActiveWbcActionTerminal(const char* status);
  void updateWbcControlState();
  bool startPolicyHandoff(const PolicyInputs& start, double duration_sec, const char* label);

 protected:
  rclcpp::Publisher<luvbot_control_interfaces::msg::PolicyObservation>::SharedPtr pubState_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joyFeedbackPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wbcActionFeedbackPub_;
  rclcpp::Subscription<luvbot_control_interfaces::msg::PolicyCommand>::SharedPtr subCmdWbc_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subWbcMotionDone_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subWbcActionCommand_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subTextWbcCancel_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr shotIdPub_;
  rclcpp::CallbackGroup::SharedPtr sequence_trigger_callback_group_;

  rclcpp::Subscription<luvbot_control_interfaces::msg::PolicyCommand>::SharedPtr subCmdRl_;
  rclcpp::Subscription<luvbot_control_interfaces::msg::PolicyCommand>::SharedPtr subCmdImitation_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subEmgstop_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr plannerCommandSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subFollowFace_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationMotionDone_;

  // State containers
  luvbot_control_interfaces::msg::PolicyObservation policy_observation_;
  sensor_msgs::msg::JointState rvizVisualPos_;
  std::vector<std::string> jointNames_;
  /// True for leg joints (name contains hip/knee/ankle).
  std::vector<bool> is_leg_joint_;
  /// True for arm joints (name contains shoulder/elbow/wrist).
  std::vector<bool> is_arm_joint_;
  /// True for head joints (name contains neck/head).
  std::vector<bool> is_head_joint_;

  // Motion source components
  HeadScriptController headCtrl_;
  FaceFollowController faceFollowCtrl_;
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
  bool joystick_imitation_enabled_{false};
  bool joystick_sequence_enabled_{false};
  bool joystick_back_emergency_sequence_enabled_{false};
  std::string joystick_imitation_config_path_;
  std::string joystick_sequence_config_path_;
  std::string joystick_back_emergency_sequence_path_;
  std::string legged_controllers_share_;
  bool radarControlEnabled_{false};
  bool faceFollowEnabled_{false};

  /// Last controller `time.seconds()` from `update()`; used for async callbacks (e.g. /imitation_motion_done).
  double last_controller_wall_sec_{0.0};
  std::atomic<bool> face_follow_allowed_{false};

  std::mutex plannerCommandMutex_;
  geometry_msgs::msg::Twist plannerCommand_;
  bool plannerCommandSeen_{false};
  bool plannerCommandValid_{false};
  int64_t plannerCommandStampNs_{0};
  int64_t plannerCommandClockNs_{0};
  std::chrono::steady_clock::time_point plannerCommandReceiveTime_{};

  double imitation_policy_switch_timeout_sec_{120.0};
  double imitation_return_blend_sec_{0.35};
  bool wbc_enabled_{false};
  double amp_to_wbc_transition_sec_{1.0};
  double wbc_to_amp_transition_sec_{0.35};
  double wbc_command_timeout_sec_{0.5};
  std::atomic<bool> wbc_return_pending_{false};
  std::atomic<bool> wbc_handoff_in_progress_{false};
  double emergency_damping_kd_{5.0};
  bool sequence_finish_emergency_pending_{false};
  std::mutex auxiliary_action_trigger_mutex_;
  bool wbc_cleanup_pending_{false};  // Guarded by auxiliary_action_trigger_mutex_.
  // Callbacks stage handoffs; only the control loop touches the shared transition.
  std::mutex wbc_transition_mutex_;
  uint64_t wbc_generation_{0};
  // Simulation-only completion policy. Pending state is guarded by wbc_transition_mutex_.
  std::vector<std::string> sim_wbc_emergency_after_actions_;
  bool sim_wbc_finish_emergency_pending_{false};
  uint64_t sim_wbc_finish_emergency_generation_{0};
  bool wbc_start_allowed_{false};
  bool wbc_controller_walking_{false};
  geometry_msgs::msg::Quaternion wbc_orientation_;
  geometry_msgs::msg::Twist wbc_velocity_;
  bool wbc_handoff_pending_{false};
  bool wbc_handoff_clear_pending_{false};
  PolicyInputs wbc_handoff_start_;
  PolicyCmdSource wbc_handoff_target_{PolicyCmdSource::kRl};
  std::mutex wbc_action_state_mutex_;
  bool wbc_action_active_{false};
  int64_t wbc_action_timestamp_ms_{0};
  std::string wbc_action_id_;
  std::deque<std::pair<std::string, std::chrono::steady_clock::time_point>> recent_wbc_action_requests_;

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
  void publishPolicyObservationWithJoystick(double wall_sec);
  void stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Duration& period,
                                  const rclcpp::Time& time);
  void maybePublishVisualization(const rclcpp::Time& time);
  void applyJointCsvPhaseTriggerBlendAndCmdVel(double wall_sec);
};

}  // namespace legged
