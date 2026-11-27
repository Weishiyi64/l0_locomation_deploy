//
// Base class for legged controllers: shared FSM, policy mux, safety, head control, and command composition.
//
#pragma once

#include <ArmSwingController.h>
#include <ControllerTypes.h>
#include <FaceFollow.h>
#include <IHardwareLayer.h>
#include <JointCommandComposer.h>
#include <JointCommandTransition.h>
#include <PolicyMux.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
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
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "JoystickInput.h"
#include "SafetyChecker.h"
#include "StateMachine.h"
#include "VelocityLimiter.h"
#include "VisualizationPublisher.h"
#include "robot_interfaces/msg/policy_command.hpp"
#include "robot_interfaces/msg/policy_observation.hpp"
#include "robot_interfaces/srv/control_wbc_motion.hpp"
#include "robot_interfaces/srv/motion_transaction.hpp"

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
  void lowCmdRlCallback(const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void lowCmdImitationCallback(const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void lowCmdWbcCallback(const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg);
  void wbcMotionDoneCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void plannerCommandCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void followFaceCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void imitationMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  std::array<double, 3> readCurrentHeadRpyRad();
  bool faceFollowAllowed() const;
  void syncFaceFollowState(bool allowed, double wall_sec);

  geometry_msgs::msg::Twist computeJoystickTrim(const JoystickCommand& command);
  geometry_msgs::msg::Twist selectedOperatorVelocity(const geometry_msgs::msg::Twist& joystick_velocity);
  void applyJoystickMotion(const JoystickCommand& command);
  void processJoystickActions(const JoystickCommand& command);
  bool startImitationAction(const std::string& policy);
  void enterWalkMode();
  virtual void resetController();
  enum StopReason : uint32_t {
    ExternalStop = 1, ManualStop = 2, OrientationStop = 4, NoValidCommand = 8
  };
  void triggerEmergencyStop(StopReason reason = ExternalStop);
  void updateSpeedMultiplier(double lt_axis_value);
  void adjustStandbyKp(double delta);
  void toggleCalibrationMode();
  void calibrateByTag(const std::string& tag, const std::string& banner);
  bool imitationActionActive() const;
  bool canStartAmpDerivedAction(const char* context, const std::string& action_label, bool require_walking,
                                bool block_imitation_active) const;
  void dataInit(const std::vector<std::string>& joint_names);
  /// Switch policy mux to RL and reset RL observation history (no kp/kd ramp).
  void returnToRlWithoutGainRamp(const char* reason);
  void initializeMotionAdmission();
  void updateMotionPreparation();
  bool motionBaseReady();
  void motionTransaction(const std::shared_ptr<robot_interfaces::srv::MotionTransaction::Request> request,
                         std::shared_ptr<robot_interfaces::srv::MotionTransaction::Response> response);
  bool startWbcMotion(const std::string& motion_id, int64_t timestamp_ms,
                      std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Response> response);
  bool requestWbcReturn(const char* reason, std::string& message, bool failed = false, bool live_preview = false);
  void softStopWbc(const std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Request> request,
                   std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Response> response);
  rclcpp::Service<robot_interfaces::srv::ControlWbcMotion>::SharedPtr soft_stop_wbc_service_;
  rclcpp::Client<robot_interfaces::srv::ControlWbcMotion>::SharedPtr joystick_soft_stop_client_;
  bool wbc_live_return_{false};
  void abortWbcImmediately(const char* reason);
  void publishWbcActionFeedback(int64_t timestamp_ms, const std::string& action_id, const char* status,
                                const char* detail = nullptr);
  void publishActiveWbcActionTerminal(const char* status, const char* detail = nullptr);
  void updateWbcControlState();
  bool startPolicyHandoff(double duration_sec, const char* label, bool include_legs = false);
  bool holdPolicyCommand(const PolicyInputs& inputs);

 protected:
  rclcpp::Publisher<robot_interfaces::msg::PolicyObservation>::SharedPtr pubState_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joyFeedbackPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wbcActionFeedbackPub_;
  rclcpp::Subscription<robot_interfaces::msg::PolicyCommand>::SharedPtr subCmdWbc_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subWbcMotionDone_;
  rclcpp::CallbackGroup::SharedPtr wbc_callback_group_;

  rclcpp::Subscription<robot_interfaces::msg::PolicyCommand>::SharedPtr subCmdRl_;
  rclcpp::Subscription<robot_interfaces::msg::PolicyCommand>::SharedPtr subCmdImitation_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subEmgstop_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr plannerCommandSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subFollowFace_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationMotionDone_;

  // State containers
  robot_interfaces::msg::PolicyObservation policy_observation_;
  sensor_msgs::msg::JointState rvizVisualPos_;
  std::vector<std::string> jointNames_;

  // Motion source components
  FaceFollowHeadOutput headCtrl_;
  FaceFollowController faceFollowCtrl_;
  ArmSwingController armSwingCtrl_;
  PolicyMux policyMux_;
  JointCommandTransition policy_handoff_;
  JointCommandComposer jointCommandComposer_;
  bool holding_policy_command_{false};  // Control-thread owned.

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
  double wbc_transition_sec_{0.35};
  double wbc_command_timeout_sec_{0.5};
  std::atomic<bool> wbc_return_pending_{false};
  std::atomic<bool> wbc_handoff_in_progress_{false};
  double emergency_damping_kd_{5.0};
  std::mutex auxiliary_action_trigger_mutex_;
  std::atomic<bool> wbc_cleanup_pending_{false};  // Guarded by auxiliary_action_trigger_mutex_.
  std::atomic<bool> motion_reserved_{false}, motion_cancelled_{false};
  std::atomic<int64_t> motion_gateway_seen_ms_{0};
  std::mutex motion_mutex_;
  std::string motion_request_id_, motion_source_, motion_action_id_;
  int64_t motion_timestamp_ms_{0}, motion_ready_since_ms_{0};
  bool motion_ready_seen_{false};
  bool motion_head_pending_{false};  // Guarded by motion_mutex_; applied by the control thread.
  int64_t text_capture_stamp_ns_{0};  // Guarded by motion_mutex_; zero means not ready to capture.
  bool motion_head_hold_active_{false};  // The following preparation output state is control-thread owned.
  bool motion_head_pause_applied_{false};
  bool motion_upper_snapshot_valid_{false};
  std::array<double, 3> motion_head_positions_{};
  std::array<double, 9> motion_upper_positions_{};
  std::deque<std::string> motion_cancelled_ids_;
  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  rclcpp::Service<robot_interfaces::srv::MotionTransaction>::SharedPtr motion_service_;
  rclcpp::TimerBase::SharedPtr motion_watchdog_;
  // Callbacks stage handoffs; only the control loop touches the shared transition.
  std::mutex wbc_transition_mutex_;
  std::condition_variable wbc_transition_cv_;
  uint64_t wbc_generation_{0};
  bool wbc_start_allowed_{false};
  bool wbc_controller_walking_{false};
  enum class WbcReturnOutcome { Completed, Cancelled, Failed };
  WbcReturnOutcome wbc_return_outcome_{WbcReturnOutcome::Completed};
  std::string wbc_return_id_;
  geometry_msgs::msg::Quaternion wbc_orientation_;
  geometry_msgs::msg::Twist wbc_velocity_;
  bool wbc_handoff_pending_{false};
  bool wbc_handoff_clear_pending_{false};
  bool wbc_preposition_pending_{false};
  bool wbc_preposition_ready_{false};
  bool wbc_preposition_cancel_pending_{false};
  std::atomic<bool> wbc_preposition_active_{false};
  std::vector<double> wbc_preposition_positions_;
  PolicyCmdSource wbc_handoff_target_{PolicyCmdSource::kRl};
  std::mutex wbc_action_state_mutex_;
  bool wbc_action_active_{false};
  int64_t wbc_action_timestamp_ms_{0};
  std::string wbc_action_id_;

  // Feedback rumble array
  // sensor_msgs::JoyFeedbackArray joyFeedbackArray_;
  sensor_msgs::msg::JoyFeedbackArray joyFeedbackArray_;

  // Components
  JoystickInput joystickInput_;
  JoystickCommand lastJoystickCommand_;
  std::unique_ptr<VelocityLimiter> velLimiter_;
  std::unique_ptr<VisualizationPublisher> vizPub_;
  std::shared_ptr<SafetyChecker> safetyChecker_;
  std::unique_ptr<StateMachine> fsm_;

  // Flags (default false for real robot; set true in sim launch)
  bool isSim_{false};
  bool enableRviz_{false};

 private:
  struct StatusSnapshot {
    std::chrono::steady_clock::time_point sampled{};
    StateMachine::State mode{StateMachine::State::Standby};
    PolicyCmdSource source{PolicyCmdSource::kRl};
    bool required{false}, valid{false}, emergency{false}, holding_last_command{false};
    double policy_age_ms{-1.0}, policy_valid_for_ms{0.0};
    uint32_t reasons{0};
  };
  std::atomic<uint32_t> stop_reasons_{0};
  std::mutex status_snapshot_mutex_;
  StatusSnapshot status_snapshot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_status_pub_;
  rclcpp::TimerBase::SharedPtr controller_status_timer_;
  void captureControllerStatus(bool walking, const PolicyInputs& inputs);
  void publishControllerStatus();
  void prepareControllerClock(const rclcpp::Time& time);
  RuntimeControlState computeRuntimeControlState() const;
  void runSafetyAndEmergencyFeedback(const rclcpp::Time& time, bool safety_armed);
  void publishPolicyObservationWithJoystick();
  void stepAuxiliaryMotionSources(bool allow_auxiliary_overlays, const rclcpp::Time& time);
  void maybePublishVisualization(const rclcpp::Time& time);
};

}  // namespace legged
