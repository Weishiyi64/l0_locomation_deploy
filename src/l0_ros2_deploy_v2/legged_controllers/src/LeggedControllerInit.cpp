#include <yaml-cpp/yaml.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ControllerFileUtils.h"
#include "LeggedController.h"
#include "ParameterManager.h"

namespace legged {

namespace {

constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees) { return degrees * kPi / 180.0; }

double radiansToDegrees(double radians) { return radians * 180.0 / kPi; }

std::string parentPath(const std::string& path) {
  const std::size_t pos = path.find_last_of('/');
  if (pos == std::string::npos) return "";
  if (pos == 0) return "/";
  return path.substr(0, pos);
}

std::string baseName(const std::string& path) {
  const std::size_t pos = path.find_last_of('/');
  if (pos == std::string::npos) return path;
  return path.substr(pos + 1);
}

std::string derivePolicyRootPath(const std::string& robot_cfg_path) {
  return parentPath(parentPath(parentPath(robot_cfg_path)));
}

std::string deriveRobotTypeFromConfigPath(const std::string& robot_cfg_path) {
  return baseName(parentPath(parentPath(robot_cfg_path)));
}

}  // namespace

bool LeggedController::init(rclcpp::Node::SharedPtr controller_nh) {
  // Parameters read in this function (launch / YAML overrides replace these defaults).
  auto declare_if_absent = [&controller_nh](const std::string& name, const auto& default_value) {
    if (!controller_nh->has_parameter(name))
    {
      controller_nh->declare_parameter(name, default_value);
    }
  };
  declare_if_absent("isSim", false);
  declare_if_absent("rviz", false);
  declare_if_absent("radar_control_enabled", false);
  declare_if_absent("face_follow_enabled", false);
  declare_if_absent("policy_root_path", std::string(""));
  declare_if_absent("imitation_policy_switch_timeout_sec", 120.0);
  declare_if_absent("imitation_return_blend_sec", 0.35);
  declare_if_absent("wbc_enabled", false);
  declare_if_absent("wbc_transition_sec", 0.35);
  declare_if_absent("wbc_command_timeout_sec", 0.5);
  declare_if_absent("arm_swing.enabled", true);
  declare_if_absent("arm_swing.stop_vx_threshold", 0.05);
  declare_if_absent("arm_swing.high_vx_threshold", 0.8);
  declare_if_absent("arm_swing.vx_hysteresis", 0.03);
  declare_if_absent("arm_swing.blend_sec", 0.35);
  declare_if_absent("safety.orientation.roll_limit_deg", 30.0);
  declare_if_absent("safety.orientation.pitch_limit_deg", 30.0);
  declare_if_absent("safety.orientation.consecutive_violation_count", 50);
  declare_if_absent("emergency.damping_kd", 5.0);

  std::vector<std::string> joint_names;
  std::string robot_cfg_path;
  if (!controller_nh->get_parameter("robot_cfg_path", robot_cfg_path) || robot_cfg_path.empty())
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "Failed to get robot_cfg_path params");
    return false;
  }

  YAML::Node root = YAML::LoadFile(robot_cfg_path);
  YAML::Node rc = root["RobotCfg"];
  requireNode(rc, "RobotCfg");

  bool single_node_publishes_imitation_command = false;
  {
    const std::string policy_type_cfg = rc["policy_type"].as<std::string>("");
    single_node_publishes_imitation_command = (policy_type_cfg == "rl_imitation");
    RCLCPP_INFO(controller_nh->get_logger(),
                "Policy mux from robot_cfg: policy_type='%s' -> use control/policy/command/imitation on walk=%s",
                policy_type_cfg.c_str(), single_node_publishes_imitation_command ? "yes" : "no");
  }

  joint_names = asStringList(rc["joint_order_mj"], "RobotCfg.joint_order_mj");

  if (joint_names.empty())
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "Failed to get joint names from URDF.");
    return false;
  }

  std::string robot_type;
  controller_nh->get_parameter("robot_type", robot_type);
  robot_type = trimCopy(robot_type);
  if (robot_type.empty()) robot_type = deriveRobotTypeFromConfigPath(robot_cfg_path);

  std::string policy_root_path;
  controller_nh->get_parameter("policy_root_path", policy_root_path);
  policy_root_path = trimCopy(policy_root_path);
  if (policy_root_path.empty()) policy_root_path = derivePolicyRootPath(robot_cfg_path);

  RCLCPP_INFO(controller_nh->get_logger(), "LeggedController::init, namespace: %s", controller_nh->get_namespace());

  controller_nh->get_parameter("isSim", isSim_);
  controller_nh->get_parameter("rviz", enableRviz_);
  controller_nh->get_parameter("radar_control_enabled", radarControlEnabled_);
  controller_nh->get_parameter("face_follow_enabled", faceFollowEnabled_);

  controller_nh->get_parameter("imitation_policy_switch_timeout_sec", imitation_policy_switch_timeout_sec_);
  controller_nh->get_parameter("imitation_return_blend_sec", imitation_return_blend_sec_);
  controller_nh->get_parameter("wbc_enabled", wbc_enabled_);
  controller_nh->get_parameter("wbc_transition_sec", wbc_transition_sec_);
  controller_nh->get_parameter("wbc_command_timeout_sec", wbc_command_timeout_sec_);
  controller_nh->get_parameter("emergency.damping_kd", emergency_damping_kd_);
  imitation_return_blend_sec_ = std::max(imitation_return_blend_sec_, 0.0);
  if (!std::isfinite(wbc_transition_sec_) || wbc_transition_sec_ < 0.0)
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "wbc_transition_sec must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(wbc_command_timeout_sec_) || wbc_command_timeout_sec_ <= 0.0)
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "wbc_command_timeout_sec must be finite and positive");
    return false;
  }
  if (emergency_damping_kd_ < 0.0)
  {
    RCLCPP_WARN(controller_nh->get_logger(), "emergency.damping_kd %.3f < 0, clamping to 0", emergency_damping_kd_);
    emergency_damping_kd_ = 0.0;
  }
  RCLCPP_INFO(controller_nh->get_logger(), "imitation_return_blend_sec: %.3f", imitation_return_blend_sec_);
  RCLCPP_INFO(controller_nh->get_logger(), "wbc_enabled: %d transition=%.3fs timeout=%.3fs", wbc_enabled_,
              wbc_transition_sec_, wbc_command_timeout_sec_);
  RCLCPP_INFO(controller_nh->get_logger(), "emergency.damping_kd: %.3f", emergency_damping_kd_);

  ArmSwingController::Params arm_swing_params;
  controller_nh->get_parameter("arm_swing.enabled", arm_swing_params.enabled);
  controller_nh->get_parameter("arm_swing.stop_vx_threshold", arm_swing_params.stop_vx_threshold);
  controller_nh->get_parameter("arm_swing.high_vx_threshold", arm_swing_params.high_vx_threshold);
  controller_nh->get_parameter("arm_swing.vx_hysteresis", arm_swing_params.vx_hysteresis);
  controller_nh->get_parameter("arm_swing.blend_sec", arm_swing_params.blend_sec);

  bool ok = true;
  policy_frequency = rc["policy_frequency"].as<int>(0);
  if (policy_frequency == 0)
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "Failed to get parameters from /RobotCfg/policy_frequency");
    return false;
  }
  policyMux_.init(controller_nh, joint_names);
  JointGainProfile rl_walking_gains;
  if (!policyMux_.loadWalkingGainProfile(rc, "RobotCfg", joint_names.size(), rl_walking_gains))
  {
    return false;
  }
  policyMux_.configureStartup(single_node_publishes_imitation_command, rl_walking_gains,
                              imitation_policy_switch_timeout_sec_, policy_root_path, robot_type);

  RCLCPP_INFO(controller_nh->get_logger(), "Loaded RL walking gains from: %s", robot_cfg_path.c_str());
  RCLCPP_INFO(controller_nh->get_logger(),
              "Dynamic imitation gains will be loaded when an imitation action switches policy");

  if (!ok)
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "Failed to get parameters from RobotCfg.");
    return false;
  }

  {
    double viz_freq = 0.0;
    controller_nh->get_parameter_or("viz_frequency", viz_freq, static_cast<double>(policy_frequency));

    if (viz_freq <= 0.0) viz_freq = static_cast<double>(policy_frequency);
    vizPeriod_ = rclcpp::Duration::from_seconds(1.0 / viz_freq);

    double joy_feedback_hz = 2.0;
    controller_nh->get_parameter_or("joy_feedback_hz", joy_feedback_hz, 2.0);

    if (joy_feedback_hz <= 0.0) joy_feedback_hz = 2.0;
    joyFeedbackPeriod_ = rclcpp::Duration::from_seconds(1.0 / joy_feedback_hz);
    lastJoyFeedbackTick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    joyFeedbackState_ = false;
  }

  dataInit(joint_names);
  armSwingCtrl_.configure(arm_swing_params, controller_nh->get_logger());

  headCtrl_.setTopicEnabled(faceFollowEnabled_);
  headCtrl_.setTopicBlendSec(0.1);
  RCLCPP_INFO(controller_nh->get_logger(), "face_follow_enabled: %d topic=/algorithm/em/follow_face",
              faceFollowEnabled_);

  velLimiter_ = std::make_unique<VelocityLimiter>(controller_nh);

  {
    const SafetyChecker::Params defaults;
    SafetyChecker::Params sp;
    double roll_limit_deg = radiansToDegrees(defaults.roll_limit);
    double pitch_limit_deg = radiansToDegrees(defaults.pitch_limit);
    controller_nh->get_parameter("safety.orientation.roll_limit_deg", roll_limit_deg);
    controller_nh->get_parameter("safety.orientation.pitch_limit_deg", pitch_limit_deg);
    controller_nh->get_parameter("safety.orientation.consecutive_violation_count", sp.consecutive_violation_count);
    sp.roll_limit = degreesToRadians(roll_limit_deg);
    sp.pitch_limit = degreesToRadians(pitch_limit_deg);

    if (sp.roll_limit <= 0.0)
    {
      RCLCPP_WARN(controller_nh->get_logger(), "Invalid roll_limit_deg; using default %.1f deg",
                  radiansToDegrees(defaults.roll_limit));
      sp.roll_limit = defaults.roll_limit;
    }
    if (sp.pitch_limit <= 0.0)
    {
      RCLCPP_WARN(controller_nh->get_logger(), "Invalid pitch_limit_deg; using default %.1f deg",
                  radiansToDegrees(defaults.pitch_limit));
      sp.pitch_limit = defaults.pitch_limit;
    }
    if (sp.consecutive_violation_count <= 0)
    {
      RCLCPP_WARN(controller_nh->get_logger(), "Invalid consecutive_violation_count; using default %d",
                  defaults.consecutive_violation_count);
      sp.consecutive_violation_count = defaults.consecutive_violation_count;
    }

    safetyChecker_ = std::make_shared<SafetyChecker>(sp);
    robot_hw_->setImuSafetyChecker(safetyChecker_);
    RCLCPP_INFO(controller_nh->get_logger(),
                "Orientation safety always active on real robot: roll_limit=%.1f deg, pitch_limit=%.1f deg, "
                "consecutive_violation_count=%d IMU frames",
                radiansToDegrees(sp.roll_limit), radiansToDegrees(sp.pitch_limit), sp.consecutive_violation_count);
  }

  fsm_ = std::make_unique<StateMachine>(controller_nh->get_logger());

  ModeSubscribe(controller_nh);
  ModePublish(controller_nh);

  return true;
}

void LeggedController::dataInit(const std::vector<std::string>& joint_names) {
  policy_observation_.joints.name = joint_names;
  policy_observation_.joints.position.resize(joint_names.size());
  policy_observation_.joints.velocity.resize(joint_names.size());
  policy_observation_.joints.effort.resize(joint_names.size());
  policyMux_.resizeCommands(joint_names.size());

  rvizVisualPos_.name = joint_names;
  jointNames_ = joint_names;
  rvizVisualPos_.position.resize(joint_names.size());
  rvizVisualPos_.velocity.resize(joint_names.size());
  rvizVisualPos_.effort.resize(joint_names.size());

  headCtrl_.initJoints(joint_names, nh_->get_logger());
  armSwingCtrl_.initJoints(joint_names, nh_->get_logger());
  policy_handoff_.resize(joint_names.size());

  joyFeedbackArray_.array.resize(2);
  joyFeedbackArray_.array[0].type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  joyFeedbackArray_.array[1].type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  joyFeedbackArray_.array[0].id = 0;
  joyFeedbackArray_.array[1].id = 1;

  policy_observation_.behavior_up_down = 0.0f;
  policy_observation_.behavior_left_right = 0.0f;
}

}  // namespace legged
