#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "legged_controllers/ControllerFileUtils.h"
#include "legged_controllers/JointGroups.h"
#include "legged_controllers/LeggedController.h"
#include "legged_controllers/ParameterManager.h"

namespace legged {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kDefaultBackEmergencySequencePath = "action_sequences/walking_back_emergency.yaml";

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
  declare_if_absent("head_script_enabled", false);
  declare_if_absent("head_script_config", std::string(""));
  declare_if_absent("joystick_imitation_enabled", false);
  declare_if_absent("joystick_imitation_config_path", std::string(""));
  declare_if_absent("joystick_sequence_enabled", false);
  declare_if_absent("joystick_sequence_config_path", std::string(""));
  declare_if_absent("joystick_back_emergency_sequence_enabled", true);
  declare_if_absent("joystick_back_emergency_sequence_path", std::string(kDefaultBackEmergencySequencePath));
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
  declare_if_absent("safety.orientation.consecutive_violation_count", 3);
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

  bool head_script_enabled = false;
  controller_nh->get_parameter("head_script_enabled", head_script_enabled);
  RCLCPP_INFO(controller_nh->get_logger(), "head_script_enabled: %d", head_script_enabled);
  controller_nh->get_parameter("joystick_imitation_enabled", joystick_imitation_enabled_);
  controller_nh->get_parameter("joystick_sequence_enabled", joystick_sequence_enabled_);
  controller_nh->get_parameter("joystick_back_emergency_sequence_enabled", joystick_back_emergency_sequence_enabled_);
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
  RCLCPP_INFO(controller_nh->get_logger(), "joystick_imitation_enabled: %d", joystick_imitation_enabled_);
  RCLCPP_INFO(controller_nh->get_logger(), "joystick_sequence_enabled: %d", joystick_sequence_enabled_);
  RCLCPP_INFO(controller_nh->get_logger(), "joystick_back_emergency_sequence_enabled: %d",
              joystick_back_emergency_sequence_enabled_);
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

  std::string head_script_config_path;
  controller_nh->get_parameter("head_script_config", head_script_config_path);
  controller_nh->get_parameter("joystick_imitation_config_path", joystick_imitation_config_path_);
  controller_nh->get_parameter("joystick_sequence_config_path", joystick_sequence_config_path_);
  controller_nh->get_parameter("joystick_back_emergency_sequence_path", joystick_back_emergency_sequence_path_);
  joystick_imitation_config_path_ = trimCopy(joystick_imitation_config_path_);
  joystick_sequence_config_path_ = trimCopy(joystick_sequence_config_path_);
  joystick_back_emergency_sequence_path_ = trimCopy(joystick_back_emergency_sequence_path_);
  sequence_engine_enabled_ = joystick_sequence_enabled_ || joystick_back_emergency_sequence_enabled_;

  bool ok = true;
  policy_frequency = rc["policy_frequency"].as<int>(0);
  if (policy_frequency == 0)
  {
    RCLCPP_ERROR(controller_nh->get_logger(), "Failed to get parameters from /RobotCfg/policy_frequency");
    return false;
  }
  policyMux_.init(controller_nh, joint_names);
  JointGainProfile rl_walking_gains;
  if (!loadWalkingGainProfile(rc, "RobotCfg", joint_names.size(), rl_walking_gains))
  {
    return false;
  }
  policyMux_.configureStartup(single_node_publishes_imitation_command, rl_walking_gains,
                              imitation_policy_switch_timeout_sec_, policy_root_path, robot_type);

  RCLCPP_INFO(controller_nh->get_logger(), "Loaded RL walking gains from: %s", robot_cfg_path.c_str());
  RCLCPP_INFO(controller_nh->get_logger(), "Dynamic imitation gains will be loaded when a sequence switches policy");

  try
  {
    legged_controllers_share_ = ament_index_cpp::get_package_share_directory("legged_controllers");
  } catch (const std::exception& e)
  { RCLCPP_WARN(controller_nh->get_logger(), "Failed to resolve legged_controllers share dir: %s", e.what()); }
  sequenceRuntime_.init(controller_nh->get_logger(), legged_controllers_share_);

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

  headCtrl_.setScriptEnabled(head_script_enabled);
  headCtrl_.setTopicEnabled(faceFollowEnabled_);
  headCtrl_.setTopicBlendSec(0.1);
  headCtrl_.setFallbackBlendSec(0.2);
  RCLCPP_INFO(controller_nh->get_logger(), "face_follow_enabled: %d topic=/algorithm/em/follow_face",
              faceFollowEnabled_);
  if (head_script_enabled)
  {
    RCLCPP_INFO(controller_nh->get_logger(), "Head script enabled: timeline will drive head");
    if (!headCtrl_.loadConfig(head_script_config_path))
    {
      RCLCPP_ERROR(controller_nh->get_logger(), "Head script config failed to load: %s",
                   head_script_config_path.c_str());
      return false;
    }
    RCLCPP_INFO(controller_nh->get_logger(), "Head script loaded from config: %s (%zu segments, duration %.1fs)",
                head_script_config_path.c_str(), headCtrl_.numSegments(), headCtrl_.duration());
  }
  else
  {
    RCLCPP_INFO(controller_nh->get_logger(), "Head script disabled by parameter");
  }

  sequenceRuntime_.setEnabled(sequence_engine_enabled_);
  if (joystick_back_emergency_sequence_enabled_)
  {
    if (joystick_back_emergency_sequence_path_.empty())
    {
      RCLCPP_ERROR(controller_nh->get_logger(),
                   "Back emergency sequence enabled but joystick_back_emergency_sequence_path is empty");
      return false;
    }
    if (!loadActionSequenceForPlayback(joystick_back_emergency_sequence_path_, "[BackEmergency] Init"))
    {
      return false;
    }
    if (!sequenceRuntime_.usesFullCsv() || sequenceRuntime_.usesRlImitation())
    {
      RCLCPP_ERROR(controller_nh->get_logger(),
                   "Back emergency sequence must use full_csv and must not use rl_imitation: %s",
                   joystick_back_emergency_sequence_path_.c_str());
      return false;
    }
    RCLCPP_INFO(controller_nh->get_logger(), "Back emergency sequence enabled: %s",
                joystick_back_emergency_sequence_path_.c_str());
  }

  if (joystick_imitation_enabled_)
  {
    if (joystick_imitation_config_path_.empty())
    {
      RCLCPP_ERROR(controller_nh->get_logger(),
                   "Joystick imitation enabled but joystick_imitation_config_path is empty");
      return false;
    }

    JoystickImitationActions::LoadOptions options;
    options.package_share_dir = legged_controllers_share_;
    options.policy_root_path = policy_root_path;
    options.robot_type = robot_type;
    options.validate_policy_assets = true;
    if (!joystickImitationActions_.load(joystick_imitation_config_path_, options, controller_nh->get_logger()))
    {
      return false;
    }
    if (joystickImitationActions_.empty())
    {
      joystick_imitation_enabled_ = false;
      RCLCPP_WARN(controller_nh->get_logger(),
                  "Joystick imitation disabled: no usable actions loaded from %s for robot_type=%s (skipped=%zu)",
                  joystick_imitation_config_path_.c_str(), robot_type.c_str(),
                  joystickImitationActions_.skippedBindingCount());
    }
    else if (joystickImitationActions_.skippedBindingCount() > 0)
    {
      RCLCPP_WARN(
          controller_nh->get_logger(), "Joystick imitation skipped %zu unavailable action(s) from %s for robot_type=%s",
          joystickImitationActions_.skippedBindingCount(), joystick_imitation_config_path_.c_str(), robot_type.c_str());
    }
    if (joystick_imitation_enabled_)
    {
      RCLCPP_INFO(
          controller_nh->get_logger(), "Joystick imitation enabled: config=%s robot_type=%s enabled_actions=%zu",
          joystick_imitation_config_path_.c_str(), robot_type.c_str(), joystickImitationActions_.bindingCount());
    }
  }

  if (joystick_sequence_enabled_)
  {
    if (joystick_sequence_config_path_.empty())
    {
      RCLCPP_ERROR(controller_nh->get_logger(), "Joystick sequence enabled but joystick_sequence_config_path is empty");
      return false;
    }

    JoystickSequenceActions::LoadOptions options;
    options.package_share_dir = legged_controllers_share_;
    options.validate_sequence_files = true;
    if (!joystickSequenceActions_.load(joystick_sequence_config_path_, options, controller_nh->get_logger()))
    {
      return false;
    }
    RCLCPP_INFO(controller_nh->get_logger(), "Joystick sequence enabled: config=%s enabled_actions=%zu",
                joystick_sequence_config_path_.c_str(), joystickSequenceActions_.bindingCount());
  }

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

    safetyChecker_ = std::make_unique<SafetyChecker>(sp);
    RCLCPP_INFO(controller_nh->get_logger(),
                "Orientation safety always active on real robot: roll_limit=%.1f deg, pitch_limit=%.1f deg, "
                "consecutive_violation_count=%d",
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
  sequenceRuntime_.initJoints(joint_names);

  const JointGroups groups = classifyJointGroups(joint_names);
  is_leg_joint_ = groups.leg;
  is_arm_joint_ = groups.arm;
  is_head_joint_ = groups.head;

  joyFeedbackArray_.array.resize(2);
  joyFeedbackArray_.array[0].type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  joyFeedbackArray_.array[1].type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  joyFeedbackArray_.array[0].id = 0;
  joyFeedbackArray_.array[1].id = 1;

  policy_observation_.behavior_up_down = 0.0f;
  policy_observation_.behavior_left_right = 0.0f;
}

}  // namespace legged
