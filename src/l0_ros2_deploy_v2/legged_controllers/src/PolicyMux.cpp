#include "PolicyMux.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <future>
#include <unordered_map>
#include <utility>

#include "ControllerFileUtils.h"
#include "IHardwareLayer.h"

namespace legged {

namespace {

void fillPolicyCommandPositions(robot_interfaces::msg::PolicyCommand& cmd, double value) {
  std::fill(cmd.target_positions.begin(), cmd.target_positions.end(), value);
}

bool isPolicyIdentifier(const std::string& value) {
  return !value.empty() && value != "." && value != ".." &&
         std::all_of(value.begin(), value.end(), [](unsigned char character) {
           return std::isalnum(character) || character == '_' || character == '-' || character == '.';
         });
}

}  // namespace

rclcpp::Logger PolicyMux::logger() const { return node_ ? node_->get_logger() : rclcpp::get_logger("policy_mux"); }

void PolicyMux::init(rclcpp::Node::SharedPtr node, const std::vector<std::string>& joint_names) {
  node_ = std::move(node);
  joint_names_ = joint_names;
  resizeCommands(joint_names.size());
}

void PolicyMux::configureStartup(bool single_node_publishes_imitation_command, const JointGainProfile& rl_gains,
                                 double policy_switch_timeout_sec, const std::string& policy_root_path,
                                 const std::string& robot_type) {
  single_node_publishes_imitation_command_ = single_node_publishes_imitation_command;
  imitation_policy_switch_timeout_sec_ = policy_switch_timeout_sec;
  policy_root_path_ = policy_root_path;
  robot_type_ = robot_type;
  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    rl_walking_gains_ = rl_gains;
    imitation_walking_gains_ = rl_gains;
    wbc_walking_gains_ = rl_gains;
  }
}

void PolicyMux::invalidateImitationCommandLocked() { pos_cmd_imitation_valid_ = false; }

void PolicyMux::createRosInterfaces() {
  if (!node_) return;

  rl_imitation_reset_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("control/policy/imitation/reset", rclcpp::QoS(10));
  rl_imitation_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>("control/policy/imitation/active",
                                                                          rclcpp::QoS(10).reliable().transient_local());
  rl_obs_history_reset_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("control/policy/observation/reset", rclcpp::QoS(10));
  if (!imitation_policy_client_callback_group_)
  {
    imitation_policy_client_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }
  switch_imitation_policy_client_ = node_->create_client<robot_interfaces::srv::SwitchPolicy>(
      "control/policy/switch", rmw_qos_profile_services_default, imitation_policy_client_callback_group_);
  publishImitationActive(false);
  wbc_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>("control/policy/wbc/active",
                                                             rclcpp::QoS(1).reliable().transient_local());
  wbc_execution_pub_ = node_->create_publisher<std_msgs::msg::String>("control/policy/wbc/execution_active",
                                                                 rclcpp::QoS(1).reliable().transient_local());
  wbc_client_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  prepare_wbc_motion_client_ = node_->create_client<robot_interfaces::srv::ControlWbcMotion>(
      "control/policy/wbc/prepare", rmw_qos_profile_services_default, wbc_client_callback_group_);
  publishWbcActive(false);
}

bool PolicyMux::loadWalkingGainProfile(const YAML::Node& rc, const std::string& profile_name, size_t expected_size,
                                       JointGainProfile& out) const {
  out.kps = asDoubleList(rc["kps"], profile_name + ".kps");
  if (out.kps.empty())
  {
    RCLCPP_ERROR(logger(), "Failed to get walking gains from %s.kps", profile_name.c_str());
    return false;
  }

  out.kds = asDoubleList(rc["kds"], profile_name + ".kds");
  if (out.kds.empty())
  {
    RCLCPP_ERROR(logger(), "Failed to get walking gains from %s.kds", profile_name.c_str());
    return false;
  }

  if (out.kps.size() != expected_size || out.kds.size() != expected_size)
  {
    RCLCPP_ERROR(logger(), "Walking gain size mismatch for %s: expected %zu joints, got kp=%zu kd=%zu",
                 profile_name.c_str(), expected_size, out.kps.size(), out.kds.size());
    return false;
  }

  return true;
}

bool PolicyMux::loadImitationWalkingGains(const std::string& policy_name) {
  if (policy_root_path_.empty() || robot_type_.empty())
  {
    RCLCPP_ERROR(logger(), "[ImitationPolicy] Controller has no policy_root_path/robot_type mapping");
    return false;
  }
  const std::string robot_cfg_path = policy_root_path_ + "/" + robot_type_ + "/" + policy_name + "/rl_config.yaml";

  try
  {
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    JointGainProfile next_gains;
    if (!loadWalkingGainProfile(rc, "RobotCfg(imitation:" + policy_name + ")", pos_cmd_rl_.target_positions.size(),
                                next_gains))
    {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(gains_mutex_);
      imitation_walking_gains_ = std::move(next_gains);
    }
    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "[ImitationPolicy] Failed to load gains for '%s' from %s: %s", policy_name.c_str(),
                 robot_cfg_path.c_str(), e.what());
    return false;
  }
}

bool PolicyMux::switchImitationPolicy(const std::string& policy_name, const char* context) {
  const std::string policy = trimCopy(policy_name);
  if (!isPolicyIdentifier(policy))
  {
    RCLCPP_ERROR(logger(), "%s failed: invalid imitation policy identifier",
                 context != nullptr ? context : "Imitation");
    return false;
  }

  if (imitation_policy_ready_ && current_imitation_policy_ == policy)
  {
    return true;
  }

  if (!switch_imitation_policy_client_)
  {
    RCLCPP_ERROR(logger(), "%s failed: switch imitation policy client is not initialized",
                 context != nullptr ? context : "Imitation");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    invalidateImitationCommandLocked();
  }
  publishImitationActive(false);
  if (!switch_imitation_policy_client_->wait_for_service(std::chrono::milliseconds(0)))
  {
    RCLCPP_ERROR(logger(), "%s failed: control/policy/switch service is not available",
                 context != nullptr ? context : "Imitation");
    return false;
  }

  auto request = std::make_shared<robot_interfaces::srv::SwitchPolicy::Request>();
  request->policy_name = policy;
  auto future = switch_imitation_policy_client_->async_send_request(request);
  const auto timeout = std::chrono::duration<double>(std::max(1.0, imitation_policy_switch_timeout_sec_));
  if (future.wait_for(timeout) != std::future_status::ready)
  {
    imitation_policy_ready_ = false;
    current_imitation_policy_.clear();
    RCLCPP_ERROR(logger(), "%s timed out switching imitation policy to '%s' after %.1fs",
                 context != nullptr ? context : "Imitation", policy.c_str(), imitation_policy_switch_timeout_sec_);
    return false;
  }

  const auto response = future.get();
  if (!response->success)
  {
    imitation_policy_ready_ = false;
    current_imitation_policy_.clear();
    RCLCPP_ERROR(logger(), "%s failed to switch imitation policy to '%s': %s",
                 context != nullptr ? context : "Imitation", policy.c_str(), response->message.c_str());
    return false;
  }

  if (!loadImitationWalkingGains(response->active_policy))
  {
    imitation_policy_ready_ = false;
    current_imitation_policy_.clear();
    return false;
  }

  current_imitation_policy_ = response->active_policy;
  imitation_policy_ready_ = true;
  return true;
}

bool PolicyMux::validateAndOrderCommand(const robot_interfaces::msg::PolicyCommand& msg,
                                        std::vector<double>& ordered_positions, uint64_t& last_sequence,
                                        bool& sequence_initialized, const char* topic_label) {
  const rclcpp::Time current = node_->get_clock()->now();
  const rclcpp::Time stamp(msg.header.stamp);
  const rclcpp::Time valid_until(msg.valid_until);
  if (msg.source_id.empty() || msg.joint_names.size() != joint_names_.size() ||
      msg.target_positions.size() != msg.joint_names.size())
  {
    RCLCPP_WARN(logger(), "Rejected %s command: source/name/value count mismatch", topic_label);
    return false;
  }
  if (valid_until <= current || valid_until <= stamp || stamp > current + rclcpp::Duration::from_seconds(0.1))
  {
    RCLCPP_WARN(logger(), "Rejected expired or invalid-time %s command from '%s'", topic_label, msg.source_id.c_str());
    return false;
  }
  if (sequence_initialized && msg.sequence_id <= last_sequence)
  {
    RCLCPP_WARN(logger(), "Rejected out-of-order %s command: sequence=%llu last=%llu", topic_label,
                static_cast<unsigned long long>(msg.sequence_id), static_cast<unsigned long long>(last_sequence));
    return false;
  }

  std::unordered_map<std::string, double> by_name;
  by_name.reserve(msg.joint_names.size());
  for (size_t i = 0; i < msg.joint_names.size(); ++i)
  {
    if (!std::isfinite(msg.target_positions[i]) || !by_name.emplace(msg.joint_names[i], msg.target_positions[i]).second)
    {
      RCLCPP_WARN(logger(), "Rejected %s command: duplicate joint or non-finite value", topic_label);
      return false;
    }
  }
  ordered_positions.clear();
  ordered_positions.reserve(joint_names_.size());
  for (const auto& joint_name : joint_names_)
  {
    const auto it = by_name.find(joint_name);
    if (it == by_name.end())
    {
      RCLCPP_WARN(logger(), "Rejected %s command: missing/unknown joint names", topic_label);
      return false;
    }
    ordered_positions.push_back(it->second);
  }
  last_sequence = msg.sequence_id;
  sequence_initialized = true;
  return true;
}

void PolicyMux::setRlCommand(const robot_interfaces::msg::PolicyCommand& msg) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  if (!amp_return_id_.empty() && msg.amp_return_id != amp_return_id_) return;
  if (rclcpp::Time(pos_cmd_rl_.valid_until) <= node_->get_clock()->now()) rl_sequence_initialized_ = false;
  std::vector<double> ordered_positions;
  if (validateAndOrderCommand(msg, ordered_positions, last_rl_sequence_, rl_sequence_initialized_, "RL"))
  {
    command_received_[0] = std::chrono::steady_clock::now();
    pos_cmd_rl_ = msg;
    pos_cmd_rl_.joint_names.assign(joint_names_.begin(), joint_names_.end());
    pos_cmd_rl_.target_positions.assign(ordered_positions.begin(), ordered_positions.end());
  }
}

void PolicyMux::setImitationCommand(const robot_interfaces::msg::PolicyCommand& msg) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  if (rclcpp::Time(pos_cmd_imitation_.valid_until) <= node_->get_clock()->now())
    imitation_sequence_initialized_ = false;
  std::vector<double> ordered_positions;
  pos_cmd_imitation_valid_ = validateAndOrderCommand(msg, ordered_positions, last_imitation_sequence_,
                                                     imitation_sequence_initialized_, "imitation");
  if (pos_cmd_imitation_valid_)
  {
    command_received_[1] = std::chrono::steady_clock::now();
    pos_cmd_imitation_ = msg;
    pos_cmd_imitation_.joint_names.assign(joint_names_.begin(), joint_names_.end());
    pos_cmd_imitation_.target_positions.assign(ordered_positions.begin(), ordered_positions.end());
  }
}

void PolicyMux::clearCommands(double value) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  amp_return_id_.clear();
  amp_return_seed_ = robot_interfaces::msg::PolicyCommand{};
  fillPolicyCommandPositions(pos_cmd_rl_, value);
  fillPolicyCommandPositions(pos_cmd_imitation_, value);
  fillPolicyCommandPositions(pos_cmd_wbc_, value);
  pos_cmd_rl_.valid_until = builtin_interfaces::msg::Time{};
  pos_cmd_wbc_valid_ = false;
  invalidateImitationCommandLocked();
}

void PolicyMux::resizeCommands(size_t joint_count) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  pos_cmd_rl_.target_positions.assign(joint_count, 0.0);
  pos_cmd_imitation_.target_positions.assign(joint_count, 0.0);
  pos_cmd_wbc_.target_positions.assign(joint_count, 0.0);
  pos_cmd_wbc_valid_ = false;
  invalidateImitationCommandLocked();
}

void PolicyMux::setSource(PolicyCmdSource source, const char* reason) {
  if (source == PolicyCmdSource::kImitation) clearRlReturn();
  const PolicyCmdSource previous = policy_cmd_source_.exchange(source);
  if (reason != nullptr && previous != source)
  {
    RCLCPP_INFO(logger(), "%s", reason);
  }
}

PolicyCmdSource PolicyMux::walkStartPolicySource() const {
  return single_node_publishes_imitation_command_ ? PolicyCmdSource::kImitation : PolicyCmdSource::kRl;
}

PolicyInputs PolicyMux::selectInputs(bool walking) const {
  PolicyInputs out;
  bool use_imitation = false;
  bool use_wbc = false;
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    use_imitation = walking && policy_cmd_source_.load() == PolicyCmdSource::kImitation;
    use_wbc = walking && policy_cmd_source_.load() == PolicyCmdSource::kWbc;
    const auto& selected = use_wbc ? pos_cmd_wbc_ : (use_imitation ? pos_cmd_imitation_ : pos_cmd_rl_);
    out.q_snapshot.assign(selected.target_positions.begin(), selected.target_positions.end());
    out.source = use_wbc ? PolicyCmdSource::kWbc : (use_imitation ? PolicyCmdSource::kImitation : PolicyCmdSource::kRl);
    const auto received = command_received_[use_wbc ? 2 : (use_imitation ? 1 : 0)];
    if (received != std::chrono::steady_clock::time_point{})
      out.last_update_age_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - received).count();
    out.valid_for_ms = std::max(0.0, (rclcpp::Time(selected.valid_until) - node_->get_clock()->now()).seconds() * 1000.0);
    out.valid = !walking || ((!use_wbc || pos_cmd_wbc_valid_) && (!use_imitation || pos_cmd_imitation_valid_) &&
                             rclcpp::Time(selected.valid_until) > node_->get_clock()->now());
  }

  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    out.gains = use_wbc ? wbc_walking_gains_ : (use_imitation ? imitation_walking_gains_ : rl_walking_gains_);
  }
  out.using_imitation = use_imitation;
  return out;
}

JointGainProfile PolicyMux::rlGainsSnapshot() const {
  std::lock_guard<std::mutex> lock(gains_mutex_);
  return rl_walking_gains_;
}

void PolicyMux::fillObservationPolicyCommand(
    robot_interfaces::msg::PolicyObservation& observation) const {
  using Observation = robot_interfaces::msg::PolicyObservation;
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  observation.amp_return_id = amp_return_id_;
  if (amp_return_live_preview_) observation.velocity_command = geometry_msgs::msg::Twist{};
  const auto source = policy_cmd_source_.load();
  observation.source_policy = source == PolicyCmdSource::kWbc ? Observation::SOURCE_WBC :
      (source == PolicyCmdSource::kImitation ? Observation::SOURCE_IMITATION : Observation::SOURCE_AMP);
  observation.source_command = source == PolicyCmdSource::kWbc ?
      (amp_return_id_.empty() || amp_return_live_preview_ ? pos_cmd_wbc_ : amp_return_seed_) :
      (source == PolicyCmdSource::kImitation ? pos_cmd_imitation_ : pos_cmd_rl_);
  if (!observation.inference_enabled ||
      (source == PolicyCmdSource::kWbc && !pos_cmd_wbc_valid_) ||
      (source == PolicyCmdSource::kImitation && !pos_cmd_imitation_valid_) ||
      rclcpp::Time(observation.source_command.valid_until) <= node_->get_clock()->now())
  {
    observation.source_policy = Observation::SOURCE_NONE;
    observation.source_command = robot_interfaces::msg::PolicyCommand{};
  }
}

std::string PolicyMux::beginRlReturn(bool live_preview) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  if (!amp_return_id_.empty()) return amp_return_id_;
  amp_return_live_preview_ = live_preview;
  amp_return_id_ = wbc_request_id_.empty() ? "amp_recovery_" + std::to_string(node_->now().nanoseconds()) : wbc_request_id_;
  amp_return_seed_ = pos_cmd_wbc_valid_ && pos_cmd_wbc_.source_id == amp_return_id_ ?
      pos_cmd_wbc_ : robot_interfaces::msg::PolicyCommand{};
  // This request ID is new for AMP; no output computed before the return may take over.
  pos_cmd_rl_.valid_until = builtin_interfaces::msg::Time{};
  return amp_return_id_;
}

void PolicyMux::finishRlReturnPreview() {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  amp_return_live_preview_ = false;
}

void PolicyMux::clearRlReturn() {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  amp_return_live_preview_ = false;
  amp_return_id_.clear();
  amp_return_seed_ = robot_interfaces::msg::PolicyCommand{};
}

void PolicyMux::publishRlObsReset() {
  if (!rl_obs_history_reset_pub_) return;

  std_msgs::msg::Bool reset_msg;
  reset_msg.data = true;
  rl_obs_history_reset_pub_->publish(reset_msg);
}

void PolicyMux::publishImitationReset() {
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    invalidateImitationCommandLocked();
  }
  if (!rl_imitation_reset_pub_) return;
  std_msgs::msg::Bool reset_msg;
  reset_msg.data = true;
  rl_imitation_reset_pub_->publish(reset_msg);
}

void PolicyMux::publishImitationActive(bool active, const char* reason) {
  imitation_active_commanded_.store(active);
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    invalidateImitationCommandLocked();
  }
  if (!rl_imitation_active_pub_) return;

  std_msgs::msg::Bool msg;
  msg.data = active;
  rl_imitation_active_pub_->publish(msg);
  if (reason != nullptr)
  {
    RCLCPP_INFO(logger(), "%s (control/policy/imitation/active=%s)", reason, active ? "true" : "false");
  }
}

void PolicyMux::returnToRlWithoutGainRamp(const char* reason) {
  publishRlObsReset();
  setSource(PolicyCmdSource::kRl, reason);
  publishImitationActive(false);
}

bool PolicyMux::loadWbcWalkingGains(const std::string& robot_cfg_path, const std::string& motion_id) {
  if (robot_cfg_path.empty())
  {
    RCLCPP_ERROR(logger(), "[WBC] Empty robot_cfg_path returned for motion '%s'", motion_id.c_str());
    return false;
  }
  try
  {
    const YAML::Node root = YAML::LoadFile(robot_cfg_path);
    const YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    JointGainProfile gains;
    if (!loadWalkingGainProfile(rc, "RobotCfg(wbc:" + motion_id + ")", joint_names_.size(), gains)) return false;
    const auto valid_gain = [](double value) { return std::isfinite(value) && value >= 0.0; };
    if (!std::all_of(gains.kps.begin(), gains.kps.end(), valid_gain) ||
        !std::all_of(gains.kds.begin(), gains.kds.end(), valid_gain))
    {
      RCLCPP_ERROR(logger(), "[WBC] Gains for '%s' must be finite and non-negative", motion_id.c_str());
      return false;
    }
    std::lock_guard<std::mutex> lock(gains_mutex_);
    wbc_walking_gains_ = std::move(gains);
    return true;
  } catch (const std::exception& exception)
  {
    RCLCPP_ERROR(logger(), "[WBC] Failed to load gains for '%s' from %s: %s", motion_id.c_str(), robot_cfg_path.c_str(),
                 exception.what());
    return false;
  }
}

std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Response> PolicyMux::prepareWbcMotion(
    uint8_t command, const std::string& motion_id, double timeout_sec, const std::string& request_id) {
  if (!prepare_wbc_motion_client_ || !prepare_wbc_motion_client_->wait_for_service(std::chrono::milliseconds(0)))
  {
    RCLCPP_ERROR(logger(), "[WBC] control/policy/wbc/prepare service is unavailable");
    return nullptr;
  }
  auto request = std::make_shared<robot_interfaces::srv::ControlWbcMotion::Request>();
  request->command = command;
  request->motion_id = motion_id;
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    if (command == robot_interfaces::srv::ControlWbcMotion::Request::START ||
        command == robot_interfaces::srv::ControlWbcMotion::Request::REFERENCE) {
      if (request_id.empty()) return nullptr;
      wbc_request_id_ = request_id;
      amp_return_id_.clear();
      amp_return_live_preview_ = false;
      amp_return_seed_ = robot_interfaces::msg::PolicyCommand{};
      pos_cmd_wbc_valid_ = false;
      wbc_sequence_initialized_ = false;
    }
    request->request_id = request_id.empty() ? wbc_request_id_ : request_id;
  }
  auto future = prepare_wbc_motion_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout_sec)) != std::future_status::ready)
  {
    RCLCPP_ERROR(logger(), "[WBC] control/policy/wbc/prepare timed out after %.2fs", timeout_sec);
    return nullptr;
  }
  return future.get();
}

void PolicyMux::publishWbcActive(bool active, const char* reason) {
  wbc_active_commanded_.store(active);
  if (wbc_active_pub_)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    wbc_active_pub_->publish(message);
  }
  if (wbc_execution_pub_)
  {
    std_msgs::msg::String message;
    {
      std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
      message.data = "{\"request_id\":\"" + wbc_request_id_ + "\",\"active\":" + (active ? "true}" : "false}");
    }
    wbc_execution_pub_->publish(message);
  }
  if (reason != nullptr) RCLCPP_DEBUG(logger(), "%s (control/policy/wbc/active=%s)", reason, active ? "true" : "false");
}

void PolicyMux::setWbcCommand(const robot_interfaces::msg::PolicyCommand& msg) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  // Freeze the valid return seed; a worker retiring asynchronously must not change the bridge command.
  if (wbc_request_id_.empty() || msg.source_id != wbc_request_id_ ||
      (!amp_return_id_.empty() && !(amp_return_live_preview_ && policy_cmd_source_.load() == PolicyCmdSource::kWbc))) return;
  if (rclcpp::Time(pos_cmd_wbc_.valid_until) <= node_->get_clock()->now()) wbc_sequence_initialized_ = false;
  std::vector<double> ordered;
  // Reject a bad incoming command without invalidating the last accepted command before its own deadline.
  if (!validateAndOrderCommand(msg, ordered, last_wbc_sequence_, wbc_sequence_initialized_, "WBC")) return;
  pos_cmd_wbc_valid_ = true;
  command_received_[2] = std::chrono::steady_clock::now();
  pos_cmd_wbc_ = msg;
  pos_cmd_wbc_.joint_names.assign(joint_names_.begin(), joint_names_.end());
  pos_cmd_wbc_.target_positions.assign(ordered.begin(), ordered.end());
  ++wbc_command_revision_;
  command_cv_.notify_all();
}

uint64_t PolicyMux::wbcCommandRevision() const {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  return wbc_command_revision_;
}

bool PolicyMux::rlReturnCommandReady(const std::string& return_id) const {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  return !return_id.empty() && amp_return_id_ == return_id && pos_cmd_rl_.amp_return_id == return_id &&
         rclcpp::Time(pos_cmd_rl_.valid_until) > node_->get_clock()->now();
}

bool PolicyMux::waitForWbcCommandAfter(uint64_t revision, double timeout_sec) {
  std::unique_lock<std::mutex> lock(pos_cmd_mutex_);
  const auto ready = [this, revision]() {
    return pos_cmd_wbc_valid_ && wbc_command_revision_ > revision &&
           rclcpp::Time(pos_cmd_wbc_.valid_until) > node_->get_clock()->now();
  };
  if (timeout_sec <= 0.0) return ready();
  return command_cv_.wait_for(lock, std::chrono::duration<double>(timeout_sec), ready);
}

}  // namespace legged
