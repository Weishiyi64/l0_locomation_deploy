#include "legged_controllers/PolicyMux.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <utility>

#include "legged_controllers/ControllerFileUtils.h"
#include "legged_controllers/IHardwareLayer.h"

namespace legged {

namespace {

void fillPosCmdPositions(legged_msgs::msg::PosCmd& cmd, double value) { std::fill(cmd.q.begin(), cmd.q.end(), value); }

}  // namespace

rclcpp::Logger PolicyMux::logger() const { return node_ ? node_->get_logger() : rclcpp::get_logger("policy_mux"); }

void PolicyMux::init(rclcpp::Node::SharedPtr node, size_t joint_count) {
  node_ = std::move(node);
  resizeCommands(joint_count);
}

void PolicyMux::configureStartup(bool single_node_publishes_imitation_poscmd, const JointGainProfile& rl_gains,
                                 double policy_switch_timeout_sec) {
  single_node_publishes_imitation_poscmd_ = single_node_publishes_imitation_poscmd;
  imitation_policy_switch_timeout_sec_ = policy_switch_timeout_sec;
  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    rl_walking_gains_ = rl_gains;
    imitation_walking_gains_ = rl_gains;
  }
}

void PolicyMux::invalidateImitationCommandLocked() { pos_cmd_imitation_valid_ = false; }

void PolicyMux::createRosInterfaces() {
  if (!node_) return;

  rl_imitation_reset_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rlImitationReset", rclcpp::QoS(10));
  rl_imitation_active_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("/rlImitationActive", rclcpp::QoS(10).reliable().transient_local());
  rl_obs_history_reset_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rl_obs_history_reset", rclcpp::QoS(10));
  if (!imitation_policy_client_callback_group_)
  {
    imitation_policy_client_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }
  switch_imitation_policy_client_ = node_->create_client<legged_msgs::srv::SwitchImitationPolicy>(
      "/switch_imitation_policy", rmw_qos_profile_services_default, imitation_policy_client_callback_group_);
  publishImitationActive(false);
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

bool PolicyMux::loadImitationWalkingGains(const std::string& robot_cfg_path, const std::string& policy_name) {
  if (robot_cfg_path.empty())
  {
    RCLCPP_ERROR(logger(), "[ImitationPolicy] Empty robot_cfg_path returned for policy '%s'", policy_name.c_str());
    return false;
  }

  try
  {
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    JointGainProfile next_gains;
    if (!loadWalkingGainProfile(rc, "RobotCfg(imitation:" + policy_name + ")", pos_cmd_rl_.q.size(), next_gains))
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
  if (policy.empty())
  {
    RCLCPP_ERROR(logger(), "%s failed: empty imitation policy", context != nullptr ? context : "Imitation");
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
    RCLCPP_ERROR(logger(), "%s failed: /switch_imitation_policy service is not available",
                 context != nullptr ? context : "Imitation");
    return false;
  }

  auto request = std::make_shared<legged_msgs::srv::SwitchImitationPolicy::Request>();
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

  if (!loadImitationWalkingGains(response->robot_cfg_path, response->policy_name))
  {
    imitation_policy_ready_ = false;
    current_imitation_policy_.clear();
    return false;
  }

  current_imitation_policy_ = response->policy_name;
  imitation_policy_ready_ = true;
  return true;
}

void PolicyMux::setRlCommand(const legged_msgs::msg::PosCmd& msg) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  pos_cmd_rl_.q = msg.q;
}

void PolicyMux::setImitationCommand(const legged_msgs::msg::PosCmd& msg) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  pos_cmd_imitation_.q = msg.q;
  pos_cmd_imitation_valid_ = !pos_cmd_imitation_.q.empty();
}

void PolicyMux::clearCommands(double value) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  fillPosCmdPositions(pos_cmd_rl_, value);
  fillPosCmdPositions(pos_cmd_imitation_, value);
  invalidateImitationCommandLocked();
}

void PolicyMux::resizeCommands(size_t joint_count) {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
  pos_cmd_rl_.q.assign(joint_count, 0.0);
  pos_cmd_imitation_.q.assign(joint_count, 0.0);
  invalidateImitationCommandLocked();
}

void PolicyMux::setSource(PolicyCmdSource source, const char* reason) {
  const PolicyCmdSource previous = policy_cmd_source_.exchange(source);
  if (reason != nullptr && previous != source)
  {
    RCLCPP_INFO(logger(), "%s", reason);
  }
}

PolicyCmdSource PolicyMux::walkStartPolicySource() const {
  return single_node_publishes_imitation_poscmd_ ? PolicyCmdSource::kImitation : PolicyCmdSource::kRl;
}

PolicyInputs PolicyMux::selectInputs(bool walking) const {
  PolicyInputs out;
  bool use_imitation = false;
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    use_imitation = walking && (policy_cmd_source_.load() == PolicyCmdSource::kImitation) && pos_cmd_imitation_valid_;
    out.q_snapshot = use_imitation ? pos_cmd_imitation_.q : pos_cmd_rl_.q;
  }

  {
    std::lock_guard<std::mutex> lock(gains_mutex_);
    out.gains = use_imitation ? imitation_walking_gains_ : rl_walking_gains_;
  }
  out.using_imitation = use_imitation;
  return out;
}

JointGainProfile PolicyMux::rlGainsSnapshot() const {
  std::lock_guard<std::mutex> lock(gains_mutex_);
  return rl_walking_gains_;
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
    RCLCPP_INFO(logger(), "%s (/rlImitationActive=%s)", reason, active ? "true" : "false");
  }
}

void PolicyMux::returnToRlWithoutGainRamp(const char* reason) {
  publishRlObsReset();
  setSource(PolicyCmdSource::kRl, reason);
  publishImitationActive(false);
}

}  // namespace legged
