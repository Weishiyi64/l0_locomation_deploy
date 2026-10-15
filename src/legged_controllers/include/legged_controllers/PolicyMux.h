#pragma once

#include <legged_controllers/ControllerTypes.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <legged_msgs/msg/pos_cmd.hpp>
#include <legged_msgs/srv/switch_imitation_policy.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

namespace legged {

class PolicyMux {
 public:
  PolicyMux() = default;

  void init(rclcpp::Node::SharedPtr node, size_t joint_count);
  void configureStartup(bool single_node_publishes_imitation_poscmd, const JointGainProfile& rl_gains,
                        double policy_switch_timeout_sec);
  void createRosInterfaces();

  bool loadWalkingGainProfile(const YAML::Node& rc, const std::string& profile_name, size_t expected_size,
                              JointGainProfile& out) const;
  bool loadImitationWalkingGains(const std::string& robot_cfg_path, const std::string& policy_name);
  bool switchImitationPolicy(const std::string& policy_name, const char* context);

  void setRlCommand(const legged_msgs::msg::PosCmd& msg);
  void setImitationCommand(const legged_msgs::msg::PosCmd& msg);
  void clearCommands(double value);
  void resizeCommands(size_t joint_count);

  void setSource(PolicyCmdSource source, const char* reason = nullptr);
  PolicyCmdSource source() const { return policy_cmd_source_.load(); }
  PolicyCmdSource walkStartPolicySource() const;
  PolicyInputs selectInputs(bool walking) const;
  JointGainProfile rlGainsSnapshot() const;

  void publishRlObsReset();
  void publishImitationReset();
  void publishImitationActive(bool active, const char* reason = nullptr);
  void returnToRlWithoutGainRamp(const char* reason);

  bool singleNodePublishesImitationPosCmd() const { return single_node_publishes_imitation_poscmd_; }
  bool imitationActiveCommanded() const { return imitation_active_commanded_.load(); }

 private:
  rclcpp::Logger logger() const;
  void invalidateImitationCommandLocked();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_imitation_reset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_imitation_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_obs_history_reset_pub_;
  rclcpp::Client<legged_msgs::srv::SwitchImitationPolicy>::SharedPtr switch_imitation_policy_client_;
  rclcpp::CallbackGroup::SharedPtr imitation_policy_client_callback_group_;

  mutable std::mutex pos_cmd_mutex_;
  legged_msgs::msg::PosCmd pos_cmd_rl_;
  legged_msgs::msg::PosCmd pos_cmd_imitation_;
  bool pos_cmd_imitation_valid_{false};
  std::atomic<PolicyCmdSource> policy_cmd_source_{PolicyCmdSource::kRl};
  bool single_node_publishes_imitation_poscmd_{false};

  mutable std::mutex gains_mutex_;
  JointGainProfile rl_walking_gains_;
  JointGainProfile imitation_walking_gains_;
  std::string current_imitation_policy_;
  bool imitation_policy_ready_{false};
  double imitation_policy_switch_timeout_sec_{120.0};
  std::atomic<bool> imitation_active_commanded_{false};
};

}  // namespace legged
